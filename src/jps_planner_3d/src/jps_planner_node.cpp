#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

#include "jps_planner_3d/jps_3d.hpp"
#include "jps_planner_3d/voxel_map.hpp"
#include "nav_interfaces/srv/set_voxel_map.hpp"
#include "nav_interfaces/srv/plan_path.hpp"
#include "nav_interfaces/srv/get_distance_field.hpp"

namespace jps_planner_3d
{

namespace srv = nav_interfaces::srv;

class JpsPlannerNode : public rclcpp::Node
{
public:
  JpsPlannerNode()
  : Node("jps_planner_node")
  {
    declare_parameter<double>("connectivity_height_threshold", 0.3);
    declare_parameter<double>("surface_clearance_height", 1.6);
    declare_parameter<int>("distance_field_max_distance", 10);
    declare_parameter<double>("heuristic_weight", 1.0);
    declare_parameter<double>("w_up", 2.0);
    declare_parameter<double>("w_down", 1.0);
    declare_parameter<double>("w_obs", 0.5);
    declare_parameter<std::string>("frame_id", "map");

    voxel_map_ = std::make_shared<VoxelMap>();
    configureVoxelMapParameters();
    rebuildPlanner();

    set_map_service_ = create_service<srv::SetVoxelMap>(
      "set_voxel_map",
      std::bind(&JpsPlannerNode::handleSetVoxelMap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    plan_service_ = create_service<srv::PlanPath>(
      "plan_path",
      std::bind(&JpsPlannerNode::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_distance_field_service_ = create_service<srv::GetDistanceField>(
      "get_distance_field",
      std::bind(&JpsPlannerNode::handleGetDistanceField, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "JPS planner node ready. Waiting for voxel map...");
  }

private:
  void configureVoxelMapParameters()
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    voxel_map_->setConnectivityHeightThreshold(get_parameter("connectivity_height_threshold").as_double());
    voxel_map_->setSurfaceClearanceHeight(get_parameter("surface_clearance_height").as_double());
    voxel_map_->setDistanceFieldMaxDistance(get_parameter("distance_field_max_distance").as_int());
  }

  void rebuildPlanner()
  {
    planner_ = std::make_shared<JPS3D>(
      voxel_map_,
      get_parameter("heuristic_weight").as_double(),
      get_parameter("w_up").as_double(),
      get_parameter("w_down").as_double(),
      get_parameter("w_obs").as_double());
  }

  void handleSetVoxelMap(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<srv::SetVoxelMap::Request> request,
    std::shared_ptr<srv::SetVoxelMap::Response> response)
  {
    RCLCPP_INFO(get_logger(), "set_voxel_map: occupied=%zu, res=%.3f, seed=(%.2f, %.2f, %.2f)",
      request->occupied.size(), request->resolution,
      request->seed_point.x, request->seed_point.y, request->seed_point.z);

    if (request->resolution <= 0.0) {
      response->success = false;
      response->message = "Resolution must be > 0";
      response->surface_voxel_count = 0;
      return;
    }

    std::vector<std::tuple<float, float, float>> voxels;
    voxels.reserve(request->occupied.size());
    for (const auto & p : request->occupied) {
      voxels.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }

    std::tuple<float, float, float> seed_point = {
      static_cast<float>(request->seed_point.x),
      static_cast<float>(request->seed_point.y),
      static_cast<float>(request->seed_point.z)
    };
    
    bool use_seed = (request->seed_point.x != 0.0 || 
                     request->seed_point.y != 0.0 || 
                     request->seed_point.z != 0.0);

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      voxel_map_->setConnectivityHeightThreshold(get_parameter("connectivity_height_threshold").as_double());
      voxel_map_->setSurfaceClearanceHeight(get_parameter("surface_clearance_height").as_double());
      voxel_map_->setDistanceFieldMaxDistance(get_parameter("distance_field_max_distance").as_int());

      bool updated = voxel_map_->updateFromPoints(voxels, request->resolution, 
                                                   use_seed ? &seed_point : nullptr);
      if (!updated) {
        response->success = false;
        response->message = "Failed to update voxel map (empty data?)";
        response->surface_voxel_count = 0;
        return;
      }
      rebuildPlanner();
    }

    const auto surface_count = voxel_map_->getValidSurfaceVoxels().size();
    response->success = true;
    response->message = "Voxel map updated";
    response->surface_voxel_count = static_cast<int64_t>(surface_count);
    response->traversability_time_ms = voxel_map_->getTraversabilityTimeMs();
    
    auto [nx, ny, nz] = voxel_map_->getGridSize();
    RCLCPP_INFO(get_logger(),
      "Voxel map ready: occupied=%zu, surface=%zu, grid=%d x %d x %d, res=%.3f",
      request->occupied.size(), surface_count, nx, ny, nz, request->resolution);
  }

  void handlePlanPath(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<srv::PlanPath::Request> request,
    std::shared_ptr<srv::PlanPath::Response> response)
  {
    RCLCPP_INFO(get_logger(),
      "plan_path: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) snap=%s",
      request->start.position.x, request->start.position.y, request->start.position.z,
      request->goal.position.x, request->goal.position.y, request->goal.position.z,
      request->snap_to_surface ? "true" : "false");

    std::shared_ptr<JPS3D> planner_copy;
    std::shared_ptr<VoxelMap> map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      planner_copy = planner_;
      map_copy = voxel_map_;
    }

    if (!map_copy || !map_copy->isReady()) {
      response->success = false;
      response->message = "Voxel map is not ready";
      response->planning_time_ms = 0.0;
      response->expanded_nodes = 0;
      response->memory_usage_bytes = 0;
      return;
    }

    const auto & s = request->start.position;
    const auto & g = request->goal.position;

    auto t_start = std::chrono::high_resolution_clock::now();
    
    auto path = planner_copy->plan(
      {s.x, s.y, s.z},
      {g.x, g.y, g.z},
      request->snap_to_surface);
    
    auto t_end = std::chrono::high_resolution_clock::now();
    double planning_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    int64_t expanded_nodes = planner_copy->getExpandedNodes();
    
    if (path.empty()) {
      response->success = false;
      response->message = "Planner returned empty path";
      response->planning_time_ms = planning_time_ms;
      response->expanded_nodes = expanded_nodes;
      response->memory_usage_bytes = 0;
      RCLCPP_WARN(get_logger(), "plan_path failed: %s (%.2f ms) expanded=%ld",
                  response->message.c_str(), planning_time_ms, expanded_nodes);
      return;
    }

    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = get_parameter("frame_id").as_string();
    ros_path.header.stamp = now();
    ros_path.poses.reserve(path.size());

    for (const auto & p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = ros_path.header;
      pose.pose.position.x = std::get<0>(p);
      pose.pose.position.y = std::get<1>(p);
      pose.pose.position.z = std::get<2>(p);
      pose.pose.orientation.w = 1.0;
      ros_path.poses.push_back(pose);
    }

    response->success = true;
    response->message = "Path planned";
    response->path = std::move(ros_path);
    response->planning_time_ms = planning_time_ms;
    response->expanded_nodes = expanded_nodes;
    response->memory_usage_bytes = 0;

    RCLCPP_INFO(get_logger(), "plan_path success: poses=%zu, time=%.2f ms, expanded=%ld", 
                path.size(), planning_time_ms, expanded_nodes);
  }

  void handleGetDistanceField(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<srv::GetDistanceField::Request> /*request*/,
    std::shared_ptr<srv::GetDistanceField::Response> response)
  {
    RCLCPP_INFO(get_logger(), "get_distance_field: retrieving distance field data");

    std::shared_ptr<VoxelMap> map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_copy = voxel_map_;
    }

    if (!map_copy || !map_copy->isReady()) {
      response->success = false;
      response->message = "Voxel map is not ready";
      return;
    }

    // Get surface voxels
    const auto& surface_voxels = map_copy->getValidSurfaceVoxels();
    
    if (surface_voxels.empty()) {
      response->success = false;
      response->message = "No surface voxels available";
      return;
    }

    // Populate response
    response->resolution = map_copy->getResolution();
    response->surface_points.reserve(surface_voxels.size());
    response->distances.reserve(surface_voxels.size());

    for (const auto& voxel : surface_voxels) {
      auto [ix, iy, iz] = voxel;
      auto [x, y, z] = map_copy->gridToWorld(ix, iy, iz);
      
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      response->surface_points.push_back(pt);
      
      // Get distance value
      int dist = map_copy->getDistanceToObstacle(ix, iy, iz);
      response->distances.push_back(static_cast<uint8_t>(dist));
    }

    response->success = true;
    response->message = "Distance field retrieved";
    
    RCLCPP_INFO(get_logger(), "get_distance_field success: %zu surface points", 
                surface_voxels.size());
  }

  std::shared_ptr<VoxelMap> voxel_map_;
  std::shared_ptr<JPS3D> planner_;
  std::mutex map_mutex_;

  rclcpp::Service<srv::SetVoxelMap>::SharedPtr set_map_service_;
  rclcpp::Service<srv::PlanPath>::SharedPtr plan_service_;
  rclcpp::Service<srv::GetDistanceField>::SharedPtr get_distance_field_service_;
};

}  // namespace jps_planner_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jps_planner_3d::JpsPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
