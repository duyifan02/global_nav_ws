#include <memory>
#include <mutex>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

#include "jps_planner_3d/jps_3d.hpp"
#include "jps_planner_3d/voxel_map.hpp"
#include "jps_planner_3d/srv/set_voxel_map.hpp"
#include "jps_planner_3d/srv/plan_path.hpp"

namespace jps_planner_3d
{

class JpsPlannerNode : public rclcpp::Node
{
public:
  JpsPlannerNode()
  : Node("jps_planner_node")
  {
    declare_parameter<double>("connectivity_height_threshold", 0.6);
    declare_parameter<double>("surface_clearance_height", 0.8);
    declare_parameter<int>("distance_field_max_distance", 5);
    declare_parameter<double>("heuristic_weight", 1.0);
    declare_parameter<double>("max_z_change", 0.5);
    declare_parameter<double>("z_penalty_weight", 3.0);
    declare_parameter<double>("potential_weight", 100.0);
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
      get_parameter("max_z_change").as_double(),
      get_parameter("z_penalty_weight").as_double(),
      get_parameter("potential_weight").as_double());
  }

  void handleSetVoxelMap(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<srv::SetVoxelMap::Request> request,
    std::shared_ptr<srv::SetVoxelMap::Response> response)
  {
    if (request->resolution <= 0.0 || request->free_space_margin < 0.0) {
      response->success = false;
      response->message = "Resolution must be > 0 and free_space_margin must be >= 0";
      return;
    }

    std::vector<std::tuple<float, float, float>> voxels;
    voxels.reserve(request->occupied.size());
    for (const auto & p : request->occupied) {
      voxels.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
    }

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      voxel_map_->setConnectivityHeightThreshold(get_parameter("connectivity_height_threshold").as_double());
      voxel_map_->setSurfaceClearanceHeight(get_parameter("surface_clearance_height").as_double());
      voxel_map_->setDistanceFieldMaxDistance(get_parameter("distance_field_max_distance").as_int());
      voxel_map_->setFreeSpaceMargin(request->free_space_margin);

      bool updated = voxel_map_->updateFromPoints(voxels, request->resolution);
      if (!updated) {
        response->success = false;
        response->message = "Failed to update voxel map (empty data?)";
        return;
      }
      rebuildPlanner();
    }

    response->success = true;
    response->message = "Voxel map updated";
    RCLCPP_INFO(get_logger(), "Voxel map updated: %zu occupied voxels", request->occupied.size());
  }

  void handlePlanPath(
    const std::shared_ptr<rmw_request_id_t> /*unused*/,
    const std::shared_ptr<srv::PlanPath::Request> request,
    std::shared_ptr<srv::PlanPath::Response> response)
  {
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
      return;
    }

    const auto & s = request->start.position;
    const auto & g = request->goal.position;

    auto path = planner_copy->plan(
      {s.x, s.y, s.z},
      {g.x, g.y, g.z},
      request->snap_to_surface);

    if (path.empty()) {
      response->success = false;
      const auto stats = planner_copy->getSearchStats();
      auto it = stats.find("failure_reason");
      if (stats.find("cancelled") != stats.end()) {
        response->message = "Planning was cancelled";
      } else if (it != stats.end()) {
        switch (it->second) {
          case 1: response->message = "Start or goal outside voxel map"; break;
          case 2: response->message = "Start is not on a valid surface"; break;
          case 3: response->message = "Goal is not on a valid surface"; break;
          default: response->message = "Planner returned empty path"; break;
        }
      } else {
        response->message = "Planner returned empty path";
      }
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
  }

  std::shared_ptr<VoxelMap> voxel_map_;
  std::shared_ptr<JPS3D> planner_;
  std::mutex map_mutex_;

  rclcpp::Service<srv::SetVoxelMap>::SharedPtr set_map_service_;
  rclcpp::Service<srv::PlanPath>::SharedPtr plan_service_;
};

}  // namespace jps_planner_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jps_planner_3d::JpsPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
