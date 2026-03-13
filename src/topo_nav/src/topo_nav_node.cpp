#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>

#include "jps_planner_3d/voxel_map.hpp"
#include "jps_planner_3d/pcd_voxel_loader.hpp"   // loadAndVoxelizePcd, PcdVoxelizeResult

#include "topo_nav/persistence.hpp"
#include "topo_nav/room_segmenter.hpp"
#include "topo_nav/floor_detector.hpp"
#include "topo_nav/hierarchical_planner.hpp"

// Reuse existing map-loading services from nav_interfaces
#include "nav_interfaces/srv/set_voxel_map.hpp"
#include "nav_interfaces/srv/load_pcd_map.hpp"
// New services
#include "nav_interfaces/srv/build_topo_map.hpp"
#include "nav_interfaces/srv/get_room_labels.hpp"
#include "nav_interfaces/srv/plan_hierarchical.hpp"

namespace topo_nav
{

namespace srv = nav_interfaces::srv;

class TopoNavNode : public rclcpp::Node
{
public:
  TopoNavNode()
  : Node("topo_nav_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("connectivity_height_threshold", 0.3);
    declare_parameter<double>("surface_clearance_height", 1.6);
    declare_parameter<int>("distance_field_max_distance", 10);
    declare_parameter<double>("w_up",   2.0);
    declare_parameter<double>("w_down", 1.0);
    declare_parameter<double>("w_obs",  0.5);
    // Default topo-map parameters
    declare_parameter<double>("persistence_tau",       3.0);
    declare_parameter<double>("floor_separation_m",    2.5);
    declare_parameter<double>("corridor_aspect_ratio", 3.0);
    declare_parameter<double>("staircase_variance_m",  0.5);
    declare_parameter<double>("robot_diameter_m",      0.4);

    // ── Shared map ──────────────────────────────────────────────────────────
    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();
    rebuildPlanner();

    // ── Services ────────────────────────────────────────────────────────────
    set_map_srv_ = create_service<srv::SetVoxelMap>(
      "topo_nav/set_voxel_map",
      std::bind(&TopoNavNode::handleSetVoxelMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    load_pcd_srv_ = create_service<srv::LoadPcdMap>(
      "topo_nav/load_pcd_map",
      std::bind(&TopoNavNode::handleLoadPcdMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    build_topo_srv_ = create_service<srv::BuildTopoMap>(
      "topo_nav/build_topo_map",
      std::bind(&TopoNavNode::handleBuildTopoMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_labels_srv_ = create_service<srv::GetRoomLabels>(
      "topo_nav/get_room_labels",
      std::bind(&TopoNavNode::handleGetRoomLabels, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    plan_hier_srv_ = create_service<srv::PlanHierarchical>(
      "topo_nav/plan_hierarchical",
      std::bind(&TopoNavNode::handlePlanHierarchical, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "TopoNav node ready. Call set_voxel_map or "
                              "load_pcd_map, then build_topo_map.");
  }

private:
  // ── Configuration ─────────────────────────────────────────────────────────
  void configureMap()
  {
    voxel_map_->setConnectivityHeightThreshold(
      get_parameter("connectivity_height_threshold").as_double());
    voxel_map_->setSurfaceClearanceHeight(
      get_parameter("surface_clearance_height").as_double());
    voxel_map_->setDistanceFieldMaxDistance(
      get_parameter("distance_field_max_distance").as_int());
  }

  void rebuildPlanner()
  {
    planner_ = std::make_unique<HierarchicalPlanner>(
      voxel_map_,
      get_parameter("w_up").as_double(),
      get_parameter("w_down").as_double(),
      get_parameter("w_obs").as_double());
  }

  // ── SetVoxelMap ───────────────────────────────────────────────────────────
  void handleSetVoxelMap(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::SetVoxelMap::Request>  req,
    const std::shared_ptr<srv::SetVoxelMap::Response> res)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    std::vector<std::tuple<float, float, float>> occupied;
    occupied.reserve(req->occupied.size());
    for (const auto & p : req->occupied) {
      occupied.emplace_back(
        static_cast<float>(p.x),
        static_cast<float>(p.y),
        static_cast<float>(p.z));
    }

    const std::tuple<float, float, float> * seed_ptr = nullptr;
    std::tuple<float, float, float> seed;
    if (req->seed_point.x != 0.0 || req->seed_point.y != 0.0 ||
        req->seed_point.z != 0.0)
    {
      seed = {static_cast<float>(req->seed_point.x),
              static_cast<float>(req->seed_point.y),
              static_cast<float>(req->seed_point.z)};
      seed_ptr = &seed;
    }

    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();

    const bool ok = voxel_map_->updateFromPoints(
      occupied, req->resolution, seed_ptr);

    rebuildPlanner();  // new planner owns new voxel_map_

    res->success             = ok;
    res->message             = ok ? "Map loaded." : "Failed to load map.";
    res->surface_voxel_count = static_cast<int64_t>(
      voxel_map_->getValidSurfaceVoxels().size());
    res->traversability_time_ms = voxel_map_->getTraversabilityTimeMs();
    res->map_memory_usage_bytes = static_cast<int64_t>(
      voxel_map_->getConstructedSceneMemoryUsageBytes());

    RCLCPP_INFO(get_logger(), "SetVoxelMap: %s (%zu surface voxels)",
      res->message.c_str(),
      voxel_map_->getValidSurfaceVoxels().size());
  }

  // ── LoadPcdMap ────────────────────────────────────────────────────────────
  void handleLoadPcdMap(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::LoadPcdMap::Request>  req,
    const std::shared_ptr<srv::LoadPcdMap::Response> res)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    jps_planner_3d::PcdVoxelizeResult pcd_result;
    std::string err_msg;
    const auto t0 = std::chrono::steady_clock::now();
    const bool ok = jps_planner_3d::loadAndVoxelizePcd(
      req->pcd_path, req->resolution,
      req->min_points_per_voxel, req->max_abs_coord,
      pcd_result, err_msg);
    const auto t1 = std::chrono::steady_clock::now();

    if (!ok) {
      res->success = false;
      res->message = "PCD load failed: " + err_msg;
      return;
    }

    const double Tp =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

    const std::tuple<float, float, float> * seed_ptr = nullptr;
    std::tuple<float, float, float> seed;
    if (req->seed_point.x != 0.0 || req->seed_point.y != 0.0 ||
        req->seed_point.z != 0.0)
    {
      seed = {static_cast<float>(req->seed_point.x),
              static_cast<float>(req->seed_point.y),
              static_cast<float>(req->seed_point.z)};
      seed_ptr = &seed;
    }

    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();
    voxel_map_->updateFromPoints(pcd_result.occupied_voxels, req->resolution, seed_ptr);
    rebuildPlanner();

    res->success               = true;
    res->message               = "PCD map loaded.";
    res->raw_point_count       = pcd_result.raw_point_count;
    res->occupied_voxel_count  = static_cast<int64_t>(pcd_result.occupied_voxels.size());
    res->surface_voxel_count   = static_cast<int64_t>(
      voxel_map_->getValidSurfaceVoxels().size());
    res->preprocessing_time_ms = Tp;
    res->traversability_time_ms = voxel_map_->getTraversabilityTimeMs();
    res->map_memory_usage_bytes = static_cast<int64_t>(
      voxel_map_->getConstructedSceneMemoryUsageBytes());

    RCLCPP_INFO(get_logger(), "LoadPcdMap: %zu surface voxels from %s",
      voxel_map_->getValidSurfaceVoxels().size(), req->pcd_path.c_str());
  }

  // ── BuildTopoMap ──────────────────────────────────────────────────────────
  void handleBuildTopoMap(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::BuildTopoMap::Request>  req,
    const std::shared_ptr<srv::BuildTopoMap::Response> res)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!voxel_map_->isReady()) {
      res->success = false;
      res->message = "No map loaded. Call set_voxel_map or load_pcd_map first.";
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    // Algorithm 1: persistent homology
    const float tau = static_cast<float>(req->persistence_tau > 0
      ? req->persistence_tau
      : get_parameter("persistence_tau").as_double());

    pr_ = computePersistence(*voxel_map_);

    // Algorithm 2: room segmentation
    rg_ = segmentRooms(pr_, *voxel_map_, tau);

    // Floor detection + room classification
    const float H_floor = static_cast<float>(req->floor_separation_m > 0
      ? req->floor_separation_m
      : get_parameter("floor_separation_m").as_double());
    const float sigma   = static_cast<float>(req->staircase_variance_m > 0
      ? req->staircase_variance_m
      : get_parameter("staircase_variance_m").as_double());
    const float gamma   = static_cast<float>(req->corridor_aspect_ratio > 0
      ? req->corridor_aspect_ratio
      : get_parameter("corridor_aspect_ratio").as_double());

    fg_ = detectFloors(rg_, *voxel_map_, H_floor, sigma, gamma);

    // Install topo map into planner
    planner_->setTopoMap(rg_, fg_);

    const auto t1 = std::chrono::steady_clock::now();
    const double build_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Count room types
    int n_stair = 0, n_corr = 0;
    for (const auto & rm : rg_.rooms) {
      if (rm.type == RoomType::STAIRCASE) ++n_stair;
      if (rm.type == RoomType::CORRIDOR)  ++n_corr;
    }

    res->success        = true;
    res->message        = "Topo map built.";
    res->num_rooms      = rg_.num_rooms;
    res->num_floors     = static_cast<int>(fg_.floors.size());
    res->num_staircases = n_stair;
    res->num_corridors  = n_corr;
    res->build_time_ms  = build_ms;

    RCLCPP_INFO(get_logger(),
      "BuildTopoMap: %d rooms (%d staircases, %d corridors), "
      "%d floors, %.1f ms",
      res->num_rooms, res->num_staircases, res->num_corridors,
      res->num_floors, res->build_time_ms);
  }

  // ── GetRoomLabels ─────────────────────────────────────────────────────────
  void handleGetRoomLabels(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::GetRoomLabels::Request>,
    const std::shared_ptr<srv::GetRoomLabels::Response> res)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!planner_->hasTopoMap()) {
      res->success = false;
      res->message = "Topo map not built.";
      return;
    }

    const auto & graph = voxel_map_->getSurfaceGraph();
    const int n = static_cast<int>(graph.size());
    res->surface_points.reserve(n);
    res->room_labels.reserve(n);
    res->floor_labels.reserve(n);
    res->room_persistence.reserve(n);

    for (int s = 0; s < n; ++s) {
      const auto & nd = graph[s];
      auto [wx, wy, wz] = voxel_map_->gridToWorld(nd.x, nd.y, nd.z);
      geometry_msgs::msg::Point p;
      p.x = wx; p.y = wy; p.z = wz;
      res->surface_points.push_back(p);

      const int rid = rg_.room_labels[s];
      res->room_labels.push_back(rid);

      const int fid = (rid >= 0) ? rg_.rooms[rid].floor_id : -1;
      res->floor_labels.push_back(fid);

      const float pers = (rid >= 0) ? rg_.rooms[rid].birth_d : 0.f;
      res->room_persistence.push_back(pers);
    }

    res->success = true;
    res->message = "OK";
  }

  // ── PlanHierarchical ──────────────────────────────────────────────────────
  void handlePlanHierarchical(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::PlanHierarchical::Request>  req,
    const std::shared_ptr<srv::PlanHierarchical::Response> res)
  {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!planner_->hasTopoMap()) {
      res->success = false;
      res->message = "Topo map not built.";
      return;
    }

    const float robot_d = static_cast<float>(
      req->robot_diameter_m > 0
        ? req->robot_diameter_m
        : get_parameter("robot_diameter_m").as_double());

    const auto start_w = std::make_tuple(
      req->start.position.x,
      req->start.position.y,
      req->start.position.z);
    const auto goal_w = std::make_tuple(
      req->goal.position.x,
      req->goal.position.y,
      req->goal.position.z);

    const HierarchicalPath hp = planner_->plan(
      start_w, goal_w, robot_d, req->snap_to_surface);

    res->success              = hp.success;
    res->message              = hp.message;
    res->planning_time_ms     = hp.total_planning_ms;
    res->floor_planning_ms    = hp.floor_planning_ms;
    res->room_planning_ms     = hp.room_planning_ms;
    res->voxel_planning_ms    = hp.voxel_planning_ms;
    res->path_length_m        = hp.path_length_m;
    res->voxel_nodes_expanded = static_cast<int64_t>(hp.voxel_nodes_expanded);

    for (int r : hp.room_sequence)  res->room_sequence.push_back(r);
    for (int f : hp.floor_sequence) res->floor_sequence.push_back(f);

    if (hp.success) {
      const std::string frame = get_parameter("frame_id").as_string();
      nav_msgs::msg::Path path_msg;
      path_msg.header.frame_id = frame;
      path_msg.header.stamp    = now();
      path_msg.poses.reserve(hp.waypoints.size());
      for (const auto & [wx, wy, wz] : hp.waypoints) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = wx;
        ps.pose.position.y = wy;
        ps.pose.position.z = wz;
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
      }
      res->path = path_msg;
    }

    RCLCPP_INFO(get_logger(),
      "PlanHierarchical: %s | %.1f ms total (floor %.2f + room %.2f + voxel %.2f ms)"
      " | %.2f m | rooms: %zu",
      hp.success ? "OK" : "FAIL",
      hp.total_planning_ms,
      hp.floor_planning_ms, hp.room_planning_ms, hp.voxel_planning_ms,
      hp.path_length_m, hp.room_sequence.size());
  }

  // ── State ─────────────────────────────────────────────────────────────────
  std::mutex map_mutex_;
  std::shared_ptr<jps_planner_3d::VoxelMap> voxel_map_;
  std::unique_ptr<HierarchicalPlanner>       planner_;

  PersistenceResult pr_;
  RoomGraph         rg_;
  FloorGraph        fg_;

  // Services
  rclcpp::Service<srv::SetVoxelMap>::SharedPtr     set_map_srv_;
  rclcpp::Service<srv::LoadPcdMap>::SharedPtr      load_pcd_srv_;
  rclcpp::Service<srv::BuildTopoMap>::SharedPtr    build_topo_srv_;
  rclcpp::Service<srv::GetRoomLabels>::SharedPtr   get_labels_srv_;
  rclcpp::Service<srv::PlanHierarchical>::SharedPtr plan_hier_srv_;
};

}  // namespace topo_nav

// ── main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<topo_nav::TopoNavNode>());
  rclcpp::shutdown();
  return 0;
}
