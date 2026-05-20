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
#include "topo_nav/grid_segmenter.hpp"
#include "topo_nav/floor_detector.hpp"
#include "topo_nav/hierarchical_planner.hpp"
#include "topo_nav/prm_planner.hpp"
#include "topo_nav/alt_planner.hpp"

// Reuse existing map-loading services from nav_interfaces
#include "nav_interfaces/srv/set_voxel_map.hpp"
#include "nav_interfaces/srv/load_pcd_map.hpp"
// New services
#include "nav_interfaces/srv/build_topo_map.hpp"
#include "nav_interfaces/srv/get_room_labels.hpp"
#include "nav_interfaces/srv/plan_hierarchical.hpp"
#include "nav_interfaces/srv/get_distance_field.hpp"

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
    w_up_   = get_parameter("w_up").as_double();
    w_down_ = get_parameter("w_down").as_double();
    w_obs_  = get_parameter("w_obs").as_double();
    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();
    rebuildPlanner();

    // ── Services ────────────────────────────────────────────────────────────
    set_map_srv_ = create_service<srv::SetVoxelMap>(
      "set_voxel_map",
      std::bind(&TopoNavNode::handleSetVoxelMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    load_pcd_srv_ = create_service<srv::LoadPcdMap>(
      "load_pcd_map",
      std::bind(&TopoNavNode::handleLoadPcdMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    build_topo_srv_ = create_service<srv::BuildTopoMap>(
      "build_topo_map",
      std::bind(&TopoNavNode::handleBuildTopoMap, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_labels_srv_ = create_service<srv::GetRoomLabels>(
      "get_room_labels",
      std::bind(&TopoNavNode::handleGetRoomLabels, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    plan_hier_srv_ = create_service<srv::PlanHierarchical>(
      "plan_hierarchical",
      std::bind(&TopoNavNode::handlePlanHierarchical, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    get_dist_srv_ = create_service<srv::GetDistanceField>(
      "get_distance_field",
      std::bind(&TopoNavNode::handleGetDistanceField, this,
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

    std::vector<std::tuple<float, float, float>> seeds;
    seeds.reserve(req->seed_points.size());
    for (const auto& p : req->seed_points) {
      seeds.emplace_back(static_cast<float>(p.x),
                         static_cast<float>(p.y),
                         static_cast<float>(p.z));
    }

    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();

    const bool ok = voxel_map_->updateFromPoints(
      occupied, req->resolution, seeds);

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

    std::vector<std::tuple<float, float, float>> seeds_pcd;
    seeds_pcd.reserve(req->seed_points.size());
    for (const auto& p : req->seed_points) {
      seeds_pcd.emplace_back(static_cast<float>(p.x),
                             static_cast<float>(p.y),
                             static_cast<float>(p.z));
    }

    voxel_map_ = std::make_shared<jps_planner_3d::VoxelMap>();
    configureMap();
    voxel_map_->updateFromPoints(pcd_result.occupied_voxels, req->resolution, seeds_pcd);
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

    const std::string mode = req->planner_mode.empty() ? "full" : req->planner_mode;
    planner_mode_ = mode;

    const auto t0 = std::chrono::steady_clock::now();
    double persistence_ms = 0.0, segmentation_ms = 0.0, precompute_ms = 0.0;

    // ── flat_surface: no topo map needed, just ensure planner is ready ──────
    if (mode == "flat_surface") {
      flat_mode_ = true;
      const auto t1 = std::chrono::steady_clock::now();
      res->success       = true;
      res->message       = "Flat surface mode (no topo map).";
      res->build_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      res->memory_usage_bytes = static_cast<int64_t>(
        voxel_map_->getConstructedSceneMemoryUsageBytes());
      RCLCPP_INFO(get_logger(), "BuildTopoMap [flat_surface]: skipped topo construction");
      return;
    }
    flat_mode_ = false;

    // ── Common parameters ───────────────────────────────────────────────────
    const float tau = static_cast<float>(req->persistence_tau > 0
      ? req->persistence_tau
      : get_parameter("persistence_tau").as_double());
    const float H_floor = static_cast<float>(req->floor_separation_m > 0
      ? req->floor_separation_m
      : get_parameter("floor_separation_m").as_double());
    const float sigma   = static_cast<float>(req->staircase_variance_m > 0
      ? req->staircase_variance_m
      : get_parameter("staircase_variance_m").as_double());
    const float gamma   = static_cast<float>(req->corridor_aspect_ratio > 0
      ? req->corridor_aspect_ratio
      : get_parameter("corridor_aspect_ratio").as_double());

    // ── Segmentation (PH or grid) ───────────────────────────────────────────
    if (mode == "grid_partition") {
      // Grid-based room segmentation (Config C / baseline)
      auto seg_t0 = std::chrono::steady_clock::now();
      const int target = req->target_room_count > 0 ? req->target_room_count : 0;
      rg_ = segmentRoomsGrid(*voxel_map_, target, tau);
      auto seg_t1 = std::chrono::steady_clock::now();
      segmentation_ms = std::chrono::duration<double, std::milli>(seg_t1 - seg_t0).count();
    } else {
      // PH-based segmentation (full, no_stitch, prm, alt_astar, ph_alt_tg)
      auto ph_t0 = std::chrono::steady_clock::now();
      pr_ = computePersistence(*voxel_map_);
      auto ph_t1 = std::chrono::steady_clock::now();
      persistence_ms = std::chrono::duration<double, std::milli>(ph_t1 - ph_t0).count();

      auto seg_t0 = std::chrono::steady_clock::now();
      rg_ = segmentRooms(pr_, *voxel_map_, tau);
      auto seg_t1 = std::chrono::steady_clock::now();
      segmentation_ms = std::chrono::duration<double, std::milli>(seg_t1 - seg_t0).count();
    }

    // Floor detection + room classification
    fg_ = detectFloors(rg_, *voxel_map_, H_floor, sigma, gamma);

    // ── Install topo map into planner ───────────────────────────────────────
    const bool skip_precompute = (mode == "no_stitch");
    auto pc_t0 = std::chrono::steady_clock::now();
    planner_->setTopoMap(rg_, fg_, skip_precompute);
    planner_->setUseStitch(mode != "no_stitch");

    // PH-ALT-TG: build transition graph ALT landmarks
    if (mode == "ph_alt_tg") {
      const int n_landmarks = req->num_landmarks > 0 ? req->num_landmarks : 8;
      planner_->precomputeTGLandmarks(n_landmarks);
      planner_->setUsePHALT(true);
    } else {
      planner_->setUsePHALT(false);
    }

    auto pc_t1 = std::chrono::steady_clock::now();
    precompute_ms = std::chrono::duration<double, std::milli>(pc_t1 - pc_t0).count();

    // ── Build alternative planners if requested ─────────────────────────────
    if (mode == "prm") {
      auto prm_t0 = std::chrono::steady_clock::now();
      prm_planner_ = std::make_unique<PRMPlanner>(voxel_map_, w_up_, w_down_, w_obs_);
      const int n_samples = req->num_prm_samples > 0 ? req->num_prm_samples : 1000;
      prm_planner_->buildRoadmap(n_samples, 15, 0.0);
      auto prm_t1 = std::chrono::steady_clock::now();
      precompute_ms = std::chrono::duration<double, std::milli>(prm_t1 - prm_t0).count();
    } else {
      prm_planner_.reset();
    }

    if (mode == "alt_astar") {
      auto alt_t0 = std::chrono::steady_clock::now();
      alt_planner_ = std::make_unique<ALTPlanner>(voxel_map_, w_up_, w_down_, w_obs_);
      const int n_landmarks = req->num_landmarks > 0 ? req->num_landmarks : 16;
      alt_planner_->precompute(n_landmarks);
      auto alt_t1 = std::chrono::steady_clock::now();
      precompute_ms = std::chrono::duration<double, std::milli>(alt_t1 - alt_t0).count();
    } else {
      alt_planner_.reset();
    }

    const auto t1 = std::chrono::steady_clock::now();
    const double build_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Count room types
    int n_stair = 0, n_corr = 0;
    for (const auto & rm : rg_.rooms) {
      if (rm.type == RoomType::STAIRCASE) ++n_stair;
      if (rm.type == RoomType::CORRIDOR)  ++n_corr;
    }

    res->success             = true;
    res->message             = "Topo map built (" + mode + ").";
    res->num_rooms           = rg_.num_rooms;
    res->num_floors          = static_cast<int>(fg_.floors.size());
    res->num_staircases      = n_stair;
    res->num_corridors       = n_corr;
    res->build_time_ms       = build_ms;
    res->persistence_time_ms = persistence_ms;
    res->segmentation_time_ms = segmentation_ms;
    res->precompute_time_ms  = precompute_ms;
    res->memory_usage_bytes  = static_cast<int64_t>(
      voxel_map_->getConstructedSceneMemoryUsageBytes());

    // Precompute diagnostics
    const auto & ps = planner_->getPrecomputeStats();
    res->tg_nodes                = ps.tg_nodes;
    res->tg_edges                = ps.tg_edges;
    res->tg_landmarks            = ps.tg_landmarks;
    res->tg_connected_components = ps.tg_connected_components;
    res->max_room_voxels         = ps.max_room_voxels;
    res->max_room_portals        = ps.max_room_portals;
    res->max_room_fraction       = ps.max_room_fraction;
    res->skipped_rooms           = ps.skipped_rooms;
    res->skipped_voxel_fraction  = ps.skipped_voxel_fraction;
    res->transfer_pairs_total    = ps.transfer_pairs_total;
    res->transfer_pairs_cached   = ps.transfer_pairs_cached;
    res->transfer_coverage       = ps.transfer_coverage;

    RCLCPP_INFO(get_logger(),
      "BuildTopoMap [%s]: %d rooms (%d stair, %d corr), %d floors, "
      "%.1f ms (PH %.1f + seg %.1f + precomp %.1f ms)",
      mode.c_str(), res->num_rooms, res->num_staircases, res->num_corridors,
      res->num_floors, res->build_time_ms,
      persistence_ms, segmentation_ms, precompute_ms);
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

    HierarchicalPath hp;

    if (flat_mode_) {
      // Config B: flat surface A*
      hp = planner_->planFlat(start_w, goal_w, robot_d, req->snap_to_surface);
    } else if (planner_mode_ == "prm" && prm_planner_) {
      hp = prm_planner_->plan(start_w, goal_w, robot_d, req->snap_to_surface);
    } else if (planner_mode_ == "alt_astar" && alt_planner_) {
      hp = alt_planner_->plan(start_w, goal_w, robot_d, req->snap_to_surface);
    } else {
      // full, no_stitch, grid_partition, ph_alt_tg all use hierarchical plan()
      if (!planner_->hasTopoMap()) {
        res->success = false;
        res->message = "Topo map not built.";
        return;
      }
      hp = planner_->plan(start_w, goal_w, robot_d, req->snap_to_surface);
    }

    res->success              = hp.success;
    res->message              = hp.message;
    res->planning_time_ms     = hp.total_planning_ms;
    res->snap_ms              = hp.snap_ms;
    res->floor_planning_ms    = hp.floor_planning_ms;
    res->room_planning_ms     = hp.room_planning_ms;
    res->voxel_planning_ms    = hp.voxel_planning_ms;
    res->path_length_m        = hp.path_length_m;
    res->voxel_nodes_expanded = static_cast<int64_t>(hp.voxel_nodes_expanded);

    // Per-query diagnostics
    const auto & d = hp.diag;
    res->diag_start_room       = d.start_room;
    res->diag_goal_room        = d.goal_room;
    res->diag_same_room        = d.same_room;
    res->diag_room_hops        = d.room_hops;
    res->diag_tg_expanded      = d.tg_expanded;
    res->diag_tg_astar_ms      = d.tg_astar_ms;
    res->diag_h_start          = d.h_start;
    res->diag_tg_cost          = d.tg_cost;
    res->diag_stitch_segments  = d.stitch_segments;
    res->diag_stitch_cached    = d.stitch_cached;
    res->diag_stitch_fallback  = d.stitch_fallback;
    res->diag_stitch_ms        = d.stitch_ms;
    res->diag_fallback_nodes   = d.fallback_nodes;
    res->diag_fallback_reason  = d.fallback_reason;
    res->diag_cache_hit_all    = d.cache_hit_all;
    res->diag_tg_nodes         = d.tg_nodes;
    res->diag_start_portals    = d.start_portals;
    res->diag_goal_portals     = d.goal_portals;

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
      "PlanHierarchical [%s]: %s | %.1f ms (snap %.2f + floor %.2f + room %.2f + voxel %.2f)"
      " | %.2f m | rooms: %zu",
      planner_mode_.c_str(),
      hp.success ? "OK" : "FAIL",
      hp.total_planning_ms,
      hp.snap_ms, hp.floor_planning_ms, hp.room_planning_ms, hp.voxel_planning_ms,
      hp.path_length_m, hp.room_sequence.size());
  }

  // ── GetDistanceField ────────────────────────────────────────────────────
  void handleGetDistanceField(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<srv::GetDistanceField::Request>,
    const std::shared_ptr<srv::GetDistanceField::Response> res)
  {
    RCLCPP_INFO(get_logger(), "get_distance_field: retrieving distance field data");

    std::shared_ptr<jps_planner_3d::VoxelMap> map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_copy = voxel_map_;
    }

    if (!map_copy || !map_copy->isReady()) {
      res->success = false;
      res->message = "Voxel map is not ready";
      return;
    }

    const auto & surface_voxels = map_copy->getValidSurfaceVoxels();

    if (surface_voxels.empty()) {
      res->success = false;
      res->message = "No surface voxels available";
      return;
    }

    res->resolution = map_copy->getResolution();
    res->surface_points.reserve(surface_voxels.size());
    res->distances.reserve(surface_voxels.size());

    for (const auto & voxel : surface_voxels) {
      auto [ix, iy, iz] = voxel;
      auto [x, y, z] = map_copy->gridToWorld(ix, iy, iz);

      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      res->surface_points.push_back(pt);

      int dist = map_copy->getDistanceToObstacle(ix, iy, iz);
      res->distances.push_back(static_cast<uint8_t>(dist));
    }

    res->success = true;
    res->message = "Distance field retrieved";

    RCLCPP_INFO(get_logger(), "get_distance_field success: %zu surface points",
                surface_voxels.size());
  }

  // ── State ─────────────────────────────────────────────────────────────────
  std::mutex map_mutex_;
  std::shared_ptr<jps_planner_3d::VoxelMap> voxel_map_;
  std::unique_ptr<HierarchicalPlanner>       planner_;
  std::unique_ptr<PRMPlanner>                prm_planner_;
  std::unique_ptr<ALTPlanner>                alt_planner_;

  double w_up_ = 2.0, w_down_ = 1.0, w_obs_ = 0.5;
  std::string planner_mode_ = "full";
  bool flat_mode_ = false;

  PersistenceResult pr_;
  RoomGraph         rg_;
  FloorGraph        fg_;

  // Services
  rclcpp::Service<srv::SetVoxelMap>::SharedPtr     set_map_srv_;
  rclcpp::Service<srv::LoadPcdMap>::SharedPtr      load_pcd_srv_;
  rclcpp::Service<srv::BuildTopoMap>::SharedPtr    build_topo_srv_;
  rclcpp::Service<srv::GetRoomLabels>::SharedPtr   get_labels_srv_;
  rclcpp::Service<srv::PlanHierarchical>::SharedPtr plan_hier_srv_;
  rclcpp::Service<srv::GetDistanceField>::SharedPtr get_dist_srv_;
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
