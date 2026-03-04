#ifndef JPS_PLANNER_3D__PCD_VOXEL_LOADER_HPP_
#define JPS_PLANNER_3D__PCD_VOXEL_LOADER_HPP_

#include <cstdint>
#include <string>
#include <tuple>
#include <vector>

namespace jps_planner_3d
{

struct PcdVoxelizeResult
{
  std::vector<std::tuple<float, float, float>> occupied_voxels;
  int64_t raw_point_count = 0;
};

bool loadAndVoxelizePcd(
  const std::string & pcd_path,
  double resolution,
  int min_points_per_voxel,
  double max_abs_coord,
  PcdVoxelizeResult & result,
  std::string & error_msg);

}  // namespace jps_planner_3d

#endif  // JPS_PLANNER_3D__PCD_VOXEL_LOADER_HPP_
