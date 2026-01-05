#include "jps_planner_3d/voxel_map.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include <queue>

namespace jps_planner_3d
{


VoxelMap::VoxelMap(double resolution,
                   double free_space_margin)
: resolution_(resolution)
, free_space_margin_(free_space_margin)
, grid_size_(std::make_tuple(0, 0, 0))
, min_bound_(std::make_tuple(0.0, 0.0, 0.0))
, max_bound_(std::make_tuple(0.0, 0.0, 0.0))
, origin_(std::make_tuple(0.0, 0.0, 0.0))
{
}

VoxelMap::~VoxelMap()
{
}

bool VoxelMap::updateFromPoints(const std::vector<std::tuple<float, float, float>>& occupied_voxels,
                                double resolution)
{
    if (occupied_voxels.empty() || resolution <= 0.0) {
        return false;
    }

    // 清空之前的数据，避免使用旧数据
    valid_surface_voxels_.clear();
    surface_array_.clear();
    distance_map_.clear();

    resolution_ = resolution;

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& voxel : occupied_voxels) {
        auto [x, y, z] = voxel;
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        min_z = std::min(min_z, z);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
        max_z = std::max(max_z, z);
    }

    double expansion = free_space_margin_;
    if (expansion < resolution_) {
        expansion = resolution_;
    } else {
        expansion += resolution_;
    }

    min_bound_ = std::make_tuple(
        std::floor(min_x / resolution_) * resolution_ - expansion,
        std::floor(min_y / resolution_) * resolution_ - expansion,
        std::floor(min_z / resolution_) * resolution_ - expansion
    );
    max_bound_ = std::make_tuple(
        std::ceil(max_x / resolution_) * resolution_ + expansion,
        std::ceil(max_y / resolution_) * resolution_ + expansion,
        std::ceil(max_z / resolution_) * resolution_ + expansion
    );
    origin_ = min_bound_;

    auto [min_x_bound, min_y_bound, min_z_bound] = min_bound_;
    auto [max_x_bound, max_y_bound, max_z_bound] = max_bound_;
    int nx = static_cast<int>(std::ceil((max_x_bound - min_x_bound) / resolution_)) + 1;
    int ny = static_cast<int>(std::ceil((max_y_bound - min_y_bound) / resolution_)) + 1;
    int nz = static_cast<int>(std::ceil((max_z_bound - min_z_bound) / resolution_)) + 1;
    grid_size_ = std::make_tuple(nx, ny, nz);

    size_t total_size = static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nz);
    voxel_array_.clear();
    voxel_array_.resize(total_size, false);

    for (const auto& voxel : occupied_voxels) {
        auto [x, y, z] = voxel;
        int ix = static_cast<int>(std::floor((x - std::get<0>(origin_)) / resolution_));
        int iy = static_cast<int>(std::floor((y - std::get<1>(origin_)) / resolution_));
        int iz = static_cast<int>(std::floor((z - std::get<2>(origin_)) / resolution_));
        if (isValidIdx(ix, iy, iz)) {
            size_t idx = index3DTo1D(ix, iy, iz);
            voxel_array_[idx] = true;
        }
    }

    // 步骤1: 构建可到达区域（使用直接索引的voxel_array_）
    buildReachableMap();
    voxel_array_.clear();
    voxel_array_.shrink_to_fit();  // 释放内存

    return true;
}

bool VoxelMap::isReady() const
{
    auto [nx, ny, nz] = grid_size_;
    // 只检查网格尺寸，因为地图构建成功与否取决于网格是否有效
    // surface_array_可能为空（如果没有可到达的surface点），这是有效的地图状态
    // 规划器应该能够处理这种情况（返回空路径或失败）
    return nx > 0 && ny > 0 && nz > 0;
}

bool VoxelMap::isOccupied(double x, double y, double z) const
{
    auto grid_idx = worldToGrid(x, y, z);
    auto [ix, iy, iz] = grid_idx;
    if (!isValidIdx(ix, iy, iz)) {
        return true;  // 超出边界视为占据
    }
    return isOccupiedIdx(ix, iy, iz);
}

bool VoxelMap::isOccupiedIdx(int ix, int iy, int iz) const
{
    if (!isValidIdx(ix, iy, iz)) {
        return false;
    }
    // 使用voxel_array_查询（构建阶段和路径验证使用）
    if (voxel_array_.empty()) {
        return false;
    }
    size_t idx = index3DTo1D(ix, iy, iz);
    return voxel_array_[idx];
}

bool VoxelMap::isValidIdx(int ix, int iy, int iz) const
{
    auto [nx, ny, nz] = grid_size_;
    return ix >= 0 && ix < nx && iy >= 0 && iy < ny && iz >= 0 && iz < nz;
}

std::tuple<int, int, int> VoxelMap::worldToGrid(double x, double y, double z) const
{
    auto [ox, oy, oz] = origin_;
    int ix = static_cast<int>(std::floor((x - ox) / resolution_));
    int iy = static_cast<int>(std::floor((y - oy) / resolution_));
    int iz = static_cast<int>(std::floor((z - oz) / resolution_));
    return std::make_tuple(ix, iy, iz);
}

std::tuple<double, double, double> VoxelMap::gridToWorld(int ix, int iy, int iz) const
{
    auto [ox, oy, oz] = origin_;
    double x = ox + (ix + 0.5) * resolution_;
    double y = oy + (iy + 0.5) * resolution_;
    double z = oz + (iz + 0.5) * resolution_;
    return std::make_tuple(x, y, z);
}

void VoxelMap::buildReachableMap()
{
    // 只检查网格尺寸，不检查 valid_surface_voxels_（避免循环依赖）
    auto [nx, ny, nz] = grid_size_;
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    
    size_t total_size = voxel_array_.size();
    if (total_size == 0) {
        return;
    }
    
    // 清空列表
    valid_surface_voxels_.clear();
    
    // 辅助数组：标记地面体素是否已访问，防止BFS重复
    std::vector<bool> visited_ground(total_size, false);
    
    // 1. 寻找起始种子点 (寻找原点附近：被占据 + 上方是空气 的体素)
    auto origin_grid = worldToGrid(0.0, 0.0, 0.0);
    int start_ix = std::get<0>(origin_grid);
    int start_iy = std::get<1>(origin_grid);
    
    int search_r = static_cast<int>(5.0 / resolution_);
    
    // 计算需要检查的上方体素数量（根据surface_clearance_height_）
    // 在整个函数中都是常量，提前计算一次即可
    int clearance_cells = static_cast<int>(std::ceil(surface_clearance_height_ / resolution_));
    // 确保上界至少为0，避免当clearance_cells >= nz时循环不执行
    int max_iz = std::max(0, nz - clearance_cells);
    
    double min_dist_sq = std::numeric_limits<double>::max();
    std::tuple<int, int, int> best_seed = {-1, -1, -1};

    // 首先在原点附近搜索
    for (int dx = -search_r; dx <= search_r; ++dx) {
        for (int dy = -search_r; dy <= search_r; ++dy) {
            int ix = start_ix + dx;
            int iy = start_iy + dy;
            
            if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) continue;
            
            // 遍历该柱子上的所有体素，寻找符合条件的地面点
            for (int iz = 0; iz < max_iz; ++iz) {  // 确保上方有足够空间检查
                // 条件1: 当前体素必须被占据 (Ground)
                if (!isOccupiedIdx(ix, iy, iz)) continue;
                
                // 条件2: 上方clearance_cells个体素都必须是空气 (可通行)
                bool has_clearance = true;
                for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                    if (isOccupiedIdx(ix, iy, iz + check_z)) {
                        has_clearance = false;
                        break;
                    }
                }
                if (!has_clearance) continue;

                // 计算距离原点的距离
                auto [wx, wy, wz] = gridToWorld(ix, iy, iz);
                double d2 = wx*wx + wy*wy + wz*wz;
                
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    best_seed = {ix, iy, iz};
                }
            }
        }
    }
    
    // 如果原点附近找不到种子点，扩大搜索范围到整个地图
    if (std::get<0>(best_seed) == -1) {
        // 在整个地图中搜索符合条件的体素
        // clearance_cells 和 max_iz 已在上面计算，直接使用
        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                for (int iz = 0; iz < max_iz; ++iz) {
                    // 条件1: 当前体素必须被占据 (Ground)
                    if (!isOccupiedIdx(ix, iy, iz)) continue;
                    
                    // 条件2: 上方clearance_cells个体素都必须是空气 (可通行)
                    bool has_clearance = true;
                    for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                        if (isOccupiedIdx(ix, iy, iz + check_z)) {
                            has_clearance = false;
                            break;
                        }
                    }
                    if (!has_clearance) continue;

                    // 找到第一个符合条件的体素就使用
                    best_seed = {ix, iy, iz};
                    auto [wx, wy, wz] = gridToWorld(ix, iy, iz);
                    min_dist_sq = wx*wx + wy*wy + wz*wz;
                    goto found_seed;
                }
            }
        }
        found_seed:;
    }
    
    // 如果仍然找不到种子点，说明地图中没有符合条件的体素
    if (std::get<0>(best_seed) == -1) {
        return;
    }

    // 2. BFS 遍历 (只在上方是空气的地面体素上扩展)
    std::queue<std::tuple<int, int, int>> q;
    
    q.push(best_seed);
    size_t seed_idx = index3DTo1D(std::get<0>(best_seed), std::get<1>(best_seed), std::get<2>(best_seed));
    visited_ground[seed_idx] = true;
    
    // 种子点已确认上方是空气，直接记录为Surface
    {
        auto [sx, sy, sz] = best_seed;
        valid_surface_voxels_.emplace_back(sx, sy, sz + 1);
    }
    
    int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    
    // 允许的台阶高度 (Z轴搜索范围)
    int z_step = static_cast<int>(std::ceil(connectivity_height_threshold_ / resolution_));
    // clearance_cells 已在上面第441行声明，直接使用

    while (!q.empty()) {
        auto [cx, cy, cz] = q.front();
        q.pop();
        
        // === 扩展邻居 ===
        for (auto& d : dirs) {
            int nx_ix = cx + d[0];
            int ny_iy = cy + d[1];
            
            if (nx_ix < 0 || nx_ix >= nx || ny_iy < 0 || ny_iy >= ny) continue;
            
            // 在邻居柱子上寻找连接的地面点
            // 搜索范围: [cz - z_step, cz + z_step]
            // 注意: max_z 限制为 nz - clearance_cells，确保上方有足够空间检查（与种子搜索保持一致）
            // 使用 std::max(0, ...) 避免当 clearance_cells >= nz 时 max_z 为负数
            int min_z = std::max(0, cz - z_step);
            int max_z = std::min(std::max(0, nz - clearance_cells), cz + z_step);
            
            for (int nz_iz = min_z; nz_iz <= max_z; ++nz_iz) {
                // 1. 必须被占据 (只在实心体素上行走)
                if (!isOccupiedIdx(nx_ix, ny_iy, nz_iz)) continue;
                
                // 2. 检查是否已访问
                size_t n_idx = index3DTo1D(nx_ix, ny_iy, nz_iz);
                if (visited_ground[n_idx]) continue;
                
                // 标记为已访问
                visited_ground[n_idx] = true;
                
                // 3. 检查上方是否有足够的空气高度（根据surface_clearance_height_）
                bool has_clearance = true;
                for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                    int check_iz = nz_iz + check_z;
                    if (check_iz >= nz || isOccupiedIdx(nx_ix, ny_iy, check_iz)) {
                        has_clearance = false;
                        break;
                    }
                }
                if (has_clearance) {
                    // 上方有足够空气 → 这是一个有效的Surface，记录并继续扩展
                    valid_surface_voxels_.emplace_back(nx_ix, ny_iy, nz_iz + 1);
                    q.push({nx_ix, ny_iy, nz_iz});
                }
                // 如果上方被堵住，只标记为已访问，不加入队列继续扩展
            }
        }
    }
    
    // BFS 完成后，构建直接索引数组（大空间换小时间，O(1)查询）
    // 使用已声明的 total_size（已在函数开头计算）
    
    // 初始化surface标记数组（全部为false）
    surface_array_.clear();
    surface_array_.resize(total_size, false);
    
    // 填充surface标记数组
    for (const auto& voxel : valid_surface_voxels_) {
        auto [sx, sy, sz] = voxel;
        if (isValidIdx(sx, sy, sz)) {
            size_t idx = index3DTo1D(sx, sy, sz);
            surface_array_[idx] = true;
        }
    }
}

bool VoxelMap::isSurfaceValid(int ix, int iy, int iz) const
{
    // 使用直接索引数组，O(1)查询
    if (!isValidIdx(ix, iy, iz) || surface_array_.empty()) {
        return false;
    }
    
    size_t idx = index3DTo1D(ix, iy, iz);
    if (idx >= surface_array_.size()) {
        return false;
    }
    
    return surface_array_[idx];
}

std::tuple<int, int, int> VoxelMap::findNearestValidSurface(
    int ix, int iy, int iz, int search_radius, const std::function<bool()>& should_cancel) const
{
    // 使用直接索引数组，在搜索半径内直接遍历，O(1)查询每个体素
    if (surface_array_.empty()) {
        return {-1, -1, -1};
    }

    int min_dist_sq = search_radius * search_radius + 1;  // 初始化为超出搜索范围
    std::tuple<int, int, int> best = {-1, -1, -1};

    // 在搜索半径内遍历所有体素
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
        // 每完成一层dx循环，检查一次取消标志（避免每个体素都检查影响性能）
        if (should_cancel && should_cancel()) {
            return {-1, -1, -1};  // 被取消，返回无效值
        }

        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            // Z方向也搜索一定范围（允许高度变化）
            for (int dz = -search_radius; dz <= search_radius; ++dz) {
                int nx_coord = ix + dx;
                int ny_coord = iy + dy;
                int nz_coord = iz + dz;

                // 检查索引有效性
                if (!isValidIdx(nx_coord, ny_coord, nz_coord)) {
                    continue;
                }

                // 使用直接索引检查是否是有效Surface（O(1)）
                size_t idx = index3DTo1D(nx_coord, ny_coord, nz_coord);
                if (idx >= surface_array_.size() || !surface_array_[idx]) {
                    continue;
                }

                // 计算距离平方（避免开方）
                int dist_sq = dx*dx + dy*dy + dz*dz;

                // 更新最近点
                if (dist_sq <= search_radius * search_radius && dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best = {nx_coord, ny_coord, nz_coord};
                }
            }
        }
    }

    return best;
}

bool VoxelMap::hasAdjacentObstacles(int ix, int iy, int iz) const
{
    // 使用距离场判断：距离<=1表示附近有障碍物
    int dist = getDistanceToObstacle(ix, iy, iz);
    return dist <= 1;
}

int VoxelMap::getDistanceToObstacle(int ix, int iy, int iz) const
{
    // 直接查询distance_map_（调用者保证传入的是surface点）
    // 如果不是surface点或索引无效，distance_map_中不存在，返回0
    if (!isValidIdx(ix, iy, iz) || distance_map_.empty()) {
        return 0;
    }
    
    size_t idx = index3DTo1D(ix, iy, iz);
    auto it = distance_map_.find(idx);
    if (it != distance_map_.end()) {
        // 返回完整的8位距离值（最大255）
        return static_cast<int>(it->second);
    }
    
    // 如果没有找到（非surface点或距离场未计算），返回0
    return 0;
}

void VoxelMap::computeDistanceFieldOnSurface()
{
    if (valid_surface_voxels_.empty() || surface_array_.empty()) {
        return;
    }

    // 限制最大距离为255（8 bits，uint8_t最大值）
    const int MAX_DISTANCE = std::min(distance_field_max_distance_, 255);
    const uint8_t INF = 255;  // 使用255作为INF（最大可表示值）
    const uint8_t MAX_DISTANCE_U8 = static_cast<uint8_t>(MAX_DISTANCE);
    
    // 清空距离场映射
    distance_map_.clear();
    distance_map_.reserve(valid_surface_voxels_.size());
    
    // Step 1: 初始化 - 检查每个surface点的邻域是否有障碍物
    // 如果邻域有障碍物，距离=1；否则距离=INF
    // 计算Z方向搜索范围（与BFS连通性保持一致）
    int z_search = static_cast<int>(std::ceil(connectivity_height_threshold_ / resolution_));

    for (const auto& voxel : valid_surface_voxels_) {
        auto [sx, sy, sz] = voxel;
        if (!isValidIdx(sx, sy, sz)) continue;

        size_t idx = index3DTo1D(sx, sy, sz);

        // 检查XY平面8邻域是否为真正的障碍物
        bool near_obstacle = false;
        for (int dx = -1; dx <= 1 && !near_obstacle; ++dx) {
            for (int dy = -1; dy <= 1 && !near_obstacle; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx_coord = sx + dx;
                int ny_coord = sy + dy;

                // 先检查这个方向是否有可通行的surface（在Z范围内）
                // 这样可以正确处理下台阶、斜坡等场景
                bool has_passable_surface = false;
                for (int dz = -z_search; dz <= z_search; ++dz) {
                    int check_z = sz + dz;
                    if (isSurfaceValid(nx_coord, ny_coord, check_z)) {
                        has_passable_surface = true;
                        break;
                    }
                }

                // 只有当该方向没有可通行surface 且 有实体障碍物时，才认为是障碍物
                if (!has_passable_surface) {
                    // 检查同层和上层是否有障碍物（墙壁检测）
                    if (isOccupiedIdx(nx_coord, ny_coord, sz) ||
                        isOccupiedIdx(nx_coord, ny_coord, sz + 1)) {
                        near_obstacle = true;
                    }
                }
            }
        }

        // 下方肯定是地面（Surface定义），不算墙壁
        // 只存储surface点的距离场，使用8 bits（值范围0-255）
        distance_map_[idx] = near_obstacle ? 1 : INF;
    }
    
    // Step 2: 多遍DP扫描直到收敛
    // 只遍历surface点，使用distance_map_存储
    bool changed = true;
    int iterations = 0;
    const int MAX_ITERATIONS = MAX_DISTANCE + 2;  // 最多需要MAX_DISTANCE次迭代

    while (changed && iterations < MAX_ITERATIONS) {
        changed = false;
        iterations++;

        // 遍历所有surface点
        for (const auto& voxel : valid_surface_voxels_) {
            auto [sx, sy, sz] = voxel;
            if (!isValidIdx(sx, sy, sz)) continue;

            size_t idx = index3DTo1D(sx, sy, sz);

            // 获取当前距离（如果不存在，使用INF）
            auto it = distance_map_.find(idx);
            if (it == distance_map_.end()) {
                continue;  // 不应该发生
            }

            uint8_t current_dist = it->second;
            if (current_dist == 0) continue;  // 跳过障碍物（不应该存在）

            // 检查8邻域的surface点（使用直接索引）
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;

                    int nx_coord = sx + dx;
                    int ny_coord = sy + dy;

                    // 在Z方向搜索可连通的surface点（与BFS建图和初始化保持一致）
                    // 使用connectivity_height_threshold_确保距离场传播范围与可达性定义一致
                    for (int dz = -z_search; dz <= z_search; ++dz) {
                        int nz_coord = sz + dz;
                        
                        if (!isValidIdx(nx_coord, ny_coord, nz_coord)) continue;
                        
                        size_t neighbor_idx = index3DTo1D(nx_coord, ny_coord, nz_coord);
                        
                        // 检查是否是有效Surface点
                        if (neighbor_idx >= surface_array_.size() || !surface_array_[neighbor_idx]) {
                            continue;
                        }
                        
                        // 从distance_map_中获取邻居距离
                        auto neighbor_it = distance_map_.find(neighbor_idx);
                        if (neighbor_it == distance_map_.end()) {
                            continue;  // 不应该发生
                        }
                        
                        uint8_t neighbor_dist = neighbor_it->second;
                        
                        // 传播距离：邻居距离+1（只有当邻居距离不是INF时才传播）
                        if (neighbor_dist < INF) {
                            uint8_t new_dist = static_cast<uint8_t>(neighbor_dist + 1);
                            // 限制在最大距离范围内
                            if (new_dist > MAX_DISTANCE_U8) {
                                new_dist = MAX_DISTANCE_U8;
                            }
                            if (new_dist < current_dist) {
                                current_dist = new_dist;
                                changed = true;
                            }
                        }
                    }
                }
            }
            
            // 更新距离值
            // 重要：只有当距离不是INF时才截断到MAX_DISTANCE_U8
            // INF值（255）表示未到达的点，应该保持为INF以区别于距离正好为max_distance的点
            if (current_dist < INF) {
                // 实际计算的距离值：截断到最大距离范围内
                distance_map_[idx] = std::min(current_dist, MAX_DISTANCE_U8);
            } else {
                // INF值：保持为INF（255），表示该点附近没有障碍物，不应被惩罚
                distance_map_[idx] = INF;
            }
        }
    }
}


}  // namespace jps_planner_3d