#include "jps_planner_3d/voxel_map.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include <queue>
#include <chrono>

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
                                double resolution,
                                const std::tuple<float, float, float>* seed_point)
{
    if (occupied_voxels.empty() || resolution <= 0.0) {
        return false;
    }

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
    voxel_array_.resize(total_size, 0);

    for (const auto& voxel : occupied_voxels) {
        auto [x, y, z] = voxel;
        int ix = static_cast<int>(std::floor((x - std::get<0>(origin_)) / resolution_));
        int iy = static_cast<int>(std::floor((y - std::get<1>(origin_)) / resolution_));
        int iz = static_cast<int>(std::floor((z - std::get<2>(origin_)) / resolution_));
        if (isValidIdx(ix, iy, iz)) {
            size_t idx = index3DTo1D(ix, iy, iz);
            voxel_array_[idx] = 1;
        }
    }

    auto t_build_start = std::chrono::high_resolution_clock::now();
    buildReachableMap(seed_point);
    auto t_build_end = std::chrono::high_resolution_clock::now();
    traversability_time_ms_ = std::chrono::duration<double, std::milli>(t_build_end - t_build_start).count();

    // 计算距离场（需要 voxel_array_ 和 surface_array_，必须在 buildSurfaceGraph 前调用）
    computeDistanceFieldOnSurface();

    // 构建 Surface Graph 用于A*搜索加速（使用 distance_map_ 填充节点距离）
    buildSurfaceGraph();
    
    voxel_array_.clear();
    voxel_array_.shrink_to_fit();

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

void VoxelMap::buildReachableMap(const std::tuple<float, float, float>* seed_point)
{
    auto [nx, ny, nz] = grid_size_;
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }
    
    size_t total_size = voxel_array_.size();
    if (total_size == 0) {
        return;
    }
    
    valid_surface_voxels_.clear();
    
    std::vector<uint8_t> visited_ground(total_size, 0);
    
    int clearance_cells = static_cast<int>(std::ceil(surface_clearance_height_ / resolution_));
    int max_iz = std::max(0, nz - clearance_cells);
    
    double min_dist_sq = std::numeric_limits<double>::max();
    std::tuple<int, int, int> best_seed = {-1, -1, -1};

    // 如果提供了种子点，优先使用
    if (seed_point != nullptr) {
        auto [sx, sy, sz] = *seed_point;
        auto [seed_ix, seed_iy, seed_iz] = worldToGrid(sx, sy, sz);
        
        // 在种子点附近搜索有效的地面体素
        int search_r = static_cast<int>(3.0 / resolution_);
        
        for (int dx = -search_r; dx <= search_r; ++dx) {
            for (int dy = -search_r; dy <= search_r; ++dy) {
                for (int dz = -search_r; dz <= search_r; ++dz) {
                    int ix = seed_ix + dx;
                    int iy = seed_iy + dy;
                    int iz = seed_iz + dz;
                    
                    if (!isValidIdx(ix, iy, iz) || iz >= max_iz) continue;
                    if (!isOccupiedIdx(ix, iy, iz)) continue;
                    
                    bool has_clearance = true;
                    for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                        if (isOccupiedIdx(ix, iy, iz + check_z)) {
                            has_clearance = false;
                            break;
                        }
                    }
                    if (!has_clearance) continue;
                    
                    double d2 = dx*dx + dy*dy + dz*dz;
                    if (d2 < min_dist_sq) {
                        min_dist_sq = d2;
                        best_seed = {ix, iy, iz};
                    }
                }
            }
        }
    }
    
    // 如果没有种子点或种子点附近找不到，从原点附近搜索
    if (std::get<0>(best_seed) == -1) {
        auto origin_grid = worldToGrid(0.0, 0.0, 0.0);
        int start_ix = std::get<0>(origin_grid);
        int start_iy = std::get<1>(origin_grid);
        
        int search_r = static_cast<int>(5.0 / resolution_);
        
        for (int dx = -search_r; dx <= search_r; ++dx) {
            for (int dy = -search_r; dy <= search_r; ++dy) {
                int ix = start_ix + dx;
                int iy = start_iy + dy;
                
                if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) continue;
                
                for (int iz = 0; iz < max_iz; ++iz) {
                    if (!isOccupiedIdx(ix, iy, iz)) continue;
                    
                    bool has_clearance = true;
                    for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                        if (isOccupiedIdx(ix, iy, iz + check_z)) {
                            has_clearance = false;
                            break;
                        }
                    }
                    if (!has_clearance) continue;

                    auto [wx, wy, wz] = gridToWorld(ix, iy, iz);
                    double d2 = wx*wx + wy*wy + wz*wz;
                    
                    if (d2 < min_dist_sq) {
                        min_dist_sq = d2;
                        best_seed = {ix, iy, iz};
                    }
                }
            }
        }
    }
    
    // 如果原点附近找不到，全图搜索
    if (std::get<0>(best_seed) == -1) {
        for (int ix = 0; ix < nx; ++ix) {
            for (int iy = 0; iy < ny; ++iy) {
                for (int iz = 0; iz < max_iz; ++iz) {
                    if (!isOccupiedIdx(ix, iy, iz)) continue;
                    
                    bool has_clearance = true;
                    for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                        if (isOccupiedIdx(ix, iy, iz + check_z)) {
                            has_clearance = false;
                            break;
                        }
                    }
                    if (!has_clearance) continue;

                    best_seed = {ix, iy, iz};
                    goto found_seed;
                }
            }
        }
        found_seed:;
    }
    
    if (std::get<0>(best_seed) == -1) {
        return;
    }

    std::queue<std::tuple<int, int, int>> q;
    
    q.push(best_seed);
    size_t seed_idx = index3DTo1D(std::get<0>(best_seed), std::get<1>(best_seed), std::get<2>(best_seed));
    visited_ground[seed_idx] = true;
    
    {
        auto [sx, sy, sz] = best_seed;
        valid_surface_voxels_.emplace_back(sx, sy, sz + 1);
    }
    
    int z_step = static_cast<int>(std::ceil(connectivity_height_threshold_ / resolution_));
    // 论文公式(3): Z连通邻域要求 ‖Δc_xy‖₁ = 1，即严格4连通（不含对角线）
    constexpr int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    while (!q.empty()) {
        auto [cx, cy, cz] = q.front();
        q.pop();

        for (int di = 0; di < 4; ++di) {
            int dx = dirs4[di][0];
            int dy = dirs4[di][1];

            int nx_ix = cx + dx;
            int ny_iy = cy + dy;

            if (nx_ix < 0 || nx_ix >= nx || ny_iy < 0 || ny_iy >= ny) continue;

            int min_z = std::max(0, cz - z_step);
            int max_z = std::min(std::max(0, nz - clearance_cells), cz + z_step);

            for (int nz_iz = min_z; nz_iz <= max_z; ++nz_iz) {
                if (!isOccupiedIdx(nx_ix, ny_iy, nz_iz)) continue;

                size_t n_idx = index3DTo1D(nx_ix, ny_iy, nz_iz);
                if (visited_ground[n_idx]) continue;

                visited_ground[n_idx] = true;

                bool has_clearance = true;
                for (int check_z = 1; check_z <= clearance_cells; ++check_z) {
                    int check_iz = nz_iz + check_z;
                    if (check_iz >= nz || isOccupiedIdx(nx_ix, ny_iy, check_iz)) {
                        has_clearance = false;
                        break;
                    }
                }
                if (has_clearance) {
                    valid_surface_voxels_.emplace_back(nx_ix, ny_iy, nz_iz + 1);
                    q.push({nx_ix, ny_iy, nz_iz});
                }
            }
        }
    }
    
    surface_array_.clear();
    surface_array_.resize(total_size, 0);
    
    for (const auto& voxel : valid_surface_voxels_) {
        auto [sx, sy, sz] = voxel;
        if (isValidIdx(sx, sy, sz)) {
            size_t idx = index3DTo1D(sx, sy, sz);
            surface_array_[idx] = 1;
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
    
    // 计算Z方向搜索范围（与BFS连通性保持一致）
    int z_search = static_cast<int>(std::ceil(connectivity_height_threshold_ / resolution_));
    
    auto [nx, ny, nz] = grid_size_;
    
    // ========== Step 1: 初始化距离场 ==========
    // 检查每个surface点的邻域是否有障碍物
    // 如果邻域有障碍物，距离=1；否则距离=INF
    
    // 论文公式(6-7): 有效障碍方向定义用4连通（与 ‖Δc_xy‖₁=1 一致）
    constexpr int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    for (const auto& voxel : valid_surface_voxels_) {
        auto [sx, sy, sz] = voxel;
        if (!isValidIdx(sx, sy, sz)) continue;

        size_t idx = index3DTo1D(sx, sy, sz);

        // 检查4个基本方向是否为有效障碍：该方向无Z连通邻居且有占据体素
        bool near_obstacle = false;
        for (int di = 0; di < 4 && !near_obstacle; ++di) {
            int nx_coord = sx + dirs4[di][0];
            int ny_coord = sy + dirs4[di][1];

            // 先检查该方向是否有Z连通可达的surface点
            bool has_passable_surface = false;
            for (int dz = -z_search; dz <= z_search; ++dz) {
                if (isSurfaceValid(nx_coord, ny_coord, sz + dz)) {
                    has_passable_surface = true;
                    break;
                }
            }

            // 论文公式(6): 无Z连通邻居且该方向有占据体素 → 有效障碍
            if (!has_passable_surface) {
                if (isOccupiedIdx(nx_coord, ny_coord, sz) ||
                    isOccupiedIdx(nx_coord, ny_coord, sz + 1)) {
                    near_obstacle = true;
                }
            }
        }

        // 论文算法1初始化: 边界处D=0，否则D=∞
        distance_map_[idx] = near_obstacle ? 0 : INF;
    }
    
    // ========== Step 2: 两遍DP扫描（优化版本）==========
    // 使用 Chamfer Distance Transform 的思想，但适配 Surface 约束和 Z 轴连通性
    
    // 辅助函数：获取指定坐标的距离值
    auto get_distance = [this](int x, int y, int z) -> uint8_t {
        if (!isValidIdx(x, y, z)) return INF;
        size_t idx = index3DTo1D(x, y, z);
        if (idx >= surface_array_.size() || !surface_array_[idx]) return INF;
        
        auto it = distance_map_.find(idx);
        return (it != distance_map_.end()) ? it->second : INF;
    };
    
    // 辅助函数：尝试从邻居更新当前点的距离
    // 返回更新后的距离值
    auto try_update_from_neighbor = [&](uint8_t current_dist, int nx, int ny, int base_z) -> uint8_t {
        uint8_t min_dist = current_dist;
        
        // 在Z轴连通范围内搜索邻居
        for (int dz = -z_search; dz <= z_search; ++dz) {
            int nz = base_z + dz;
            uint8_t neighbor_dist = get_distance(nx, ny, nz);
            
            if (neighbor_dist < INF) {
                uint8_t new_dist = static_cast<uint8_t>(neighbor_dist + 1);
                if (new_dist > MAX_DISTANCE_U8) {
                    new_dist = MAX_DISTANCE_U8;
                }
                min_dist = std::min(min_dist, new_dist);
            }
        }
        
        return min_dist;
    };
    
    // ===== 前向扫描 (Forward Pass): 从左上到右下 =====
    // 只检查已处理的邻居：左(-1,0)、上(0,-1)
    
    // 外层循环：XY平面从左上到右下
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            // 内层循环：遍历该XY位置上的所有Z层Surface点
            for (int z = 0; z < nz; ++z) {
                if (!isValidIdx(x, y, z)) continue;
                size_t idx = index3DTo1D(x, y, z);
                
                // 检查是否是Surface点
                if (idx >= surface_array_.size() || !surface_array_[idx]) continue;
                
                auto it = distance_map_.find(idx);
                if (it == distance_map_.end()) continue;
                
                uint8_t current_dist = it->second;
                if (current_dist == 0 || current_dist == 1) continue;
                
                // 检查2个前向邻居方向（考虑Z轴连通性）
                current_dist = try_update_from_neighbor(current_dist, x - 1, y, z);
                current_dist = try_update_from_neighbor(current_dist, x, y - 1, z);
                
                // 更新距离值
                distance_map_[idx] = current_dist;
            }
        }
    }
    
    // ===== 后向扫描 (Backward Pass): 从右下到左上 =====
    // 只检查已处理的邻居：右(1,0)、下(0,1)
    
    // 外层循环：XY平面从右下到左上
    for (int y = ny - 1; y >= 0; --y) {
        for (int x = nx - 1; x >= 0; --x) {
            // 内层循环：遍历该XY位置上的所有Z层Surface点
            for (int z = nz - 1; z >= 0; --z) {
                if (!isValidIdx(x, y, z)) continue;
                size_t idx = index3DTo1D(x, y, z);
                
                // 检查是否是Surface点
                if (idx >= surface_array_.size() || !surface_array_[idx]) continue;
                
                auto it = distance_map_.find(idx);
                if (it == distance_map_.end()) continue;
                
                uint8_t current_dist = it->second;
                if (current_dist == 0 || current_dist == 1) continue;
                
                // 检查2个后向邻居方向（考虑Z轴连通性）
                current_dist = try_update_from_neighbor(current_dist, x + 1, y, z);
                current_dist = try_update_from_neighbor(current_dist, x, y + 1, z);
                
                // 更新距离值，并限制在MAX_DISTANCE范围内
                if (current_dist < INF) {
                    distance_map_[idx] = std::min(current_dist, MAX_DISTANCE_U8);
                } else {
                    distance_map_[idx] = INF;
                }
            }
        }
    }
}


int VoxelMap::coordToSurfaceIdx(int ix, int iy, int iz) const
{
    size_t grid_idx = index3DTo1D(ix, iy, iz);
    auto it = coord_to_surface_idx_.find(grid_idx);
    if (it != coord_to_surface_idx_.end()) {
        return it->second;
    }
    return -1;
}

void VoxelMap::buildSurfaceGraph()
{
    if (valid_surface_voxels_.empty()) {
        return;
    }
    
    const size_t n = valid_surface_voxels_.size();
    int z_search = static_cast<int>(std::ceil(connectivity_height_threshold_ / resolution_));
    
    // 清空旧数据
    surface_graph_.clear();
    surface_graph_.reserve(n);
    coord_to_surface_idx_.clear();
    coord_to_surface_idx_.reserve(n);
    
    // Step 1: 创建所有 SurfaceNode 并建立坐标到索引的映射
    for (size_t i = 0; i < n; ++i) {
        auto [x, y, z] = valid_surface_voxels_[i];
        size_t grid_idx = index3DTo1D(x, y, z);
        
        SurfaceNode node;
        node.x = x;
        node.y = y;
        node.z = z;
        node.distance = 255;
        node.neighbor_start = -1;
        node.neighbor_count = 0;
        
        // 从 distance_map_ 获取距离值
        auto it = distance_map_.find(grid_idx);
        if (it != distance_map_.end()) {
            node.distance = it->second;
        }
        
        surface_graph_.push_back(node);
        coord_to_surface_idx_[grid_idx] = static_cast<int>(i);
    }
    
    // 论文公式(3): 4连通邻域，‖Δc_xy‖₁ = 1
    constexpr int dirs4[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};

    // Step 2: 第一遍 - 统计每个节点的邻居数量
    // 修复：遍历 [-z_search, z_search] 全范围内所有 surface 邻居，
    // 而不是找到第一个就停止。原逻辑只连接最近的 z 邻居，会导致
    // surface graph 不连通（BFS 可达的节点在 graph 中却无法互达）。
    std::vector<int> neighbor_counts(n, 0);

    for (size_t i = 0; i < n; ++i) {
        const auto& node = surface_graph_[i];

        for (int di = 0; di < 4; ++di) {
            int nx = node.x + dirs4[di][0];
            int ny = node.y + dirs4[di][1];

            for (int dz = -z_search; dz <= z_search; ++dz) {
                if (coordToSurfaceIdx(nx, ny, node.z + dz) >= 0) {
                    neighbor_counts[i]++;
                }
            }
        }
    }

    // Step 3: 计算 all_neighbors_ 的总大小并预分配
    size_t total_neighbors = 0;
    for (size_t i = 0; i < n; ++i) {
        surface_graph_[i].neighbor_start = static_cast<int>(total_neighbors);
        surface_graph_[i].neighbor_count = neighbor_counts[i];
        total_neighbors += neighbor_counts[i];
    }
    all_neighbors_.clear();
    all_neighbors_.reserve(total_neighbors);

    // Step 4: 第二遍 - 填充 all_neighbors_（与Step2逻辑完全一致）
    for (size_t i = 0; i < n; ++i) {
        const auto& node = surface_graph_[i];

        for (int di = 0; di < 4; ++di) {
            int nx = node.x + dirs4[di][0];
            int ny = node.y + dirs4[di][1];

            for (int dz = -z_search; dz <= z_search; ++dz) {
                int neighbor_idx = coordToSurfaceIdx(nx, ny, node.z + dz);
                if (neighbor_idx >= 0) {
                    all_neighbors_.push_back(neighbor_idx);
                }
            }
        }
    }
}

}  // namespace jps_planner_3d