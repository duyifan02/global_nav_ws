#ifndef JPS_PLANNER_3D__VOXEL_MAP_HPP_
#define JPS_PLANNER_3D__VOXEL_MAP_HPP_

#include <memory>
#include <vector>
#include <tuple>
#include <limits>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace jps_planner_3d
{

/**
 * @brief 体素地图类
 * 
 * 基于八叉树地图的体素地图，提供高效的查询功能
 */
class VoxelMap
{
public:
    /**
     * @brief 构造函数 (JPS V2接口)
     * 
    * @param resolution 体素分辨率（米）
    */
    VoxelMap(double resolution = 0.2);

    /**
     * @brief 析构函数
     */
    ~VoxelMap();

    /**
     * @brief 从点云更新体素地图（多种子版本）
     *
     * @param occupied_voxels 占据体素的世界坐标列表
     * @param resolution 体素分辨率
     * @param seed_points BFS种子点列表（世界坐标）。推荐传入 [start, goal] 双种子，
     *                    确保起点和终点所在的连通分量都被包含在surface中。
     *                    为空时从原点附近自动搜索。
     * @return true表示更新成功，false表示失败
     */
    bool updateFromPoints(const std::vector<std::tuple<float, float, float>>& occupied_voxels,
                          double resolution,
                          const std::vector<std::tuple<float, float, float>>& seed_points = {});

    /**
     * @brief 检查地图是否就绪
     * 
     * @return true表示就绪，false表示未就绪
     */
    bool isReady() const;

    /**
     * @brief 检查世界坐标点是否被占据
     * 
     * @param x 世界坐标x
     * @param y 世界坐标y
     * @param z 世界坐标z
     * @return true表示被占据，false表示空闲
     */
    bool isOccupied(double x, double y, double z) const;

    /**
     * @brief 检查网格索引是否被占据
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return true表示被占据，false表示空闲
     */
    bool isOccupiedIdx(int ix, int iy, int iz) const;

    /**
     * @brief 检查网格索引是否有效
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return true表示有效，false表示无效
     */
    bool isValidIdx(int ix, int iy, int iz) const;

    /**
     * @brief 世界坐标转网格索引
     * 
     * @param x 世界坐标x
     * @param y 世界坐标y
     * @param z 世界坐标z
     * @return 网格索引 (ix, iy, iz)，如果无效则返回nullptr
     */
    std::tuple<int, int, int> worldToGrid(double x, double y, double z) const;

    /**
     * @brief 网格索引转世界坐标
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return 世界坐标 (x, y, z)
     */
    std::tuple<double, double, double> gridToWorld(int ix, int iy, int iz) const;

    /**
     * @brief 获取所有有效的表面体素索引列表
     * 
     * @return 表面体素的索引列表 (ix, iy, iz)
     */
    const std::vector<std::tuple<int, int, int>>& getValidSurfaceVoxels() const { return valid_surface_voxels_; }

    /**
     * @brief 检查某个坐标是否是有效的Surface（使用直接索引，O(1)查询）
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return true表示是有效Surface，false表示不是
     */
    bool isSurfaceValid(int ix, int iy, int iz) const;

    /**
     * @brief 在指定坐标附近搜索有效的Surface点
     *
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @param search_radius 搜索半径（格子数）
     * @param should_cancel 可选的取消检查回调，返回true表示应该中止搜索
     * @return 找到的最近Surface点索引，未找到或被取消返回(-1,-1,-1)
     */
    std::tuple<int, int, int> findNearestValidSurface(
        int ix, int iy, int iz,
        int search_radius = 10,
        const std::function<bool()>& should_cancel = nullptr) const;

    /**
     * @brief 检查指定坐标附近是否有障碍物（用于贴墙惩罚）
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return true表示附近有障碍物
     */
    bool hasAdjacentObstacles(int ix, int iy, int iz) const;

    /**
     * @brief 获取指定位置到最近障碍物的距离（XY平面曼哈顿距离，网格单位）
     * 
     * @param ix 网格索引x
     * @param iy 网格索引y
     * @param iz 网格索引z
     * @return 到最近障碍物的距离（网格单位），如果是障碍物返回0
     */
    int getDistanceToObstacle(int ix, int iy, int iz) const;

    /**
     * @brief 获取体素分辨率
     * 
     * @return 分辨率（米）
     */
    double getResolution() const { return resolution_; }

    /**
     * @brief 获取距离场最大距离
     * 
     * @return 最大距离（网格单位）
     */
    int getDistanceFieldMaxDistance() const { return distance_field_max_distance_; }

    /**
     * @brief 设置BFS连通性高度阈值
     * 
     * @param threshold 高度阈值（米），控制BFS建图时可跨越的台阶高度
     */
    void setConnectivityHeightThreshold(double threshold) { connectivity_height_threshold_ = threshold; }

    /**
     * @brief 设置Surface上方空气高度要求
     * 
     * @param height 高度（米），只有上方这个范围内都无障碍物才标记为Surface
     */
    void setSurfaceClearanceHeight(double height) { surface_clearance_height_ = height; }

    /**
     * @brief 设置距离场最大距离
     * 
     * @param max_distance 最大距离（网格单位），用于贴墙惩罚计算，最大255（8 bits）
     */
    void setDistanceFieldMaxDistance(int max_distance) { 
        distance_field_max_distance_ = std::min(max_distance, 255); // 限制最大值为255（uint8_t最大值）
    }


    /**
     * @brief 获取网格尺寸
     * 
     * @return 网格尺寸 (nx, ny, nz)
     */
    std::tuple<int, int, int> getGridSize() const { return grid_size_; }

    /**
     * @brief 获取可通行性估计时间 (Te: traversability estimation time)
     * 
     * @return buildReachableMap 耗时（毫秒）
     */
    double getTraversabilityTimeMs() const { return traversability_time_ms_; }

    /**
     * @brief 获取构建后场景表示占用内存估算（字节）
     * 
     * 统计地图构建后用于规划的主要数据结构（surface、distance field、graph 等）
     * 的近似内存占用，不包含进程其他组件与分配器额外开销。
     */
    size_t getConstructedSceneMemoryUsageBytes() const;

    /**
     * @brief Surface Graph 节点结构（扁平化邻接表）
     * 
     * 用于A*搜索的优化数据结构，预计算邻居关系
     * 邻居存储在全局扁平数组 all_neighbors 中
     */
    struct SurfaceNode {
        int x, y, z;           // 网格坐标
        int distance;          // 到障碍物距离（来自距离场）
        int neighbor_start;    // 邻居在 all_neighbors 中的起始位置
        int neighbor_count;   // 邻居数量
    };

    /**
     * @brief 获取Surface Graph
     * 
     * @return Surface节点数组的常量引用
     */
    const std::vector<SurfaceNode>& getSurfaceGraph() const { return surface_graph_; }

    /**
     * @brief 获取扁平化邻居数组
     * 
     * @return 所有邻居的连续数组
     */
    const std::vector<int>& getAllNeighbors() const { return all_neighbors_; }

    /**
     * @brief 将网格坐标转换为Surface Graph索引
     * 
     * @param ix, iy, iz 网格坐标
     * @return 节点索引，如果不存在返回-1
     */
    int coordToSurfaceIdx(int ix, int iy, int iz) const;

private:
    /**
     * @brief 从多个种子点开始BFS构建可到达区域
     * @param seed_points 种子点列表（世界坐标），为空则自动搜索
     */
    void buildReachableMap(const std::vector<std::tuple<float, float, float>>& seed_points = {});

    /**
     * @brief 只在Surface上使用DP算法计算距离场
     * 
     * 优化流程：
     * 1. 初始化：检查每个surface邻域，有障碍物则距离=1，否则距离=MAX
     * 2. 多遍DP扫描直到收敛，只在surface点之间传播距离
     * 3. 结果存储为直接索引数组，O(1)查询
     */
    void computeDistanceFieldOnSurface();

    /**
     * @brief 构建Surface Graph
     * 
     * 预计算所有surface节点的邻居关系，用于A*搜索加速
     */
    void buildSurfaceGraph();

    double resolution_;                    // 体素分辨率
    double connectivity_height_threshold_ = 0.6; // 连通性高度阈值
    double surface_clearance_height_ = 0.8; // Surface上方空气高度要求（米）
    int distance_field_max_distance_ = 5;   // 距离场最大距离（网格单位，最大255，使用8 bits存储）
    // BFS邻域搜索距离固定为1（网格单位）

    // 体素网格数据（直接索引形式，用于构建阶段和路径验证）
    std::vector<uint8_t> voxel_array_;        // 体素占据状态（扁平化存储，uint8_t避免vector<bool>内存问题）
    
    // Surface数据（直接索引形式，O(1)查询，优化存储空间）
    std::vector<std::tuple<int, int, int>> valid_surface_voxels_; // 有效Surface体素列表（用于遍历）
    std::vector<uint8_t> surface_array_;      // Surface标记数组（直接索引，O(1)查询）
    std::unordered_map<size_t, uint8_t> distance_map_; // 距离场映射（只存储surface点，8 bits per value，最大255）
    
    // Surface Graph 优化结构（扁平化邻接表）
    std::vector<SurfaceNode> surface_graph_;      // Surface节点数组，连续存储
    std::vector<int> all_neighbors_;              // 所有邻居的扁平数组
    std::unordered_map<size_t, int> coord_to_surface_idx_;  // 网格坐标→graph索引映射（构建时用）
    
    std::tuple<int, int, int> grid_size_;   // 网格尺寸 (nx, ny, nz)
    std::tuple<double, double, double> min_bound_;  // 最小边界
    std::tuple<double, double, double> max_bound_;  // 最大边界
    std::tuple<double, double, double> origin_;     // 原点
    
    double traversability_time_ms_ = 0.0;  // Te: traversability estimation time (buildReachableMap)

    /**
     * @brief 将3D索引转换为1D索引
     */
    size_t index3DTo1D(int ix, int iy, int iz) const
    {
        auto [nx, ny, nz] = grid_size_;
        return static_cast<size_t>(ix) * static_cast<size_t>(ny) * static_cast<size_t>(nz) +
               static_cast<size_t>(iy) * static_cast<size_t>(nz) +
               static_cast<size_t>(iz);
    }
};

}  // namespace jps_planner_3d

#endif  // JPS_PLANNER_3D__VOXEL_MAP_HPP_
