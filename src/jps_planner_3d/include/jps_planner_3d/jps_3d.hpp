#ifndef JPS_PLANNER_3D__JPS_3D_HPP_
#define JPS_PLANNER_3D__JPS_3D_HPP_

#include "jps_planner_3d/voxel_map.hpp"
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <unordered_map>
#include <queue>
#include <limits>
#include <functional>

namespace jps_planner_3d
{

// 用于tuple的哈希函数
struct TupleHash {
    size_t operator()(const std::tuple<int, int, int>& t) const {
        auto [x, y, z] = t;
        return std::hash<int>()(x) ^ (std::hash<int>()(y) << 1) ^ (std::hash<int>()(z) << 2);
    }
};

/**
 * @brief 3D Jump Point Search路径规划器
 */
class JPS3D
{
public:
    /**
     * @brief 构造函数 (JPS V2接口)
     * 
     * @param voxel_map 体素地图指针（必须已构建Surface和距离场）
     * @param heuristic_weight 启发式权重 (1.0=最优路径, >1=加速搜索但可能非最优)
     * @param max_z_change 最大允许Z轴变化(米)，限制单步Z轴移动距离
     * @param z_penalty_weight Z轴变化惩罚权重，用于惩罚垂直移动
     * @param potential_weight 贴墙惩罚权重，基于距离场计算，0表示禁用贴墙惩罚
     */
    JPS3D(
        std::shared_ptr<VoxelMap> voxel_map,
        double heuristic_weight = 1.0,
        double max_z_change = 0.5,
        double z_penalty_weight = 3.0,
        double potential_weight = 100.0);

    /**
     * @brief 析构函数
     */
    ~JPS3D();

    /**
     * @brief 规划从起点到终点的路径
     *
     * @param start_pos 起点世界坐标 (x, y, z)
     * @param goal_pos 终点世界坐标 (x, y, z)
     * @return 路径点列表（世界坐标），如果失败返回空列表
     */
	std::vector<std::tuple<double, double, double>> plan(
		const std::tuple<double, double, double>& start_pos,
		const std::tuple<double, double, double>& goal_pos,
		bool snap_to_surface = true);

    /**
     * @brief 获取搜索统计信息
     * 
     * @return 统计信息映射
     */
    std::unordered_map<std::string, int> getSearchStats() const { return search_stats_; }

    /**
     * @brief 获取探索的节点列表（用于可视化）
     * 
     * @return 探索的节点索引列表
     */
    std::vector<std::tuple<int, int, int>> getExploredNodes() const { return explored_nodes_; }

private:
    /**
     * @brief JPS搜索主函数
     * 
     * @param start 起点网格索引
     * @param goal 终点网格索引
     * @return 路径索引列表，失败返回空列表
     */
    std::vector<std::tuple<int, int, int>> jpsSearch(
        const std::tuple<int, int, int>& start,
        const std::tuple<int, int, int>& goal);

    /**
     * @brief 获取当前节点的跳点后继节点
     * 
     * @param current 当前节点
     * @param parent 父节点
     * @param goal 目标节点
     * @return 跳点列表
     */
    std::vector<std::tuple<int, int, int>> getSuccessors(
        const std::tuple<int, int, int>& current,
        const std::tuple<int, int, int>& parent,
        const std::tuple<int, int, int>& goal);

    /**
     * @brief 启发式函数（欧几里得距离 + 地面机器人Z惩罚）
     * 
     * @param node1 节点1
     * @param node2 节点2
     * @return 距离
     */
    double heuristic(
        const std::tuple<int, int, int>& node1,
        const std::tuple<int, int, int>& node2) const;

    /**
     * @brief 计算两个节点之间的实际距离（考虑代价）
     * 
     * @param node1 节点1
     * @param node2 节点2
     * @return 距离（包含代价权重）
     */
    double distance(
        const std::tuple<int, int, int>& node1,
        const std::tuple<int, int, int>& node2) const;

    /**
     * @brief 重建路径
     * 
     * @param came_from 父节点映射
     * @param current 当前节点（终点）
     * @return 路径节点列表
     */
    std::vector<std::tuple<int, int, int>> reconstructPath(
        const std::unordered_map<std::tuple<int, int, int>, std::tuple<int, int, int>,
            TupleHash>& came_from,
        const std::tuple<int, int, int>& current);

    std::shared_ptr<VoxelMap> voxel_map_;
    double heuristic_weight_;
    double max_z_change_;
    double z_penalty_weight_;
    double potential_weight_;

    int max_iterations_;

    std::vector<std::tuple<int, int, int>> explored_nodes_;  // 探索的节点
    std::unordered_map<std::string, int> search_stats_;       // 搜索统计
};

}  // namespace jps_planner_3d

#endif  // JPS_PLANNER_3D__JPS_3D_HPP_