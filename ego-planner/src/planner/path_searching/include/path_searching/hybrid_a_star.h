#ifndef _HYBRID_A_STAR_H_
#define _HYBRID_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <unordered_map>
#include <memory>
#include "path_searching/reeds_shepp.h"

constexpr double inf = 1e9;

/**
 * @brief 混合 A* 的节点结构，包含 3D 状态 (x, y, theta)
 */
struct HybridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    enum search_direction
    {
        FORWARD = 0,
        BACKWARD = 1
    };

    int rounds{0};
    enum enum_state state{UNDEFINED};
    enum search_direction direction{FORWARD};

    // 3D 位姿状态
    Eigen::Vector3d pose;     // (x, y, theta)
    Eigen::Vector2i grid_idx; // 所在栅格索引

    double gScore{inf}, fScore{inf};
    std::shared_ptr<HybridNode> cameFrom{nullptr};

    // 从父节点到本节点的转向角度
    double control_input{0.0};  // 方向盘转角或曲率

    // 双向搜索连接点
    std::shared_ptr<HybridNode> meet_node{nullptr};  // 相遇的节点
};

class HybridNodeComparator
{
public:
    bool operator()(const std::shared_ptr<HybridNode>& node1,
                    const std::shared_ptr<HybridNode>& node2) const
    {
        return node1->fScore > node2->fScore;
    }
};

/**
 * @brief 双向混合 A* 路径规划器
 * 融合栅格搜索的快速性和连续路径的光滑性
 * 支持双向搜索以提高效率，同时保持海事避碰规则
 */
class HybridAStar
{
private:
    GridMap::Ptr grid_map_;
    ReedsSheppCurve reed_shepp_curve_;

    // 状态空间参数
    double step_size_;                    // 栅格步长
    double inv_step_size_;
    double angle_resolution_;             // 方向角离散化精度 (rad)
    double turning_radius_;               // 转向半径
    double max_steering_angle_;           // 最大转向角
    double velocity_;                     // 规划速度 (用于运动学约束)

    Eigen::Vector2d center_;
    Eigen::Vector2i CENTER_IDX_, POOL_SIZE_;

    // 开闭集管理 - 正向搜索
    std::priority_queue<std::shared_ptr<HybridNode>,
                        std::vector<std::shared_ptr<HybridNode>>,
                        HybridNodeComparator> openSet_forward_;

    // 开闭集管理 - 反向搜索
    std::priority_queue<std::shared_ptr<HybridNode>,
                        std::vector<std::shared_ptr<HybridNode>>,
                        HybridNodeComparator> openSet_backward_;

    // 节点缓存 (加速查询，减少内存分配)
    std::unordered_map<std::string, std::shared_ptr<HybridNode>> closed_nodes_forward_;
    std::unordered_map<std::string, std::shared_ptr<HybridNode>> closed_nodes_backward_;

    std::vector<std::shared_ptr<HybridNode>> hybrid_path_;
    int rounds_{0};

    // ========== 海事规则相关 ==========
    int colregs_mode_{0};
    Eigen::Vector2d start_pos_;
    Eigen::Vector2d ts_pos_, ts_vel_;
    Eigen::Vector2d os_vel_;
    bool has_target_ship_{false};

    double dcpa_{10.0};
    double tcpa_{0.0};
    double safe_dcpa_{3.5};

    double calculateThreatCost(const Eigen::Vector2d& node_pos) const;
    double calculateInflatedMapProximityCost(const Eigen::Vector2d& node_pos) const;
    double calculateSteeringSmoothnessCost(const std::shared_ptr<HybridNode>& current,
                                           double steering_angle) const;
    double angleDiff(double a, double b) const;
    bool isLineCollisionFree(const Eigen::Vector3d& start,
                             const Eigen::Vector3d& goal) const;
    void shortcutHybridPath();
    // ===================================

    // 私有辅助方法
    std::string stateKey(const Eigen::Vector3d& pose) const;
    std::shared_ptr<HybridNode> getOrCreateNode(const Eigen::Vector3d& pose, HybridNode::search_direction direction);

    std::vector<std::shared_ptr<HybridNode>> retrievePath(std::shared_ptr<HybridNode> current);

    /**
     * @brief 获取候选动作（控制输入）
     * @return 候选转向角列表
     */
    std::vector<double> getMotionPrimitives();

    /**
     * @brief 根据转向角生成正向运动轨迹
     * @param start 起始位姿
     * @param steering_angle 转向角
     * @param step_dist 运动距离
     * @return 生成的位姿点集
     */
    std::vector<Eigen::Vector3d> generateMotion(const Eigen::Vector3d& start,
                                                 double steering_angle,
                                                 double step_dist);

    /**
     * @brief 根据转向角生成反向运动轨迹（逆运动学）
     * @param start 起始位姿
     * @param steering_angle 转向角（会取反）
     * @param step_dist 运动距离
     * @return 生成的位姿点集
     */
    std::vector<Eigen::Vector3d> generateReverseMotion(const Eigen::Vector3d& start,
                                                        double steering_angle,
                                                        double step_dist);

    /**
     * @brief 检查两个方向的节点是否相遇
     * @param forward_node 正向节点
     * @param backward_node 反向节点
     * @return 是否相遇
     */
    bool checkNodesMeet(const std::shared_ptr<HybridNode>& forward_node,
                        const std::shared_ptr<HybridNode>& backward_node) const;

    /**
     * @brief 重建双向搜索的完整路径
     * @param forward_meet 正向相遇节点
     * @param backward_meet 反向相遇节点
     */
    void reconstructBidirectionalPath(const std::shared_ptr<HybridNode>& forward_meet,
                                      const std::shared_ptr<HybridNode>& backward_meet);

    /**
     * @brief Reeds-Shepp 连接尝试
     * 尝试用 Reeds-Shepp 曲线直接连接到目标
     */
    bool tryReedSheppConnection(const std::shared_ptr<HybridNode>& current,
                                const Eigen::Vector3d& goal_pose,
                                std::vector<Eigen::Vector3d>& connection_path);

    /**
     * @brief 检查轨迹碰撞
     */
    bool checkTrajectoryCollision(const std::vector<Eigen::Vector3d>& trajectory);
    bool hasCollisionFreeMotion(const Eigen::Vector2d& point,
                                double theta,
                                bool reverse);
    bool adjustEndpointOutOfObstacle(Eigen::Vector2d& point,
                                     const Eigen::Vector2d& away_from,
                                     const char* point_name);
    bool adjustEndpointForSearch(Eigen::Vector2d& point,
                                 double theta,
                                 const Eigen::Vector2d& away_from,
                                 const char* point_name,
                                 bool reverse);
    bool tryLateralBypass(const Eigen::Vector3d& start_pose,
                          const Eigen::Vector3d& goal_pose);

    inline Eigen::Vector2d Index2Coord(const Eigen::Vector2i& index) const;
    inline bool Coord2Index(const Eigen::Vector2d& pt, Eigen::Vector2i& idx) const;

public:
    typedef std::shared_ptr<HybridAStar> Ptr;

    HybridAStar(double angle_resolution = 0.1745,  // 10 度
                double turning_radius = 1.0);
    ~HybridAStar() = default;

    /**
     * @brief 初始化地图和搜索空间
     */
    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size);

    /**
     * @brief 执行混合 A* 搜索
     * @param step_size 栅格步长
     * @param start_pt 起始位置
     * @param start_theta 起始方向角
     * @param goal_pt 目标位置
     * @param goal_theta 目标方向角
     * @return 是否搜索成功
     */
    bool search(double step_size,
                const Eigen::Vector2d& start_pt, double start_theta,
                const Eigen::Vector2d& goal_pt, double goal_theta);

    /**
     * @brief 获取规划的路径（3D 位姿）
     */
    std::vector<Eigen::Vector3d> getPath() const;

    /**
     * @brief 获取仅包含位置的路径（2D）
     */
    std::vector<Eigen::Vector3d> getPositionPath() const;

    // ========== 参数设置接口 ==========
    void setVelocity(double vel) { velocity_ = vel; }
    void setMaxSteeringAngle(double angle) { max_steering_angle_ = angle; }
    void setTurningRadius(double radius)
    {
        turning_radius_ = radius;
        reed_shepp_curve_.setTurningRadius(radius);
    }

    void setColregsMode(int mode) { colregs_mode_ = mode; }
    void setStartPos(const Eigen::Vector2d& pos) { start_pos_ = pos; }
    void setTargetShipInfo(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel,
                          const Eigen::Vector2d& os_vel)
    {
        // 指定舰船信息用于海事避碰规则计算
        ts_pos_ = pos;      // 目标舰船位置
        ts_vel_ = vel;      // 目标舰船速度
        os_vel_ = os_vel;   // 本舰船正向速度向量
                            // 注意：基于位置的COLREGS判断对正向和反向搜索都不改变，
                            // 但os_vel_的符号在反向搜索中应明确衣不同的含义
        has_target_ship_ = true;
    }
    void setSafetyParams(double dcpa, double tcpa)
    {
        dcpa_ = dcpa;
        tcpa_ = tcpa;
    }
    void setSafeDcpa(double safe_dcpa) { safe_dcpa_ = safe_dcpa; }

    inline bool checkOccupancy(const Eigen::Vector2d& pos)
    {
        if (grid_map_ == nullptr)
            return true;
        return (bool)grid_map_->getInflateOccupancy2d(pos);
    }

    // ========== 获取内部状态 ==========
    int getSearchIterations() const { return rounds_; }
    double getTurningRadius() const { return turning_radius_; }
};

#endif  // _HYBRID_A_STAR_H_
