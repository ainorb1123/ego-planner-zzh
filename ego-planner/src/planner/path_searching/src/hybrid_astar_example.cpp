/**
 * @file hybrid_astar_example.cpp
 * @brief 混合 A* 路径规划算法的使用示例
 * 
 * 本文件展示如何在 ego-planner 系统中集成混合 A* 算法
 */

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <path_searching/hybrid_a_star.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class HybridAStarPlanner
{
private:
    ros::NodeHandle nh_;
    HybridAStar::Ptr hybrid_astar_;
    GridMap::Ptr grid_map_;

    // ROS 发布器
    ros::Publisher path_marker_pub_;
    ros::Publisher trajectory_pub_;

    // 规划参数
    double angle_resolution_;
    double turning_radius_;
    double velocity_;
    double max_steering_angle_;

public:
    HybridAStarPlanner() : nh_("~")
    {
        // 从参数服务器读取配置
        nh_.param("angle_resolution", angle_resolution_, 0.1745);      // 10°
        nh_.param("turning_radius", turning_radius_, 1.0);
        nh_.param("velocity", velocity_, 1.0);
        nh_.param("max_steering_angle", max_steering_angle_, 0.5);

        // 初始化规划器
        hybrid_astar_ = std::make_shared<HybridAStar>(angle_resolution_, turning_radius_);
        hybrid_astar_->setVelocity(velocity_);
        hybrid_astar_->setMaxSteeringAngle(max_steering_angle_);

        // 初始化发布器
        path_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("hybrid_astar_path", 10);
        trajectory_pub_ = nh_.advertise<visualization_msgs::Marker>("hybrid_astar_trajectory", 10);

        ROS_INFO("Hybrid A* Planner initialized with:");
        ROS_INFO("  Angle Resolution: %.2f rad (%.1f°)", angle_resolution_, 
                 angle_resolution_ * 180.0 / M_PI);
        ROS_INFO("  Turning Radius: %.2f m", turning_radius_);
        ROS_INFO("  Velocity: %.2f m/s", velocity_);
        ROS_INFO("  Max Steering Angle: %.2f rad (%.1f°)", max_steering_angle_,
                 max_steering_angle_ * 180.0 / M_PI);
    }

    /**
     * @brief 初始化地图
     */
    void initializeMap(GridMap::Ptr map)
    {
        grid_map_ = map;
        Eigen::Vector2i pool_size(200, 200);
        hybrid_astar_->initGridMap(grid_map_, pool_size);
        ROS_INFO("Map initialized with pool size: %d x %d", pool_size.x(), pool_size.y());
    }

    /**
     * @brief 规划路径
     * @param start_pos 起始位置 (x, y)
     * @param start_angle 起始方向角 (rad)
     * @param goal_pos 目标位置 (x, y)
     * @param goal_angle 目标方向角 (rad)
     * @return 是否规划成功
     */
    bool planPath(const Eigen::Vector2d& start_pos, double start_angle,
                  const Eigen::Vector2d& goal_pos, double goal_angle)
    {
        ROS_INFO("Planning path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                 start_pos.x(), start_pos.y(), start_angle * 180.0 / M_PI,
                 goal_pos.x(), goal_pos.y(), goal_angle * 180.0 / M_PI);

        // 执行搜索
        bool success = hybrid_astar_->search(0.1, start_pos, start_angle, goal_pos, goal_angle);

        if (success)
        {
            std::vector<Eigen::Vector3d> path = hybrid_astar_->getPath();
            ROS_INFO("Path found with %zu waypoints", path.size());

            // 可视化路径
            visualizePath(path);

            return true;
        }
        else
        {
            ROS_WARN("Path planning failed!");
            return false;
        }
    }

    /**
     * @brief 规划考虑海事规则的路径
     */
    bool planPathWithColregs(const Eigen::Vector2d& start_pos, double start_angle,
                             const Eigen::Vector2d& goal_pos, double goal_angle,
                             int colregs_mode,
                             const Eigen::Vector2d& target_ship_pos,
                             const Eigen::Vector2d& target_ship_vel,
                             const Eigen::Vector2d& own_vel)
    {
        ROS_INFO("Planning with COLREGS mode: %d", colregs_mode);

        // 设置海事规则参数
        hybrid_astar_->setColregsMode(colregs_mode);
        hybrid_astar_->setStartPos(start_pos);
        hybrid_astar_->setTargetShipInfo(target_ship_pos, target_ship_vel, own_vel);
        hybrid_astar_->setSafetyParams(5.0, 3.0);  // DCPA 5.0 m, TCPA 3.0 s

        // 执行搜索
        return planPath(start_pos, start_angle, goal_pos, goal_angle);
    }

    /**
     * @brief 可视化规划的路径
     */
    void visualizePath(const std::vector<Eigen::Vector3d>& path)
    {
        visualization_msgs::MarkerArray marker_array;

        // 路径点标记
        visualization_msgs::Marker points;
        points.header.frame_id = "world";
        points.header.stamp = ros::Time::now();
        points.ns = "hybrid_astar";
        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.scale.z = 0.1;
        points.color.g = 1.0;
        points.color.a = 0.8;

        // 路径线
        visualization_msgs::Marker line;
        line.header.frame_id = "world";
        line.header.stamp = ros::Time::now();
        line.ns = "hybrid_astar";
        line.id = 1;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.b = 1.0;
        line.color.a = 0.8;

        // 方向箭头
        visualization_msgs::Marker arrows;
        arrows.header.frame_id = "world";
        arrows.header.stamp = ros::Time::now();
        arrows.ns = "hybrid_astar";
        arrows.id = 2;
        arrows.type = visualization_msgs::Marker::ARROW;
        arrows.action = visualization_msgs::Marker::ADD;
        arrows.scale.x = 0.3;
        arrows.scale.y = 0.1;
        arrows.scale.z = 0.1;
        arrows.color.r = 1.0;
        arrows.color.a = 0.8;

        for (size_t i = 0; i < path.size(); ++i)
        {
            geometry_msgs::Point p;
            p.x = path[i].x();
            p.y = path[i].y();
            p.z = 0.0;

            points.points.push_back(p);
            line.points.push_back(p);
        }

        marker_array.markers.push_back(points);
        marker_array.markers.push_back(line);

        // 发布标记
        path_marker_pub_.publish(marker_array);

        ROS_DEBUG("Published path visualization with %zu points", path.size());
    }

    /**
     * @brief 获取规划的路径
     */
    std::vector<Eigen::Vector3d> getPath() const
    {
        return hybrid_astar_->getPath();
    }

    /**
     * @brief 调整运动学参数
     */
    void adjustKinematicParams(double velocity, double turning_radius, 
                               double max_steering_angle)
    {
        hybrid_astar_->setVelocity(velocity);
        hybrid_astar_->setTurningRadius(turning_radius);
        hybrid_astar_->setMaxSteeringAngle(max_steering_angle);

        ROS_INFO("Updated kinematic parameters:");
        ROS_INFO("  Velocity: %.2f m/s", velocity);
        ROS_INFO("  Turning Radius: %.2f m", turning_radius);
        ROS_INFO("  Max Steering Angle: %.2f rad", max_steering_angle);
    }
};

// ============= 示例用法 =============

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hybrid_astar_example");
    ros::NodeHandle nh;

    // 创建规划器
    HybridAStarPlanner planner;

    // 创建模拟的地图
    // 注意: 实际应用中应使用真实的 GridMap 实例
    GridMap::Ptr grid_map = std::make_shared<GridMap>();

    // 初始化地图
    planner.initializeMap(grid_map);

    // ===== 示例 1: 基础路径规划 =====
    ROS_INFO("\n========== Example 1: Basic Path Planning ==========");
    Eigen::Vector2d start_pos(0.0, 0.0);
    double start_angle = 0.0;
    Eigen::Vector2d goal_pos(10.0, 10.0);
    double goal_angle = M_PI / 4.0;

    if (planner.planPath(start_pos, start_angle, goal_pos, goal_angle))
    {
        std::vector<Eigen::Vector3d> path = planner.getPath();
        ROS_INFO("Successfully planned path with %zu waypoints", path.size());
    }

    // ===== 示例 2: 带海事规则的路径规划 =====
    ROS_INFO("\n========== Example 2: Path Planning with COLREGS ==========");
    Eigen::Vector2d target_ship_pos(5.0, 15.0);
    Eigen::Vector2d target_ship_vel(1.0, 0.5);
    Eigen::Vector2d own_vel(1.0, 0.0);

    planner.planPathWithColregs(
        start_pos, start_angle,
        goal_pos, goal_angle,
        2,  // CROSS_GIVE_WAY
        target_ship_pos, target_ship_vel, own_vel);

    // ===== 示例 3: 调整运动学参数 =====
    ROS_INFO("\n========== Example 3: Adjusting Kinematic Parameters ==========");
    planner.adjustKinematicParams(2.0, 0.5, 0.7);

    // 重新规划
    planner.planPath(start_pos, start_angle, goal_pos, goal_angle);

    // ===== 示例 4: 不同的方向角 =====
    ROS_INFO("\n========== Example 4: Path Planning with Different Final Angle ==========");
    double goal_angle_2 = -M_PI / 2.0;
    planner.planPath(start_pos, start_angle, goal_pos, goal_angle_2);

    ros::spin();

    return 0;
}

/**
 * @brief 集成到 plan_manage 的示例
 * 
 * 在 plan_manage.cpp 中替换 A* 调用：
 * 
 * 旧代码:
 * {
 *     AStar::Ptr a_star = std::make_shared<AStar>();
 *     a_star->initGridMap(grid_map_, grid_size_);
 *     a_star->AstarSearch(step_size_, start_pt, goal_pt);
 *     path = a_star->getPath();
 * }
 * 
 * 新代码:
 * {
 *     HybridAStar::Ptr hybrid_astar = std::make_shared<HybridAStar>(0.1745, 1.0);
 *     hybrid_astar->initGridMap(grid_map_, grid_size_);
 *     
 *     // 可选: 设置海事规则
 *     hybrid_astar->setColregsMode(colregs_mode_);
 *     hybrid_astar->setTargetShipInfo(target_pos, target_vel, own_vel);
 *     
 *     hybrid_astar->search(step_size_, start_pt, start_angle_, goal_pt, goal_angle_);
 *     path = hybrid_astar->getPath();
 * }
 */
