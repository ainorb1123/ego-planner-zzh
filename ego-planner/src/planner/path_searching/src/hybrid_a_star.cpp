#include "path_searching/hybrid_a_star.h"
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <limits>

HybridAStar::HybridAStar(double angle_resolution, double turning_radius)
    : angle_resolution_(angle_resolution),
      turning_radius_(turning_radius),
      reed_shepp_curve_(turning_radius),
      max_steering_angle_(0.5),
      velocity_(1.0),
      step_size_(0.1),
      inv_step_size_(10.0)
{
    ROS_INFO("Bidirectional Hybrid A* initialized with angle_resolution=%.3f, turning_radius=%.3f",
             angle_resolution, turning_radius);
}

void HybridAStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size)
{
    grid_map_ = occ_map;
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;
}

std::string HybridAStar::stateKey(const Eigen::Vector3d& pose) const
{
    // 创建离散化的状态键用于哈希表查�?
    int x_idx = static_cast<int>(std::floor(pose.x() / step_size_));
    int y_idx = static_cast<int>(std::floor(pose.y() / step_size_));
    int theta_idx = static_cast<int>(std::floor(pose.z() / angle_resolution_));

    return std::to_string(x_idx) + "_" + std::to_string(y_idx) + "_" +
           std::to_string(theta_idx);
}

std::shared_ptr<HybridNode> HybridAStar::getOrCreateNode(const Eigen::Vector3d& pose, HybridNode::search_direction direction)
{
    std::string key = stateKey(pose) + "_" + (direction == HybridNode::FORWARD ? "F" : "B");
    auto& node_map = (direction == HybridNode::FORWARD) ? closed_nodes_forward_ : closed_nodes_backward_;

    auto it = node_map.find(key);
    if (it != node_map.end())
    {
        return it->second;
    }

    auto node = std::make_shared<HybridNode>();
    node->pose = pose;
    node->grid_idx = CENTER_IDX_;
    node->rounds = rounds_;
    node->direction = direction;

    // 计算网格索引（检查有效性）
    Eigen::Vector2d pos_2d(pose.x(), pose.y());
    if (!Coord2Index(pos_2d, node->grid_idx))
    {
        // 保持默认的中心索�?
        ROS_DEBUG_STREAM("Position (" << pos_2d.x() << ", " << pos_2d.y() << ") out of grid bounds");
    }

    node_map[key] = node;
    return node;
}

std::vector<double> HybridAStar::getMotionPrimitives()
{
    std::vector<double> primitives;

    // 生成离散的转向角�?
    int num_angles = static_cast<int>(2.0 * max_steering_angle_ / 0.1) + 1;

    for (int i = -num_angles / 2; i <= num_angles / 2; ++i)
    {
        double steering_angle = i * 0.1;  // 0.1 rad 间隔
        if (std::abs(steering_angle) <= max_steering_angle_)
        {
            primitives.push_back(steering_angle);
        }
    }

    return primitives;
}

std::vector<Eigen::Vector3d> HybridAStar::generateMotion(const Eigen::Vector3d& start,
                                                          double steering_angle,
                                                          double step_dist)
{
    std::vector<Eigen::Vector3d> trajectory;
    trajectory.push_back(start);

    double x = start.x();
    double y = start.y();
    double theta = start.z();

    // 使用简化的运动学模型（自行车模型）
    // dx = v * cos(theta) * dt
    // dy = v * sin(theta) * dt
    // dtheta = (v/L) * tan(steering_angle) * dt
    // 其中 L 是转向半�?

    double dt = step_size_ / velocity_;  // 时间步长
    // 修复Bug #7: 使用ceil处理边界情况
    int num_steps = static_cast<int>(std::ceil(step_dist / step_size_));
    
    if (num_steps <= 0) {
        return trajectory;  // 返回仅包含起点的轨迹
    }

    for (int i = 0; i < static_cast<int>(num_steps); ++i)
    {
        // 自行车模�?
        double dx = velocity_ * cos(theta) * dt;
        double dy = velocity_ * sin(theta) * dt;
        
        // 处理转向角为0时的特殊情况
        double dtheta = 0.0;
        if (std::abs(steering_angle) > 1e-6)
        {
            dtheta = (velocity_ / turning_radius_) * std::tan(steering_angle) * dt;
        }

        x += dx;
        y += dy;
        theta += dtheta;

        // 标准化角�?
        while (theta > M_PI)
            theta -= 2 * M_PI;
        while (theta < -M_PI)
            theta += 2 * M_PI;

        trajectory.push_back(Eigen::Vector3d(x, y, theta));
    }

    return trajectory;
}

std::vector<Eigen::Vector3d> HybridAStar::generateReverseMotion(const Eigen::Vector3d& start,
                                                                double steering_angle,
                                                                double step_dist)
{
    std::vector<Eigen::Vector3d> trajectory;
    trajectory.push_back(start);

    double x = start.x();
    double y = start.y();
    double theta = start.z();

    // 逆运动学：速度方向相反，转向角取反
    // dx = -v * cos(theta) * dt  (反向移动)
    // dy = -v * sin(theta) * dt  (反向移动)
    // dtheta = -(v/L) * tan(-steering_angle) * dt = (v/L) * tan(steering_angle) * dt
    // 其中 L 是转向半�?

    double dt = step_size_ / velocity_;  // 时间步长
    int num_steps = static_cast<int>(std::ceil(step_dist / step_size_));

    if (num_steps <= 0) {
        return trajectory;  // 返回仅包含起点的轨迹
    }

    for (int i = 0; i < num_steps; ++i)
    {
        // 逆运动学：反向移�?
        double dx = -velocity_ * cos(theta) * dt;  // 负号表示反向
        double dy = -velocity_ * sin(theta) * dt;  // 负号表示反向

        // 处理转向角为0时的特殊情况（转向角符号已取反）
        double dtheta = 0.0;
        if (std::abs(steering_angle) > 1e-6)
        {
            // 注意：逆运动学中转向角符号不变，但由于反向移动，角度变化方向相�?
            dtheta = -(velocity_ / turning_radius_) * std::tan(steering_angle) * dt;
        }

        x += dx;
        y += dy;
        theta += dtheta;

        // 标准化角�?
        while (theta > M_PI)
            theta -= 2 * M_PI;
        while (theta < -M_PI)
            theta += 2 * M_PI;

        trajectory.push_back(Eigen::Vector3d(x, y, theta));
    }

    return trajectory;
}

bool HybridAStar::checkTrajectoryCollision(const std::vector<Eigen::Vector3d>& trajectory)
{
    for (const auto& pose : trajectory)
    {
        if (checkOccupancy(Eigen::Vector2d(pose.x(), pose.y())))
        {
            return true;
        }
    }
    return false;
}

bool HybridAStar::adjustEndpointOutOfObstacle(Eigen::Vector2d& point,
                                              const Eigen::Vector2d& away_from,
                                              const char* point_name)
{
    Eigen::Vector2i point_idx;
    if (!Coord2Index(point, point_idx))
    {
        ROS_ERROR("Hybrid A*: %s point is out of search grid before adjustment: (%.3f, %.3f)",
                  point_name, point.x(), point.y());
        return false;
    }

    if (!checkOccupancy(point))
    {
        return true;
    }

    Eigen::Vector2d direction = point - away_from;
    if (direction.norm() < 1e-6)
    {
        direction = Eigen::Vector2d(1.0, 0.0);
    }
    else
    {
        direction.normalize();
    }

    const Eigen::Vector2d original = point;
    const int max_adjust_steps = 50;
    for (int i = 0; i < max_adjust_steps; ++i)
    {
        point += direction * step_size_;
        if (!Coord2Index(point, point_idx))
        {
            ROS_ERROR("Hybrid A*: %s point adjustment left search grid: original=(%.3f, %.3f), current=(%.3f, %.3f)",
                      point_name, original.x(), original.y(), point.x(), point.y());
            return false;
        }

        if (!checkOccupancy(point))
        {
            ROS_WARN("Hybrid A*: adjusted %s point out of inflated obstacle: (%.3f, %.3f) -> (%.3f, %.3f)",
                     point_name, original.x(), original.y(), point.x(), point.y());
            return true;
        }
    }

    ROS_ERROR("Hybrid A*: failed to adjust %s point out of inflated obstacle after %.2f m: original=(%.3f, %.3f)",
              point_name, max_adjust_steps * step_size_, original.x(), original.y());
    return false;
}

bool HybridAStar::hasCollisionFreeMotion(const Eigen::Vector2d& point,
                                         double theta,
                                         bool reverse)
{
    Eigen::Vector3d pose(point.x(), point.y(), theta);
    auto primitives = getMotionPrimitives();
    for (double steering_angle : primitives)
    {
        std::vector<Eigen::Vector3d> trajectory = reverse
            ? generateReverseMotion(pose, steering_angle, step_size_ * 2)
            : generateMotion(pose, steering_angle, step_size_ * 2);
        if (!trajectory.empty() && !checkTrajectoryCollision(trajectory))
        {
            return true;
        }
    }
    return false;
}

bool HybridAStar::adjustEndpointForSearch(Eigen::Vector2d& point,
                                          double theta,
                                          const Eigen::Vector2d& away_from,
                                          const char* point_name,
                                          bool reverse)
{
    if (!adjustEndpointOutOfObstacle(point, away_from, point_name))
    {
        return false;
    }

    if (hasCollisionFreeMotion(point, theta, reverse))
    {
        return true;
    }

    Eigen::Vector2d direction = point - away_from;
    if (direction.norm() < 1e-6)
    {
        direction = Eigen::Vector2d(1.0, 0.0);
    }
    else
    {
        direction.normalize();
    }

    const Eigen::Vector2d original = point;
    Eigen::Vector2i point_idx;
    const int max_adjust_steps = 50;
    for (int i = 0; i < max_adjust_steps; ++i)
    {
        point += direction * step_size_;
        if (!Coord2Index(point, point_idx))
        {
            ROS_ERROR("Hybrid A*: %s point search adjustment left search grid: original=(%.3f, %.3f), current=(%.3f, %.3f)",
                      point_name, original.x(), original.y(), point.x(), point.y());
            return false;
        }

        if (!checkOccupancy(point) && hasCollisionFreeMotion(point, theta, reverse))
        {
            ROS_WARN("Hybrid A*: adjusted %s point to allow first expansion: (%.3f, %.3f) -> (%.3f, %.3f)",
                     point_name, original.x(), original.y(), point.x(), point.y());
            return true;
        }
    }

    ROS_ERROR("Hybrid A*: failed to make %s point expandable after %.2f m: original=(%.3f, %.3f)",
              point_name, max_adjust_steps * step_size_, original.x(), original.y());
    return false;
}

bool HybridAStar::checkNodesMeet(const std::shared_ptr<HybridNode>& forward_node,
                                 const std::shared_ptr<HybridNode>& backward_node) const
{
    // 检查位置距�?
    double pos_dist = (forward_node->pose.head(2) - backward_node->pose.head(2)).norm();
    if (pos_dist > step_size_ * 2.0)  // 位置距离阈�?
        return false;

    // 检查角度差�?
    double theta_diff = std::abs(forward_node->pose.z() - backward_node->pose.z());
    if (theta_diff > M_PI)
        theta_diff = 2 * M_PI - theta_diff;

    if (theta_diff > angle_resolution_ * 2.0)  // 角度差异阈�?
        return false;

    return true;
}

void HybridAStar::reconstructBidirectionalPath(const std::shared_ptr<HybridNode>& forward_meet,
                                               const std::shared_ptr<HybridNode>& backward_meet)
{
    hybrid_path_.clear();

    // 重建正向路径（从起点到相遇点�?
    std::vector<std::shared_ptr<HybridNode>> forward_path = retrievePath(forward_meet);

    // 重建反向路径（从相遇点到终点），然后反转
    std::vector<std::shared_ptr<HybridNode>> backward_path = retrievePath(backward_meet);
    std::reverse(backward_path.begin(), backward_path.end());

    // 合并路径：正向路�?+ 反向路径（去掉重复的相遇点）
    hybrid_path_.insert(hybrid_path_.end(), forward_path.begin(), forward_path.end());
    if (!backward_path.empty()) {
        hybrid_path_.insert(hybrid_path_.end(), backward_path.begin() + 1, backward_path.end());
    }
}

bool HybridAStar::tryReedSheppConnection(const std::shared_ptr<HybridNode>& current,
                                         const Eigen::Vector3d& goal_pose,
                                         std::vector<Eigen::Vector3d>& connection_path)
{
    // 计算 Reeds-Shepp 路径
    ReedsSheppCurve::Path rs_path = reed_shepp_curve_.shortestPath(
        current->pose.x(), current->pose.y(), current->pose.z(),
        goal_pose.x(), goal_pose.y(), goal_pose.z(),
        step_size_);

    // 检查碰�?
    if (!checkTrajectoryCollision(rs_path.points))
    {
        connection_path = rs_path.points;
        return true;
    }

    return false;
}

double HybridAStar::calculateThreatCost(const Eigen::Vector2d& node_pos)
{
    if (!has_target_ship_ || colregs_mode_ == 0)
        return 0.0;

    const double predict_time = std::max(0.5, std::min(tcpa_, 8.0));
    const Eigen::Vector2d ts_pred = ts_pos_ + ts_vel_ * predict_time;
    const Eigen::Vector2d os_pred = start_pos_ + os_vel_ * predict_time;
    const Eigen::Vector2d encounter_center = 0.5 * (ts_pred + os_pred);

    Eigen::Vector2d route_vec = ts_pred - start_pos_;
    if (route_vec.norm() < 1e-3)
    {
        route_vec = os_vel_.norm() > 1e-3 ? os_vel_ : Eigen::Vector2d(1.0, 0.0);
    }
    route_vec.normalize();

    const Eigen::Vector2d rel_to_route = node_pos - start_pos_;
    const double along = rel_to_route.dot(route_vec);
    const double encounter_along = (encounter_center - start_pos_).dot(route_vec);
    const double lateral = route_vec.x() * rel_to_route.y() - route_vec.y() * rel_to_route.x();

    const double desired_clearance = std::max(safe_dcpa_, 5.0);
    const double corridor_half_width = desired_clearance;
    const double active_before = std::max(8.0, 2.0 * desired_clearance);
    const double active_after = std::max(4.0, desired_clearance);
    const bool in_encounter_window =
        along > encounter_along - active_before && along < encounter_along + active_after;

    double cost = 0.0;
    const Eigen::Vector2d rel_to_ts_pred = node_pos - ts_pred;
    const double ts_along = rel_to_ts_pred.dot(route_vec);
    const double ts_lateral = route_vec.x() * rel_to_ts_pred.y() - route_vec.y() * rel_to_ts_pred.x();
    const double longitudinal_clearance = std::max(12.0, safe_dcpa_ * 2.8);
    const double lateral_clearance = std::max(7.0, safe_dcpa_ * 1.4);
    const double ellipse_value =
        (ts_along * ts_along) / (longitudinal_clearance * longitudinal_clearance) +
        (ts_lateral * ts_lateral) / (lateral_clearance * lateral_clearance);
    const double normalized_boundary_dist = std::sqrt(std::max(0.0, ellipse_value));
    const double boundary_margin = 0.45;
    if (normalized_boundary_dist < 1.0 + boundary_margin)
    {
        const double penetration = std::max(0.0, 1.0 - normalized_boundary_dist);
        const double near_boundary = std::max(0.0, 1.0 + boundary_margin - normalized_boundary_dist) /
                                     boundary_margin;
        cost += 3200.0 * penetration * penetration;
        cost += 1200.0 * near_boundary * near_boundary;
    }

    if (colregs_mode_ == 1)
    {
        const double head_on_lookahead = std::max(encounter_along + active_after,
                                                 std::max(12.0, 2.5 * desired_clearance));
        const bool in_head_on_approach = along > 0.0 && along < head_on_lookahead;
        if (in_head_on_approach)
        {
            const double right_lateral = -lateral;
            if (right_lateral < desired_clearance)
            {
                const double miss = desired_clearance - right_lateral;
                cost += 1800.0 * (miss / desired_clearance) * (miss / desired_clearance);
            }

            if (std::abs(lateral) < corridor_half_width * 1.2)
            {
                const double ratio = (corridor_half_width * 1.2 - std::abs(lateral)) /
                                     (corridor_half_width * 1.2);
                cost += 1200.0 * ratio * ratio;
            }

            if (lateral > 0.0)
            {
                const double wrong_side = std::min(lateral / desired_clearance, 2.0);
                cost += 800.0 * wrong_side * wrong_side;
            }
        }
    }

    if (!in_encounter_window)
    {
        return cost;
    }

    double preferred_side = 0.0;
    if (colregs_mode_ == 1 || colregs_mode_ == 2)
    {
        preferred_side = -1.0;
    }
    else if (colregs_mode_ == 4)
    {
        preferred_side = 1.0;
    }
    else
    {
        return cost;
    }

    const double side_value = preferred_side * lateral;
    if (side_value < desired_clearance)
    {
        const double miss = desired_clearance - side_value;
        cost += 1000.0 * (miss / desired_clearance) * (miss / desired_clearance);
    }

    if (std::abs(lateral) < corridor_half_width)
    {
        const double ratio = (corridor_half_width - std::abs(lateral)) / corridor_half_width;
        cost += 500.0 * ratio * ratio;
    }

    return cost;
}

double HybridAStar::calculateInflatedMapProximityCost(const Eigen::Vector2d& node_pos) const
{
    if (!grid_map_)
        return 0.0;

    const double resolution = std::max(0.05, grid_map_->getResolution());
    const double influence_dist = std::max(2.5, safe_dcpa_ * 0.6);
    const int max_step = static_cast<int>(std::ceil(influence_dist / resolution));

    if (grid_map_->getInflateOccupancy2d(node_pos) > 0)
    {
        return 6000.0;
    }

    double min_dist = influence_dist + resolution;
    for (int dx = -max_step; dx <= max_step; ++dx)
    {
        for (int dy = -max_step; dy <= max_step; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;

            const double dist = resolution * std::sqrt(static_cast<double>(dx * dx + dy * dy));
            if (dist >= min_dist || dist > influence_dist)
                continue;

            Eigen::Vector2d sample = node_pos + Eigen::Vector2d(dx * resolution, dy * resolution);
            if (grid_map_->getInflateOccupancy2d(sample) > 0)
            {
                min_dist = dist;
            }
        }
    }

    if (min_dist > influence_dist)
        return 0.0;

    const double ratio = (influence_dist - min_dist) / influence_dist;
    return 900.0 * ratio * ratio;
}
double HybridAStar::calculateSteeringSmoothnessCost(const std::shared_ptr<HybridNode>& current,
                                                    double steering_angle) const
{
    const double steering_abs_weight = 0.25;
    const double steering_change_weight = 1.2;
    const double steering_reverse_weight = 0.8;

    double cost = steering_abs_weight * std::abs(steering_angle);

    if (current && current->cameFrom)
    {
        const double steering_change = std::abs(steering_angle - current->control_input);
        cost += steering_change_weight * steering_change;

        if (std::abs(steering_angle) > 0.05 &&
            std::abs(current->control_input) > 0.05 &&
            steering_angle * current->control_input < 0.0)
        {
            cost += steering_reverse_weight;
        }
    }

    return cost;
}

bool HybridAStar::tryLateralBypass(const Eigen::Vector3d& start_pose,
                                   const Eigen::Vector3d& goal_pose)
{
    Eigen::Vector2d start = start_pose.head<2>();
    Eigen::Vector2d goal = goal_pose.head<2>();
    Eigen::Vector2d forward = goal - start;
    double length = forward.norm();
    if (length < 1e-3)
    {
        return false;
    }

    forward /= length;
    Eigen::Vector2d left_normal(-forward.y(), forward.x());

    double preferred_side = -1.0;
    if (colregs_mode_ == 4)
    {
        preferred_side = 1.0;
    }

    const double predict_time = has_target_ship_ ? std::max(0.5, std::min(tcpa_, 8.0)) : 2.0;
    const Eigen::Vector2d ts_pred = has_target_ship_ ? ts_pos_ + ts_vel_ * predict_time : goal;
    const Eigen::Vector2d os_pred = has_target_ship_ ? start_pos_ + os_vel_ * predict_time : start;
    Eigen::Vector2d encounter_center = has_target_ship_ ? 0.5 * (ts_pred + os_pred) : 0.5 * (start + goal);

    double encounter_along = (encounter_center - start).dot(forward);
    encounter_along = std::max(0.3 * length, std::min(0.85 * length, encounter_along));
    Eigen::Vector2d entry = start + forward * std::max(0.15 * length, encounter_along - 6.0);
    Eigen::Vector2d exit = start + forward * std::min(length, encounter_along + 6.0);

    std::vector<double> side_order = {preferred_side, -preferred_side};
    const double min_offset = std::max(safe_dcpa_, 5.0);
    const std::vector<double> offset_extra = {0.0, 1.0, 2.0, 3.0, 4.0};
    const double sample_step = std::max(0.05, step_size_);

    for (double side : side_order)
    {
        for (double extra : offset_extra)
        {
            double offset = min_offset + extra;
            Eigen::Vector2d shift = side * offset * left_normal;
            std::vector<Eigen::Vector2d> anchors;
            anchors.push_back(start);
            anchors.push_back(entry + shift);
            anchors.push_back(exit + shift);
            anchors.push_back(goal);

            std::vector<std::shared_ptr<HybridNode>> candidate;
            bool collision = false;

            for (size_t i = 0; i + 1 < anchors.size() && !collision; ++i)
            {
                Eigen::Vector2d segment = anchors[i + 1] - anchors[i];
                double segment_len = segment.norm();
                int samples = std::max(1, static_cast<int>(std::ceil(segment_len / sample_step)));

                for (int j = 0; j <= samples; ++j)
                {
                    if (i > 0 && j == 0)
                    {
                        continue;
                    }

                    double ratio = static_cast<double>(j) / static_cast<double>(samples);
                    Eigen::Vector2d pos = anchors[i] + ratio * segment;
                    if (checkOccupancy(pos))
                    {
                        collision = true;
                        break;
                    }

                    double theta = segment_len > 1e-6 ? std::atan2(segment.y(), segment.x()) : start_pose.z();
                    auto node = std::make_shared<HybridNode>();
                    node->pose = Eigen::Vector3d(pos.x(), pos.y(), theta);
                    candidate.push_back(node);
                }
            }

            if (!collision && candidate.size() >= 2)
            {
                hybrid_path_ = candidate;
                ROS_WARN("Hybrid A*: using predictive COLREGs bypass, side=%.0f, offset=%.2f, points=%zu",
                         side, offset, hybrid_path_.size());
                return true;
            }
        }
    }

    return false;
}

std::vector<std::shared_ptr<HybridNode>> HybridAStar::retrievePath(std::shared_ptr<HybridNode> current)
{
    std::vector<std::shared_ptr<HybridNode>> path;
    path.push_back(current);

    while (current->cameFrom != nullptr)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

bool HybridAStar::search(double step_size,
                         const Eigen::Vector2d& start_pt, double start_theta,
                         const Eigen::Vector2d& goal_pt, double goal_theta)
{
    // 验证输入参数
    if (step_size <= 0.0)
    {
        ROS_ERROR("Invalid step_size: %f", step_size);
        return false;
    }

    if (!grid_map_)
    {
        ROS_ERROR("Grid map not initialized");
        return false;
    }

    ros::Time time_start = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1.0 / step_size;
    Eigen::Vector2d adjusted_start = start_pt;
    Eigen::Vector2d adjusted_goal = goal_pt;
    center_ = (adjusted_start + adjusted_goal) / 2.0;

    if (!adjustEndpointForSearch(adjusted_start, start_theta, adjusted_goal, "start", false) ||
        !adjustEndpointForSearch(adjusted_goal, goal_theta, adjusted_start, "goal", true))
    {
        return false;
    }

    center_ = (adjusted_start + adjusted_goal) / 2.0;

    Eigen::Vector3d start_pose(adjusted_start.x(), adjusted_start.y(), start_theta);
    Eigen::Vector3d goal_pose(adjusted_goal.x(), adjusted_goal.y(), goal_theta);

    // 清空开闭集
    std::priority_queue<std::shared_ptr<HybridNode>,
                        std::vector<std::shared_ptr<HybridNode>>,
                        HybridNodeComparator> empty;

    openSet_forward_.swap(empty);
    openSet_backward_.swap(empty);
    closed_nodes_forward_.clear();
    closed_nodes_backward_.clear();

    // 创建起始节点（正向）
    auto start_node = getOrCreateNode(start_pose, HybridNode::FORWARD);
    start_node->gScore = 0.0;
    start_node->fScore = reed_shepp_curve_.distance(start_pose.x(), start_pose.y(), start_pose.z(),
                                                    goal_pose.x(), goal_pose.y(), goal_pose.z());
    start_node->state = HybridNode::OPENSET;
    openSet_forward_.push(start_node);

    // 创建目标节点（反向）
    auto goal_node = getOrCreateNode(goal_pose, HybridNode::BACKWARD);
    goal_node->gScore = 0.0;
    goal_node->fScore = reed_shepp_curve_.distance(goal_pose.x(), goal_pose.y(), goal_pose.z(),
                                                   start_pose.x(), start_pose.y(), start_pose.z());
    goal_node->state = HybridNode::OPENSET;
    openSet_backward_.push(goal_node);

    int iter_count = 0;
    const int MAX_ITER = 50000;
    const double TIMEOUT = 0.5;  // 500ms 超时

    while ((!openSet_forward_.empty() || !openSet_backward_.empty()) && iter_count < MAX_ITER)
    {
        ++iter_count;

        // 交替处理正向和反向搜�?
        bool process_forward = (iter_count % 2 == 0);

        if (process_forward && !openSet_forward_.empty())
        {
            // 处理正向搜索
            auto current = openSet_forward_.top();
            openSet_forward_.pop();

            if (current->state == HybridNode::CLOSEDSET)
                continue;

            current->state = HybridNode::CLOSEDSET;

            // 检查是否与反向搜索相遇
            for (const auto& backward_pair : closed_nodes_backward_)
            {
                auto backward_node = backward_pair.second;
                if (backward_node->state == HybridNode::CLOSEDSET &&
                    checkNodesMeet(current, backward_node))
                {
                    // 找到相遇点，重构路径
                    reconstructBidirectionalPath(current, backward_node);
                    ROS_INFO("Bidirectional Hybrid A* found path after %d iterations", iter_count);
                    return true;
                }
            }

            // 尝试所有可能的动作
            auto primitives = getMotionPrimitives();
            for (double steering_angle : primitives)
            {
                // 生成正向轨迹
                std::vector<Eigen::Vector3d> trajectory = generateMotion(current->pose, steering_angle, step_size_ * 2);

                if (trajectory.empty() || checkTrajectoryCollision(trajectory))
                {
                    continue;
                }

                Eigen::Vector3d new_pose = trajectory.back();
                auto new_node = getOrCreateNode(new_pose, HybridNode::FORWARD);

                if (new_node->state == HybridNode::CLOSEDSET)
                {
                    continue;
                }

                // 计算实际轨迹长度
                double edge_cost = 0.0;
                for (size_t i = 1; i < trajectory.size(); ++i)
                {
                    edge_cost += (trajectory[i] - trajectory[i-1]).norm();
                }
                edge_cost = std::max(edge_cost, step_size_);

                Eigen::Vector2d new_pos2d(new_pose.x(), new_pose.y());
                double threat_cost = calculateThreatCost(new_pos2d);
                double map_proximity_cost = calculateInflatedMapProximityCost(new_pos2d);
                double smoothness_cost = calculateSteeringSmoothnessCost(current, steering_angle);
                double tentative_g = current->gScore + edge_cost + threat_cost + map_proximity_cost + smoothness_cost;

                if (new_node->state != HybridNode::OPENSET || tentative_g < new_node->gScore)
                {
                    new_node->cameFrom = current;
                    new_node->gScore = tentative_g;
                    new_node->fScore = tentative_g + reed_shepp_curve_.distance(
                        new_pose.x(), new_pose.y(), new_pose.z(),
                        goal_pose.x(), goal_pose.y(), goal_pose.z());
                    new_node->state = HybridNode::OPENSET;
                    new_node->control_input = steering_angle;
                    openSet_forward_.push(new_node);
                }
            }
        }
        else if (!openSet_backward_.empty())
        {
            // 处理反向搜索（从第一次齐次迭代开始）
            // 注意：反向搜索时本船后退运动，但COLREGS规则基于起始位置(start_pos_)的相对方位判�?
            auto current = openSet_backward_.top();
            openSet_backward_.pop();

            if (current->state == HybridNode::CLOSEDSET)
                continue;

            current->state = HybridNode::CLOSEDSET;

            // 检查是否与正向搜索相遇（双向收敛）
            for (const auto& forward_pair : closed_nodes_forward_)
            {
                auto forward_node = forward_pair.second;
                if (forward_node->state == HybridNode::CLOSEDSET &&
                    checkNodesMeet(forward_node, current))
                {
                    // 找到相遇点，重构路径
                    reconstructBidirectionalPath(forward_node, current);
                    ROS_INFO("Bidirectional Hybrid A* found path after %d iterations", iter_count);
                    return true;
                }
            }

            // 尝试所有可能的动作（反向转向）
            auto primitives = getMotionPrimitives();
            for (double steering_angle : primitives)
            {
                // 生成反向轨迹
                std::vector<Eigen::Vector3d> trajectory = generateReverseMotion(current->pose, steering_angle, step_size_ * 2);

                if (trajectory.empty() || checkTrajectoryCollision(trajectory))
                {
                    continue;
                }

                Eigen::Vector3d new_pose = trajectory.back();
                auto new_node = getOrCreateNode(new_pose, HybridNode::BACKWARD);

                if (new_node->state == HybridNode::CLOSEDSET)
                {
                    continue;
                }

                // 计算实际轨迹长度
                double edge_cost = 0.0;
                for (size_t i = 1; i < trajectory.size(); ++i)
                {
                    edge_cost += (trajectory[i] - trajectory[i-1]).norm();
                }
                edge_cost = std::max(edge_cost, step_size_);

                // 反向搜索也应用海事避碰规�?
                // 关键：尽管本船正向自上而下(反向转向)，但需要遵守基于起始位置的COLREGS规则
                // 每一个反向路点都需要检查其COLREGS威胁
                Eigen::Vector2d new_pos2d(new_pose.x(), new_pose.y());
                double threat_cost = calculateThreatCost(new_pos2d);
                double map_proximity_cost = calculateInflatedMapProximityCost(new_pos2d);
                double smoothness_cost = calculateSteeringSmoothnessCost(current, steering_angle);
                double tentative_g = current->gScore + edge_cost + threat_cost + map_proximity_cost + smoothness_cost;

                if (new_node->state != HybridNode::OPENSET || tentative_g < new_node->gScore)
                {
                    new_node->cameFrom = current;
                    new_node->gScore = tentative_g;
                    new_node->fScore = tentative_g + reed_shepp_curve_.distance(
                        new_pose.x(), new_pose.y(), new_pose.z(),
                        start_pose.x(), start_pose.y(), start_pose.z());
                    new_node->state = HybridNode::OPENSET;
                    new_node->control_input = steering_angle;
                    openSet_backward_.push(new_node);
                }
            }
        }

        // 超时检�?
        if (iter_count % 500 == 0)
        {
            if ((ros::Time::now() - time_start).toSec() > TIMEOUT)
            {
                ROS_WARN("Bidirectional Hybrid A* timeout after %d iterations", iter_count);
                return false;
            }
        }
    }

    if (tryLateralBypass(start_pose, goal_pose))
    {
        return true;
    }
    ROS_WARN("Bidirectional Hybrid A* failed: max iterations reached or open sets empty");
    return false;
}

std::vector<Eigen::Vector3d> HybridAStar::getPath() const
{
    std::vector<Eigen::Vector3d> path;
    for (const auto& node : hybrid_path_)
    {
        path.push_back(node->pose);
    }
    return path;
}

std::vector<Eigen::Vector3d> HybridAStar::getPositionPath() const
{
    std::vector<Eigen::Vector3d> path;
    for (const auto& node : hybrid_path_)
    {
        path.push_back(Eigen::Vector3d(node->pose.x(), node->pose.y(), 0.0));
    }
    return path;
}

Eigen::Vector2d HybridAStar::Index2Coord(const Eigen::Vector2i& index) const
{
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
}

bool HybridAStar::Coord2Index(const Eigen::Vector2d& pt, Eigen::Vector2i& idx) const
{
    idx = ((pt - center_) * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + CENTER_IDX_;
    if (idx.x() < 0 || idx.x() >= POOL_SIZE_.x() || idx.y() < 0 || idx.y() >= POOL_SIZE_.y())
    {
        return false;
    }
    return true;
}
