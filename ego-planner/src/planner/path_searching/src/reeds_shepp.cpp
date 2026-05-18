#include "path_searching/reeds_shepp.h"
#include <cmath>
#include <algorithm>

double ReedsSheppCurve::normalizeAngle(double angle) const
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

double ReedsSheppCurve::angleDiff(double a1, double a2) const
{
    double diff = a2 - a1;
    return normalizeAngle(diff);
}

ReedsSheppCurve::Path ReedsSheppCurve::shortestPath(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta,
    double step_size)
{
    Path result;
    result.length = 0.0;

    // 坐标变换到局部坐标系
    double dx = goal_x - start_x;
    double dy = goal_y - start_y;
    double local_x = cos(start_theta) * dx + sin(start_theta) * dy;
    double local_y = -sin(start_theta) * dx + cos(start_theta) * dy;
    double local_theta = goal_theta - start_theta;

    // 标准化角度到 [-pi, pi]
    local_theta = normalizeAngle(local_theta);

    // 计算标准化的路径参数
    double rho = sqrt(local_x * local_x + local_y * local_y) / (2.0 * turning_radius_);

    // 检查可达性
    if (rho > 1.0)
    {
        // 使用简化的圆弧-直线-圆弧（CSC）路径
        RSPath rs_path = computeCSC(0, 0, 0, local_x, local_y, local_theta);
        result.length = rs_path.length * turning_radius_;
        discretizePath(rs_path, step_size / turning_radius_, result.points, result.angles);
    }
    else
    {
        // 使用圆弧-圆弧-圆弧（CCC）路径
        RSPath rs_path = computeCCC(0, 0, 0, local_x, local_y, local_theta);
        result.length = rs_path.length * turning_radius_;
        discretizePath(rs_path, step_size / turning_radius_, result.points, result.angles);
    }

    // 变换回全局坐标系
    for (auto& point : result.points)
    {
        double x_global = cos(start_theta) * point.x() - sin(start_theta) * point.y() + start_x;
        double y_global = sin(start_theta) * point.x() + cos(start_theta) * point.y() + start_y;
        double theta_global = normalizeAngle(point.z() + start_theta);
        point = Eigen::Vector3d(x_global, y_global, theta_global);
    }

    for (auto& angle : result.angles)
    {
        angle = normalizeAngle(angle + start_theta);
    }

    return result;
}

ReedsSheppCurve::RSPath ReedsSheppCurve::computeCSC(
    double x0, double y0, double theta0,
    double x1, double y1, double theta1) const
{
    RSPath best_path;
    best_path.length = std::numeric_limits<double>::infinity();

    // 获取起点和终点处的圆心（修复：乘以转向半径）
    // 左圆
    double cx_start_l = x0 - turning_radius_ * sin(theta0);
    double cy_start_l = y0 + turning_radius_ * cos(theta0);
    // 右圆
    double cx_start_r = x0 + turning_radius_ * sin(theta0);
    double cy_start_r = y0 - turning_radius_ * cos(theta0);

    double cx_end_l = x1 - turning_radius_ * sin(theta1);
    double cy_end_l = y1 + turning_radius_ * cos(theta1);
    double cx_end_r = x1 + turning_radius_ * sin(theta1);
    double cy_end_r = y1 - turning_radius_ * cos(theta1);

    // LSL (Left-Straight-Left)
    double dist_ll = sqrt(pow(cx_end_l - cx_start_l, 2) + pow(cy_end_l - cy_start_l, 2));
    if (dist_ll >= 0.0 && dist_ll <= 4.0)
    {  
        double alpha = asin(std::min(1.0, dist_ll / 2.0));
        double straight_len = sqrt(std::max(0.0, dist_ll * dist_ll - 4.0 * sin(alpha) * sin(alpha)));
        double path_length = 2.0 * alpha + straight_len;
        
        if (path_length < best_path.length)
        {
            best_path.length = path_length;
            best_path.actions = {STRAIGHT_LEFT, STRAIGHT_STRAIGHT, STRAIGHT_LEFT};
            best_path.lengths = {alpha, straight_len, alpha};
        }
    }

    // RSR (Right-Straight-Right)
    double dist_rr = sqrt(pow(cx_end_r - cx_start_r, 2) + pow(cy_end_r - cy_start_r, 2));
    if (dist_rr >= 0.0 && dist_rr <= 4.0)
    {
        double alpha = asin(std::min(1.0, dist_rr / 2.0));
        double straight_len = sqrt(std::max(0.0, dist_rr * dist_rr - 4.0 * sin(alpha) * sin(alpha)));
        double path_length = 2.0 * alpha + straight_len;
        
        if (path_length < best_path.length)
        {
            best_path.length = path_length;
            best_path.actions = {STRAIGHT_RIGHT, STRAIGHT_STRAIGHT, STRAIGHT_RIGHT};
            best_path.lengths = {alpha, straight_len, alpha};
        }
    }

    // LSR (Left-Straight-Right)
    double dist_lr = sqrt(pow(cx_end_r - cx_start_l, 2) + pow(cy_end_r - cy_start_l, 2));
    if (dist_lr >= 0.0 && dist_lr <= 4.0)
    {
        double alpha = asin(std::min(1.0, dist_lr / 2.0));
        double straight_len = sqrt(std::max(0.0, dist_lr * dist_lr - 4.0 * cos(alpha) * cos(alpha)));
        double path_length = M_PI / 2.0 - alpha + straight_len + M_PI / 2.0 - alpha;
        
        if (path_length < best_path.length)
        {
            best_path.length = path_length;
            best_path.actions = {STRAIGHT_LEFT, STRAIGHT_STRAIGHT, STRAIGHT_RIGHT};
            best_path.lengths = {M_PI / 2.0 - alpha, straight_len, M_PI / 2.0 - alpha};
        }
    }

    // RSL (Right-Straight-Left)
    double dist_rl = sqrt(pow(cx_end_l - cx_start_r, 2) + pow(cy_end_l - cy_start_r, 2));
    if (dist_rl >= 0.0 && dist_rl <= 4.0)
    {
        double alpha = asin(std::min(1.0, dist_rl / 2.0));
        double straight_len = sqrt(std::max(0.0, dist_rl * dist_rl - 4.0 * cos(alpha) * cos(alpha)));
        double path_length = M_PI / 2.0 - alpha + straight_len + M_PI / 2.0 - alpha;
        
        if (path_length < best_path.length)
        {
            best_path.length = path_length;
            best_path.actions = {STRAIGHT_RIGHT, STRAIGHT_STRAIGHT, STRAIGHT_LEFT};
            best_path.lengths = {M_PI / 2.0 - alpha, straight_len, M_PI / 2.0 - alpha};
        }
    }

    // 如果没有找到有效路径，返回一个退化的路径
    if (std::isinf(best_path.length))
    {
        double dist = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
        best_path.length = dist;
        best_path.actions = {STRAIGHT_STRAIGHT};
        best_path.lengths = {dist};
    }

    return best_path;
}

ReedsSheppCurve::RSPath ReedsSheppCurve::computeCCC(
    double x0, double y0, double theta0,
    double x1, double y1, double theta1) const
{
    RSPath best_path;
    best_path.length = std::numeric_limits<double>::infinity();

    // 获取起点和终点处的圆心（修复：乘以转向半径）
    double cx_start_l = x0 - turning_radius_ * sin(theta0);
    double cy_start_l = y0 + turning_radius_ * cos(theta0);
    double cx_start_r = x0 + turning_radius_ * sin(theta0);
    double cy_start_r = y0 - turning_radius_ * cos(theta0);

    double cx_end_l = x1 - turning_radius_ * sin(theta1);
    double cy_end_l = y1 + turning_radius_ * cos(theta1);
    double cx_end_r = x1 + turning_radius_ * sin(theta1);
    double cy_end_r = y1 - turning_radius_ * cos(theta1);

    // LRL (Left-Right-Left) 路径
    {
        double dist_lr = sqrt(pow(cx_end_r - cx_start_l, 2) + pow(cy_end_r - cy_start_l, 2));
        if (dist_lr >= 0.0 && dist_lr <= 4.0)
        {
            double alpha = acos(std::min(1.0, dist_lr / 2.0));
            double path_length = M_PI - 2.0 * alpha + 2.0 * (M_PI - alpha);
            
            if (path_length < best_path.length && path_length < 2.0 * M_PI)
            {
                best_path.length = path_length;
                best_path.actions = {STRAIGHT_LEFT, STRAIGHT_RIGHT, STRAIGHT_LEFT};
                best_path.lengths = {alpha, 2.0 * (M_PI - alpha), alpha};
            }
        }
    }

    // RLR (Right-Left-Right) 路径
    {
        double dist_rl = sqrt(pow(cx_end_l - cx_start_r, 2) + pow(cy_end_l - cy_start_r, 2));
        if (dist_rl >= 0.0 && dist_rl <= 4.0)
        {
            double alpha = acos(std::min(1.0, dist_rl / 2.0));
            double path_length = M_PI - 2.0 * alpha + 2.0 * (M_PI - alpha);
            
            if (path_length < best_path.length && path_length < 2.0 * M_PI)
            {
                best_path.length = path_length;
                best_path.actions = {STRAIGHT_RIGHT, STRAIGHT_LEFT, STRAIGHT_RIGHT};
                best_path.lengths = {alpha, 2.0 * (M_PI - alpha), alpha};
            }
        }
    }

    // LLL (Left-Left-Left) 路径 - 三个左转弯
    {
        double dist_ll = sqrt(pow(cx_end_l - cx_start_l, 2) + pow(cy_end_l - cy_start_l, 2));
        if (dist_ll >= 0.0 && dist_ll <= 4.0)
        {
            double alpha = asin(std::min(1.0, dist_ll / 2.0));
            double path_length = 2.0 * M_PI - 2.0 * alpha;
            
            if (path_length < best_path.length && path_length < 2.0 * M_PI)
            {
                best_path.length = path_length;
                best_path.actions = {STRAIGHT_LEFT, STRAIGHT_LEFT, STRAIGHT_LEFT};
                best_path.lengths = {M_PI - alpha, 2.0 * alpha, M_PI - alpha};
            }
        }
    }

    // RRR (Right-Right-Right) 路径 - 三个右转弯
    {
        double dist_rr = sqrt(pow(cx_end_r - cx_start_r, 2) + pow(cy_end_r - cy_start_r, 2));
        if (dist_rr >= 0.0 && dist_rr <= 4.0)
        {
            double alpha = asin(std::min(1.0, dist_rr / 2.0));
            double path_length = 2.0 * M_PI - 2.0 * alpha;
            
            if (path_length < best_path.length && path_length < 2.0 * M_PI)
            {
                best_path.length = path_length;
                best_path.actions = {STRAIGHT_RIGHT, STRAIGHT_RIGHT, STRAIGHT_RIGHT};
                best_path.lengths = {M_PI - alpha, 2.0 * alpha, M_PI - alpha};
            }
        }
    }

    // 如果没有找到有效的 CCC 路径，回退到 CSC
    if (std::isinf(best_path.length))
    {
        return computeCSC(x0, y0, theta0, x1, y1, theta1);
    }

    return best_path;
}

void ReedsSheppCurve::discretizePath(const RSPath& rs_path, double step_size,
                                      std::vector<Eigen::Vector3d>& points,
                                      std::vector<double>& angles) const
{
    points.clear();
    angles.clear();

    double current_x = 0.0, current_y = 0.0, current_theta = 0.0;
    points.push_back(Eigen::Vector3d(current_x, current_y, current_theta));
    angles.push_back(current_theta);

    // 遍历每个路径段
    for (size_t i = 0; i < rs_path.actions.size(); ++i)
    {
        double segment_length = rs_path.lengths[i];
        ActionType action = rs_path.actions[i];

        // 计算实际曲率（修复：考虑转向半径）
        double curvature = 0.0;
        if (action == STRAIGHT_LEFT)
            curvature = 1.0 / turning_radius_;
        else if (action == STRAIGHT_RIGHT)
            curvature = -1.0 / turning_radius_;
        // STRAIGHT_STRAIGHT: curvature = 0

        // 沿着段离散化
        double accumulated_dist = 0.0;
        while (accumulated_dist < segment_length - 1e-6)
        {
            double next_step = std::min(step_size, segment_length - accumulated_dist);

            // 使用欧拉方法进行运动学积分（修复：正确计算角度增量）
            double theta_delta = curvature * next_step * turning_radius_;
            double mid_theta = current_theta + theta_delta / 2.0;
            
            current_x += cos(mid_theta) * next_step;
            current_y += sin(mid_theta) * next_step;
            current_theta += theta_delta;

            // 标准化角度
            while (current_theta > M_PI)
                current_theta -= 2 * M_PI;
            while (current_theta < -M_PI)
                current_theta += 2 * M_PI;

            accumulated_dist += next_step;

            // 避免重复添加末点
            if (accumulated_dist < segment_length - 1e-6 || i < rs_path.actions.size() - 1)
            {
                points.push_back(Eigen::Vector3d(current_x, current_y, current_theta));
                angles.push_back(current_theta);
            }
        }
    }

    // 确保末点被添加
    if (points.empty() || 
        (points.back() - Eigen::Vector3d(current_x, current_y, current_theta)).norm() > 1e-6)
    {
        points.push_back(Eigen::Vector3d(current_x, current_y, current_theta));
        angles.push_back(current_theta);
    }
}

double ReedsSheppCurve::distance(double x1, double y1, double theta1,
                                  double x2, double y2, double theta2) const
{
    // 欧氏距离作为启发函数的下界
    double eucl_dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

    // 考虑方向差异
    double angle_diff = normalizeAngle(theta2 - theta1);
    double angle_penalty = std::abs(angle_diff) * turning_radius_ * 0.1;

    return eucl_dist + angle_penalty;
}
