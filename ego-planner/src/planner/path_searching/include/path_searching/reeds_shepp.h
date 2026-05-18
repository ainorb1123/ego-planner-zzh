#ifndef _REEDS_SHEPP_H_
#define _REEDS_SHEPP_H_

#include <Eigen/Eigen>
#include <vector>
#include <cmath>

/**
 * @brief Reeds-Shepp 曲线类，用于计算具有转向半径约束的最优路径
 * 支持前进和后退运动
 */
class ReedsSheppCurve
{
public:
    struct Path
    {
        double length;  // 总路径长度
        std::vector<Eigen::Vector3d> points;  // 路径点集
        std::vector<double> angles;           // 各点的方向角
    };

    explicit ReedsSheppCurve(double turning_radius = 1.0)
        : turning_radius_(turning_radius) {}

    /**
     * @brief 计算从起始配置到目标配置的 Reeds-Shepp 路径
     * @param start_x, start_y, start_theta 起始位置和方向
     * @param goal_x, goal_y, goal_theta 目标位置和方向
     * @param step_size 路径离散化步长
     * @return 路径结构体，包含路径长度和点集
     */
    Path shortestPath(double start_x, double start_y, double start_theta,
                      double goal_x, double goal_y, double goal_theta,
                      double step_size = 0.1);

    /**
     * @brief 计算两点间的 Reeds-Shepp 距离（启发函数用）
     */
    double distance(double x1, double y1, double theta1,
                    double x2, double y2, double theta2) const;

    void setTurningRadius(double radius) { turning_radius_ = radius; }
    double getTurningRadius() const { return turning_radius_; }

private:
    double turning_radius_;

    // Reeds-Shepp 基本动作类型
    enum ActionType
    {
        STRAIGHT_LEFT,      // 直线左转
        STRAIGHT_RIGHT,     // 直线右转
        STRAIGHT_STRAIGHT,  // 直线直行
    };

    struct RSPath
    {
        double length;
        std::vector<ActionType> actions;
        std::vector<double> radii;
        std::vector<double> lengths;
    };

    // 计算各种基本 Reeds-Shepp 曲线
    RSPath computeCSC(double x0, double y0, double theta0,
                      double x1, double y1, double theta1) const;

    RSPath computeCCC(double x0, double y0, double theta0,
                      double x1, double y1, double theta1) const;

    // 辅助计算函数
    double normalizeAngle(double angle) const;
    double angleDiff(double a1, double a2) const;

    // 离散化路径
    void discretizePath(const RSPath& rs_path, double step_size,
                        std::vector<Eigen::Vector3d>& points,
                        std::vector<double>& angles) const;
};

#endif  // _REEDS_SHEPP_H_
