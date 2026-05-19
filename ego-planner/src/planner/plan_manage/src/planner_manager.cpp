#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <thread>

namespace ego_planner
{
    EGOPlannerManager::EGOPlannerManager() : ts_pos_(Eigen::Vector3d::Zero()), ts_vel_(Eigen::Vector3d::Zero()) {}
    EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

    void EGOPlannerManager::updateDynamicObstacle(const std::string& name, const Eigen::Vector3d& pos,
                                                  const Eigen::Vector3d& vel, const ros::Time& stamp)
    {
        for (auto& obs : dynamic_obstacles_)
        {
            if (obs.name == name)
            {
                obs.pos = pos;
                obs.vel = vel;
                obs.stamp = stamp;
                obs.valid = true;
                return;
            }
        }

        DynamicObstacleState obs;
        obs.name = name;
        obs.pos = pos;
        obs.vel = vel;
        obs.stamp = stamp;
        obs.valid = true;
        dynamic_obstacles_.push_back(obs);
    }

    bool EGOPlannerManager::selectActiveObstacle(const Eigen::Vector3d& os_pos, const Eigen::Vector3d& os_vel)
    {
        const ros::Time now = ros::Time::now();
        const double stale_timeout = 1.0;

        bool found = false;
        double best_score = std::numeric_limits<double>::infinity();
        DynamicObstacleState best_obs;

        for (const auto& obs : dynamic_obstacles_)
        {
            if (!obs.valid)
                continue;
            if (!obs.stamp.isZero() && (now - obs.stamp).toSec() > stale_timeout)
                continue;

            Eigen::Vector2d rel_pos = obs.pos.head<2>() - os_pos.head<2>();
            Eigen::Vector2d rel_vel = os_vel.head<2>() - obs.vel.head<2>();
            const double dist = rel_pos.norm();
            const double rel_speed_sq = rel_vel.squaredNorm();

            double tcpa = 0.0;
            double dcpa = dist;
            if (rel_speed_sq > 0.0025)
            {
                const double dot_product = rel_pos.dot(rel_vel);
                tcpa = dot_product / rel_speed_sq;
                const double squared_dcpa = rel_pos.squaredNorm() - dot_product * dot_product / rel_speed_sq;
                dcpa = std::sqrt(std::max(0.0, squared_dcpa));
            }

            double score = dist;
            if (tcpa > 0.0)
                score = dcpa + 0.1 * std::max(0.0, tcpa) + 0.02 * dist;

            if (score < best_score)
            {
                best_score = score;
                best_obs = obs;
                found = true;
            }
        }

        has_active_obstacle_ = found;
        if (!found)
        {
            active_obstacle_name_ = "none";
            current_scenario_ = NONE;
            head_on_maneuver_lock_ = false;
            head_on_lock_obstacle_ = "none";
            head_on_lock_course_ = Eigen::Vector2d(1.0, 0.0);
            last_dcpa_ = 0.0;
            last_tcpa_ = 0.0;
            return false;
        }

        active_obstacle_name_ = best_obs.name;
        ts_pos_ = best_obs.pos;
        ts_vel_ = best_obs.vel;
        return true;
    }

    void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
    {
        /* 璇诲彇鍩虹鍙傛暟 */
        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
        nh.param("manager/colregs_dist_threshold", colregs_dist_threshold_, 25.0);
        nh.param("manager/safe_dcpa", safe_dcpa_, 3.5);

        local_data_.traj_id_ = 0;
        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam(nh);
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        
        // 鍒濆鍖栧弻鍚戞贩鍚圓* 骞舵寕杞藉埌浼樺寲鍣ㄤ笅
        a_star_.reset(new HybridAStar(0.1745, 1.0)); // 鍙屽悜娣峰悎A*锛氳浆寮搴?0搴︼紝杞悜鍗婂緞1.0m
        a_star_->setVelocity(1.0);
        a_star_->setMaxSteeringAngle(0.5);
        a_star_->initGridMap(grid_map_, Eigen::Vector2i(100, 100));
        bspline_optimizer_rebound_->a_star_ = a_star_; 

        visualization_ = vis;
    }

    // !SECTION

    // SECTION rebond replanning

bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                          Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                          Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
    {
        static int count = 0;
        std::cout << "\n[rebo replan]: ------------------------------------- " << count++ << std::endl;
        
        if (selectActiveObstacle(start_pt, start_vel))
        {
            checkCOLREGs(start_pt, start_vel, ts_pos_, ts_vel_);
        }
        else
        {
            current_scenario_ = NONE;
            head_on_maneuver_lock_ = false;
            head_on_lock_obstacle_ = "none";
            head_on_lock_course_ = Eigen::Vector2d(1.0, 0.0);
        }

        // 2. 灏嗛伩纰板眬闈俊鎭敞鍏?A*
        if (a_star_) {
            a_star_->setTargetShipInfo(ts_pos_.head<2>(), ts_vel_.head<2>(), start_vel.head<2>());
            a_star_->setColregsMode((int)current_scenario_);
            a_star_->setStartPos(start_pt.head<2>());
            // 浼犲叆DCPA/TCPA瀹夊叏鏉冮噸鍙傛暟
            a_star_->setSafetyParams(last_dcpa_, last_tcpa_);
            a_star_->setSafeDcpa(safe_dcpa_);
        }

        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            continuous_failures_count_ = 0; // 绂荤洰鏍囪繎涓嶇畻澶辫触
            return true; 
        }

        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt; // 瀹氫箟璁℃椂鍙橀噺

        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
        
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        static bool flag_first_call = true, flag_force_polynomial = false;
        bool flag_regenerate = false;

        /*** STEP 1: 璺緞鍒濆鍖?***/
        do {
            point_set.clear();
            start_end_derivatives.clear();
            flag_regenerate = false;

            if (flag_first_call || flag_polyInit || flag_force_polynomial) 
            {
                flag_first_call = false;
                flag_force_polynomial = false;
                PolynomialTraj gl_traj;
                double dist = (start_pt - local_target_pt).norm();
                double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

                gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);

                double t_tmp;
                bool flag_too_far;
                double ts_tmp = ts * 1.5;
                do {
                    ts_tmp /= 1.5;
                    point_set.clear();
                    flag_too_far = false;
                    Eigen::Vector3d last_pt = gl_traj.evaluate(0);
                    for (t_tmp = 0; t_tmp < time; t_tmp += ts_tmp) {
                        Eigen::Vector3d pt = gl_traj.evaluate(t_tmp);
                        if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) { flag_too_far = true; break; }
                        last_pt = pt;
                        point_set.push_back(pt);
                    }
                } while ((flag_too_far || point_set.size() < 7) && ts_tmp > 0.01); // 澧炲姞 ts 淇濇姢
                
                start_end_derivatives.push_back(gl_traj.evaluateVel(0));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
                start_end_derivatives.push_back(gl_traj.evaluateAcc(std::max(0.0, t_tmp - ts_tmp)));
            }
            else 
            {
                double t;
                double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

                vector<double> pseudo_arc_length;
                vector<Eigen::Vector3d> segment_point;
                pseudo_arc_length.push_back(0.0);
                for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
                {
                    segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                    if (segment_point.size() > 1)
                    {
                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                    }
                }
                t -= ts;

                double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
                if (poly_time > ts)
                {
                    PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                                  local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

                    for (double tp = ts; tp < poly_time; tp += ts)
                    {
                        segment_point.push_back(gl_traj.evaluate(tp));
                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                    }
                }

                double sample_length = 0;
                double cps_dist = pp_.ctrl_pt_dist * 1.5; 
                size_t id = 0;
                do
                {
                    cps_dist /= 1.5;
                    point_set.clear();
                    sample_length = 0;
                    id = 0;
                    while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                    {
                        if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                        {
                            point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                                (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                            sample_length += cps_dist;
                        }
                        else
                            id++;
                    }
                    point_set.push_back(local_target_pt);
                } while (point_set.size() < 7 && cps_dist > 0.01); 

                start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(Eigen::Vector3d::Zero());

                if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) 
                {
                    flag_force_polynomial = true;
                    flag_regenerate = true;
                }
            }
        } while (flag_regenerate);

        if (current_scenario_ == HEAD_ON && point_set.size() >= 4)
        {
            Eigen::Vector2d forward = head_on_maneuver_lock_ ? head_on_lock_course_ : start_vel.head<2>();
            if (forward.norm() < 0.1)
            {
                forward = (local_target_pt - start_pt).head<2>();
            }

            if (forward.norm() > 1e-3)
            {
                forward.normalize();
                const Eigen::Vector2d right_normal(forward.y(), -forward.x());
                const Eigen::Vector2d start2d = start_pt.head<2>();
                const Eigen::Vector2d target2d = local_target_pt.head<2>();
                const double path_len = std::max((target2d - start2d).dot(forward), 1.0);
                const double obs_along = std::max(0.0, (ts_pos_.head<2>() - start2d).dot(forward));
                const double desired_offset = std::max(5.5, safe_dcpa_ * 1.1);
                const double bias_start = 1.0;
                const double bias_full = std::max(5.0, std::min(10.0, obs_along * 0.55));
                const double pass_along = obs_along + std::max(10.0, safe_dcpa_ * 2.5);
                const bool can_return_after_pass = path_len > pass_along + 3.0;
                const double return_start = can_return_after_pass ? pass_along : path_len + 1.0;

                for (size_t i = 1; i < point_set.size(); ++i)
                {
                    Eigen::Vector2d rel = point_set[i].head<2>() - start2d;
                    const double along = rel.dot(forward);
                    if (along <= bias_start)
                    {
                        continue;
                    }

                    double ramp = std::min(1.0, std::max(0.0, (along - bias_start) / bias_full));
                    double taper = 1.0;
                    if (along > return_start)
                    {
                        const double taper_len = std::max(2.0, path_len - return_start);
                        taper = std::max(0.0, 1.0 - (along - return_start) / taper_len);
                    }

                    const double offset = desired_offset * ramp * ramp * (3.0 - 2.0 * ramp) * taper;
                    Eigen::Vector2d shifted = point_set[i].head<2>() + right_normal * offset;
                    point_set[i].x() = shifted.x();
                    point_set[i].y() = shifted.y();
                }

                if (!start_end_derivatives.empty())
                {
                    const double start_speed =
                        std::max(0.4, std::min(pp_.max_vel_, start_end_derivatives[0].head<2>().norm()));
                    Eigen::Vector2d biased_start_dir = forward + 0.35 * right_normal;
                    if (biased_start_dir.norm() > 1e-3)
                    {
                        biased_start_dir.normalize();
                        start_end_derivatives[0].x() = biased_start_dir.x() * start_speed;
                        start_end_derivatives[0].y() = biased_start_dir.y() * start_speed;
                    }

                    if (!can_return_after_pass && start_end_derivatives.size() >= 2)
                    {
                        Eigen::Vector2d biased_end_dir = forward + 0.25 * right_normal;
                        if (biased_end_dir.norm() > 1e-3)
                        {
                            const double end_speed = start_end_derivatives[1].head<2>().norm();
                            biased_end_dir.normalize();
                            start_end_derivatives[1].x() = biased_end_dir.x() * end_speed;
                            start_end_derivatives[1].y() = biased_end_dir.y() * end_speed;
                        }
                    }
                }

                ROS_WARN("COLREGs HEAD_ON: applied early starboard bias to local initial trajectory, offset=%.2f m, points=%zu",
                         desired_offset, point_set.size());
                ROS_WARN("COLREGs HEAD_ON: holding starboard lane until along=%.2f m (obs_along=%.2f, path_len=%.2f)",
                         return_start, obs_along, path_len);
            }
        }

        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        /* 3. 璋冪敤甯﹂伩纰颁唬浠风殑 A* */
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        if(current_scenario_ != NONE) {
            ROS_INFO("[COLREGs] Planner adjusted for Scenario: %d", (int)current_scenario_);
        }

        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        if (!flag_step_1_success) {
            continuous_failures_count_++;
            return false;
        }

        t_opt = ros::Time::now() - t_start;

/*** STEP 3: REFINE ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        if (!pos.checkFeasibility(ratio, false)) {
            Eigen::MatrixXd optimal_control_points;
            if (refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points))
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        updateTrajInfo(pos, ros::Time::now());
        continuous_failures_count_ = 0;
        return true;
    }

    bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
    {
        Eigen::MatrixXd control_points(3, 6);
        for (int i = 0; i < 6; i++)
        {
            control_points.col(i) = stop_pos;
        }

        updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

        return true;
    }

    bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);

        for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
        {
            points.push_back(waypoints[wp_i]);
        }

        double total_len = 0;
        total_len += (start_pos - waypoints[0]).norm();
        for (size_t i = 0; i < waypoints.size() - 1; i++)
        {
            total_len += (waypoints[i + 1] - waypoints[i]).norm();
        }

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        double dist_thresh = max(total_len / 8, 4.0);

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // for ( int i=0; i<inter_points.size(); i++ )
        // {
        //   cout << inter_points[i].transpose() << endl;
        // }

        // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);
        points.push_back(end_pos);

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        const double dist_thresh = 4.0;

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // write position matrix
        int pt_num = inter_points.size();
        //ROS_INFO("point num : %d",inter_points.size());
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

        // std::cout << "ratio: " << ratio << std::endl;
        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
        bspline_optimizer_rebound_->ref_pts_.clear();
        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
    {
        local_data_.start_time_ = time_now;
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                           Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;
        // double length = bspline.getLength(0.1);
        // int seg_num = ceil(length / pp_.ctrl_pt_dist);

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

void EGOPlannerManager::checkCOLREGs(const Eigen::Vector3d& os_pos, const Eigen::Vector3d& os_vel, 
                                     const Eigen::Vector3d& ts_pos, const Eigen::Vector3d& ts_vel) {
        Eigen::Vector2d p_os = os_pos.head<2>();
        Eigen::Vector2d v_os = os_vel.head<2>();
        Eigen::Vector2d p_ts = ts_pos.head<2>();
        Eigen::Vector2d v_ts = ts_vel.head<2>();

        Eigen::Vector2d rel_pos = p_ts - p_os;
        Eigen::Vector2d rel_vel = v_os - v_ts; 
        double dist = rel_pos.norm();

        double dcpa = dist;
        double tcpa = 0.0;
        double v_rel_norm = rel_vel.norm();
        
        if (v_rel_norm > 0.05) { 
            double dot_product = rel_pos.dot(rel_vel);
            tcpa = dot_product / (v_rel_norm * v_rel_norm);
            double squared_dcpa = rel_pos.squaredNorm() - pow(dot_product, 2) / (v_rel_norm * v_rel_norm);
            dcpa = sqrt(std::max(0.0, squared_dcpa));
        }
        
        last_dcpa_ = dcpa;
        last_tcpa_ = tcpa;

        double os_yaw = atan2(v_os.y(), v_os.x());
        double ts_yaw = atan2(v_ts.y(), v_ts.x());
        
        double bearing = atan2(rel_pos.y(), rel_pos.x()) - os_yaw;
        while (bearing >  M_PI) bearing -= 2 * M_PI;
        while (bearing < -M_PI) bearing += 2 * M_PI;
        double bearing_deg = bearing * 180.0 / M_PI;

        Eigen::Vector2d os_course = v_os.norm() > 0.05 ? v_os.normalized() : Eigen::Vector2d(cos(os_yaw), sin(os_yaw));
        double lateral_to_ts = os_course.x() * rel_pos.y() - os_course.y() * rel_pos.x();
        double along_to_ts = rel_pos.dot(os_course);

        if (head_on_maneuver_lock_)
        {
            const bool same_obstacle = head_on_lock_obstacle_ == active_obstacle_name_;
            const double lock_time = (ros::Time::now() - head_on_lock_start_).toSec();
            const bool passed_target = along_to_ts < -safe_dcpa_;
            const bool safely_separated = lock_time > 3.0 && tcpa < -1.0 && dcpa > safe_dcpa_ * 1.2;

            if (same_obstacle && !passed_target && !safely_separated)
            {
                current_scenario_ = HEAD_ON;
                ROS_INFO_THROTTLE(0.5, "[COLREGs] Mode: 1 | locked starboard maneuver | lateral: %.2f | along: %.2f | DCPA: %.2f | TCPA: %.2f",
                                  lateral_to_ts, along_to_ts, dcpa, tcpa);
                return;
            }

            head_on_maneuver_lock_ = false;
            head_on_lock_obstacle_ = "none";
        }

        if (dist > colregs_dist_threshold_ || tcpa <= 0 || dcpa > safe_dcpa_) {
            current_scenario_ = NONE;
            ROS_INFO_THROTTLE(0.5, "[COLREGs] Mode: 0 | filtered | dist: %.2f/%.2f | DCPA: %.2f/%.2f | TCPA: %.2f",
                              dist, colregs_dist_threshold_, dcpa, safe_dcpa_, tcpa);
            return;
        }
        bool has_clear_colregs_side = false;
        if ((current_scenario_ == HEAD_ON || current_scenario_ == CROSS_GIVE_WAY) &&
            lateral_to_ts > safe_dcpa_ * 0.8)
        {
            has_clear_colregs_side = true;
        }
        else if (current_scenario_ == OVERTAKING &&
                 lateral_to_ts < -safe_dcpa_ * 0.8)
        {
            has_clear_colregs_side = true;
        }

        if (!head_on_maneuver_lock_ && has_clear_colregs_side && dcpa > safe_dcpa_ * 0.8)
        {
            current_scenario_ = NONE;
            ROS_INFO_THROTTLE(0.5, "[COLREGs] Mode: 0 | cleared by maneuver | lateral: %.2f | DCPA: %.2f/%.2f | TCPA: %.2f",
                              lateral_to_ts, dcpa, safe_dcpa_, tcpa);
            return;
        }

        double rel_bearing_to_ts = atan2(-rel_pos.y(), -rel_pos.x()) - ts_yaw;
        while (rel_bearing_to_ts >  M_PI) rel_bearing_to_ts -= 2 * M_PI;
        while (rel_bearing_to_ts < -M_PI) rel_bearing_to_ts += 2 * M_PI;
        double rel_bearing_to_ts_deg = rel_bearing_to_ts * 180.0 / M_PI;

        double angle_diff = ts_yaw - os_yaw;
        while (angle_diff >  M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        double angle_diff_deg = angle_diff * 180.0 / M_PI;

        COLREGS_SCENARIO detected_scenario = NONE;

        // 鍒ゅ畾閫昏緫
        if (abs(rel_bearing_to_ts_deg) > 112.5 && v_os.norm() > v_ts.norm()) {
            detected_scenario = OVERTAKING;
        }
        else if (abs(bearing_deg) < 15.0 && abs(angle_diff_deg) > 160.0) {
            detected_scenario = HEAD_ON;
        }
        else if (abs(bearing_deg) <= 112.5) {
            if (bearing_deg < 0) {
                detected_scenario = CROSS_GIVE_WAY;
            } else {
                detected_scenario = CROSS_STAND_ON;
            }
        }

        // 娑堟姈澶勭悊
        static int consistency_count = 0;
        static COLREGS_SCENARIO last_detected = NONE;
        if (detected_scenario == last_detected && detected_scenario != NONE) {
            consistency_count++;
        } else {
            consistency_count = 0;
            last_detected = detected_scenario;
        }

        if (consistency_count >= 3 || detected_scenario == NONE) {
            current_scenario_ = detected_scenario;
        }

        if (current_scenario_ == HEAD_ON && !head_on_maneuver_lock_)
        {
            head_on_maneuver_lock_ = true;
            head_on_lock_obstacle_ = active_obstacle_name_;
            head_on_lock_start_ = ros::Time::now();
            head_on_lock_course_ = os_course;
            if (head_on_lock_course_.norm() < 1e-3)
            {
                head_on_lock_course_ = Eigen::Vector2d(1.0, 0.0);
            }
            else
            {
                head_on_lock_course_.normalize();
            }
            ROS_WARN("[COLREGs] HEAD_ON maneuver locked for obstacle: %s",
                     head_on_lock_obstacle_.c_str());
        }

        ROS_INFO_THROTTLE(0.5, "[COLREGs] Mode: %d | DCPA: %.2f | TCPA: %.2f", (int)current_scenario_, dcpa, tcpa);
    }

} // namespace ego_planner
