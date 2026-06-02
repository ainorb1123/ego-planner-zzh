#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include <traj_utils/planning_visualization.h>
#include <path_searching/hybrid_a_star.h>  // ???A*??????

namespace ego_planner
{
    struct DynamicObstacleState
    {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        ros::Time stamp;
        std::string name;
        bool valid{false};
    };

    class EGOPlannerManager
    {
    public:
        EGOPlannerManager();
        ~EGOPlannerManager();

        /* --- COLREGs ??????????????? --- */
        
        // COLREGs scenario: 0 none, 1 head-on, 2 crossing give-way, 3 crossing stand-on, 4 overtaking
        enum COLREGS_SCENARIO {
            NONE = 0,
            HEAD_ON = 1,
            CROSS_GIVE_WAY = 2,
            CROSS_STAND_ON = 3,
            OVERTAKING = 4
        };

        COLREGS_SCENARIO current_scenario_ = NONE;

        Eigen::Vector3d ts_pos_, ts_vel_;
        std::string active_obstacle_name_{"none"};
        bool has_active_obstacle_{false};

        void checkCOLREGs(const Eigen::Vector3d& os_pos, const Eigen::Vector3d& os_vel,
                          const Eigen::Vector3d& ts_pos, const Eigen::Vector3d& ts_vel);

        void updateDynamicObstacle(const std::string& name, const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& vel, const ros::Time& stamp);
        bool selectActiveObstacle(const Eigen::Vector3d& os_pos, const Eigen::Vector3d& os_vel);

        /* --- ???????????--- */

        bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                           Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
        
        bool EmergencyStop(Eigen::Vector3d stop_pos);
        // ?????????
        bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                            const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
        
        bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                     const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

        void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

        /* --- ????????? --- */
        PlanParameters pp_;              // ??????
        LocalTrajData local_data_;       // ?????????
        GlobalTrajData global_data_;     // ?????????
        GridMap::Ptr grid_map_;          // ??????
        HybridAStar::Ptr a_star_;        // ???A*???????????
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double getLastDCPA() const { return last_dcpa_; }
        double getLastTCPA() const { return last_tcpa_; }
        double getSafeDCPA() const { return safe_dcpa_; }
        bool hasHeadOnManeuverLock() const { return head_on_maneuver_lock_; }
        Eigen::Vector2d getHeadOnLockCourse() const { return head_on_lock_course_; }
        Eigen::Vector2d getHeadOnLockOrigin() const { return head_on_lock_origin_; }
        Eigen::Vector2d getHeadOnObstacleTrackOrigin() const { return head_on_obstacle_track_origin_; }
        bool hasOvertakingManeuverLock() const { return overtaking_maneuver_lock_; }
        Eigen::Vector2d getOvertakingLockCourse() const { return overtaking_lock_course_; }
        Eigen::Vector2d getOvertakingLockOrigin() const { return overtaking_lock_origin_; }
        Eigen::Vector2d getOvertakingObstacleTrackOrigin() const { return overtaking_obstacle_track_origin_; }
        bool hasCrossingManeuverLock() const { return crossing_maneuver_lock_; }
        Eigen::Vector2d getCrossingTrackCourse() const { return crossing_track_course_; }
        Eigen::Vector2d getCrossingTrackOrigin() const { return crossing_track_origin_; }
        double getCrossingInitialSide() const { return crossing_initial_side_; }

    private:
        /* --- ???????????a??--- */
        PlanningVisualization::Ptr visualization_;
        BsplineOptimizer::Ptr bspline_optimizer_rebound_;

        // ????$???????????$???????
        double last_dcpa_{0.0};
        double last_tcpa_{0.0};

        // ?????????
        double colregs_dist_threshold_{25.0}; // ?????????
        double colregs_tcpa_threshold_{30.0}; // ignore CPA events too far in the future
        double safe_dcpa_{3.5};               // ??????????
        int continuous_failures_count_{0};
        std::vector<DynamicObstacleState> dynamic_obstacles_;
        bool head_on_maneuver_lock_{false};
        std::string head_on_lock_obstacle_{"none"};
        ros::Time head_on_lock_start_;
        Eigen::Vector2d head_on_lock_course_{Eigen::Vector2d(1.0, 0.0)};
        Eigen::Vector2d head_on_lock_origin_{Eigen::Vector2d::Zero()};
        Eigen::Vector2d head_on_obstacle_track_origin_{Eigen::Vector2d::Zero()};
        bool overtaking_maneuver_lock_{false};
        std::string overtaking_lock_obstacle_{"none"};
        ros::Time overtaking_lock_start_;
        Eigen::Vector2d overtaking_lock_course_{Eigen::Vector2d(1.0, 0.0)};
        Eigen::Vector2d overtaking_lock_origin_{Eigen::Vector2d::Zero()};
        Eigen::Vector2d overtaking_obstacle_track_origin_{Eigen::Vector2d::Zero()};
        bool crossing_maneuver_lock_{false};
        std::string crossing_lock_obstacle_{"none"};
        ros::Time crossing_lock_start_;
        Eigen::Vector2d crossing_track_course_{Eigen::Vector2d(1.0, 0.0)};
        Eigen::Vector2d crossing_track_origin_{Eigen::Vector2d::Zero()};
        double crossing_initial_side_{0.0};

        void resetHeadOnManeuver();
        void resetOvertakingManeuver();
        void resetCrossingManeuver();
        void updateAStarColregsContext(const Eigen::Vector3d& start_pt,
                                       const Eigen::Vector3d& start_vel);
        void applyHeadOnInitialBias(const Eigen::Vector3d& start_pt,
                                    const Eigen::Vector3d& start_vel,
                                    const Eigen::Vector3d& local_target_pt,
                                    std::vector<Eigen::Vector3d>& point_set,
                                    std::vector<Eigen::Vector3d>& start_end_derivatives);
        void applyOvertakingInitialBias(const Eigen::Vector3d& start_pt,
                                        const Eigen::Vector3d& start_vel,
                                        const Eigen::Vector3d& local_target_pt,
                                        std::vector<Eigen::Vector3d>& point_set,
                                        std::vector<Eigen::Vector3d>& start_end_derivatives);
        void applyCrossingInitialBias(const Eigen::Vector3d& start_pt,
                                      const Eigen::Vector3d& local_target_pt,
                                      std::vector<Eigen::Vector3d>& point_set,
                                      std::vector<Eigen::Vector3d>& start_end_derivatives);

        // ?????????
        void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

        // B?????????
        void reparamBspline(UniformBspline &bspline, std::vector<Eigen::Vector3d> &start_end_derivative, 
                            double ratio, Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

        // ?????????
        bool refineTrajAlgo(UniformBspline &traj, std::vector<Eigen::Vector3d> &start_end_derivative, 
                            double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    public:
        typedef std::unique_ptr<EGOPlannerManager> Ptr;
    };

} // namespace ego_planner

#endif
