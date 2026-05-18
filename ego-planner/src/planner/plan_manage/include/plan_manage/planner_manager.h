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
#include <path_searching/hybrid_a_star.h>  // 娣峰悎A*璺緞鎼滅储

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

        /* --- COLREGs 娴蜂簨閬跨鐩稿叧鏁版嵁缁撴瀯 --- */
        
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

        /* --- 瑙勫垝鍣ㄦ牳蹇冩帴鍙?--- */

        bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                           Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
        
        bool EmergencyStop(Eigen::Vector3d stop_pos);
        // 鍏ㄥ眬璺緞瑙勫垝
        bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                            const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
        
        bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                     const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

        void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);

        /* --- 鍏叡鎴愬憳鍙橀噺 --- */
        PlanParameters pp_;              // 瑙勫垝鍙傛暟
        LocalTrajData local_data_;       // 鏈湴杞ㄨ抗鏁版嵁
        GlobalTrajData global_data_;     // 鍏ㄥ眬杞ㄨ抗鏁版嵁
        GridMap::Ptr grid_map_;          // 鍦板浘鎸囬拡
        HybridAStar::Ptr a_star_;        // 娣峰悎A*璺緞鎼滅储鍣ㄦ寚閽?
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double getLastDCPA() const { return last_dcpa_; }
        double getLastTCPA() const { return last_tcpa_; }
        double getSafeDCPA() const { return safe_dcpa_; }

    private:
        /* --- 鍐呴儴绉佹湁绠楁硶涓庢ā鍧?--- */
        PlanningVisualization::Ptr visualization_;
        BsplineOptimizer::Ptr bspline_optimizer_rebound_;

        // 鐢ㄤ簬璁＄畻鍜屼繚瀛樻渶杩戜竴娆＄殑閬跨鎸囨爣
        double last_dcpa_{0.0};
        double last_tcpa_{0.0};

        // 閬跨閫昏緫鍙傛暟
        double colregs_dist_threshold_{25.0}; // 瑙﹀彂鍒ゅ畾璺濈
        double safe_dcpa_{3.5};               // 瀹夊叏璺濈闃堝€?
        int continuous_failures_count_{0};
        std::vector<DynamicObstacleState> dynamic_obstacles_;

        // 鏇存柊杞ㄨ抗淇℃伅
        void updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now);

        // B鏍锋潯閲嶅弬鏁板寲
        void reparamBspline(UniformBspline &bspline, std::vector<Eigen::Vector3d> &start_end_derivative, 
                            double ratio, Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

        // 杞ㄨ抗绮惧寲绠楁硶
        bool refineTrajAlgo(UniformBspline &traj, std::vector<Eigen::Vector3d> &start_end_derivative, 
                            double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    public:
        typedef std::unique_ptr<EGOPlannerManager> Ptr;
    };

} // namespace ego_planner

#endif
