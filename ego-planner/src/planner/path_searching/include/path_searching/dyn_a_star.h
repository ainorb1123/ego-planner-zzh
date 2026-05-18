#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <memory>

constexpr double inf = 1e9; // 淇涓烘洿鏍囧噯鐨勭瀛﹁鏁版硶琛ㄧず
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0}; 
    enum enum_state state{UNDEFINED};
    Eigen::Vector2i index;

    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
    bool operator()(GridNodePtr node1, GridNodePtr node2)
    {
        return node1->fScore > node2->fScore;
    }
};

class AStar
{
private:
    GridMap::Ptr grid_map_;

    int colregs_mode_{0};
    Eigen::Vector2d start_pos_;

    Eigen::Vector2d ts_pos_, ts_vel_; 
    Eigen::Vector2d os_vel_; // 鏈綋褰撳墠閫熷害
    bool has_target_ship_{false};
    
    // DCPA/TCPA 瀹夊叏鏉冮噸鍙傛暟
    double dcpa_{10.0};
    double tcpa_{0.0};
    double safe_dcpa_{3.5};

    double calculateThreatCost(const Eigen::Vector2d& node_pos) {
        if (!has_target_ship_ || colregs_mode_ == 0) return 0.0;
        
        // TCPA <= 0 琛ㄧず姝ｅ湪杩滅鎴栧凡杩囨渶杩戜細閬囩偣锛屼笉澧炲姞浠ｄ环
        if (tcpa_ <= 0.0) return 0.0;
        
        double dist = (node_pos - ts_pos_).norm();
        if (dcpa_ > safe_dcpa_ && dist > 15.0) return 0.0;

        // 璁＄畻瀹夊叏鏉冮噸锛欴CPA瓒婂皬銆乀CPA瓒婄揣杩紝鏉冮噸瓒婂ぇ
        // 浣跨敤DCPA浠ｆ浛璺濈锛屾洿鍑嗙‘鍙嶆槧纰版挒椋庨櫓
        double safety_weight = safe_dcpa_ / std::max(dcpa_, 0.1);  // 鍩虹瀹夊叏鏉冮噸
        double urgency = std::min(1.0, 5.0 / std::max(tcpa_, 0.1)); // TCPA瓒婄揣杩紝urgency瓒婃帴杩?
        double base_cost = 50.0 * safety_weight * urgency;  // 鍩虹鏂瑰悜浠ｄ环
        
        double colregs_extra_cost = 0.0;
        
        Eigen::Vector2d move_vec = node_pos - start_pos_;
        Eigen::Vector2d ts_rel_vec = ts_pos_ - start_pos_;
        
        // 2D 鍙変箻鍒ゆ柇宸﹀彸: side > 0 琛ㄧず鑺傜偣鍦ㄦ湰鑸硅埅鍚戝乏渚э紝side < 0 琛ㄧず鍙充晶
        double side = move_vec.x() * ts_rel_vec.y() - move_vec.y() * ts_rel_vec.x();
        
        // 瑙勫垯寮曞閫昏緫
        if (colregs_mode_ == 1 || colregs_mode_ == 2) {
            // 瀵瑰ご(HEAD_ON, 瑙勫垯14) 鎴?浜ゅ弶璁╄矾(CROSS_GIVE_WAY, 瑙勫垯15)
            // 瑙勫垯瑕佹眰锛氬簲鍚戝彸杞悜锛堝崡渚э級锛屼粠浠栬埞宸﹁埛閫氳繃
            // 鍥犳鎯╃綒宸︿晶锛堝寳渚э紝side > 0锛夌殑璺緞
            if (side > 0) { 
                colregs_extra_cost = base_cost; 
            }
        }
        else if (colregs_mode_ == 4) {
            // 杩借秺(OVERTAKING, 瑙勫垯13)
            // 娴蜂簨鎯緥锛氬缓璁粠浠栬埞宸﹁埛锛堝寳渚э級杩借秺
            // 鍥犳鎯╃綒鍙充晶锛堝崡渚э紝side < 0锛夌殑璺緞
            if (side < 0) {
                colregs_extra_cost = base_cost;
            }
        }
        // 浜ゅ弶鐩磋埅(CROSS_STAND_ON, mode==3)锛氫繚鍚戜繚閫燂紝涓嶆坊鍔犳柟鍚戞€т唬浠?        
        return colregs_extra_cost;
    }

    // ... 鍏朵粬鍐呰仈鍑芥暟 ...
    inline double getHeu(GridNodePtr node1, GridNodePtr node2);

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);

    /* 璺緞鍥炴函澹版槑 */
    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    /* 鍧愭爣杞崲涓庤皟鏁村０鏄?*/
    bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector2d start_pt, 
                                               Eigen::Vector2d end_pt, 
                                               Eigen::Vector2i &start_idx, 
                                               Eigen::Vector2i &end_idx);

    bool ConvertToIndexAndAdjustStartEndPointsReverse(Eigen::Vector2d start_pt, 
                                                      Eigen::Vector2d end_pt, 
                                                      Eigen::Vector2i &start_idx, 
                                                      Eigen::Vector2i &end_idx);

public:
    typedef std::shared_ptr<AStar> Ptr;
       // 璁剧疆瀹夊叏鏉冮噸鍙傛暟
    void setSafetyParams(double dcpa, double tcpa) {
        dcpa_ = dcpa;
        tcpa_ = tcpa;
    }
    AStar(){};
    ~AStar();
            // 渚涘閮ㄨ皟鐢ㄧ殑鎺ュ彛
    void setColregsMode(int mode) { colregs_mode_ = mode; }
    void setStartPos(const Eigen::Vector2d& pos) { start_pos_ = pos; }
    void setTargetShipInfo(const Eigen::Vector2d& pos, const Eigen::Vector2d& vel, const Eigen::Vector2d& os_vel) {
        ts_pos_ = pos;
        ts_vel_ = vel;
        os_vel_ = os_vel;
        has_target_ship_ = true;
    }

    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size);
    bool AstarSearch(const double step_size, Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool is_adjust=true);
    std::vector<Eigen::Vector3d> getPath();

    inline bool checkOccupancy(const Eigen::Vector2d &pos) {
        if (grid_map_ == nullptr) return true;
        return (bool)grid_map_->getInflateOccupancy2d(pos);
    }

    inline Eigen::Vector2d Index2Coord(const Eigen::Vector2i &index) const;
    inline bool Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const;

    double step_size_, inv_step_size_;
    Eigen::Vector2d center_;
    Eigen::Vector2i CENTER_IDX_, POOL_SIZE_;
    const double tie_breaker_ = 1.0 + 1.0 / 1000;
    std::vector<GridNodePtr> gridPath_;
    GridNodePtr **GridNodeMap_;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;
    int rounds_{0};
};

// ... 浠ヤ笅鏄?inline 鍑芥暟鐨勫叿浣撳疄鐜?...
inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2) {
    double eucl_dist = (Index2Coord(node1->index) - Index2Coord(node2->index)).norm();
    
    // 濡傛灉瀛樺湪閬跨灞€闈笖TCPA绱ц揩锛屽鍔犲惎鍙戝€间互鍋忓悜鏇翠繚瀹堢殑鎼滅储
    if (colregs_mode_ != 0 && tcpa_ > 0 && tcpa_ < 3.0 && dcpa_ < safe_dcpa_ * 2) {
        // 绱ц揩鎯呭喌涓嬬暐寰鍔犲惎鍙戝€硷紝榧撳姳鏇翠繚瀹堢殑璺緞閫夋嫨
        eucl_dist *= 1.1;
    }
    
    return tie_breaker_ * eucl_dist;
}

inline Eigen::Vector2d AStar::Index2Coord(const Eigen::Vector2i &index) const {
    return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
}

inline bool AStar::Coord2Index(const Eigen::Vector2d &pt, Eigen::Vector2i &idx) const {
    idx = ((pt - center_) * inv_step_size_ + Eigen::Vector2d(0.5, 0.5)).cast<int>() + CENTER_IDX_;
    if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) ) {
        return false;
    }
    return true;
}

#endif
