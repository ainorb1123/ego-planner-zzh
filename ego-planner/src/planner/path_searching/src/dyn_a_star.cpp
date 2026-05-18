#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    if (GridNodeMap_ == nullptr) return;
    
    for (int i = 0; i < POOL_SIZE_(0); ++i)
    {
        if (GridNodeMap_[i] == nullptr) continue;
        for (int j = 0; j < POOL_SIZE_(1); ++j)
        {
            delete GridNodeMap_[i][j];
        }
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
    GridNodeMap_ = nullptr;
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr [POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }

    grid_map_ = occ_map;
}


double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));


    int diag = min(dx, dy);
    double remaining = abs(dx - dy);

    return diag * sqrt(2.0) + remaining * 1.0;  // 缁熶竴澶勭悊鎵€鏈夋儏鍐?
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy ;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector2d start_pt, Vector2d end_pt, Vector2i &start_idx, Vector2i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::ConvertToIndexAndAdjustStartEndPointsReverse(Eigen::Vector2d start_pt, Eigen::Vector2d end_pt,Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx) {
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (end_pt - start_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (start_pt - end_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}


bool AStar::AstarSearch(const double step_size, Vector2d start_pt, Vector2d end_pt, bool is_adjust)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1.0 / step_size;
    center_ = (start_pt + end_pt) / 2.0;

    Vector2i start_idx, end_idx;

    if(is_adjust)
    {
        if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
        {
            ROS_ERROR("Unable to handle the initial or end point, force return!");
            return false;
        }
    }
    else
    {
        if (!ConvertToIndexAndAdjustStartEndPointsReverse(start_pt, end_pt, start_idx, end_idx))
        {
            ROS_ERROR("Unable to handle the initial or end point, force return!");
            return false;
        }
    }

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    // 娓呯┖ OpenSet
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    // 3. 璁剧疆璧风偣灞炴€у苟鍔犲叆 OpenSet
    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET;
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr);

    endPtr->index = end_idx;

    GridNodePtr current = NULL;
    GridNodePtr neighborPtr = NULL;
    int num_iter = 0;

    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET;

        // 閬嶅巻 8 閭诲眳
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0) continue;

                Vector2i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 ||
                    neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 )
                {
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET) continue;

                if (checkOccupancy(Index2Coord(neighborIdx))) continue;

                neighborPtr->index = neighborIdx;
                neighborPtr->rounds = rounds_;

                // --- 鏍稿績锛氳绠椾唬浠?---
                double edge_cost = sqrt(dx * dx + dy * dy); 
                Eigen::Vector2d neighbor_pos = Index2Coord(neighborIdx);
                
                double colregs_cost = calculateThreatCost(neighbor_pos);

                double tentative_gScore = current->gScore + edge_cost + colregs_cost;

                if (!flag_explored)
                {
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr);
                }
                else if (tentative_gScore < neighborPtr->gScore)
                {
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        }

        if (num_iter % 500 == 0) {
            if ((ros::Time::now() - time_1).toSec() > 0.1)
            {
                ROS_WARN("A* Timeout! 0.1s exceeded.");
                return false;
            }
        }
    }

    return false;
}


vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;
    Eigen::Vector3d pos;
    Eigen::Vector2d pos2d;
    for (auto ptr : gridPath_)
    {
        pos2d = Index2Coord(ptr->index);
        pos<<pos2d(0),pos2d(1),0;
        path.push_back(pos);
    }
    reverse(path.begin(), path.end());
    return path;
}
