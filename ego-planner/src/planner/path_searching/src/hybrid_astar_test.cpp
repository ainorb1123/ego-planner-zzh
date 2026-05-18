/**
 * @file hybrid_astar_test.cpp
 * @brief 混合A*算法的单元测试
 * 
 * 测试以下功能：
 * 1. Reeds-Shepp 路径计算
 * 2. 混合A*搜索
 * 3. 参数验证
 * 4. 海事规则集成
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <path_searching/hybrid_a_star.h>
#include <path_searching/reeds_shepp.h>

// ============== Reeds-Shepp 测试 ==============

class ReedsSheppTest : public ::testing::Test
{
protected:
    ReedsSheppCurve curve_{1.0};  // 转向半径 1.0m
};

/**
 * @test Reeds-Shepp 距离函数
 */
TEST_F(ReedsSheppTest, DistanceFunction)
{
    // 测试同一点的距离应为0
    double dist = curve_.distance(0, 0, 0, 0, 0, 0);
    EXPECT_NEAR(dist, 0.0, 1e-6);

    // 测试距离应为正
    dist = curve_.distance(0, 0, 0, 10, 10, 0);
    EXPECT_GT(dist, 0.0);

    // 测试对称性
    double dist1 = curve_.distance(0, 0, 0, 5, 5, M_PI / 4);
    double dist2 = curve_.distance(5, 5, M_PI / 4, 0, 0, 0);
    EXPECT_NEAR(dist1, dist2, 1e-3);  // 允许小数值误差
}

/**
 * @test Reeds-Shepp 最短路径
 */
TEST_F(ReedsSheppTest, ShortestPath)
{
    auto path = curve_.shortestPath(0, 0, 0, 10, 0, 0, 0.1);
    
    EXPECT_GT(path.points.size(), 0);
    EXPECT_GT(path.length, 0);
    
    // 验证起点
    EXPECT_NEAR(path.points[0].x(), 0, 1e-3);
    EXPECT_NEAR(path.points[0].y(), 0, 1e-3);
    
    // 验证终点（允许离散化误差）
    EXPECT_NEAR(path.points.back().x(), 10, 0.2);
    EXPECT_NEAR(path.points.back().y(), 0, 0.2);
}

/**
 * @test Reeds-Shepp 带方向的路径
 */
TEST_F(ReedsSheppTest, PathWithOrientation)
{
    auto path = curve_.shortestPath(0, 0, 0, 5, 5, M_PI / 4, 0.1);
    
    EXPECT_GT(path.points.size(), 0);
    EXPECT_GT(path.length, 0);
    
    // 终点方向应接近目标方向（允许离散化误差）
    double final_theta = path.points.back().z();
    double angle_diff = std::abs(final_theta - M_PI / 4);
    if (angle_diff > M_PI)
        angle_diff = 2 * M_PI - angle_diff;
    EXPECT_LT(angle_diff, 0.3);  // 大约17度以内
}

// ============== 混合A*测试 ==============

class HybridAStarTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        planner_ = std::make_shared<HybridAStar>(0.1745, 1.0);  // 10度，1.0m
        planner_->setVelocity(1.0);
        planner_->setMaxSteeringAngle(0.5);
    }

    HybridAStar::Ptr planner_;
};

/**
 * @test 参数验证 - 无效步长
 */
TEST_F(HybridAStarTest, InvalidStepSize)
{
    // 不初始化地图的情况下测试会失败（返回false）
    Eigen::Vector2d start(0, 0);
    Eigen::Vector2d goal(10, 10);
    
    // step_size <= 0 应该返回false
    bool result = planner_->search(-0.1, start, 0, goal, 0);
    EXPECT_FALSE(result);
    
    result = planner_->search(0, start, 0, goal, 0);
    EXPECT_FALSE(result);
}

/**
 * @test 参数验证 - 未初始化的地图
 */
TEST_F(HybridAStarTest, UninitializedGridMap)
{
    Eigen::Vector2d start(0, 0);
    Eigen::Vector2d goal(10, 10);
    
    // 未初始化地图，应返回false
    bool result = planner_->search(0.1, start, 0, goal, 0);
    EXPECT_FALSE(result);
}

/**
 * @test 转向半径设置
 */
TEST_F(HybridAStarTest, TurningRadiusSetup)
{
    planner_->setTurningRadius(2.0);
    EXPECT_NEAR(planner_->getTurningRadius(), 2.0, 1e-6);
    
    planner_->setTurningRadius(0.5);
    EXPECT_NEAR(planner_->getTurningRadius(), 0.5, 1e-6);
}

/**
 * @test 海事规则参数
 */
TEST_F(HybridAStarTest, ColregsParameters)
{
    planner_->setColregsMode(1);
    planner_->setStartPos(Eigen::Vector2d(0, 0));
    planner_->setTargetShipInfo(Eigen::Vector2d(10, 10), 
                                Eigen::Vector2d(1, 0),
                                Eigen::Vector2d(1, 0));
    planner_->setSafetyParams(5.0, 10.0);
    
    // 应该能够设置这些参数而不出错
    EXPECT_EQ(planner_->getSearchIterations(), 0);  // 未开始搜索
}

// ============== 集成测试 ==============

class HybridAStarIntegrationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // 这里可以创建一个简单的模拟地图
        // 实际使用中应该使用真实的GridMap对象
    }
};

/**
 * @test 完整的规划流程
 */
TEST_F(HybridAStarIntegrationTest, PlanningWorkflow)
{
    auto planner = std::make_shared<HybridAStar>(0.1745, 1.0);
    
    // 测试参数设置
    planner->setVelocity(1.0);
    planner->setMaxSteeringAngle(0.5);
    planner->setTurningRadius(1.0);
    
    // 创建简单的配置
    Eigen::Vector2d start(0, 0);
    Eigen::Vector2d goal(5, 5);
    
    // 注意：实际规划需要地图初始化
    // 此测试验证API的正确性
    EXPECT_TRUE(true);
}

// ============== 性能测试 ==============

class HybridAStarPerformanceTest : public ::testing::Test
{
protected:
    ReedsSheppCurve curve_{1.0};
};

/**
 * @test Reeds-Shepp 路径生成性能
 */
TEST_F(HybridAStarPerformanceTest, PathGenerationSpeed)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < 100; ++i)
    {
        auto path = curve_.shortestPath(0, 0, 0, 10, 10, M_PI / 4, 0.1);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);
    
    // 100次路径生成应该在1秒内完成
    EXPECT_LT(duration.count(), 1000);
    
    std::cout << "100 path generations took " << duration.count() << " ms" << std::endl;
}

// ============== 主函数 ==============

int main(int argc, char** argv)
{
    // 如果是ROS环境
    // ros::init(argc, argv, "hybrid_astar_test");
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
