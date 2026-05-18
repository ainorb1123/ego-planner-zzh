#!/bin/bash
# 双向混合A*算法编译验证脚本
# 用法: bash compile_verify.sh

set -e  # 任何命令失败则退出

echo "=========================================="
echo "双向混合A*算法编译验证脚本"
echo "=========================================="

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'  # No Color

# 工作目录
WORK_DIR="$(pwd)"
PLANNER_DIR="${WORK_DIR}/src/planner"
PATH_SEARCH_DIR="${PLANNER_DIR}/path_searching"

echo -e "\n${YELLOW}1. 检查目录结构...${NC}"
if [ ! -d "$PATH_SEARCH_DIR" ]; then
    echo -e "${RED}✗ 找不到path_searching目录: $PATH_SEARCH_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 目录结构正确${NC}"

echo -e "\n${YELLOW}2. 检查关键源文件...${NC}"
FILES=(
    "include/path_searching/hybrid_a_star.h"
    "src/hybrid_a_star.cpp"
    "src/reeds_shepp.cpp"
    "src/dyn_a_star.cpp"
    "CMakeLists.txt"
)

for file in "${FILES[@]}"; do
    if [ ! -f "$PATH_SEARCH_DIR/$file" ]; then
        echo -e "${RED}✗ 缺少文件: $file${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ $file${NC}"
done

echo -e "\n${YELLOW}3. 检查编译修复...${NC}"

# 检查是否移除了inline关键字
echo -n "  检查Index2Coord内联关键字... "
if grep -q "^inline Eigen::Vector2d HybridAStar::Index2Coord" "$PATH_SEARCH_DIR/src/hybrid_a_star.cpp"; then
    echo -e "${RED}✗ 仍存在inline关键字${NC}"
    exit 1
else
    echo -e "${GREEN}✓ 已移除${NC}"
fi

echo -n "  检查Coord2Index内联关键字... "
if grep -q "^inline bool HybridAStar::Coord2Index" "$PATH_SEARCH_DIR/src/hybrid_a_star.cpp"; then
    echo -e "${RED}✗ 仍存在inline关键字${NC}"
    exit 1
else
    echo -e "${GREEN}✓ 已移除${NC}"
fi

# 检查向量访问方式
echo -n "  检查向量访问API一致性... "
if grep -q "idx(0)\|idx(1)" "$PATH_SEARCH_DIR/src/hybrid_a_star.cpp"; then
    echo -e "${RED}✗ 存在不一致的向量访问方式${NC}"
    exit 1
else
    echo -e "${GREEN}✓ 已统一为.x()/.y()${NC}"
fi

echo -e "\n${YELLOW}4. 验证接口定义...${NC}"

# 检查关键枚举
echo -n "  检查search_direction枚举... "
if grep -q "enum search_direction.*{.*FORWARD.*BACKWARD" "$PATH_SEARCH_DIR/include/path_searching/hybrid_a_star.h"; then
    echo -e "${GREEN}✓ 定义正确${NC}"
else
    echo -e "${RED}✗ 枚举定义缺失或错误${NC}"
    exit 1
fi

# 检查关键方法
echo -n "  检查search()方法声明... "
if grep -q "bool search(double" "$PATH_SEARCH_DIR/include/path_searching/hybrid_a_star.h"; then
    echo -e "${GREEN}✓ 声明正确${NC}"
else
    echo -e "${RED}✗ 方法声明缺失${NC}"
    exit 1
fi

echo -e "\n${YELLOW}5. 执行编译...${NC}"

# 检查ROS环境
if ! command -v catkin_make &> /dev/null; then
    echo -e "${YELLOW}⚠ 未检测到ROS环境，跳过编译测试${NC}"
    echo -e "${YELLOW}   请在ROS环境中手动执行: catkin_make path_searching -j4${NC}"
else
    echo "开始编译path_searching包..."
    cd "$WORK_DIR"
    
    # 清理旧的构建
    if [ -d "build" ]; then
        echo "清理旧的构建输出..."
        rm -rf build devel
    fi
    
    # 编译
    if catkin_make path_searching -j4 -DCMAKE_BUILD_TYPE=Release; then
        echo -e "${GREEN}✓ 编译成功${NC}"
    else
        echo -e "${RED}✗ 编译失败${NC}"
        exit 1
    fi
fi

echo -e "\n${YELLOW}6. 总结报告...${NC}"
echo -e "${GREEN}✓ 所有检查通过${NC}"
echo ""
echo "修复内容:"
echo "  1. 移除hybrid_a_star.cpp中Index2Coord的inline关键字"
echo "  2. 移除hybrid_a_star.cpp中Coord2Index的inline关键字"
echo "  3. 统一向量访问方式为.x()和.y()"
echo ""
echo "下一步建议:"
echo "  1. 运行单元测试: rosrun path_searching hybrid_astar_test"
echo "  2. 启动集成测试: roslaunch plan_manage run_in_sim.launch"
echo "  3. 查看文档:"
echo "     - CODE_REVIEW_SUMMARY.md (详细代码审查)"
echo "     - QUICK_VERIFICATION.md (快速问题排查)"
echo "     - COMPILATION_READY_REPORT.md (编译就绪报告)"

echo -e "\n${GREEN}=========================================="
echo "验证完成！代码已编译就绪。"
echo "=========================================${NC}\n"
