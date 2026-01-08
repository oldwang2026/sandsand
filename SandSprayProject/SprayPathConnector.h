#ifndef SPRAYPATHCONNECTOR_H_INCLUDED
#define SPRAYPATHCONNECTOR_H_INCLUDED

// 标准库
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

// NXOpen 及项目依赖
#include <uf.h>
#include <NXOpen/TaggedObject.hxx>
#include <NXOpen/Point.hxx>
#include "regionstruct.h"

// ==================== 数据结构定义 (保持兼容) ====================

// 喷砂路径信息结构体
// (保持与原 SprayPathSequencer 中的定义一致，以便无缝迁移)
struct SprayPath {
    NXOpen::Point3d start, end;
    int id;
    tag_t tag;
    int regionId;
    FaceRegionType regionType;
    double length;

    SprayPath(const NXOpen::Point3d& s, const NXOpen::Point3d& e,
        int pathId, tag_t curveTag, int regId, FaceRegionType type, double len)
        : start(s), end(e), id(pathId), tag(curveTag), regionId(regId), regionType(type), length(len) {
    }
};

// ==================== 核心连接器类 ====================

class SprayPathConnector {
public:
    // ==================== 构造与析构 ====================
    SprayPathConnector();
    ~SprayPathConnector();

    // ==================== 1. 数据输入接口 (UI调用) ====================

    // 从UI选中的曲线中添加路径
    void addPathsFromUI(const std::vector<NXOpen::TaggedObject*>& selectedCurves);

    // 清空所有路径数据
    void clearPaths();

    // 获取当前路径数量
    int getPathCount() const;

    // ==================== 2. 起点约束设置 (UI调用) ====================

    // 通过拾取点设置起始路径 (自动查找最近的路径端点)
    void setStartByPickedPoint(const NXOpen::Point3d& pickedPoint);

    // 通过UI拾取曲线设置起始路径
    void setStartByPickedCurve(NXOpen::TaggedObject* pickedCurve);

    // 清除起始约束
    void clearStartConstraint();

    // ==================== 3. 核心求解算法 ====================

    // 执行优化：多起点贪心初始化 -> SA 模拟退火搜索顺序 -> DP 动态规划确定方向
    void solve();

    // ==================== 4. 结果获取 ====================

    // 获取最优路径序列
    // 返回 pair<路径索引, entryIsStart>
    // entryIsStart == true  => 路径正向执行 (Start -> End)
    // entryIsStart == false => 路径反向执行 (End -> Start)
    std::vector<std::pair<int, bool>> getFinalSequence() const;

    // 获取优化后的曲线对象列表（按优化顺序）
    std::vector<NXOpen::TaggedObject*> getOptimizedCurves() const;

    // 获取最优总空移距离
    double getBestDistance() const;

    // ==================== 5. 辅助与可视化 ====================

    // 打印结果日志到文件
    void printResult();

    // 在 NX 中创建空移连接线 (可视化调试用)
    void createConnectionLines();

private:
    // ==================== 私有数据成员 ====================

    std::vector<SprayPath> paths;

    // 起点约束 (-1 表示无约束)
    int startPathIndex;
    // 约束起点的进入方向 (true=从Start进, false=从End进)
    bool startEntryIsStart;

    // 最终结果
    std::vector<std::pair<int, bool>> finalSequence;
    double bestDistance;

    // 随机数生成器与日志
    std::mt19937 rng;
    std::ofstream logFile;

    // 距离缓存矩阵 (4D Array)
    // distCache[i][j][entryA][entryB]
    // i, j: 路径索引
    // entryA, entryB: bool值 (0或1)，表示该路径是否从 Start 点进入
    // 存储值: 从 Path A 的出口 到 Path B 的入口 的距离
    std::vector<std::vector<std::vector<std::vector<double>>>> distCache;

    // ==================== 私有算法方法 ====================

    // --- 几何计算辅助 ---
    static double PointDistance(const NXOpen::Point3d& p1, const NXOpen::Point3d& p2);

    // 根据方向标志获取路径的入口点和出口点
    static NXOpen::Point3d EntryPoint(const SprayPath& p, bool entryIsStart);
    static NXOpen::Point3d ExitPoint(const SprayPath& p, bool entryIsStart);

    // --- 缓存管理 ---
    void buildDistanceCache();

    // 从缓存获取两路径间的空移代价 (Exit A -> Entry B)
    double transitionCost(int aIdx, bool aEntryIsStart, int bIdx, bool bEntryIsStart) const;

    // --- 步骤1: 初始化 ---
    // 生成初始路径顺序 (Multi-start Nearest Neighbor)
    std::vector<int> generateInitialSequence();

    // --- 步骤2: DP 评估 ---
    // 给定路径顺序，计算最优方向组合及总距离
    // 返回: <总空移距离, 对应的方向标志位图>
    std::pair<double, std::vector<bool>> evaluateSequenceWithDP(const std::vector<int>& order) const;

    // --- 步骤3: SA 模拟退火算子 ---
    // 注意：这些函数需要修改 rng 状态，因此不能是 const
    void applySwap(std::vector<int>& order);
    void applyRelocate(std::vector<int>& order);
    void apply2Opt(std::vector<int>& order);

    // SA 主循环
    std::vector<int> simulatedAnnealing(const std::vector<int>& initOrder);

    // --- 结果提交 ---
    void commitResult(const std::vector<int>& order,
        const std::vector<bool>& entryFlags,
        double totalAirMove);
};

#endif // SPRAYPATHCONNECTOR_H_INCLUDED