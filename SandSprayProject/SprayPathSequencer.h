#ifndef SPRAYPATHSEQUENCER_H_INCLUDED
#define SPRAYPATHSEQUENCER_H_INCLUDED

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>
#include <chrono>
#include <uf.h>
#include <NXOpen/TaggedObject.hxx>
#include "regionstruct.h"


// 喷砂路径信息
struct SprayPath {
    NXOpen::Point3d start, end;
    int id;
    tag_t tag;
    int regionId;
    FaceRegionType regionType;  // 区域类型
    double length;

    SprayPath(const NXOpen::Point3d& s, const NXOpen::Point3d& e,
        int pathId, tag_t curveTag, int regId, FaceRegionType type, double len);
};

// 存储路径之间连接的边
struct PathEdge {
    int path1, path2;          // 两条路径索引
    bool isStart1, isStart2;   // 是否使用起点
    double distance;           // 空移距离

    PathEdge(int p1, int p2, bool s1, bool s2, double dist);

    // 按距离排序
    bool operator<(const PathEdge& other) const;
};

// 前向声明
class GeneticAlgorithm;

// 喷砂路径序列优化器
class SprayPathSequencer {
    // 友元类声明：允许 GeneticAlgorithm 访问私有成员
    friend class GeneticAlgorithm;

private:
    std::vector<SprayPath> paths;
    std::vector<std::vector<PathEdge>> edgeList;
    std::vector<bool> usedPathStart;
    std::vector<bool> usedPathEnd;

    // 最优路径序列
    std::vector<std::pair<int, bool>> finalSequence;  // <path index, isReversed>
    double bestDistance;
    double initialDistance;  // 优化前的初始距离

    // 随机数生成器
    std::mt19937 rng;

    // 起始路径约束（-1表示无约束）
    int startPathIndex;
    bool startFromPathStart;   // true=从起点开始, false=从终点开始
    std::ofstream logFile;
    // ==================== 私有方法 ====================

    // 预计算所有路径之间的边
    void computeAllEdges();

    // 贪心算法生成初始解
    std::vector<std::pair<int, bool>> greedyInitialSolution(int startPath);

    // 计算路径序列总长度
    double calculatePathLength(const std::vector<std::pair<int, bool>>& sequence);

    // 使用2-opt局部优化
    void optimizeWith2Opt(std::vector<std::pair<int, bool>>& sequence);

public:
    // ==================== 构造与析构 ====================

    SprayPathSequencer();
    ~SprayPathSequencer();

    // ==================== 数据管理接口(UI调用) ====================

    // 从UI选中的曲线中添加路径
    void addPathsFromUI(const std::vector<NXOpen::TaggedObject*>& selectedCurves);


    // 清空所有路径数据
    void clearPaths();

    // 获取当前路径数量
    int getPathCount() const;

    // ==================== 起始约束设置(UI调用) ====================

    // 通过拾取点设置起始路径(自动查找最近的路径端点)
    void setStartByPickedPoint(const NXOpen::Point3d& pickedPoint);

    // 通过UI拾取曲线设置起始路径
    void setStartByPickedCurve(NXOpen::TaggedObject* pickedCurve);

    // 清除起始约束
    void clearStartConstraint();

    // ==================== 求解算法 ====================

    // 主求解函数(UI点击"优化"按钮后调用)
    void solve();

    // ==================== 结果获取 ====================

    // 获取最优路径序列
    std::vector<std::pair<int, bool>> getFinalSequence() const;

    // 获取优化后的曲线对象列表（按优化顺序）
    std::vector<NXOpen::TaggedObject*> getOptimizedCurves() const;

    // 获取总空移距离
    double getBestDistance() const;

    // 打印结果日志
    void printResult();

    // ==================== 可视化 ====================

    // 创建空移连接线(虚线)
    void createConnectionLines();
};

#endif // SPRAYPATHSEQUENCER_H_INCLUDED