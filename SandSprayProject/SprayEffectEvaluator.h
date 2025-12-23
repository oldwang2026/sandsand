#ifndef SPRAYEFFECTEVALUATOR_H
#define SPRAYEFFECTEVALUATOR_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <array>
#include <NXOpen/Point.hxx>
#include <uf.h>
#include <uf_facet.h>
#include "regionstruct.h"

using namespace NXOpen;

// ==================== 数据存储结构 ====================

// 三角形评估数据
struct TriangleEvalData {
    double vertices[3][3];     // 三个顶点坐标 [顶点索引][x,y,z]
    Point3d center;            // 三角形中心点
    double minDistance;        // 到最近路径的距离（保留用于调试）
    double sprayIntensity;     // 喷射强度（正态分布叠加）
    int colorCode;             // 对应的颜色代码
    int regionId;              // 所属的Region ID

    // === 新增字段（用于BFS聚类） === 
    int facetId;               // NX facet ID，用于构建邻接关系
    int clusterIndex;          // 所属badArea聚类的索引，-1表示不属于任何不良区域

    TriangleEvalData()
        : minDistance(1e10), sprayIntensity(0.0), colorCode(186), regionId(-1),
          facetId(UF_FACET_NULL_FACET_ID), clusterIndex(-1)
    {
        // 初始化顶点数组
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                vertices[i][j] = 0.0;
            }
        }
    }
};

// 评估统计数据
struct EvaluationStats {
    int totalTriangles;        // 总三角形数
    int excessiveCount;        // 过度喷射（≥1.8）紫红色
    int excellentCount;        // 优秀（0.6~1.8）深绿色
    int goodCount;             // 良好（0.3~0.6）浅绿色
    int fairCount;             // 一般（0.1~0.3）黄色
    int poorCount;             // 较差（0.01~0.1）橙色
    int badCount;              // 差（<0.01）红色
    double coverageRate;       // 覆盖率 = (优秀+良好) / 总数
    double excessiveRate;      // 过度喷射率 = 过度 / 总数
    double averageIntensity;   // 平均喷射强度

    EvaluationStats()
        : totalTriangles(0), excessiveCount(0), excellentCount(0), goodCount(0),
          fairCount(0), poorCount(0), badCount(0),
          coverageRate(0.0), excessiveRate(0.0), averageIntensity(0.0) {}
};

// 不良覆盖聚集区域结构（用于BFS聚类）
struct BadAreaCluster {
    int id;                                       // 区域编号 (0, 1, 2, ...)
    int regionId;                                 // 所属Region的ID
    tag_t regionBodyTag;                          // 所属Region的Body Tag（用于曲线属性）
    FaceRegionType regionType;                    // 所属Region的类型（用于后续管理）
    std::vector<int> triangleIndices;             // 包含的三角形索引
    int triangleCount;                            // 三角形数量（冗余字段，便于访问）
    std::vector<std::array<double, 3>> vertices;  // 所有顶点坐标 [[x,y,z], ...]

    BadAreaCluster()
        : id(-1), regionId(-1), regionBodyTag(NULL_TAG), regionType(FaceRegionType::UNSORTED), triangleCount(0) {}
};



class SprayEffectEvaluator
{
public:
    // 构造与析构
    // sprayPathCurves: 喷砂路径曲线列表，会从第一条曲线读取SprayDiameter属性
    // regionBodies: 要评估的片体（Region）列表，来自对话框选择
    SprayEffectEvaluator(const std::vector<tag_t>& sprayPathCurves, const std::vector<NXOpen::Body*>& regionBodies);
    ~SprayEffectEvaluator();

    // 设置喷砂直径（用于距离分级）
    void SetSprayDiameter(double diameter);

    // 设置网格化精度（tolerance越小，三角形越密，默认0.5）
    void SetFacetTolerance(double tolerance);

    // 设置要评估的喷砂路径曲线
    void SetSprayPaths(const std::vector<tag_t>& pathCurves);

    // 核心功能：评估所有区域的喷砂覆盖效果
    bool EvaluateAllRegions();

    // 辅助功能：获取指定region的三角形数据（在EvaluateAllRegions之后调用）
    std::vector<TriangleEvalData> GetTrianglesByRegionId(int regionId) const;

    // 辅助功能：获取指定region的统计数据（在EvaluateAllRegions之后调用）
    EvaluationStats GetStatisticsByRegionId(int regionId) const;

    // 获取评估统计数据
    const EvaluationStats& GetStatistics() const { return m_stats; }

    // 获取所有三角形数据
    const std::vector<TriangleEvalData>& GetTriangles() const { return m_triangles; }

    // 获取所有Region对象
    const std::vector<Region>& GetRegions() const { return m_regions; }

    // 获取喷砂直径
    double GetSprayDiameter() const { return m_sprayDiameter; }
    double GetStepDistance() const { return m_stepDistance; }
    double GetOverlapRatio() const { return m_overlapRatio; }

    // 绘制所有三角形的边（使用临时线显示评估结果）
    void DrawAllTriangles();

    // 清除评估数据
    void Clear();

    // === 新增：BFS聚类相关公共方法 ===

    // 分析不良覆盖区域的聚类（在EvaluateAllRegions后调用）
    void AnalyzeBadAreaClusters();

    // 获取badArea聚类结果
    const std::vector<BadAreaCluster>& GetBadAreaClusters() const { return m_badAreaClusters; }

    // === 新增：临时点显示功能 ===

    // 为指定的badArea创建临时点（三角形顶点和中心点）
    void CreatePointsForBadArea(int badAreaIndex);

    // 只绘制指定badArea的三角形（不重新计算intensity）
    void DrawBadAreaTriangles(int badAreaIndex);

    // 重新计算指定badArea的intensity并重绘（用于补充路径后的更新）
    void RecalculateAndDrawBadArea(int badAreaIndex, tag_t newCurveTag);

    // 清除所有临时点
    void ClearTempPoints();

private:
    // ==================== 成员变量 ====================

    double m_sprayDiameter;              // 喷砂直径
    double m_stepDistance;               // 路径步距
    double m_overlapRatio;               // 路径重叠率
    double m_facetTolerance;             // 网格化精度
    std::vector<tag_t> m_sprayPaths;          // 喷砂路径曲线列表
    std::vector<TriangleEvalData> m_triangles; // 所有三角形数据
    EvaluationStats m_stats;             // 统计数据
    std::vector<Region> m_regions;       // 从图层80收集的所有Region对象（每个region都有自己的facet_model）

    // === 新增：BFS聚类相关成员变量 ===
    std::unordered_map<int, std::unordered_map<int, int>> m_facetIdToTriangleIndex;
    // regionId → (facet_id → triangle_index) 映射，用于快速查找三角形索引

    std::vector<int> m_poorBadTriangleIndices;  // poor/bad三角形索引列表（intensity<0.1）

    std::unordered_map<int, std::vector<int>> m_triangleAdjacencyMap;
    // triangle_index → 邻接的triangle_indices（只包含poor/bad之间的邻接）

    std::vector<BadAreaCluster> m_badAreaClusters;  // 聚类结果

    std::vector<tag_t> m_tempPoints;  // 临时点标签（用于可视化选中的badArea）

    // ==================== 内部方法 ====================

    // 步骤1：对区域内所有Face进行三角网格化
    bool FacetRegionFaces(Region& region);

    // 步骤2：提取所有三角形数据
    void ExtractTrianglesFromFacetModel(tag_t facetModel, int regionId);

    // 步骤3：计算每个三角形中心的喷射强度（正态分布叠加）
    void CalculateIntensitiesForAllTriangles();

    // 步骤4：根据强度为三角形分配颜色
    void AssignColorsToTriangles();

    // 步骤5：更新统计数据
    void UpdateStatistics();

    // 辅助方法：计算三角形中心点
    Point3d CalculateTriangleCenter(double vertices[3][3]);

    // 辅助方法：计算点到所有路径的最小距离（保留用于调试）
    double CalculateMinDistanceToPath(const Point3d& point);

    // 辅助方法：计算点的喷射强度（正态分布叠加）
    double CalculateSprayIntensity(const Point3d& point);

    // 辅助方法：根据强度映射到颜色代码
    int MapIntensityToColor(double intensity);

    // 辅助方法：绘制单个三角形的三条边
    void DrawTriangleEdges(const TriangleEvalData& triangle);

    // === 新增：BFS聚类相关私有方法 ===

    // 为poor/bad三角形构建邻接图（过滤后的邻接）
    void BuildAdjacencyMapForBadTriangles();

    // 获取指定三角形的邻接三角形索引列表
    std::vector<int> GetAdjacentTriangles(int triangleIndex);

    // BFS遍历生成单个badArea聚类
    BadAreaCluster ClusterBadAreaBFS(int seedIndex, std::unordered_set<int>& visited);
};

#endif // SPRAYEFFECTEVALUATOR_H
