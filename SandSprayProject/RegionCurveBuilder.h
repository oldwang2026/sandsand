#ifndef REGIONCURVEBUILDER_H
#define REGIONCURVEBUILDER_H

#include <vector>
#include <NXOpen/NXObject.hxx>
#include <NXOpen/Point.hxx>
#include <NXOpen/Edge.hxx>
#include <NXOpen/Curve.hxx>
#include <NXOpen/DatumCollection.hxx>

#include "RegionBuilder.h"
#include "regionstruct.h"

using namespace NXOpen;
using namespace std;

struct IntersectionCurveData {
    tag_t curveTag;               // 相交曲线
    tag_t datumPlaneTag;          // 对应的基准平面（用于删除重建）
    Vector3d planeNormal;         // 当前使用的法向量
    Point3d controlPoint;         // 对应的控制点
    int controlPointIndex;        // 控制点索引
    bool intersectsWithNeighbor;  // 是否与邻居相交

    IntersectionCurveData()
        : curveTag(NULL_TAG)
        , datumPlaneTag(NULL_TAG)
        , controlPointIndex(-1)
        , intersectsWithNeighbor(false)
    {}
};


class RegionCurveBuilder
{
public:
    // 构造函数
    RegionCurveBuilder();
    ~RegionCurveBuilder();

    // 设置喷砂参数
    void SetSprayDiameter(double diameter) { m_sprayDiameter = diameter; }
    void SetOverlapRatio(double ratio) { m_overlapRatio = ratio; }

    // 核心功能：为单个区域生成路径
    //bool GeneratePath( Region& region);
    int GetBoundaryGroupCount(Region& region);
    std::vector<RegionBoundaryEdgeInfo> GetBoundaryGroup(Region& region, int index);
    // 获取生成的路径点
    const vector<Point3d>& GetPathPoints() const { return m_pathPoints; }
    bool AnalyzeBoundaries(Region& region);  // 分析边界，生成边界组
    bool AnalyzeRingBoundaries(Region& region);
    // 清空路径数据
    //void Clear();

    std::vector<Point3d> GenerateControlPoints(tag_t splineTag);

    bool GenerateSprayPathForRegion(Region& region);



    bool GenerateSplineFromBoundaryGroup(const std::vector<RegionBoundaryEdgeInfo>& boundaryGroup, int pointsPerEdge, tag_t* splineTag);

    // 设置工艺参数的便捷方法

    static void CleanupNonPathCurves();

    void UpdateParametersFromUI(
        double sprayDiameter,
        double overlapRatio,
        double smallEdgeThreshold,
        double tinyEdgeThreshold
    );

private:
    // 喷砂参数
    double m_sprayDiameter;    // 喷砂直径
    double m_overlapRatio;      // 重叠率 (0.6-0.8)0
    double m_stepDistance;      // 步距 = 直径 * (1 - 重叠率)
    double m_angleThreshold;
    double m_tinyEdgeThreshold = 5.0;    // 极小边阈值
    double m_smallEdgeThreshold = 15.0;  // 小边阈值

    // 边界覆盖参数（新增）
    double m_maxBoundaryOffset;   // 最大边界偏移 = 喷砂半径（确保端点覆盖）
    double m_transitionThreshold; // 过渡区间上限（80-136mm使用三等分策略）  

    // 生成的路径数据
    vector<Point3d> m_pathPoints;  // 路径点序列

    // 当前处理的区域边界信息

    // 内部方法
    bool UpdateRegionBoundaryEdges(Region& region);

    //void CalculateStepDistance();

    std::vector<Point3d> ExtractPointsFromBoundaryGroup(const std::vector<RegionBoundaryEdgeInfo>& boundaryGroup, int pointsPerEdge);

    double CalculateEdgeAngle(RegionBoundaryEdgeInfo edge1, RegionBoundaryEdgeInfo edge2);

    std::vector<RegionBoundaryEdgeInfo> OrderBoundaryEdges(const std::vector<Edge*>& boundaryEdges);

    void GenerateAllControlPointsForRegion(Region& region, int pointsPerEdge = 10);

    void CalculateStepDistance();

    std::vector<Point3d> GetPointsOnIntersectionCurve(tag_t curve, const Point3d& referencePoint);

    // 针对平面环形区域：在相交曲线两端点连成的直线上取点
    std::vector<Point3d> GetPointsOnLineForPlanarRingRegion(tag_t curve, const Point3d& controlPoint);

    tag_t get_intersect(tag_t body, DatumPlane* dplane, double ref_point[3]);

    void CreateDatumPlane(Point3d org, Vector3d dir, tag_t& dplane);

    tag_t CreateSplineFromPoints(const std::vector<Point3d>& points);

    bool FilterSmallFeatureEdges(const std::vector<RegionBoundaryEdgeInfo>& orderedBoundary, int index);

    /**
   * 智能连接交点形成喷砂路径
   * @param intersectionPointsMatrix 二维点矩阵[控制点索引][交点索引]
   * @return 生成的路径段集合，每个路径段是一个点序列
   */
    std::vector<std::vector<Point3d>> ConnectPathPointsWithRingSupport(
        const std::vector<std::vector<Point3d>>& intersectionPointsMatrix);

    /**
     * 合并断开的路径段
     * @param segments 初始路径段集合
     * @param connectThreshold 连接距离阈值
     * @return 合并优化后的路径段集合
     */
    std::vector<std::vector<Point3d>> MergePathSegments(
        const std::vector<std::vector<Point3d>>& segments,
        double connectThreshold);

    /*检测相交并修改*/

    bool CheckCurvesIntersect(tag_t curve1, tag_t curve2, double tolerance = 0.1);

    // 查找不相交的邻居
    int FindNonIntersectingPrev(const std::vector<tag_t>& curves, int currentIdx, bool isRing);
    int FindNonIntersectingNext(const std::vector<tag_t>& curves, int currentIdx, bool isRing);

    // 法向量处理
    Vector3d AverageNormals(const Vector3d& n1, const Vector3d& n2);
    Vector3d GetNormalAtIndex(const std::vector<IntersectionCurveData>& curveDataList, int index);

    // 相交检测
    bool CheckIntersectionWithNeighbors(const std::vector<tag_t>& curves, int currentIdx, bool isRing);

    // 重新生成相交曲线
    bool RegenerateIntersectionCurve(IntersectionCurveData& curveData, const Vector3d& newNormal, tag_t bodyTag);

    // 为单条路径曲线设置属性
    void SetPathCurveAttributes(
        tag_t curveTag,
        tag_t regionTag,
        FaceRegionType regionType,
        int pathIndex);

    std::vector<Point3d> GetPointsOnIntersectionCurve2(tag_t curve, const Point3d& referencePoint);

    std::vector<double> OptimizedBoundaryBufferedSampling(double totalLength, double stepDistance, bool reverseDirection);
};

#endif // REGIONCURVEBUILDER_H