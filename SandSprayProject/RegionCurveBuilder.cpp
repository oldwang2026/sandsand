#include "RegionCurveBuilder.h"
#include "HelpFunction.h"
#include <NXOpen/NXObjectManager.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Features_SectionCurveBuilder.hxx>
#include <NXOpen/Features_FeatureCollection.hxx>
#include <NXOpen/SelectObject.hxx>
#include <NXOpen/SelectObjectList.hxx>
#include <NXOpen/DatumCollection.hxx>
#include <NXOpen/DatumPlane.hxx>

#include <NXOpen/Features_DatumPlaneBuilder.hxx>
#include <NXOpen/Features_DatumPlaneFeature.hxx>
#include <NXOpen/Direction.hxx>
#include <NXOpen/DirectionCollection.hxx>
#include <NXOpen/NXObject.hxx>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <uf_obj.h>
#include <uf_layer.h>
#include <vector>
#include <uf_vec.h>  // 必须包含，用于调用 UF_VEC3_angle_between
#include <cmath>     // 用于 PI 和 abs

// ========== 多候选优化算法的约束和权重常量 ==========
// 修订版：基于物理覆盖约束的分级策略
namespace {
    // === 阶段3严格约束（优先尝试）===
    const double STRICT_MIN_OFFSET_RATIO = 0.25;    // s/4（中心化覆盖）
    const double STRICT_MAX_OFFSET_RATIO = 0.50;    // s/2（物理硬约束上限）
    const double STRICT_MIN_STEP_RATIO = 0.80;      // 0.8s（步距下限）
    const double STRICT_MAX_STEP_RATIO = 1.00;      // s（步距上限，避免覆盖空洞）

    // === 宽松级1：优先放开offset下限（允许贴近边界）===
    const double RELAXED1_MIN_OFFSET_RATIO = 0.10;  // 0.1s（保留小缓冲）

    // === 宽松级2：同时放宽步距下限 ===
    const double RELAXED2_MIN_OFFSET_RATIO = 0.00;  // 0（允许贴边界）
    const double RELAXED2_MIN_STEP_RATIO = 0.60;    // 0.6s（允许更密集）

    // === 终极回退：保证可解性 ===
    const double FALLBACK_MIN_STEP_RATIO = 0.40;    // 0.4s（极端情况）
    const double FALLBACK_MAX_STEP_RATIO = 1.10;    // 1.1s（略微超出）

    // === 评分权重（步距优先）===
    const double WEIGHT_STEP = 0.9;                 // 步距稳定性优先
    const double WEIGHT_OFFSET = 0.1;               // offset次要（接近s/4为佳）

    // === 枚举精度 ===
    const double STEP_SEARCH_PRECISION = 0.01;      // 步距搜索步长 = 0.01s
    const double OFFSET_SEARCH_PRECISION = 0.01;    // offset搜索步长 = 0.01s
}

bool RegionCurveBuilder::AnalyzeBoundaries(Region& region)  
{
    // 检查是否已经分析过
    if (region.boundaryAnalyzed && !region.boundaryGroups.empty()) {
        // 已经分析过，直接返回成功
        return true;
    }
    
    // 清除之前的分析结果
    region.boundaryGroups.clear();
    UpdateRegionBoundaryEdges(region);
    // 只处理非环形区域
    if (region.isRing) {

        return AnalyzeRingBoundaries(region);
    }

    // 获取边界边
    if (region.boundaryEdges.empty()) {
        return false;
    }

    // 1. 将离散的边界边组装成有序环
    std::vector<RegionBoundaryEdgeInfo> orderedBoundary = OrderBoundaryEdges(region.boundaryEdges);

    if (orderedBoundary.empty()) {
        return false;
    }

    // 2. 按夹角阈值分组（65度）
    std::vector<std::vector<RegionBoundaryEdgeInfo>> boundarySegments;
    std::vector<RegionBoundaryEdgeInfo> currentSegment;

    // 添加第一条边到当前段
    currentSegment.push_back(orderedBoundary[0]);

    for (size_t i = 1; i < orderedBoundary.size(); i++) {
        // 计算当前边与前一条边的夹角
        const RegionBoundaryEdgeInfo& prevEdge = orderedBoundary[i - 1];
        const RegionBoundaryEdgeInfo& currEdge = orderedBoundary[i];

        double angle = CalculateEdgeAngle(prevEdge, currEdge);

        if (angle > m_angleThreshold) {  // 默认65度
            // 夹角超过阈值，完成当前段，开始新段
            boundarySegments.push_back(currentSegment);
            currentSegment.clear();
        }

        currentSegment.push_back(currEdge);
    }

    // 添加最后一段
    if (!currentSegment.empty()) {
        boundarySegments.push_back(currentSegment);
    }

    // 3. 处理环的首尾连接
    // 检查第一段和最后一段是否可以合并
    if (boundarySegments.size() > 1) {
        const RegionBoundaryEdgeInfo& lastEdgeOfLastSegment = boundarySegments.back().back();
        const RegionBoundaryEdgeInfo& firstEdgeOfFirstSegment = boundarySegments.front().front();

        double angle = CalculateEdgeAngle(lastEdgeOfLastSegment,firstEdgeOfFirstSegment);

        if (angle <= m_angleThreshold) {
            // 可以合并首尾段
            std::vector<RegionBoundaryEdgeInfo> mergedSegment = boundarySegments.back();
            mergedSegment.insert(mergedSegment.end(),
                boundarySegments.front().begin(),
                boundarySegments.front().end());

            boundarySegments.erase(boundarySegments.begin());
            boundarySegments.back() = mergedSegment;
        }
    }

    // 4. 转换格式存储到region的boundaryGroups中
    for (const auto& segment : boundarySegments) {
        region.boundaryGroups.push_back(segment);
    }

    //// 5. 选择主边界
    //if (!SelectMainBoundary(region)) {
    //    return false;
    //}
    region.boundaryAnalyzed = true;
    return true;
}

std::vector<RegionBoundaryEdgeInfo> RegionCurveBuilder::OrderBoundaryEdges(
    const std::vector<Edge*>& boundaryEdges)
{
    std::vector<RegionBoundaryEdgeInfo> orderedEdges;

    if (boundaryEdges.empty()) {
        return orderedEdges;
    }

    // 创建所有边的信息
    std::vector<RegionBoundaryEdgeInfo> edgeInfos;
    edgeInfos.reserve(boundaryEdges.size());
    for (Edge* edge : boundaryEdges) {
        edgeInfos.emplace_back(edge);
    }

    // 使用标记数组跟踪已处理的边
    std::vector<bool> used(edgeInfos.size(), false);

    // 从第一条边开始
    orderedEdges.push_back(edgeInfos[0]);
    used[0] = true;

    const double tolerance = 0.1;  // 0.01mm

    // 记录当前边未使用的端点（即下一条边应该连接的点）
    Point3d currentConnectPoint = edgeInfos[0].endpoint;  // 假设从终点开始连接

    // 连接剩余的边
    while (orderedEdges.size() < edgeInfos.size()) {
        bool found = false;

        for (size_t i = 0; i < edgeInfos.size(); i++) {
            if (used[i]) continue;

            const RegionBoundaryEdgeInfo& edge = edgeInfos[i];

            // 检查起点是否与当前连接点相连
            if (PointDistance(currentConnectPoint, edge.startpoint) < tolerance) {
                orderedEdges.push_back(edge);
                currentConnectPoint = edge.endpoint;  // 下一个连接点是该边的终点
                used[i] = true;
                found = true;
                break;
            }
            // 检查终点是否与当前连接点相连
            else if (PointDistance(currentConnectPoint, edge.endpoint) < tolerance) {
                orderedEdges.push_back(edge);
                currentConnectPoint = edge.startpoint;  // 下一个连接点是该边的起点
                used[i] = true;
                found = true;
                break;
            }
        }

        if (!found) {
            // 如果没找到，尝试从另一端连接（处理第一条边选择错误的情况）
            currentConnectPoint = orderedEdges[0].startpoint;

            for (size_t i = 0; i < edgeInfos.size(); i++) {
                if (used[i]) continue;

                const RegionBoundaryEdgeInfo& edge = edgeInfos[i];

                if (PointDistance(currentConnectPoint, edge.startpoint) < tolerance) {
                    orderedEdges.insert(orderedEdges.begin(), edge);
                    currentConnectPoint = edge.endpoint;
                    used[i] = true;
                    found = true;
                    break;
                }
                else if (PointDistance(currentConnectPoint, edge.endpoint) < tolerance) {
                    orderedEdges.insert(orderedEdges.begin(), edge);
                    currentConnectPoint = edge.startpoint;
                    used[i] = true;
                    found = true;
                    break;
                }
            }
        }

        if (!found) {
            // 真的找不到连接的边了
            break;
        }
    }

    return orderedEdges;
}

bool RegionCurveBuilder::UpdateRegionBoundaryEdges(Region& region)
{
    // 清空现有边界边
    region.boundaryEdges.clear();

    // 检查region是否有有效的body
    if (!region.body) {
        return false;
    }

    // 获取body的tag
    tag_t body_tag = region.body->Tag();

    // UF函数输出参数
    int num_boundaries = 0;
    int* num_edges = nullptr;
    tag_t* edge_tags = nullptr;

    // 调用UF函数获取边界信息
    int error = UF_MODL_ask_body_boundaries(
        body_tag,
        &num_boundaries,
        &num_edges,
        &edge_tags
    );

    if (error != 0 || num_boundaries == 0) {
        // 错误或没有边界
        if (num_edges) UF_free(num_edges);
        if (edge_tags) UF_free(edge_tags);
        return false;
    }

    // 将tag转换为Edge对象

    int edge_index = 0;

    for (int i = 0; i < num_boundaries; i++) {
        for (int j = 0; j < num_edges[i]; j++) {
            try {
                // 将tag转换为NXOpen::Edge对象
                NXOpen::TaggedObject* taggedObj =
                    NXOpen::NXObjectManager::Get(edge_tags[edge_index]);
                NXOpen::Edge* edge = dynamic_cast<NXOpen::Edge*>(taggedObj);

                if (edge) {
                    region.boundaryEdges.push_back(edge);
                }
            }
            catch (...) {
                // 忽略无效的edge tag
            }
            edge_index++;
        }
    }

    // 释放UF函数分配的内存
    UF_free(num_edges);
    UF_free(edge_tags);

    // 更新环形区域标志（如果有多个边界环）
    if (num_boundaries >= 2) {
        region.isRing = true;
    }

    return !region.boundaryEdges.empty();
}

RegionCurveBuilder::RegionCurveBuilder()
{
    m_sprayDiameter = 80;
    m_overlapRatio = 0.1;
    m_angleThreshold = 65 *  PI / 180;

    // 初始化边界覆盖参数（基于重叠率的统一模型）
    // 步距 = 直径 × (1 - 重叠率)
    m_stepDistance = m_sprayDiameter * (1.0 - m_overlapRatio);
    // 边界偏移 = 步距/2（确保端点强度与路径中间一致）
    m_maxBoundaryOffset = m_stepDistance / 2.0;
    // 过渡区上限 = 2倍步距（数学推导保证）
    m_transitionThreshold = 2 * m_stepDistance;
}

RegionCurveBuilder::~RegionCurveBuilder()
{

}

int RegionCurveBuilder::GetBoundaryGroupCount( Region& region)
{
    return static_cast<int>(region.boundaryGroups.size());
}

std::vector<RegionBoundaryEdgeInfo> RegionCurveBuilder::GetBoundaryGroup(
     Region& region, int index) 
{
    // 检查索引有效性
    if (index < 0 || index >= static_cast<int>(region.boundaryGroups.size())) {
        // 返回空vector
        return std::vector<RegionBoundaryEdgeInfo>();
    }

    return region.boundaryGroups[index];
}

bool RegionCurveBuilder::AnalyzeRingBoundaries(Region& region)
{
    if (region.boundaryAnalyzed && !region.boundaryGroups.empty()) {
        return true;
    }


    // 清除之前的分析结果
    region.boundaryGroups.clear();
    UpdateRegionBoundaryEdges(region);

    // 只处理环形区域
    if (!region.isRing) {
        return false;
    }

    // 获取边界边
    if (region.boundaryEdges.empty()) {
        return false;
    }

    // 为每条边创建信息
    std::vector<RegionBoundaryEdgeInfo> edgeInfos;
    for (Edge* edge : region.boundaryEdges) {
        edgeInfos.emplace_back(edge);
    }

    // 标记已使用的边
    std::vector<bool> used(edgeInfos.size(), false);
    const double tolerance = 0.01;

    // 组装所有独立的环
    for (size_t startIdx = 0; startIdx < edgeInfos.size(); startIdx++) {
        if (used[startIdx]) continue;

        // 开始新的环
        std::vector<RegionBoundaryEdgeInfo> currentLoop;
        std::vector<size_t> loopIndices;  // 记录环中边的索引

        // 使用简单的贪心算法组装环
        loopIndices.push_back(startIdx);
        used[startIdx] = true;

        Point3d currentEnd = edgeInfos[startIdx].endpoint;
        Point3d loopStart = edgeInfos[startIdx].startpoint;

        // 继续连接直到回到起点
        while (PointDistance(currentEnd, loopStart) > tolerance) {
            bool found = false;

            for (size_t i = 0; i < edgeInfos.size(); i++) {
                if (used[i]) continue;

                // 检查连接
                if (PointDistance(currentEnd, edgeInfos[i].startpoint) < tolerance ||
                    PointDistance(currentEnd, edgeInfos[i].endpoint) < tolerance) {

                    loopIndices.push_back(i);
                    used[i] = true;

                    // 更新当前端点
                    if (PointDistance(currentEnd, edgeInfos[i].startpoint) < tolerance) {
                        currentEnd = edgeInfos[i].endpoint;
                    }
                    else {
                        currentEnd = edgeInfos[i].startpoint;
                    }

                    found = true;
                    break;
                }
            }

            if (!found) break;  // 无法继续连接
        }

        // 将索引转换为边信息
        if (loopIndices.size() > 2) {  // 至少3条边才能形成有效环
            for (size_t idx : loopIndices) {
                currentLoop.push_back(edgeInfos[idx]);
            }
            region.boundaryGroups.push_back(currentLoop);
        }
    }

    // 环形区域至少有2个环
    return region.boundaryGroups.size() >= 2;
}

double RegionCurveBuilder::CalculateEdgeAngle(RegionBoundaryEdgeInfo edge1, RegionBoundaryEdgeInfo edge2)
{
    const double tolerance = 0.1;

    // 准备 UF 函数需要的 double 数组
    double vec_from[3]; // 前一条边的切向
    double vec_to[3];   // 后一条边的切向
    double vec_ccw[3] = { 0.0, 0.0, 1.0 }; // 参考法向 Z轴
    double angle_rad = 0.0;

    // ==========================================
    // 步骤 1: 确定连接顺序并修正向量方向
    // 目标：将切向量统一为“沿路径前进”的方向，并存入 double 数组
    // ==========================================

    // 情况A: End -> Start (标准顺接)
    if (PointDistance(edge1.endpoint, edge2.startpoint) < tolerance) {
        vec_from[0] = edge1.end_tangent.X;
        vec_from[1] = edge1.end_tangent.Y;
        vec_from[2] = edge1.end_tangent.Z;

        vec_to[0] = edge2.start_tangent.X;
        vec_to[1] = edge2.start_tangent.Y;
        vec_to[2] = edge2.start_tangent.Z;
    }
    // 情况B: End -> End (Edge2 反向)
    else if (PointDistance(edge1.endpoint, edge2.endpoint) < tolerance) {
        vec_from[0] = edge1.end_tangent.X;
        vec_from[1] = edge1.end_tangent.Y;
        vec_from[2] = edge1.end_tangent.Z;

        // Edge2反向
        vec_to[0] = -edge2.end_tangent.X;
        vec_to[1] = -edge2.end_tangent.Y;
        vec_to[2] = -edge2.end_tangent.Z;
    }
    // 情况C: Start -> Start (Edge1 反向)
    else if (PointDistance(edge1.startpoint, edge2.startpoint) < tolerance) {
        // Edge1反向
        vec_from[0] = -edge1.start_tangent.X;
        vec_from[1] = -edge1.start_tangent.Y;
        vec_from[2] = -edge1.start_tangent.Z;

        vec_to[0] = edge2.start_tangent.X;
        vec_to[1] = edge2.start_tangent.Y;
        vec_to[2] = edge2.start_tangent.Z;
    }
    // 情况D: Start -> End (Edge1 反向, Edge2 反向)
    else if (PointDistance(edge1.startpoint, edge2.endpoint) < tolerance) {
        // Edge1反向
        vec_from[0] = -edge1.start_tangent.X;
        vec_from[1] = -edge1.start_tangent.Y;
        vec_from[2] = -edge1.start_tangent.Z;

        // Edge2反向
        vec_to[0] = -edge2.end_tangent.X;
        vec_to[1] = -edge2.end_tangent.Y;
        vec_to[2] = -edge2.end_tangent.Z;
    }
    else {
        return 180.0; // 未连接，返回最大角度以确保断开
    }

    // ==========================================
    // 步骤 2: 调用 UF_VEC3_angle_between
    // 返回值 angle_rad 范围是 0 到 2*PI (0 到 360度)
    // ==========================================
    UF_VEC3_angle_between(vec_from, vec_to, vec_ccw, &angle_rad);

    // ==========================================
    // 步骤 3: 转换与归一化
    // 目标：将 0-360 的方向角转换为 0-180 的“弯曲程度”
    // ==========================================

    // 弧度转角度
    double angle_deg = angle_rad * (180.0 / PI);

    // 逻辑修正：
    // UF_VEC3_angle_between 返回的是逆时针角度。
    // 直行 = 0度
    // 左转90度 = 90度
    // 右转90度 = 270度
    // 右转10度 = 350度

    // 我们用来判断分段的阈值通常是绝对偏转量（例如 > 65度）。
    // 所以我们需要把 > 180 的角度折叠回来。
    if (angle_deg > 180.0) {
        angle_deg = 360.0 - angle_deg;
    }

    // 现在 angle_deg 代表了偏离直线的绝对角度 (0-180)
    // 90度左转 -> 90
    // 90度右转 (270) -> 360-270 = 90
    // 10度右转 (350) -> 360-350 = 10 (不会误触阈值)

    return angle_deg;
}

std::vector<Point3d> RegionCurveBuilder::ExtractPointsFromBoundaryGroup(
    const std::vector<RegionBoundaryEdgeInfo>& boundaryGroup,
    int pointsPerEdge)
{
    std::vector<Point3d> allPoints;
    const double tolerance = 0.01;

    if (boundaryGroup.empty()) {
        return allPoints;
    }

    // 如果只有一条边，默认从0到1
    if (boundaryGroup.size() == 1) {
        const auto& edge = boundaryGroup[0];
        for (int i = 0; i < pointsPerEdge; i++) {
            double parameter = static_cast<double>(i) / (pointsPerEdge - 1);
            double point[3];
            ask_curve_point(edge.boundarycurvetag, parameter, point);
            allPoints.emplace_back(point[0], point[1], point[2]);
        }
        return allPoints;
    }

    // 确定第一条边的方向
    const auto& firstEdge = boundaryGroup[0];
    const auto& secondEdge = boundaryGroup[1];
    bool firstEdgeReverse = false;
    Point3d currentConnectPoint;

    // 判断第一条边与第二条边的连接方式，确定第一条边的方向
    if (PointDistance(firstEdge.endpoint, secondEdge.startpoint) < tolerance) {
        // 第一条边终点 → 第二条边起点：第一条边0→1方向
        firstEdgeReverse = false;
        currentConnectPoint = firstEdge.endpoint;
    }
    else if (PointDistance(firstEdge.endpoint, secondEdge.endpoint) < tolerance) {
        // 第一条边终点 → 第二条边终点：第一条边0→1方向
        firstEdgeReverse = false;
        currentConnectPoint = firstEdge.endpoint;
    }
    else if (PointDistance(firstEdge.startpoint, secondEdge.startpoint) < tolerance) {
        // 第一条边起点 → 第二条边起点：第一条边1→0方向
        firstEdgeReverse = true;
        currentConnectPoint = firstEdge.startpoint;
    }
    else if (PointDistance(firstEdge.startpoint, secondEdge.endpoint) < tolerance) {
        // 第一条边起点 → 第二条边终点：第一条边1→0方向
        firstEdgeReverse = true;
        currentConnectPoint = firstEdge.startpoint;
    }
    else {
        // 连接异常，使用默认方向
        firstEdgeReverse = false;
        currentConnectPoint = firstEdge.endpoint;
    }

    // 第一条边取点
    for (int i = 0; i < pointsPerEdge; i++) {
        double parameter;
        if (firstEdgeReverse) {
            parameter = 1.0 - static_cast<double>(i) / (pointsPerEdge - 1); // 从1到0
        }
        else {
            parameter = static_cast<double>(i) / (pointsPerEdge - 1); // 从0到1
        }

        double point[3];
        ask_curve_point(firstEdge.boundarycurvetag, parameter, point);
        allPoints.emplace_back(point[0], point[1], point[2]);
    }

    // 处理后续边
    for (size_t i = 1; i < boundaryGroup.size(); i++) {
        const auto& currentEdge = boundaryGroup[i];
        bool reverseDirection = false;

        double edgeLength = ask_curve_length(0.0, 1.0, currentEdge.boundarycurvetag);
        if (edgeLength < m_tinyEdgeThreshold)
        {
            continue;
        }
        // 判断连接方式，决定取点方向
        if (PointDistance(currentConnectPoint, currentEdge.startpoint) < tolerance) {
            // 标准连接：上一条边终点 → 当前边起点
            reverseDirection = false; // 从0到1取点
            currentConnectPoint = currentEdge.endpoint; // 更新连接点为当前边终点
        }
        else if (PointDistance(currentConnectPoint, currentEdge.endpoint) < tolerance) {
            // 反向连接：上一条边终点 → 当前边终点
            reverseDirection = true; // 从1到0取点
            currentConnectPoint = currentEdge.startpoint; // 更新连接点为当前边起点
        }
        else {
            // 连接异常，尝试其他连接方式或使用默认方向
            reverseDirection = false;
            currentConnectPoint = currentEdge.endpoint;
        }

        // 根据方向取点（跳过第一个点避免重复）
        for (int j = 1; j < pointsPerEdge; j++) {
            double parameter;
            if (reverseDirection) {
                parameter = 1.0 - static_cast<double>(j) / (pointsPerEdge - 1); // 从1到0
            }
            else {
                parameter = static_cast<double>(j) / (pointsPerEdge - 1); // 从0到1
            }

            double point[3];
            ask_curve_point(currentEdge.boundarycurvetag, parameter, point);
            allPoints.emplace_back(point[0], point[1], point[2]);
        }
    }

    return allPoints;
}

bool RegionCurveBuilder::GenerateSplineFromBoundaryGroup(
    const std::vector<RegionBoundaryEdgeInfo>& boundaryGroup,
    int pointsPerEdge,
    tag_t* splineTag)
{
    if (boundaryGroup.empty()) {
        return false;
    }
    // 过滤小特征边
    //std::vector<RegionBoundaryEdgeInfo> filteredBoundaryGroup =
    //    FilterSmallFeatureEdges(boundaryGroup);


    // 提取所有点
    std::vector<Point3d> points = ExtractPointsFromBoundaryGroup(boundaryGroup, pointsPerEdge);

    if (points.size() < 2) {
        return false;
    }

    // 将Point3d数组转换为连续的double数组
    std::vector<double> pointArray;
    pointArray.reserve(points.size() * 3);

    for (const auto& pt : points) {
        pointArray.push_back(pt.X);
        pointArray.push_back(pt.Y);
        pointArray.push_back(pt.Z);
    }

    tag_t spline_id;

    // 修正：使用正确的参数调用样条曲线创建函数
    int result = PDCAPP_COM_create_spline_f_pt(
        pointArray.data(),                    // 点数组
        static_cast<int>(points.size()),      // 点数量  
        0,                                    // start_tang = 0
        NULL,                                 // tangent1 = NULL
        0,                                    // end_tang = 0
        NULL,                                 // tangent2 = NULL
        &spline_id                           // 输出样条曲线ID
    );

    if (splineTag) {
        *splineTag = spline_id;
    }

    // 将样条曲线放到第88图层


    return (result == 0); // 假设0表示成功
}

std::vector<Point3d> RegionCurveBuilder::GenerateControlPoints(tag_t splineTag)
{
    std::vector<Point3d> controlPoints;

    if (splineTag == NULL_TAG) {
        return controlPoints;
    }

    double stepDistance = m_sprayDiameter * (1.0 - m_overlapRatio) /4 ;
  
    double totalLength = 0.0;
    int status = UF_CURVE_ask_arc_length(
        splineTag,
        0.0,      // 起始参数
        1.0,      // 终止参数
        UF_MODL_UNITS_PART,  // 使用部件单位
        &totalLength
    );

    if (status != 0 || totalLength <= 0.0) {
        return controlPoints;
    }

   
    int numPoints = static_cast<int>(totalLength / stepDistance) + 1;

    
    if (numPoints < 2) {
        numPoints = 2;
    }

    
    double actualStepDistance = totalLength / (numPoints - 1);

    
    for (int i = 0; i < numPoints-1; i++) {
        double currentLength = i * actualStepDistance;

       
        double parameter = 0.0;
        int result = PDCAPP_COM_ask_param_to_arclength(
            splineTag,
            currentLength,
            &parameter
        );

        if (result == 0) {
            double point[3];
            ask_curve_point(splineTag, parameter, point);
            controlPoints.emplace_back(point[0], point[1], point[2]);
        }
    }



    return controlPoints;
}

void RegionCurveBuilder::GenerateAllControlPointsForRegion(Region& region, int pointsPerEdge)
{
    // 清空之前的数据
    region.controlPointGroups.clear();
    region.splineTags.clear();

    // 确保边界已分析
    if (!region.boundaryAnalyzed) {
        AnalyzeBoundaries(region);
    }

    // 为每个边界组生成样条曲线和控制点
    for (const auto& boundaryGroup : region.boundaryGroups) {
        tag_t splineTag = NULL_TAG;

        // 生成样条曲线
        bool success = GenerateSplineFromBoundaryGroup(
            boundaryGroup,
            pointsPerEdge,
            &splineTag
        );

        if (success && splineTag != NULL_TAG) {
            // 保存样条曲线tag
            region.splineTags.push_back(splineTag);

            // 生成控制点
            std::vector<Point3d> controlPoints = GenerateControlPoints(splineTag);
            region.controlPointGroups.push_back(controlPoints);
        }
        else {
            // 如果失败，添加空数据以保持索引对应
            region.splineTags.push_back(NULL_TAG);
            region.controlPointGroups.push_back(std::vector<Point3d>());
        }
    }
}

void RegionCurveBuilder::CalculateStepDistance()
{
    m_stepDistance = m_sprayDiameter * (1.0 - m_overlapRatio);
}

// 辅助函数:根据点数和偏移生成点位序列(长度)
// 返回值为每个点在曲线上的弧长位置
// 点位生成函数（支持对称和非对称offset）
// endOffset默认-1表示对称模式（使用startOffset作为两端offset）
std::vector<double> GeneratePointPositions(
    int numPoints,
    double startOffset,
    double totalLength,
    bool reverseDirection,
    double endOffset = -1.0)
{
    // 如果未指定endOffset，使用对称模式
    if (endOffset < 0) {
        endOffset = startOffset;
    }

    std::vector<double> lengths;

    if (numPoints == 1) {
        // 单点情况:取中点
        lengths.push_back(totalLength / 2.0);
    }
    else {
        // 多点情况:起点用startOffset，终点用endOffset，中间均匀分布
        double step = (totalLength - startOffset - endOffset) / (numPoints - 1);

        for (int i = 0; i < numPoints; i++) {
            double pos = startOffset + i * step;
            lengths.push_back(pos);
        }
    }

    // 根据方向反转
    if (reverseDirection) {
        std::reverse(lengths.begin(), lengths.end());
    }

    return lengths;
}

// ========== 多候选优化算法（暴力枚举版）==========
// 修订版：基于物理覆盖约束的四级分级策略 + 三层暴力枚举
// 核心目标：步距稳定性优先（s'接近s），offset次要（接近s/4）
std::vector<double> OptimizedMultiCandidateAlgorithm(
    double totalLength,
    double stepDistance,
    bool reverseDirection)
{
    const double s = stepDistance;

    // 候选配置结构体（支持非对称offset）
    struct CandidateConfig {
        int numPoints;
        double startOffset;
        double endOffset;
        double actualStep;
        double score;
    };

    // 计算理想点数（四舍五入）
    int idealN = static_cast<int>(round(totalLength / s));
    if (idealN < 2) idealN = 2;

    // 点数候选：从idealN开始 → idealN-1 → idealN+1
    std::vector<int> nCandidates = {idealN, idealN - 1, idealN + 1};

    // ========== 定义四级约束级别 ==========
    struct ConstraintLevel {
        double minStep, maxStep;
        double minOffset, maxOffset;
        std::string levelName;  // 用于调试日志
    };

    std::vector<ConstraintLevel> levels = {
        // 严格级：中心化覆盖
        {STRICT_MIN_STEP_RATIO * s, STRICT_MAX_STEP_RATIO * s,
         STRICT_MIN_OFFSET_RATIO * s, STRICT_MAX_OFFSET_RATIO * s, "Strict"},

        // 宽松级1：优先放开offset下限（允许贴近边界）
        {STRICT_MIN_STEP_RATIO * s, STRICT_MAX_STEP_RATIO * s,
         RELAXED1_MIN_OFFSET_RATIO * s, STRICT_MAX_OFFSET_RATIO * s, "Relaxed1"},

        // 宽松级2：同时放宽步距下限
        {RELAXED2_MIN_STEP_RATIO * s, STRICT_MAX_STEP_RATIO * s,
         RELAXED2_MIN_OFFSET_RATIO * s, STRICT_MAX_OFFSET_RATIO * s, "Relaxed2"},

        // 终极回退：保证可解性
        {FALLBACK_MIN_STEP_RATIO * s, FALLBACK_MAX_STEP_RATIO * s,
         RELAXED2_MIN_OFFSET_RATIO * s, STRICT_MAX_OFFSET_RATIO * s, "Fallback"}
    };

    // ========== 逐级尝试（找到解立即返回）==========
    for (const auto& level : levels) {
        std::vector<CandidateConfig> candidates;

        // 三层暴力枚举
        for (int n : nCandidates) {
            if (n < 2) continue;

            // 层1：遍历步距（从大到小，优先接近s）
            for (double s_prime = level.maxStep;
                 s_prime >= level.minStep;
                 s_prime -= STEP_SEARCH_PRECISION * s) {

                double totalOffsetNeeded = totalLength - (n - 1) * s_prime;

                // 层2：遍历起始offset（从小到大，优先接近s/4）
                for (double startOffset = level.minOffset;
                     startOffset <= level.maxOffset;
                     startOffset += OFFSET_SEARCH_PRECISION * s) {

                    // 层3：通过约束推导endOffset
                    double endOffset = totalOffsetNeeded - startOffset;

                    // 检查endOffset约束
                    if (endOffset < level.minOffset || endOffset > level.maxOffset) {
                        continue;
                    }

                    // ========== 评分计算（步距优先）==========
                    double stepError = std::abs(s_prime - s) / s;
                    double offsetError = (std::abs(startOffset - s/4) +
                                         std::abs(endOffset - s/4)) / (s/4);
                    double score = WEIGHT_STEP * stepError +
                                  WEIGHT_OFFSET * offsetError;

                    candidates.push_back({n, startOffset, endOffset, s_prime, score});
                }
            }
        }

        // 如果本级找到解，选择最优
        if (!candidates.empty()) {
            auto best = std::min_element(candidates.begin(), candidates.end(),
                [](const CandidateConfig& a, const CandidateConfig& b) {
                    return a.score < b.score;
                });

            return GeneratePointPositions(
                best->numPoints,
                best->startOffset,
                totalLength,
                reverseDirection,
                best->endOffset
            );
        }
    }

    // ========== 终极回退：均匀分布 ==========
    int fallbackN = idealN;
    double fallbackOffset = totalLength / (2 * fallbackN);
    return GeneratePointPositions(fallbackN, fallbackOffset, totalLength, reverseDirection);
}

// 辅助函数：在截线上按三段式自适应取点(参数化优化版本)
// 段1 (≤stepDistance): 单点中心覆盖
// 段2 (stepDistance ~ 2.2×stepDistance): 强制2点三等分
// 段3 (>2.2×stepDistance): 多候选优化算法
std::vector<Point3d> RegionCurveBuilder::GetPointsOnIntersectionCurve(
    tag_t curve,
    const Point3d& referencePoint)
{
    std::vector<Point3d> points;
    if (curve == NULL_TAG) {
        return points;
    }

    // ===== 首尾方向判断（放在最开始，优先于所有取点操作）=====
    double startPoint[3], endPoint[3];
    // 获取曲线起点（参数0.0）和终点（参数1.0）
    ask_curve_point(curve, 0.0, startPoint);
    ask_curve_point(curve, 1.0, endPoint);
    Point3d curveStart(startPoint[0], startPoint[1], startPoint[2]);
    Point3d curveEnd(endPoint[0], endPoint[1], endPoint[2]);
    // 计算参考点到首尾的距离，判断方向
    double distToStart = PointDistance(referencePoint, curveStart);
    double distToEnd = PointDistance(referencePoint, curveEnd);
    bool reverseDirection = (distToEnd < distToStart);  // 终点更近则反向

    // 确保步距已计算
    if (m_stepDistance <= 0) {
        CalculateStepDistance();
    }

    // 获取曲线总长度
    double totalLength = 0.0;
    int status = UF_CURVE_ask_arc_length(
        curve, 0.0, 1.0, UF_MODL_UNITS_PART, &totalLength
    );

    if (status != 0 || totalLength <= 0.0) {
        return points;
    }

    // ========== 日志记录：打开文件 ==========
#ifdef _MYDEBUG
    std::ofstream logFile("E:\\vsproject\\SandSprayProject\\IntersectionCurve_Log.txt", std::ios::app);
#endif // _DEBUG

   

    // ========== 参数化阈值定义 ==========
    const double SUPER_SHORT_THRESHOLD = m_stepDistance;        // 超短线段上限
    const double SHORT_SEGMENT_THRESHOLD = 2.2* m_stepDistance; // 三等分策略上限

    std::vector<double> lengths;  // 点的弧长位置序列
    std::string stage;  // 记录使用的阶段

    // ===== 第1段：超短曲线（≤stepDistance），取中点 =====
    if (totalLength <= SUPER_SHORT_THRESHOLD) {
        lengths.push_back(totalLength / 2.0);
        stage = "Stage1(SuperShort)";
    }
    // ===== 第2段：短线段（s < L ≤ 2.2s），固定两点在L/4和3L/4 =====
    else if (totalLength <= SHORT_SEGMENT_THRESHOLD) {
        double offset = totalLength / 4.0;  // L/4
        lengths = GeneratePointPositions(2, offset, totalLength, reverseDirection, offset);
        stage = "Stage2(Short)";
    }
    // ===== 第3段：正常线段（>2.2×stepDistance），多候选优化算法 =====
    else {
        lengths = OptimizedMultiCandidateAlgorithm(totalLength, m_stepDistance, reverseDirection);
        stage = "Stage3(Normal)";
    }

    // ========== 根据弧长序列生成实际3D点 ==========
    for (double len : lengths) {
        double parameter = 0.0;
        int result = PDCAPP_COM_ask_param_to_arclength(curve, len, &parameter);
        if (result == 0) {
            double point[3];
            ask_curve_point(curve, parameter, point);
            points.emplace_back(point[0], point[1], point[2]);
        }
    }

    // ========== 日志记录：输出取点信息 ==========
#ifdef _MYDEBUG
    if (logFile.is_open()) {
        logFile << stage << " | Length=" << std::fixed << std::setprecision(2) << totalLength
            << " | StepDist=" << std::fixed << std::setprecision(2) << m_stepDistance
            << " | Points=" << points.size() << " | Spacings: ";

        // 计算并记录每对相邻点之间的实际距离
        if (points.size() > 1) {
            for (size_t i = 1; i < points.size(); i++) {
                double spacing = PointDistance(points[i - 1], points[i]);
                logFile << std::fixed << std::setprecision(2) << spacing;
                if (i < points.size() - 1) {
                    logFile << ", ";
                }
            }
        }
        else if (points.size() == 1) {
            logFile << "N/A(SinglePoint)";
        }

        logFile << "\n";
        logFile.close();
    }
#endif // !_DEBUG



    return points;
}

// 针对平面环形区域：在相交曲线两端点连成的直线上按三段式自适应取点
// 段1 (≤stepdistance): 单点中心覆盖
// 段2 (<2.2*stepditance): 强制2点三等分
// 段3 (>2.2*stepdistance): 自适应均匀分布
std::vector<Point3d> RegionCurveBuilder::GetPointsOnLineForPlanarRingRegion(
    tag_t curve,
    const Point3d& controlPoint)
{
    std::vector<Point3d> points;

    if (curve == NULL_TAG) {
        return points;
    }

    // 确保步距已计算
    if (m_stepDistance <= 0) {
        CalculateStepDistance();
    }

    // 1. 获取相交曲线的两个端点
    double startPoint[3], endPoint[3];
    ask_curve_point(curve, 0.0, startPoint);
    ask_curve_point(curve, 1.0, endPoint);

    Point3d curveStart(startPoint[0], startPoint[1], startPoint[2]);
    Point3d curveEnd(endPoint[0], endPoint[1], endPoint[2]);

    // 2. 判断哪个端点靠近controlPoint
    double distToStart = PointDistance(controlPoint, curveStart);
    double distToEnd = PointDistance(controlPoint, curveEnd);

    // 确定起点和终点
    Point3d lineStart, lineEnd;
    if (distToStart < distToEnd) {
        lineStart = curveStart;
        lineEnd = curveEnd;
    }
    else {
        lineStart = curveEnd;
        lineEnd = curveStart;
    }

    // 3. 计算直线的总长度
    double totalLength = PointDistance(lineStart, lineEnd);

    // ========== 日志记录：打开文件 ==========
#ifdef _MYDEBUG
    std::ofstream logFile("E:\\vsproject\\SandSprayProject\\PlanarRingRegion_Log.txt", std::ios::app);
#endif // _DEBUG

    

    // 4. 计算方向向量（归一化）
    Vector3d direction;
    direction.X = lineEnd.X - lineStart.X;
    direction.Y = lineEnd.Y - lineStart.Y;
    direction.Z = lineEnd.Z - lineStart.Z;

    double dirLength = std::sqrt(direction.X * direction.X +
                                  direction.Y * direction.Y +
                                  direction.Z * direction.Z);

    if (dirLength > 0) {
        direction.X /= dirLength;
        direction.Y /= dirLength;
        direction.Z /= dirLength;
    }

    double lowerThreshold = m_stepDistance;  //

    // ===== 第1段：超短直线），取中点 =====
    if (totalLength <= lowerThreshold) {
        Point3d midPoint;
        midPoint.X = (lineStart.X + lineEnd.X) / 2.0;
        midPoint.Y = (lineStart.Y + lineEnd.Y) / 2.0;
        midPoint.Z = (lineStart.Z + lineEnd.Z) / 2.0;
        points.push_back(midPoint);
        return points;
    }

    // ===== 第2段：短线段（s < L ≤ 2.2s），固定两点在L/4和3L/4 =====
    if (totalLength <= 2.2 * m_stepDistance) {
        double offset = totalLength / 4.0;  // L/4

        // 第1个点（位于L/4处）
        Point3d point1;
        point1.X = lineStart.X + direction.X * offset;
        point1.Y = lineStart.Y + direction.Y * offset;
        point1.Z = lineStart.Z + direction.Z * offset;
        points.push_back(point1);

        // 第2个点（位于3L/4处）
        Point3d point2;
        point2.X = lineStart.X + direction.X * (3.0 * offset);
        point2.Y = lineStart.Y + direction.Y * (3.0 * offset);
        point2.Z = lineStart.Z + direction.Z * (3.0 * offset);
        points.push_back(point2);

        return points;
    }

    // ===== 第3段：正常区间（>2.2*stepdistance），多候选优化算法 =====
    // 判断方向：哪个端点更靠近controlPoint
    double distToStart1 = PointDistance(controlPoint, lineStart);
    double distToEnd1 = PointDistance(controlPoint, lineEnd);
    bool reverseDirection = (distToEnd1 < distToStart1);  // 终点更近则反向

    // 调用优化算法获取点位序列（弧长位置）
    std::vector<double> lengths = OptimizedMultiCandidateAlgorithm(
        totalLength,
        m_stepDistance,
        reverseDirection
    );

    // 根据弧长位置生成实际3D点（沿直线分布）
    for (double len : lengths) {
        Point3d currentPoint;
        currentPoint.X = lineStart.X + direction.X * len;
        currentPoint.Y = lineStart.Y + direction.Y * len;
        currentPoint.Z = lineStart.Z + direction.Z * len;
        points.push_back(currentPoint);
    }

    // 日志记录（仅第3段：正常区间）
#ifdef _MYDEBUG
    if (logFile.is_open()) {
        logFile << "Length=" << std::fixed << std::setprecision(2) << totalLength
            << " | Points=" << points.size() << " | Spacings: ";

        // 计算并记录每对相邻点之间的实际距离
        for (size_t i = 1; i < points.size(); i++) {
            double spacing = PointDistance(points[i - 1], points[i]);
            logFile << std::fixed << std::setprecision(2) << spacing;
            if (i < points.size() - 1) {
                logFile << ", ";
            }
        }
        logFile << "\n";
        logFile.close();
    }
#endif // _DEBUG

   

    return points;
}


tag_t RegionCurveBuilder::get_intersect(tag_t body, DatumPlane* dplane, double ref_point[3])
{
    try
    {
        NXOpen::Session* theSession = NXOpen::Session::GetSession();
        NXOpen::Part* workPart(theSession->Parts()->Work());
        NXOpen::Part* displayPart(theSession->Parts()->Display());


        NXOpen::Features::Feature* nullNXOpen_Features_Feature(NULL);
        NXOpen::Features::SectionCurveBuilder* sectionCurveBuilder1;
        sectionCurveBuilder1 = workPart->Features()->CreateSectionCurveBuilder(nullNXOpen_Features_Feature);

        sectionCurveBuilder1->CurveFitData()->SetTolerance(0.08);
        sectionCurveBuilder1->CurveFitData()->SetAngleTolerance(0.5);
        sectionCurveBuilder1->SetChordalTolerance(0.01);
        sectionCurveBuilder1->SetAssociative(false);
        sectionCurveBuilder1->CurveFitData()->SetCurveJoinMethod(NXOpen::GeometricUtilities::CurveFitData::JoinGeneral);

        TaggedObject* bodytag = NXObjectManager::Get(body);

        NXOpen::Body* body1(dynamic_cast<NXOpen::Body*>(bodytag));
        bool added1;
        added1 = sectionCurveBuilder1->ObjectsToSection()->Add(body1);
        bool added2;
        added2 = sectionCurveBuilder1->SectionPlanes()->Add(dplane);


        NXOpen::NXObject* nXObject1;
        nXObject1 = sectionCurveBuilder1->Commit();

        std::vector<NXOpen::NXObject*> objects1;
        objects1 = sectionCurveBuilder1->GetCommittedObjects();
       

        sectionCurveBuilder1->Destroy();
        theSession->CleanUpFacetedFacesAndEdges();


        Point3d ref(ref_point[0], ref_point[1], ref_point[2]);
        NXObject* returncurve = nullptr;
        double min = ask_min_dis(objects1[0],ref);
        for (auto intcurve:objects1 )
        {
            double dis = ask_min_dis(intcurve, ref);
            if (dis<=min)
            {
                returncurve = intcurve;
                min = dis;
            }
            
        }
        for (auto intcurve2 : objects1)
        {
            if (intcurve2->Tag() == returncurve->Tag()) 
            {
                continue;
            }
            else
            {
                UF_OBJ_delete_object(intcurve2->Tag());
            }
        }
        return returncurve->Tag();

    }
    catch (exception& ex)
    {
        //---- Enter your exception handling code here -----
       // HS_SP_point::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
        Point3d aaa(ref_point[0], ref_point[1], ref_point[2]);
        tag_t icurves = NULL_TAG;
        return icurves;
    }


}

void RegionCurveBuilder::CreateDatumPlane(Point3d org, Vector3d dir, tag_t& dplane)
{
    NXOpen::Session* theSession = NXOpen::Session::GetSession();
    NXOpen::Part* workPart(theSession->Parts()->Work());
    NXOpen::Part* displayPart(theSession->Parts()->Display());

    NXOpen::Features::Feature* nullNXOpen_Features_Feature(NULL);
    NXOpen::Features::DatumPlaneBuilder* datumPlaneBuilder1;
    datumPlaneBuilder1 = workPart->Features()->CreateDatumPlaneBuilder(nullNXOpen_Features_Feature);

    Point* orgPoint;
    orgPoint = workPart->Points()->CreatePoint(org);

    Direction* direction;
    direction = workPart->Directions()->CreateDirection(org, dir, SmartObject::UpdateOptionWithinModeling);

    datumPlaneBuilder1->SetPointAndDirection(orgPoint, direction);

    datumPlaneBuilder1->SetResizeDuringUpdate(true);

    NXOpen::Features::Feature* feature1;
    feature1 = datumPlaneBuilder1->CommitFeature();

    NXOpen::Features::DatumPlaneFeature* datumPlaneFeature1(dynamic_cast<NXOpen::Features::DatumPlaneFeature*>(feature1));

    DatumPlane* plane = datumPlaneFeature1->DatumPlane();

    dplane = plane->Tag();

    plane->SetReverseSection(false);
    datumPlaneBuilder1->Destroy();

}

tag_t RegionCurveBuilder::CreateSplineFromPoints(const std::vector<Point3d>& points)
{
    if (points.size() < 2) {
        return NULL_TAG;
    }
    tag_t splineTag = NULL_TAG;
    if ( points.size() == 2) {
        double start_point[3] = { points[0].X, points[0].Y, points[0].Z };
        double end_point[3] = { points[1].X, points[1].Y, points[1].Z };
        UF_CURVE_line_s lines;
        lines.start_point[0] = points[0].X;
        lines.start_point[1] = points[0].Y;
        lines.start_point[2] = points[0].Z;

        lines.end_point[0] = points[1].X;
        lines.end_point[1] = points[1].Y;
        lines.end_point[2] = points[1].Z;

        UF_CURVE_create_line(&lines, &splineTag);

        return splineTag;
    }

    // 转换点格式
    std::vector<double> pointArray;
    pointArray.reserve(points.size() * 3);

    for (const auto& pt : points) {
        pointArray.push_back(pt.X);
        pointArray.push_back(pt.Y);
        pointArray.push_back(pt.Z);
    }

   

    // 创建样条曲线（不带切向约束）
    int result = PDCAPP_COM_create_spline_f_pt(
        pointArray.data(),
        static_cast<int>(points.size()),
        0,      // 无起始切向约束
        NULL,   // 无起始切向量
        0,      // 无终止切向约束
        NULL,   // 无终止切向量
        &splineTag
    );



    return splineTag;
}

bool RegionCurveBuilder::FilterSmallFeatureEdges(
    const std::vector<RegionBoundaryEdgeInfo>& orderedBoundary, int i)
{
    
    if (orderedBoundary.size() < 3) {
        return true;
    }

    // 特征角度阈值（略小于分组阈值）
    double featureAngleThreshold = m_angleThreshold * 0.8;  // 65度 * 0.8 = 52度

    // 遍历所有边

        const RegionBoundaryEdgeInfo& currentEdge = orderedBoundary[i];

        // 计算当前边的长度
        double edgeLength = ask_curve_length(0.0, 1.0, currentEdge.boundarycurvetag);

        // 级别1：极小边直接过滤
        if (edgeLength < m_tinyEdgeThreshold) {
            return false;
        }

        //// 级别2：小边智能过滤
        //if (edgeLength < m_smallEdgeThreshold) {
        //    // 获取相邻边索引（处理循环边界）
        //    size_t prevIdx = (i == 0) ? orderedBoundary.size() - 1 : i - 1;
        //    size_t nextIdx = (i == orderedBoundary.size() - 1) ? 0 : i + 1;

        //    // 计算与前一条边的夹角
        //    double angleToPrev = CalculateEdgeAngle(
        //        orderedBoundary[prevIdx],
        //        currentEdge
        //    );

        //    // 计算与后一条边的夹角
        //    double angleToNext = CalculateEdgeAngle(
        //        currentEdge,
        //        orderedBoundary[nextIdx]
        //    );

        //    // 判断是否为突出的小特征
        //    if (angleToPrev > featureAngleThreshold && angleToNext > featureAngleThreshold) {

        //     
        //    }

        //    // 如果只有一个夹角很大，可能是转角处的必要边，保留
        //    // 如果两个夹角都较小，说明是主轮廓的一部分，保留
        /*}*/



    return true;
}

void RegionCurveBuilder::UpdateParametersFromUI(
    double sprayDiameter,
    double overlapRatio,
    double smallEdgeThreshold,
    double tinyEdgeThreshold)
{
    // 更新喷砂参数
    SetSprayDiameter(sprayDiameter);
    SetOverlapRatio(overlapRatio);

    // 更新边界过滤参数
    m_smallEdgeThreshold = smallEdgeThreshold;
    m_tinyEdgeThreshold = tinyEdgeThreshold;

    // 重新计算步距
    CalculateStepDistance();

    // 重新计算边界覆盖参数（基于重叠率的统一模型）
    // 边界偏移 = 步距/2（确保端点强度与路径中间一致）
    m_maxBoundaryOffset = m_stepDistance / 2.0;
    // 过渡区上限 = 2倍步距
    m_transitionThreshold = 2 * m_stepDistance;
}

// 辅助函数：尝试合并断开的路径段
// 通过检查段之间的4种连接方式（头尾、尾头、头头、尾尾），找到最优连接并合并
std::vector<std::vector<Point3d>> RegionCurveBuilder::MergePathSegments(
    const std::vector<std::vector<Point3d>>& segments,
    double connectThreshold)
{
    if (segments.size() <= 1) {
        return segments;  // 只有0或1段，无需合并
    }

    std::vector<std::vector<Point3d>> result = segments;
    bool merged = true;

    // 迭代合并，直到无法再合并为止
    while (merged) {
        merged = false;

        // 遍历所有段对
        for (size_t i = 0; i < result.size() && !merged; i++) {
            for (size_t j = i + 1; j < result.size() && !merged; j++) {
                auto& segA = result[i];
                auto& segB = result[j];

                if (segA.empty() || segB.empty()) continue;

                // 检查4种连接方式的距离
                double dist_A尾B头 = PointDistance(segA.back(), segB.front());
                double dist_A尾B尾 = PointDistance(segA.back(), segB.back());
                double dist_A头B头 = PointDistance(segA.front(), segB.front());
                double dist_A头B尾 = PointDistance(segA.front(), segB.back());

                // 找到最短距离
                double minDist = std::min({dist_A尾B头, dist_A尾B尾, dist_A头B头, dist_A头B尾});

                // 如果最短距离小于阈值，执行合并
                if (minDist < connectThreshold) {
                    if (minDist == dist_A尾B头) {
                        // A尾 → B头：A + B
                        segA.insert(segA.end(), segB.begin(), segB.end());
                    }
                    else if (minDist == dist_A尾B尾) {
                        // A尾 → B尾：A + 反转B
                        segA.insert(segA.end(), segB.rbegin(), segB.rend());
                    }
                    else if (minDist == dist_A头B头) {
                        // A头 → B头：反转A + B
                        std::reverse(segA.begin(), segA.end());
                        segA.insert(segA.end(), segB.begin(), segB.end());
                    }
                    else { // dist_A头B尾
                        // A头 → B尾：B + A
                        segB.insert(segB.end(), segA.begin(), segA.end());
                        segA = segB;
                    }

                    // 删除已合并的段B
                    result.erase(result.begin() + j);
                    merged = true;  // 标记已合并，继续下一轮
                }
            }
        }
    }

    return result;
}

std::vector<std::vector<Point3d>> RegionCurveBuilder::ConnectPathPointsWithRingSupport(
    const std::vector<std::vector<Point3d>>& intersectionPointsMatrix)
{
    std::vector<std::vector<Point3d>> pathSegments;  // 返回结果：所有路径段

    if (intersectionPointsMatrix.empty()) {
        return pathSegments;
    }

    // 设置连接阈值(基于喷砂直径)
    double connectThreshold = m_stepDistance ;

    // 找出最大的点数（用于确定中间层的数量）
    size_t maxPoints = 0;
    for (const auto& points : intersectionPointsMatrix) {
        maxPoints = std::max(maxPoints, points.size());
    }

    // ========== 阶段1：首边界路径（每条相交曲线的第一个点）==========
    {
        std::vector<std::vector<Point3d>> firstBoundarySegments;
        std::vector<Point3d> currentPath;

        for (size_t controlIdx = 0; controlIdx < intersectionPointsMatrix.size(); controlIdx++) {
            // 每条曲线至少有一个点，取第一个点
            if (!intersectionPointsMatrix[controlIdx].empty()) {
                Point3d firstPoint = intersectionPointsMatrix[controlIdx][0];

                if (currentPath.empty()) {
                    currentPath.push_back(firstPoint);
                }
                else {
                    Point3d lastPoint = currentPath.back();
                    double distance = PointDistance(lastPoint, firstPoint);

                    if (distance < connectThreshold) {
                        currentPath.push_back(firstPoint);
                    }
                    else {
                        // 距离超过阈值，断开
                        if (currentPath.size() >= 2) {
                            firstBoundarySegments.push_back(currentPath);
                        }
                        currentPath.clear();
                        currentPath.push_back(firstPoint);
                    }
                }
            }
        }

        // 保存最后一段路径
        if (currentPath.size() >= 2) {
            firstBoundarySegments.push_back(currentPath);
        }

        // 合并首边界内的断开段
        firstBoundarySegments = MergePathSegments(firstBoundarySegments, connectThreshold);

        // 添加到最终结果
        pathSegments.insert(pathSegments.end(), firstBoundarySegments.begin(), firstBoundarySegments.end());
    }

    // ========== 阶段2：末边界路径（每条相交曲线的最后一个点，排除单点曲线）==========
    {
        std::vector<std::vector<Point3d>> lastBoundarySegments;
        std::vector<Point3d> currentPath;

        for (size_t controlIdx = 0; controlIdx < intersectionPointsMatrix.size(); controlIdx++) {
            const auto& curvePoints = intersectionPointsMatrix[controlIdx];

            // 只处理有多个点的曲线（排除单点曲线，因为单点曲线的唯一点已经在首边界处理过）
            if (curvePoints.size() > 1) {
                Point3d lastPoint = curvePoints[curvePoints.size() - 1];

                if (currentPath.empty()) {
                    currentPath.push_back(lastPoint);
                }
                else {
                    Point3d prevPoint = currentPath.back();
                    double distance = PointDistance(prevPoint, lastPoint);

                    if (distance < connectThreshold) {
                        currentPath.push_back(lastPoint);
                    }
                    else {
                        // 距离超过阈值，断开
                        if (currentPath.size() >= 2) {
                            lastBoundarySegments.push_back(currentPath);
                        }
                        currentPath.clear();
                        currentPath.push_back(lastPoint);
                    }
                }
            }
        }

        // 保存最后一段路径
        if (currentPath.size() >= 2) {
            lastBoundarySegments.push_back(currentPath);
        }

        // 合并末边界内的断开段
        lastBoundarySegments = MergePathSegments(lastBoundarySegments, connectThreshold);

        // 添加到最终结果
        pathSegments.insert(pathSegments.end(), lastBoundarySegments.begin(), lastBoundarySegments.end());
    }

    // ========== 阶段3：中间层路径（排除首末点的所有中间点）==========
    // 从第1层到第maxPoints-2层（索引1到maxPoints-2）
    for (size_t layerIdx = 1; layerIdx + 1 < maxPoints; layerIdx++) {
        std::vector<std::vector<Point3d>> middleLayerSegments;
        std::vector<Point3d> currentPath;

        for (size_t controlIdx = 0; controlIdx < intersectionPointsMatrix.size(); controlIdx++) {
            const auto& curvePoints = intersectionPointsMatrix[controlIdx];

            // 检查该点是否存在，且不是该曲线的首点或末点
            if (layerIdx < curvePoints.size()) {
                // 确保该点不是曲线的第一个点或最后一个点
                bool isFirstPoint = (layerIdx == 0);
                bool isLastPoint = (layerIdx == curvePoints.size() - 1);

                if (!isFirstPoint && !isLastPoint) {
                    Point3d middlePoint = curvePoints[layerIdx];

                    if (currentPath.empty()) {
                        currentPath.push_back(middlePoint);
                    }
                    else {
                        Point3d lastPoint = currentPath.back();
                        double distance = PointDistance(lastPoint, middlePoint);

                        if (distance < connectThreshold) {
                            currentPath.push_back(middlePoint);
                        }
                        else {
                            // 距离超过阈值，断开
                            if (currentPath.size() >= 2) {
                                middleLayerSegments.push_back(currentPath);
                            }
                            currentPath.clear();
                            currentPath.push_back(middlePoint);
                        }
                    }
                }
                else {
                    // 遇到首点或末点（该曲线长度导致），断开当前路径
                    if (currentPath.size() >= 2) {
                        middleLayerSegments.push_back(currentPath);
                    }
                    currentPath.clear();
                }
            }
            else {
                // 该曲线没有该层的点，断开当前路径
                if (currentPath.size() >= 2) {
                    middleLayerSegments.push_back(currentPath);
                }
                currentPath.clear();
            }
        }

        // 保存最后一段路径
        if (currentPath.size() >= 2) {
            middleLayerSegments.push_back(currentPath);
        }

        // 合并当前中间层内的断开段
        middleLayerSegments = MergePathSegments(middleLayerSegments, connectThreshold);

        // 添加到最终结果
        pathSegments.insert(pathSegments.end(), middleLayerSegments.begin(), middleLayerSegments.end());
    }

    return pathSegments;
}



bool RegionCurveBuilder::CheckCurvesIntersect(tag_t curve1, tag_t curve2, double tolerance)
{
    // 1. 检测两条曲线的最小距离
    double guess[3] = { 0, 0, 0 };
    double distance;
    double pt1[3], pt2[3];

    int error = UF_MODL_ask_minimum_dist(
        curve1, curve2,
        0, guess,
        0, guess,
        &distance, pt1, pt2
    );

    if (error != 0 || distance >= tolerance) {
        return false;
    }

    // 2. 检查是否是端点相交
    const double endpointTolerance = 0.5;

    double curve1_start[3], curve1_end[3];
    ask_curve_point(curve1, 0.0, curve1_start);
    ask_curve_point(curve1, 1.0, curve1_end);

    double curve2_start[3], curve2_end[3];
    ask_curve_point(curve2, 0.0, curve2_start);
    ask_curve_point(curve2, 1.0, curve2_end);

    Point3d p1(pt1[0], pt1[1], pt1[2]);
    Point3d p2(pt2[0], pt2[1], pt2[2]);

    Point3d c1_start(curve1_start[0], curve1_start[1], curve1_start[2]);
    Point3d c1_end(curve1_end[0], curve1_end[1], curve1_end[2]);
    Point3d c2_start(curve2_start[0], curve2_start[1], curve2_start[2]);
    Point3d c2_end(curve2_end[0], curve2_end[1], curve2_end[2]);

    bool pt1_is_endpoint =
        (PointDistance(p1, c1_start) < endpointTolerance) ||
        (PointDistance(p1, c1_end) < endpointTolerance);

    bool pt2_is_endpoint =
        (PointDistance(p2, c2_start) < endpointTolerance) ||
        (PointDistance(p2, c2_end) < endpointTolerance);

    if (pt1_is_endpoint || pt2_is_endpoint) {
        return false;
    }

    return true;
}


Vector3d RegionCurveBuilder::AverageNormals(const Vector3d& n1, const Vector3d& n2)
{
    Vector3d avg;
    avg.X = (n1.X + n2.X) / 2.0;
    avg.Y = (n1.Y + n2.Y) / 2.0;
    avg.Z = (n1.Z + n2.Z) / 2.0;

    double length = sqrt(avg.X * avg.X + avg.Y * avg.Y + avg.Z * avg.Z);
    if (length > 1e-6) {
        avg.X /= length;
        avg.Y /= length;
        avg.Z /= length;
    }
    else {
        avg.X = 0.0;
        avg.Y = 1.0;
        avg.Z = 0.0;
    }

    return avg;
}

int RegionCurveBuilder::FindNonIntersectingPrev(
    const std::vector<tag_t>& curves,
    int currentIdx,
    bool isRing)
{
    int n = curves.size();

    if (currentIdx == 0) {
        if (isRing) {
            for (int i = n - 1; i >= 0; i--) {
                if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
                    return i;
                }
            }
        }
        else {
            if (n > 1 && !CheckCurvesIntersect(curves[currentIdx], curves[1])) {
                return 1;
            }
            if (n > 2 && !CheckCurvesIntersect(curves[currentIdx], curves[2])) {
                return 2;
            }
        }
        return -1;
    }

    for (int i = currentIdx - 1; i >= 0; i--) {
        if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
            return i;
        }
    }

    if (isRing) {
        for (int i = n - 1; i > currentIdx; i--) {
            if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
                return i;
            }
        }
    }

    return -1;
}

int RegionCurveBuilder::FindNonIntersectingNext(
    const std::vector<tag_t>& curves,
    int currentIdx,
    bool isRing)
{
    int n = curves.size();

    if (currentIdx == n - 1) {
        if (isRing) {
            for (int i = 0; i < n; i++) {
                if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
                    return i;
                }
            }
        }
        else {
            if (n > 1 && !CheckCurvesIntersect(curves[currentIdx], curves[n - 2])) {
                return n - 2;
            }
            if (n > 2 && !CheckCurvesIntersect(curves[currentIdx], curves[n - 3])) {
                return n - 3;
            }
        }
        return -1;
    }

    for (int i = currentIdx + 1; i < n; i++) {
        if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
            return i;
        }
    }

    if (isRing) {
        for (int i = 0; i < currentIdx; i++) {
            if (!CheckCurvesIntersect(curves[currentIdx], curves[i])) {
                return i;
            }
        }
    }

    return -1;
}

bool RegionCurveBuilder::CheckIntersectionWithNeighbors(
    const std::vector<tag_t>& curves,
    int currentIdx,
    bool isRing)
{
    int n = curves.size();
    if (n < 2) return false;

    tag_t currentCurve = curves[currentIdx];

    int prevIdx = -1;
    if (currentIdx > 0) {
        prevIdx = currentIdx - 1;
    }
    else if (isRing) {
        prevIdx = n - 1;
    }

    if (prevIdx >= 0) {
        if (CheckCurvesIntersect(currentCurve, curves[prevIdx])) {
            return true;
        }
    }

    int nextIdx = -1;
    if (currentIdx < n - 1) {
        nextIdx = currentIdx + 1;
    }
    else if (isRing) {
        nextIdx = 0;
    }

    if (nextIdx >= 0) {
        if (CheckCurvesIntersect(currentCurve, curves[nextIdx])) {
            return true;
        }
    }

    return false;
}


bool RegionCurveBuilder::RegenerateIntersectionCurve(
    IntersectionCurveData& curveData,
    const Vector3d& newNormal,
    tag_t bodyTag)
{
    try {
        if (curveData.datumPlaneTag != NULL_TAG) {
            UF_OBJ_delete_object(curveData.datumPlaneTag);
            curveData.datumPlaneTag = NULL_TAG;
        }
        if (curveData.curveTag != NULL_TAG) {
            UF_OBJ_delete_object(curveData.curveTag);
            curveData.curveTag = NULL_TAG;
        }

        tag_t newDatumPlane = NULL_TAG;
        CreateDatumPlane(curveData.controlPoint, newNormal, newDatumPlane);

        if (newDatumPlane == NULL_TAG) {
            return false;
        }

        double refPoint[3] = {
            curveData.controlPoint.X,
            curveData.controlPoint.Y,
            curveData.controlPoint.Z
        };

        DatumPlane* dplane = dynamic_cast<DatumPlane*>(
            NXObjectManager::Get(newDatumPlane));
        tag_t newCurve = get_intersect(bodyTag, dplane, refPoint);

        if (newCurve == NULL_TAG) {
            UF_OBJ_delete_object(newDatumPlane);
            return false;
        }

        curveData.curveTag = newCurve;
        curveData.datumPlaneTag = newDatumPlane;
        curveData.planeNormal = newNormal;

        return true;
    }
    catch (const std::exception&) {
        return false;
    }
}


Vector3d RegionCurveBuilder::GetNormalAtIndex(
    const std::vector<IntersectionCurveData>& curveDataList,
    int index)
{
    if (index >= 0 && index < curveDataList.size()) {
        return curveDataList[index].planeNormal;
    }

    return Vector3d(0.0, 1.0, 0.0);
}



void RegionCurveBuilder::SetPathCurveAttributes(
    tag_t curveTag,
    tag_t regionTag,
    FaceRegionType regionType,
    int pathIndex)
{
    if (curveTag == NULL_TAG) {
        return;
    }

    try {
        NXOpen::TaggedObject* taggedObj = NXOpen::NXObjectManager::Get(curveTag);
        NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(taggedObj);

        if (nxObj == nullptr) {
            return;
        }

        // 1. 标记这是喷砂路径
        nxObj->SetAttribute("IsSprayPath", 1, NXOpen::Update::Option::OptionNow);

        // 2. 记录所属区域的Body Tag
        nxObj->SetAttribute("RegionTag", static_cast<int>(regionTag), NXOpen::Update::Option::OptionNow);

        // 3. 记录区域类型（使用整数）
        int typeValue = static_cast<int>(regionType);
        nxObj->SetAttribute("RegionType", typeValue, NXOpen::Update::Option::OptionNow);

        // 4. 记录路径索引（在该区域中的第几条路径）
        nxObj->SetAttribute("PathIndex", pathIndex, NXOpen::Update::Option::OptionNow);

        // 5. 添加字符串属性，便于识别
        std::string regionTypeName;
        switch (regionType) {
        case FaceRegionType::HORIZONTAL:
            regionTypeName = "Horizontal";
            break;
        case FaceRegionType::CONVEX:
            regionTypeName = "Convex";
            break;
        case FaceRegionType::CONCAVE:
            regionTypeName = "Concave";
            break;
        case FaceRegionType::SIDEWALL:
            regionTypeName = "Sidewall";
            break;
        case FaceRegionType::UNSORTED:
            regionTypeName = "Unsorted";
            break;
        default:
            regionTypeName = "Unknown";
            break;
        }

        nxObj->SetAttribute("RegionTypeName", regionTypeName.c_str(),
            NXOpen::Update::Option::OptionNow);

        // 6. 记录步距（喷砂工艺参数）
        nxObj->SetAttribute("StepDistance", m_stepDistance,
            NXOpen::Update::Option::OptionNow);

        // 7. 记录其他喷砂参数（可选，便于追溯）
        nxObj->SetAttribute("SprayDiameter", m_sprayDiameter,
            NXOpen::Update::Option::OptionNow);

        nxObj->SetAttribute("OverlapRatio", m_overlapRatio,
            NXOpen::Update::Option::OptionNow);

    }
    catch (const std::exception& ex) {
        // 使用 NX 的消息框显示错误
        NXOpen::UI* theUI = NXOpen::UI::GetUI();

        std::stringstream errorMsg;
        errorMsg << "设置路径属性失败！\n";
        errorMsg << "================\n";
        errorMsg << "曲线Tag: " << curveTag << "\n";
        errorMsg << "区域TAG: " << regionTag << "\n";
        errorMsg << "路径索引: " << pathIndex << "\n";
        errorMsg << "错误信息: " << ex.what() << "\n";
        errorMsg << "================\n";

        theUI->NXMessageBox()->Show("属性设置错误",
            NXOpen::NXMessageBox::DialogTypeError,
            errorMsg.str().c_str());
    }
}


bool RegionCurveBuilder::GenerateSprayPathForRegion(Region& region)
{
    // 清空之前的路径数据
    region.sprayPathPoints.clear();
    region.sprayPathCurves.clear();

    // 确保步距已计算
    CalculateStepDistance();

    // 确保有控制点数据
    if (region.controlPointGroups.empty() || region.splineTags.empty()) {
        GenerateAllControlPointsForRegion(region, 10);
    }

    // 对每个边界组处理
    for (size_t groupIdx = 0; groupIdx < region.controlPointGroups.size(); groupIdx++) {
        const std::vector<Point3d>& controlPoints = region.controlPointGroups[groupIdx];
        tag_t splineTag = region.splineTags[groupIdx];

        if (controlPoints.empty() || splineTag == NULL_TAG) {
            continue;
        }

        // ============ 第一阶段：生成所有初始相交曲线 ============
        std::vector<IntersectionCurveData> curveDataList;
        curveDataList.resize(controlPoints.size());

        for (size_t i = 0; i < controlPoints.size(); i++) {
            Point3d controlPoint = controlPoints[i];

            // 获取该点处的切向量
            double parameter = 0.0;
            double point[3] = { controlPoint.X, controlPoint.Y, controlPoint.Z };
            ask_curve_point_parm(splineTag, point, parameter);

            double tangent[3];
            ask_curve_tangent(splineTag, parameter, tangent);

            
            Vector3d normal;
            normal.X = tangent[0];
            normal.Y = tangent[1];
            normal.Z = 0.0;

            // 归一化
            double length = std::sqrt(normal.X * normal.X + normal.Y * normal.Y);
            if (length > 1e-6) {
                normal.X /= length;
                normal.Y /= length;
            }
            else {
                normal.X = 1.0;
                normal.Y = 0.0;
                normal.Z = 0.0;
            }

            // 创建基准平面
            tag_t datumPlane = NULL_TAG;
            CreateDatumPlane(controlPoint, normal, datumPlane);

            if (datumPlane == NULL_TAG) {
                continue;
            }

            // 获取截线
            double refPoint[3] = { controlPoint.X, controlPoint.Y, controlPoint.Z };
            DatumPlane* dplane = dynamic_cast<DatumPlane*>(NXObjectManager::Get(datumPlane));
            tag_t intersectionCurve = get_intersect(region.body->Tag(), dplane, refPoint);

            // 保存到数据结构中
            curveDataList[i].curveTag = intersectionCurve;
            curveDataList[i].datumPlaneTag = datumPlane;
            curveDataList[i].planeNormal = normal;
            curveDataList[i].controlPoint = controlPoint;
            curveDataList[i].controlPointIndex = i;
            curveDataList[i].intersectsWithNeighbor = false;
        }

        // ============ 第二阶段：迭代检测和调整相交曲线 ============
        int maxIterations = 100;
        for (int iter = 0; iter < maxIterations; iter++) {

            // 提取当前所有曲线tag用于检测
            std::vector<tag_t> currentCurves;
            for (const auto& data : curveDataList) {
                currentCurves.push_back(data.curveTag);
            }

            // 检测每条曲线是否与前后邻居相交
            bool hasIntersection = false;
            for (size_t i = 0; i < curveDataList.size(); i++) {
                if (curveDataList[i].curveTag == NULL_TAG) {
                    curveDataList[i].intersectsWithNeighbor = false;
                    continue;
                }

                bool intersects = CheckIntersectionWithNeighbors(
                    currentCurves,
                    i,
                    region.isRing
                );

                curveDataList[i].intersectsWithNeighbor = intersects;
                if (intersects) {
                    hasIntersection = true;
                }
            }

            // 如果没有相交了，退出迭代
            if (!hasIntersection) {
                break;
            }

            // 调整相交的曲线
            for (size_t i = 0; i < curveDataList.size(); i++) {
                if (!curveDataList[i].intersectsWithNeighbor) {
                    continue;  // 不相交，跳过
                }

                // 查找前后不相交的邻居
                int prevIdx = FindNonIntersectingPrev(currentCurves, i, region.isRing);
                int nextIdx = FindNonIntersectingNext(currentCurves, i, region.isRing);

                // 如果找不到不相交的邻居，跳过此曲线
                if (prevIdx < 0 || nextIdx < 0) {
                    continue;
                }

                // 计算新的法向量（两个不相交邻居的法向量平均）
                Vector3d prevNormal = GetNormalAtIndex(curveDataList, prevIdx);
                Vector3d nextNormal = GetNormalAtIndex(curveDataList, nextIdx);
                Vector3d newNormal = AverageNormals(prevNormal, nextNormal);

                // 使用新法向量重新生成截平面和相交曲线
                bool success = RegenerateIntersectionCurve(
                    curveDataList[i],
                    newNormal,
                    region.body->Tag()
                );

                if (!success) {
                    // 如果重新生成失败，恢复原法向量
                    // (当前已经删除了旧的，无法恢复，只能标记为失败)
                    curveDataList[i].curveTag = NULL_TAG;
                }
            }
        }

        // ============ 第三阶段：在优化后的相交曲线上取点 ============
        std::vector<std::vector<Point3d>> intersectionPointsMatrix;
        intersectionPointsMatrix.resize(controlPoints.size());

        // 判断是否为平面环形区域
        //bool isPlanarRingRegion = (region.type == FaceRegionType::HORIZONTAL && region.isRing);
        bool isFilletRegion = (region.type == FaceRegionType::CONVEX || region.type == FaceRegionType::CONCAVE);
        for (size_t i = 0; i < curveDataList.size(); i++) {
            if (curveDataList[i].curveTag != NULL_TAG) {
                std::vector<Point3d> pointsOnCurve;

                if (!isFilletRegion) {
                    // 平面环形区域：在相交曲线两端点连成的直线上取点
                    pointsOnCurve = GetPointsOnLineForPlanarRingRegion(
                        curveDataList[i].curveTag,
                        curveDataList[i].controlPoint
                    );
                }
                else {
                    // 其他区域：在截线上按固定步距取点
                    pointsOnCurve = GetPointsOnIntersectionCurve(
                        curveDataList[i].curveTag,
                        curveDataList[i].controlPoint
                    );
                }

                intersectionPointsMatrix[i] = pointsOnCurve;

                // 清理基准平面（曲线保留用于后续处理）
                if (curveDataList[i].datumPlaneTag != NULL_TAG) {
                    UF_OBJ_delete_object(curveDataList[i].datumPlaneTag);
                }
            }
            else {
                intersectionPointsMatrix[i] = std::vector<Point3d>();
            }
        }

        // ============ 第四阶段：智能连接交点形成路径 ============
        std::vector<std::vector<Point3d>> pathSegments = ConnectPathPointsWithRingSupport(
            intersectionPointsMatrix
        );

        // ============ 第五阶段：将路径段转换为曲线并保存 ============
        for (const auto& pathSegment : pathSegments) {
            if (pathSegment.size() >= 2) {
                // 保存点序列
                region.sprayPathPoints.push_back(pathSegment);

                // 创建样条曲线
                tag_t pathCurve = CreateSplineFromPoints(pathSegment);
                //光顺曲线


                if (pathCurve != NULL_TAG) {
                    UF_OBJ_set_layer(pathCurve, 82);
                }
                if (pathCurve != NULL_TAG) {
                    region.sprayPathCurves.push_back(pathCurve);
                }
            }
        }

        // ============ 第六阶段：设置所有路径曲线的属性 ============
        for (size_t i = 0; i < region.sprayPathCurves.size(); i++) {
            SetPathCurveAttributes(
                region.sprayPathCurves[i],
                region.body->Tag(),
                region.type,
                static_cast<int>(i)
            );
        }  
    }
    if (true)
    {
        for (size_t i = 2; i < region.sprayPathCurves.size(); i++)
        {
            SmoothSpline(region.sprayPathCurves[i], 8);
        }
        for (size_t i = 0; i < 2; i++)
        {
            SmoothSpline(region.sprayPathCurves[i], 8);
        }
    }



    return !region.sprayPathPoints.empty();
}


void RegionCurveBuilder::CleanupNonPathCurves()
{
    const int LAYER_NUMBER = 81;  // 辅助曲线图层
    std::vector<tag_t> objectsToDelete;  // 存储待删除的对象

    // 第一步：遍历第81图层的所有对象
    tag_t objectTag = NULL_TAG;
    int status = UF_LAYER_cycle_by_layer(LAYER_NUMBER, &objectTag);

    while (objectTag != NULL_TAG && status == 0) {
        // 收集该图层上的所有对象
        objectsToDelete.push_back(objectTag);

        // 继续遍历下一个对象
        status = UF_LAYER_cycle_by_layer(LAYER_NUMBER, &objectTag);
    }

    // 第二步：统一删除所有收集的对象
    if (!objectsToDelete.empty()) {
        for (tag_t& todelete:objectsToDelete)
        {
            UF_OBJ_delete_object(todelete);
        }
    }
}
