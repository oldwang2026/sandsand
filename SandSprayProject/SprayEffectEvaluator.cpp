#include "SprayEffectEvaluator.h"
#include "HelpFunction.h"
#include <uf_layer.h>
#include <uf_obj.h>
#include <uf_object_types.h>
#include <uf_curve.h>
#include <NXOpen/NXObjectManager.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/Part.hxx>
#include <cmath>
#include <uf_disp.h>

using namespace NXOpen;

// ==================== 构造与析构 ====================

SprayEffectEvaluator::SprayEffectEvaluator(const std::vector<tag_t>& sprayPathCurves, const std::vector<NXOpen::Body*>& regionBodies)
    : m_sprayDiameter(80.0)
    , m_facetTolerance(0.5)
    , m_sprayPaths(sprayPathCurves)
{
    // 从第一条曲线读取SprayDiameter属性
    if (!m_sprayPaths.empty() && m_sprayPaths[0] != NULL_TAG) {
        try {
            NXOpen::TaggedObject* taggedObj = NXOpen::NXObjectManager::Get(m_sprayPaths[0]);
            NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(taggedObj);

            if (nxObj != nullptr) {
                // 尝试读取SprayDiameter属性
                try {
                    m_sprayDiameter = nxObj->GetRealAttribute("SprayDiameter");
                }
                catch (...) {
                    // 如果读取失败，使用默认值80.0
                    m_sprayDiameter = 80.0;
                }

                // 尝试读取OverlapRatio属性
                try {
                    m_overlapRatio = nxObj->GetRealAttribute("OverlapRatio");
                }
                catch (...) {
                    // 如果读取失败，使用默认值0.3
                    m_overlapRatio = 0.3;
                }

                // 尝试读取StepDistance属性
                try {
                    m_stepDistance = nxObj->GetRealAttribute("StepDistance");
                }
                catch (...) {
                    // 如果读取失败，根据SprayDiameter和OverlapRatio计算
                    m_stepDistance = m_sprayDiameter * (1.0 - m_overlapRatio);
                }
            }
        }
        catch (...) {
            // 如果转换失败，使用默认值
            m_sprayDiameter = 80.0;
            m_overlapRatio = 0.3;
            m_stepDistance = m_sprayDiameter * (1.0 - m_overlapRatio);  // 80.0 * 0.7 = 56.0
        }
    }

    // 处理传入的片体对象列表
    // SprayEffectEvaluator中的Region使用独立编号系统
    int regionIdCounter = 0;

    for (NXOpen::Body* body : regionBodies) {
        if (body != nullptr) {
            // 从Body对象读取Region类型属性
            FaceRegionType regionType = FaceRegionType::UNSORTED;

            try {
                // 读取RegionType属性
                int typeValue = body->GetIntegerAttribute("RegionType");
                regionType = static_cast<FaceRegionType>(typeValue);
            }
            catch (...) {
                // 如果读取失败，使用默认值UNSORTED
                regionType = FaceRegionType::UNSORTED;
            }

            // 创建Region对象，使用SprayEffectEvaluator内部的独立编号
            Region region(regionIdCounter, regionType);
            region.body = body;
            region.faces = body->GetFaces();
            region.regionobj = body;

            // 对Region进行网格化
            // 调用FacetRegionFaces进行三角网格化
            if (FacetRegionFaces(region)) {
                // 存储Region（不在构造函数中提取三角形和评估）
                m_regions.push_back(region);
                regionIdCounter++;  // 成功添加后递增编号
            }
        }
    }
}

SprayEffectEvaluator::~SprayEffectEvaluator()
{
    // 清除临时点
    ClearTempPoints();

    // 清理所有Region的Facet模型
    for (Region& region : m_regions) {
        if (region.facet_model != NULL_TAG) {
            UF_OBJ_delete_object(region.facet_model);
            region.facet_model = NULL_TAG;
        }
    }
}

void SprayEffectEvaluator::SetSprayDiameter(double diameter)
{
    m_sprayDiameter = diameter;
}

void SprayEffectEvaluator::SetFacetTolerance(double tolerance)
{
    m_facetTolerance = tolerance;
}

void SprayEffectEvaluator::SetSprayPaths(const std::vector<tag_t>& pathCurves)
{
    m_sprayPaths = pathCurves;

    // 如果设置了新的路径，尝试从第一条曲线重新读取SprayDiameter
    if (!m_sprayPaths.empty() && m_sprayPaths[0] != NULL_TAG) {
        try {
            NXOpen::TaggedObject* taggedObj = NXOpen::NXObjectManager::Get(m_sprayPaths[0]);
            NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(taggedObj);

            if (nxObj != nullptr) {
                try {
                    m_sprayDiameter = nxObj->GetRealAttribute("StepDistance");
                }
                catch (...) {
                    // 读取失败时保持当前值
                }
            }
        }
        catch (...) {
            // 转换失败时保持当前值
        }
    }
}

bool SprayEffectEvaluator::EvaluateAllRegions()
{
    // 清除之前的评估数据
    Clear();

    if (m_regions.empty()) {
        return false;
    }

    // 遍历所有region，提取三角形数据
    for (Region& region : m_regions) {
        // 确保region已经网格化
        if (region.facet_model == NULL_TAG) {
            if (!FacetRegionFaces(region)) {
                continue; // 跳过失败的region
            }
        }

        // 提取三角形数据（关联regionId）
        ExtractTrianglesFromFacetModel(region.facet_model, region.id);
    }

    if (m_triangles.empty()) {
        return false;
    }

    // 计算所有三角形的喷射强度（正态分布叠加）
    CalculateIntensitiesForAllTriangles();

    // === 新增：为poor/bad三角形构建邻接图 ===
    BuildAdjacencyMapForBadTriangles();

    // === 新增：BFS聚类分析bad区域 ===
    AnalyzeBadAreaClusters();

    // 根据强度分配颜色
    AssignColorsToTriangles();

    // 更新统计数据
    UpdateStatistics();

    // 绘制所有三角形的边，使用颜色表示评估结果
    DrawAllTriangles();

    return true;
}

std::vector<TriangleEvalData> SprayEffectEvaluator::GetTrianglesByRegionId(int regionId) const
{
    std::vector<TriangleEvalData> regionTriangles;

    // 从所有三角形中过滤出指定region的三角形
    for (const TriangleEvalData& tri : m_triangles) {
        if (tri.regionId == regionId) {
            regionTriangles.push_back(tri);
        }
    }

    return regionTriangles;
}

EvaluationStats SprayEffectEvaluator::GetStatisticsByRegionId(int regionId) const
{
    EvaluationStats regionStats;

    double totalIntensity = 0.0;

    // 统计指定region的三角形
    for (const TriangleEvalData& tri : m_triangles) {
        if (tri.regionId == regionId) {
            regionStats.totalTriangles++;
            totalIntensity += tri.sprayIntensity;

            // 根据强度分级（6级标准）
            if (tri.sprayIntensity >= 1.8) {
                regionStats.excessiveCount++;
            } else if (tri.sprayIntensity >= 0.6) {
                regionStats.excellentCount++;
            } else if (tri.sprayIntensity >= 0.3) {
                regionStats.goodCount++;
            } else if (tri.sprayIntensity >= 0.1) {
                regionStats.fairCount++;
            } else if (tri.sprayIntensity >= 0.01) {
                regionStats.poorCount++;
            } else {
                regionStats.badCount++;
            }
        }
    }

    // 计算平均强度和覆盖率
    if (regionStats.totalTriangles > 0) {
        regionStats.averageIntensity = totalIntensity / regionStats.totalTriangles;
        regionStats.coverageRate = static_cast<double>(regionStats.excellentCount + regionStats.goodCount)
                                  / regionStats.totalTriangles * 100.0;
        regionStats.excessiveRate = static_cast<double>(regionStats.excessiveCount)
                                   / regionStats.totalTriangles * 100.0;
    }

    return regionStats;
}

void SprayEffectEvaluator::Clear()
{
    m_triangles.clear();
    m_stats = EvaluationStats();
}

// ==================== 内部方法 ====================

bool SprayEffectEvaluator::FacetRegionFaces(Region& region)
{
    // 如果region已经有facet模型，先清理
    if (region.facet_model != NULL_TAG) {
        UF_OBJ_delete_object(region.facet_model);
        region.facet_model = NULL_TAG;
    }

    if (!region.body) {
        return false;
    }

    // 设置Facet参数（参考HelpFunction.cpp中的FacetBody）
    UF_FACET_parameters_t faceting_params;

    // 初始化参数结构
    UF_FACET_INIT_PARAMETERS(&faceting_params);
    UF_FACET_ask_default_parameters(&faceting_params);

    // 设置三角形网格参数
    faceting_params.max_facet_edges = 3;  // 三角形
    faceting_params.number_storage_type = 1;  // UF_FACET_TYPE_DOUBLE

    // 表面容差
    faceting_params.specify_surface_tolerance = true;
    faceting_params.surface_dist_tolerance = m_facetTolerance;  // 使用成员变量设置的容差
    faceting_params.surface_angular_tolerance = 0.4;

    // 曲线容差
    faceting_params.specify_curve_tolerance = true;
    faceting_params.curve_dist_tolerance = m_facetTolerance;
    faceting_params.curve_angular_tolerance = 0.4;
    faceting_params.curve_max_length = 10;

    // 参数和视图方向
    faceting_params.specify_parameters = true;
    faceting_params.specify_view_direction = true;
    faceting_params.silh_view_direction[0] = 0;
    faceting_params.silh_view_direction[1] = 0;
    faceting_params.silh_view_direction[2] = 1;
    faceting_params.silh_chord_tolerance = 0.015;

    // 网格大小和凸性
    faceting_params.specify_max_facet_size = true;
    faceting_params.max_facet_size = 10;
    faceting_params.specify_convex_facets = true;

    // 对Body进行facet化
    tag_t faceted_model = NULL_TAG;
    int err = UF_FACET_facet_solid(region.body->Tag(), &faceting_params, &faceted_model);

    if (err != 0) {
        // Facet化失败
        return false;
    }

    // 检查生成的facet数量
    int num_facet = 0;
    UF_FACET_ask_num_faces(faceted_model, &num_facet);

    if (num_facet == 0) {
        // 没有生成任何facet
        if (faceted_model != NULL_TAG) {
            UF_OBJ_delete_object(faceted_model);
        }
        return false;
    }

    // 断开与solid的关联，使facet模型独立
    UF_FACET_disassoc_from_solid(faceted_model);

    // 将facet模型保存到region中
    region.facet_model = faceted_model;

    return true;
}

void SprayEffectEvaluator::ExtractTrianglesFromFacetModel(tag_t facetModel, int regionId)
{
    if (facetModel == NULL_TAG) {
        return;
    }

    // 记录当前三角形数组的起始索引
    int baseIndex = m_triangles.size();
    int localIndex = 0;

    // 直接遍历模型中的所有facet
    int facet_id = UF_FACET_NULL_FACET_ID;
    UF_FACET_cycle_facets(facetModel, &facet_id);

    while (facet_id != UF_FACET_NULL_FACET_ID) {
        TriangleEvalData triData;

        // 获取facet的顶点数量和坐标
        int num_vertices = 0;
        double vertices[3][3]; // 最多3个顶点（三角形）

        UF_FACET_ask_vertices_of_facet(facetModel, facet_id, &num_vertices, vertices);

        // 只处理三角形facet
        if (num_vertices == 3) {
            // 复制顶点坐标
            for (int i = 0; i < 3; i++) {
                triData.vertices[i][0] = vertices[i][0];
                triData.vertices[i][1] = vertices[i][1];
                triData.vertices[i][2] = vertices[i][2];
            }

            // 计算三角形中心点
            triData.center = CalculateTriangleCenter(triData.vertices);

            // 设置所属的Region ID
            triData.regionId = regionId;

            // === 新增：保存facet_id用于构建邻接关系 ===
            triData.facetId = facet_id;

            // 计算当前三角形在m_triangles中的索引
            int currentIndex = baseIndex + localIndex;

            // 添加到列表
            m_triangles.push_back(triData);

            // === 新增：建立facet_id到triangle索引的映射 ===
            m_facetIdToTriangleIndex[regionId][facet_id] = currentIndex;

            localIndex++;
        }

        // 获取下一个facet
        UF_FACET_cycle_facets(facetModel, &facet_id);
    }
}

void SprayEffectEvaluator::CalculateIntensitiesForAllTriangles()
{
    // === 新增：清空之前的poor/bad索引列表 ===
    m_poorBadTriangleIndices.clear();

    // poor/bad阈值（合并poor和bad）
    const double INTENSITY_THRESHOLD = 0.1;

    for (int i = 0; i < m_triangles.size(); i++) {
        TriangleEvalData& tri = m_triangles[i];

        // 计算喷射强度（正态分布叠加）
        tri.sprayIntensity = CalculateSprayIntensity(tri.center);

        // 同时计算最小距离（保留用于调试对比）
        tri.minDistance = CalculateMinDistanceToPath(tri.center);

        // === 新增：收集poor/bad三角形索引 ===
        if (tri.sprayIntensity < INTENSITY_THRESHOLD) {
            m_poorBadTriangleIndices.push_back(i);
        }
    }
}

void SprayEffectEvaluator::AssignColorsToTriangles()
{
    for (TriangleEvalData& tri : m_triangles) {
        tri.colorCode = MapIntensityToColor(tri.sprayIntensity);
    }
}

void SprayEffectEvaluator::UpdateStatistics()
{
    m_stats.totalTriangles = static_cast<int>(m_triangles.size());
    m_stats.excessiveCount = 0;
    m_stats.excellentCount = 0;
    m_stats.goodCount = 0;
    m_stats.fairCount = 0;
    m_stats.poorCount = 0;
    m_stats.badCount = 0;

    double totalIntensity = 0.0;

    for (const TriangleEvalData& tri : m_triangles) {
        totalIntensity += tri.sprayIntensity;

        // 根据强度分级（6级标准）
        if (tri.sprayIntensity >= 1.8) {
            m_stats.excessiveCount++;
        } else if (tri.sprayIntensity >= 0.6) {
            m_stats.excellentCount++;
        } else if (tri.sprayIntensity >= 0.3) {
            m_stats.goodCount++;
        } else if (tri.sprayIntensity >= 0.1) {
            m_stats.fairCount++;
        } else if (tri.sprayIntensity >= 0.01) {
            m_stats.poorCount++;
        } else {
            m_stats.badCount++;
        }
    }

    // 计算平均强度和覆盖率
    if (m_stats.totalTriangles > 0) {
        m_stats.averageIntensity = totalIntensity / m_stats.totalTriangles;
        m_stats.coverageRate = static_cast<double>(m_stats.excellentCount + m_stats.goodCount)
                              / m_stats.totalTriangles * 100.0;
        m_stats.excessiveRate = static_cast<double>(m_stats.excessiveCount)
                               / m_stats.totalTriangles * 100.0;
    }
}

Point3d SprayEffectEvaluator::CalculateTriangleCenter(double vertices[3][3])
{
    Point3d center;
    center.X = (vertices[0][0] + vertices[1][0] + vertices[2][0]) / 3.0;
    center.Y = (vertices[0][1] + vertices[1][1] + vertices[2][1]) / 3.0;
    center.Z = (vertices[0][2] + vertices[1][2] + vertices[2][2]) / 3.0;
    return center;
}

double SprayEffectEvaluator::CalculateMinDistanceToPath(const Point3d& point)
{
    if (m_sprayPaths.empty()) {
        return 1e10; // 返回一个很大的值表示没有路径
    }

    double minDist = 1e10;

    // 遍历所有喷砂路径曲线
    for (tag_t pathTag : m_sprayPaths) {

        double dist = ask_min_dis(pathTag, point);

        if (dist < minDist) {
            minDist = dist;
        }
    }


    return minDist;
}

double SprayEffectEvaluator::CalculateSprayIntensity(const Point3d& point)
{
    if (m_sprayPaths.empty()) {
        return 0.0; // 无路径返回0强度
    }

    // 计算正态分布参数: σ = D/6 (6σ覆盖喷射宽度)
    double sigma = m_sprayDiameter / 6.0;
    double sigma_squared = sigma * sigma;
    double two_sigma_squared = 2.0 * sigma_squared;

    double totalIntensity = 0.0;

    // 遍历所有喷射路径，计算叠加强度
    for (tag_t pathTag : m_sprayPaths) {
        // 计算点到当前路径的最小距离
        double dist = ask_min_dis(pathTag, point);

        // 正态分布强度公式: exp(-d²/(2σ²))
        // 距离为0时强度最大(1.0)，随距离增大呈正态衰减
        double intensity = std::exp(-dist * dist / two_sigma_squared);

        totalIntensity += intensity;
    }

    return totalIntensity;
}

int SprayEffectEvaluator::MapIntensityToColor(double intensity)
{
    // 根据正态分布叠加强度映射颜色代码
    // 6级分级标准（含过度喷射级别）

    if (intensity >= 1.8) {
        return 211; // 紫红色（过度喷射）
    } else if (intensity >= 0.6) {
        return 36;  // 深绿色（优秀）
    } else if (intensity >= 0.3) {
        return 108; // 浅绿色（良好）
    } else if (intensity >= 0.1) {
        return 31;  // 黄色（一般）
    } else if (intensity >= 0.01) {
        return 42;  // 橙色（较差）
    } else {
        return 186; // 红色（差）
    }
}

void SprayEffectEvaluator::DrawTriangleEdges(const TriangleEvalData& triangle)
{
    // 获取三角形的颜色
    int color = triangle.colorCode;

    // 绘制三条边：v0-v1, v1-v2, v2-v0
    CreateTempLine(const_cast<double*>(triangle.vertices[0]),
                   const_cast<double*>(triangle.vertices[1]),
                   color);
    CreateTempLine(const_cast<double*>(triangle.vertices[1]),
                   const_cast<double*>(triangle.vertices[2]),
                   color);
    CreateTempLine(const_cast<double*>(triangle.vertices[2]),
                   const_cast<double*>(triangle.vertices[0]),
                   color);
}

void SprayEffectEvaluator::DrawAllTriangles()
{
    // 遍历所有三角形并绘制它们的边
    for (const TriangleEvalData& tri : m_triangles) {
        DrawTriangleEdges(tri);
    }
}

// ==================== BFS聚类相关方法实现 ====================

void SprayEffectEvaluator::BuildAdjacencyMapForBadTriangles()
{
    // 清空之前的邻接图
    m_triangleAdjacencyMap.clear();

    // poor/bad阈值
    const double INTENSITY_THRESHOLD = 0.1;

    // 只遍历已识别的poor/bad三角形
    for (int i : m_poorBadTriangleIndices) {
        int regionId = m_triangles[i].regionId;
        int facet_id = m_triangles[i].facetId;

        // 获取对应的facet_model
        tag_t facetModel = NULL_TAG;
        for (const Region& r : m_regions) {
            if (r.id == regionId) {
                facetModel = r.facet_model;
                break;
            }
        }

        if (facetModel == NULL_TAG) {
            continue;
        }

        std::vector<int> adjacentIndices;

        // 查询三条边的邻接三角形
        for (int edge_id = 0; edge_id < 3; edge_id++) {
            int adj_facet_id = UF_FACET_NULL_FACET_ID;
            int edge_in_adj = 0;

            UF_FACET_ask_adjacent_facet(facetModel, facet_id, edge_id,
                                        &adj_facet_id, &edge_in_adj);

            if (adj_facet_id != UF_FACET_NULL_FACET_ID) {
                // 使用映射快速查找邻居索引
                auto regionMapIt = m_facetIdToTriangleIndex.find(regionId);
                if (regionMapIt != m_facetIdToTriangleIndex.end()) {
                    auto facetMapIt = regionMapIt->second.find(adj_facet_id);
                    if (facetMapIt != regionMapIt->second.end()) {
                        int neighborIdx = facetMapIt->second;

                        // === 关键：只记录也是poor/bad的邻居 ===
                        if (m_triangles[neighborIdx].sprayIntensity < INTENSITY_THRESHOLD) {
                            adjacentIndices.push_back(neighborIdx);
                        }
                        // good三角形直接跳过，不记录
                    }
                }
            }
        }

        m_triangleAdjacencyMap[i] = adjacentIndices;
    }
}

std::vector<int> SprayEffectEvaluator::GetAdjacentTriangles(int triangleIndex)
{
    auto it = m_triangleAdjacencyMap.find(triangleIndex);
    if (it != m_triangleAdjacencyMap.end()) {
        return it->second;
    }
    return std::vector<int>();  // 无邻接或无效索引
}

BadAreaCluster SprayEffectEvaluator::ClusterBadAreaBFS(
    int seedIndex,
    std::unordered_set<int>& visited)
{
    BadAreaCluster cluster;
    std::queue<int> queue;

    // 初始化BFS
    queue.push(seedIndex);
    visited.insert(seedIndex);

    // 从种子三角形获取regionId
    cluster.regionId = m_triangles[seedIndex].regionId;

    // BFS遍历
    while (!queue.empty()) {
        int currentIdx = queue.front();
        queue.pop();

        // 添加到聚类
        cluster.triangleIndices.push_back(currentIdx);

        // 收集当前三角形的3个顶点
        for (int i = 0; i < 3; i++) {
            std::array<double, 3> vertex = {
                m_triangles[currentIdx].vertices[i][0],
                m_triangles[currentIdx].vertices[i][1],
                m_triangles[currentIdx].vertices[i][2]
            };
            cluster.vertices.push_back(vertex);
        }

        // 获取邻接三角形（已过滤，只包含poor/bad）
        std::vector<int> neighbors = GetAdjacentTriangles(currentIdx);

        for (int neighborIdx : neighbors) {
            // 只需判断是否已访问（不需要再判断intensity和regionId）
            if (visited.find(neighborIdx) == visited.end()) {
                queue.push(neighborIdx);
                visited.insert(neighborIdx);
            }
        }
    }

    cluster.triangleCount = cluster.triangleIndices.size();
    return cluster;
}

void SprayEffectEvaluator::AnalyzeBadAreaClusters()
{
    // 清空之前的聚类结果
    m_badAreaClusters.clear();

    // 重置所有三角形的clusterIndex
    for (auto& tri : m_triangles) {
        tri.clusterIndex = -1;
    }

    // 已访问标记
    std::unordered_set<int> visited;

    // 聚类ID计数器
    int clusterId = 0;

    // 只遍历poor/bad三角形
    for (int i : m_poorBadTriangleIndices) {
        if (visited.find(i) == visited.end()) {
            // 执行BFS聚类
            BadAreaCluster cluster = ClusterBadAreaBFS(i, visited);

            // 设置聚类ID
            cluster.id = clusterId++;

            // 填充regionType和regionBodyTag
            for (const Region& r : m_regions) {
                if (r.id == cluster.regionId) {
                    cluster.regionType = r.type;
                    cluster.regionBodyTag = r.body ? r.body->Tag() : NULL_TAG;
                    break;
                }
            }

            // 更新三角形的clusterIndex
            for (int triIdx : cluster.triangleIndices) {
                m_triangles[triIdx].clusterIndex = cluster.id;
            }

            // 添加到结果列表
            m_badAreaClusters.push_back(cluster);
        }
    }
}

// ==================== 临时点显示功能实现 ====================

void SprayEffectEvaluator::CreatePointsForBadArea(int badAreaIndex)
{
    // 清除之前的临时点
    ClearTempPoints();

    // 验证索引有效性
    if (badAreaIndex < 0 || badAreaIndex >= m_badAreaClusters.size()) {
        return;
    }

    const BadAreaCluster& cluster = m_badAreaClusters[badAreaIndex];

    // 遍历该聚类中的所有三角形
    for (int triIdx : cluster.triangleIndices) {
        if (triIdx < 0 || triIdx >= m_triangles.size()) {
            continue;
        }

        const TriangleEvalData& tri = m_triangles[triIdx];

        // 创建三个顶点点
        for (int i = 0; i < 3; i++) {
            double coords[3] = {
                tri.vertices[i][0],
                tri.vertices[i][1],
                tri.vertices[i][2]
            };

            tag_t point_tag = NULL_TAG;
            int err = UF_CURVE_create_point(coords, &point_tag);

            if (err == 0 && point_tag != NULL_TAG) {
                m_tempPoints.push_back(point_tag);
            }
        }

        // 创建中心点
        double center_coords[3] = {
            tri.center.X,
            tri.center.Y,
            tri.center.Z
        };

        tag_t center_point_tag = NULL_TAG;
        int err = UF_CURVE_create_point(center_coords, &center_point_tag);

        if (err == 0 && center_point_tag != NULL_TAG) {
            m_tempPoints.push_back(center_point_tag);
        }
    }
}

void SprayEffectEvaluator::ClearTempPoints()
{
    // 删除所有临时点
    for (tag_t point_tag : m_tempPoints) {
        if (point_tag != NULL_TAG) {
            UF_OBJ_delete_object(point_tag);
        }
    }

    // 清空点列表
    m_tempPoints.clear();
}

void SprayEffectEvaluator::DrawBadAreaTriangles(int badAreaIndex)
{
    // 1. 验证badArea索引有效性
    if (badAreaIndex < 0 || badAreaIndex >= m_badAreaClusters.size()) {
        return;
    }

    // 2. 刷新显示（清除之前的templine）
    UF_DISP_refresh();

    // 3. 获取指定badArea的三角形索引列表
    const BadAreaCluster& cluster = m_badAreaClusters[badAreaIndex];

    // 4. 绘制该badArea中的所有三角形
    for (int triIdx : cluster.triangleIndices) {
        if (triIdx < 0 || triIdx >= m_triangles.size()) {
            continue;
        }

        const TriangleEvalData& tri = m_triangles[triIdx];
        DrawTriangleEdges(tri);
    }
}

void SprayEffectEvaluator::RecalculateAndDrawBadArea(int badAreaIndex, tag_t newCurveTag)
{
    // 1. 验证badArea索引有效性
    if (badAreaIndex < 0 || badAreaIndex >= m_badAreaClusters.size()) {
        return;
    }

    // 2. 将新曲线添加到喷涂路径列表
    if (newCurveTag != NULL_TAG) {
        m_sprayPaths.push_back(newCurveTag);
    }

    // 3. 获取指定badArea的三角形索引列表
    const BadAreaCluster& cluster = m_badAreaClusters[badAreaIndex];

    // 4. 对badArea中的所有三角形重新计算intensity和颜色
    for (int triIdx : cluster.triangleIndices) {
        if (triIdx < 0 || triIdx >= m_triangles.size()) {
            continue;
        }

        TriangleEvalData& tri = m_triangles[triIdx];

        // 重新计算spray intensity（使用更新后的喷涂路径）
        tri.sprayIntensity = CalculateSprayIntensity(tri.center);

        // 更新颜色代码
        tri.colorCode = MapIntensityToColor(tri.sprayIntensity);
    }

    // 5. 刷新显示（清除之前的templine）
    UF_DISP_refresh();

    // 6. 重新绘制badArea中的三角形
    for (int triIdx : cluster.triangleIndices) {
        if (triIdx < 0 || triIdx >= m_triangles.size()) {
            continue;
        }

        const TriangleEvalData& tri = m_triangles[triIdx];
        DrawTriangleEdges(tri);
    }
}
