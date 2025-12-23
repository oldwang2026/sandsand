#ifndef REGIONSTRUCT_H_INCLUDED
#define REGIONSTRUCT_H_INCLUDED

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>    

#include <NXOpen/TaggedObject.hxx>
#include <NXOpen/Face.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/Edge.hxx>
#include <NXOpen/MeasureManager.hxx>
#include <NXOpen/MeasureFaceBuilder.hxx>
#include <NXOpen/ScCollector.hxx>
#include <NXOpen/Measurement.hxx>

#include <uf.h>
#include <uf_facet.h>
#include <uf_vec.h>
#include <uf_modl.h>
#include <uf_obj.h>

#include "FaceProcessor.h"


// 区域结构


#include <unordered_set>
#include <queue>
#include <stack>

// 前置声明NXOpen相关类
namespace NXOpen {
    class Face;
    class Edge;
}


struct VertexInfo {
    NXOpen::Point3d point;
    int id;  // 顶点的唯一ID
};

// 辅助结构：边信息
struct EdgeInfo {
    int vertex1Id;
    int vertex2Id;
    NXOpen::Edge* edge;
};

struct RegionBoundaryEdgeInfo
{
    Edge* boundaryedge;
    tag_t boundarycurvetag;
    Point3d startpoint;
    Point3d endpoint;
    Vector3d start_tangent;
    Vector3d end_tangent;

    RegionBoundaryEdgeInfo(Edge* edge);
};

struct Region {
    int id;
    bool isRing = false;
    FaceRegionType type;
    std::vector<NXOpen::Face*> faces;
    std::vector<NXOpen::Edge*> boundaryEdges;
    std::vector<std::vector<RegionBoundaryEdgeInfo>> boundaryGroups;
    NXObject* regionobj;
    Body* body;
    bool boundaryAnalyzed = false;  // 标记边界是否已分析过

    tag_t facet_model = NULL_TAG;
     // 新增：用于保存生成的控制点和曲线
    std::vector<std::vector<Point3d>> controlPointGroups;  // 每个边界组对应的控制点集合
    std::vector<tag_t> splineTags;  // 每个边界组生成的样条曲线tag

    std::vector<std::vector<Point3d>> sprayPathPoints;  // 最终的喷砂路径点
    std::vector<tag_t> sprayPathCurves;  // 生成的路径曲线

    bool processed = false;  // 标记该区域是否已生成路径

    Region(int _id, FaceRegionType _type) : id(_id), type(_type) {}
};



#endif // !REGIONSTRUCT_H_INCLUDED


