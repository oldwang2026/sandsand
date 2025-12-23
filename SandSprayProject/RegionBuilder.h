#ifndef REGIONBUILDER_H_INCLUDED
#define REGIONBUILDER_H_INCLUDED

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
#include "regionstruct.h"
// 区域结构


#include <unordered_set>
#include <queue>
#include <stack>

// 前置声明NXOpen相关类
namespace NXOpen {
    class Face;
    class Edge;
}


// 辅助结构：顶点信息

class RegionBuilder {
  public:


    RegionBuilder();
   // 构建所有区域
   
    // 获取所有区域
    std::vector<Region>& getRegions() ;

    // 获取特定类型的所有区域  
    std::vector<const Region*> getRegionsByType(FaceRegionType type) const;

    // 打印区域信息
    void printRegionInfo() const;

    void Commit(NXOpen::TaggedObject* body);

  private:
    std::vector<NXOpen::Face*> m_allfaces;
    std::vector<Region> m_regions;  // 直接存储Region对象
    std::unordered_map<NXOpen::Face*, FaceRegionType> m_faceTypeMap;
    std::unordered_set<NXOpen::Face*> m_visited;

    //std::ofstream m_countFile;

    // 计算两点距离
   

    // 获取或创建顶点ID
    int getOrCreateVertexId(const NXOpen::Point3d& point,
        std::vector<VertexInfo>& vertices,
        double tolerance) const;

    // 计算边界环的数量
    int countBoundaryLoops(const std::vector<NXOpen::Edge*>& boundaryEdges,double tolerance);


    // 获取相邻面
    std::vector<NXOpen::Face*> getAdjacentFaces(NXOpen::Face* face);

    // 使用BFS收集一个区域
    void collectRegionBFS(NXOpen::Face* seedFace, FaceRegionType targetType, int regionId);

    // 计算区域的边界边
    vector<Edge*> calculateBoundaryEdges(Region& region);

    std::vector<NXOpen::Face*> GetBoundaryFaces(const std::vector<NXOpen::Face*>& inputFaces);

    void ClassifyAllFacesbyColor(TaggedObject* selectbody);


    NXOpen::NXObject* GetExtractRegion(std::vector<Face*> intputfaces);

    void updateRegioninfo(Region& reg);

    bool isRingRegion(const Region& region, double tolerance = 0.01) ;
};

#endif // REGION_BUILDER_H

