#ifndef FACEPROCESSOR_H_INCLUDED
#define FACEPROCESSOR_H_INCLUDED

#include <vector>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>    

#include <NXOpen/TaggedObject.hxx>
#include <NXOpen/Face.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/MeasureManager.hxx>
#include <NXOpen/MeasureFaceBuilder.hxx>
#include <NXOpen/ScCollector.hxx>
#include <NXOpen/Measurement.hxx>
#include <uf.h>
#include <uf_facet.h>
#include <uf_vec.h>
#include <uf_modl.h>
#include <uf_obj.h>

using namespace std;
using namespace NXOpen;

enum class FaceRegionType
{
    HORIZONTAL,     // 水平区域
    CONVEX,        // 凸曲面区域
    CONCAVE,       // 凹曲面区域  
    SIDEWALL,       // 侧壁区域
    UNSORTED
};

class FaceProcessor
{ 
    
public:

	FaceProcessor();
	~FaceProcessor();
	/*void test(vector<TaggedObject*> selectbody);*/
    void classifyFaces(vector<TaggedObject*> selectbody);
    void ClassifyAllFacesbyColor(vector<TaggedObject*> selectbody);
    void ClassifyAllFacesbyType(vector<TaggedObject*> selectbody);
    std::vector<TaggedObject*> GetOneRegionFaces(FaceRegionType regiontype) const;
    double m_minRadiusThreshold;  // 最小半径阈值（用于判断是否为曲面）
    void HandleUserFaceEdits(FaceRegionType currentDisplayType, const std::vector<TaggedObject*>& userSelectedFaces);
    void SetFacesColor();
private:
    std::ofstream m_logfile;
    UF_FACET_parameters_t m_facetParams;  // Facet参数
    std::vector<NXOpen::Face*> m_all_faces;
    std::vector<NXOpen::Face*> m_concave_faces;
    std::vector<NXOpen::Face*> m_convex_faces;
    std::vector<NXOpen::Face*> m_horizontal_faces;
    std::vector<NXOpen::Face*> m_sidewall_faces;
    std::vector<NXOpen::Face*> m_unsorted_faces;
    std::unordered_map<NXOpen::TaggedObject*, FaceRegionType> m_faceClassificationMap;
    const double m_horizontalThreshold = cos(PI / 4);


    void clearallFacevectors();

    
    int GetColorByType(FaceRegionType regiontype);
    // 判断面类型的函数
    FaceRegionType classifyPlanarFace(NXOpen::Face* face);
    FaceRegionType classifyCurvedFaceByMinRadius(NXOpen::Face* face);

    std::vector<Face*>& GetRegionFacesRef(FaceRegionType regiontype);
    void FastRemoveFromVector(std::vector<Face*>& vec, Face* obj);

};

#endif
