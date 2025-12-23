#include "RegionBuilder.h"
#include <iostream>
#include <NXOpen/Features_ExtractFaceBuilder.hxx>
#include <NXOpen/Features_Feature.hxx>
#include <NXOpen/Section.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/BasePart.hxx>
#include <NXOpen/SelectFace.hxx>
#include <NXOpen/SelectFaceList.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Features_FeatureCollection.hxx>
#include <uf_modl.h>
#include <fstream>
#include "HelpFunction.h"
#include "regionstruct.h"
using namespace std;

std::vector<NXOpen::Face*> RegionBuilder::getAdjacentFaces(NXOpen::Face* face) {
    std::unordered_set<NXOpen::Face*> adjacentFaces;

    std::vector<NXOpen::Edge*> edges = face->GetUnsortedEdges();

    for (NXOpen::Edge* edge : edges) {
        std::vector<NXOpen::Face*> facesOnEdge = edge->GetFaces();

        for (NXOpen::Face* adjFace : facesOnEdge) {
            if (adjFace != face) {
                adjacentFaces.insert(adjFace);
            }
        }
    }

    return std::vector<NXOpen::Face*>(adjacentFaces.begin(), adjacentFaces.end());
}

void RegionBuilder::collectRegionBFS(NXOpen::Face* seedFace, FaceRegionType targetType, int regionId) {
    if (m_visited.contains(seedFace)) {
        return;
    }

    Region region(regionId, targetType);
    std::queue<NXOpen::Face*> queue;

    
    queue.push(seedFace);
    m_visited.insert(seedFace);  
    while (!queue.empty()) {
        NXOpen::Face* currentFace = queue.front();
        queue.pop();

        region.faces.push_back(currentFace);
        
        std::vector<NXOpen::Face*> neighbors = getAdjacentFaces(currentFace);

        for (NXOpen::Face* neighbor : neighbors) {
            if (!m_visited.contains(neighbor) &&m_faceTypeMap[neighbor] == targetType) 
            {
                queue.push(neighbor);
                m_visited.insert(neighbor);  
            }
        }
    }

    if (!region.faces.empty()) {
        m_regions.push_back(std::move(region));
    }


}

RegionBuilder::RegionBuilder() 
{

}

std::vector<Region>& RegionBuilder::getRegions()  {
    return m_regions;
}

std::vector<const Region*> RegionBuilder::getRegionsByType(FaceRegionType type) const {
    std::vector<const Region*> typedRegions;

    for (const auto& region : m_regions) {
        if (region.type == type) {
            typedRegions.push_back(&region);
        }
    }

    return typedRegions;
}

void RegionBuilder::printRegionInfo() const {
    std::cout << "Total regions found: " << m_regions.size() << std::endl;

    for (const auto& region : m_regions) {
        std::cout << "Region " << region.id << ":" << std::endl;
        std::cout << "  Type: " << static_cast<int>(region.type) << std::endl;
        std::cout << "  Faces: " << region.faces.size() << std::endl;
        std::cout << "  Boundary edges: " << region.boundaryEdges.size() << std::endl;
        std::cout << std::endl;
    }
}

std::vector<NXOpen::Face*> RegionBuilder::GetBoundaryFaces(const std::vector<NXOpen::Face*>& inputFaces) {
    // ��������ת��Ϊset���ڿ��ٲ���
    std::unordered_set<NXOpen::Face*> inputFaceSet(inputFaces.begin(), inputFaces.end());

    // �洢���б߽���
    std::unordered_set<NXOpen::Face*> boundaryFaces;

    // �������������棬��ȡ���ǵı�
    for (NXOpen::Face* face : inputFaces) {
        std::vector<NXOpen::Edge*> edges = face->GetUnsortedEdges();

        for (NXOpen::Edge* edge : edges) {
            std::vector<NXOpen::Face*> edgeFaces = edge->GetFaces();

            for (NXOpen::Face* edgeFace : edgeFaces) {

                if (!inputFaceSet.contains(edgeFace)) {
                    boundaryFaces.insert(edgeFace);
                }
            }
        }
    }

    return std::vector<NXOpen::Face*>(boundaryFaces.begin(), boundaryFaces.end());
}

NXOpen::NXObject* RegionBuilder::GetExtractRegion(std::vector<Face*> intputfaces)
{
    NXOpen::Session* theSession = NXOpen::Session::GetSession();
    NXOpen::Part* workPart(theSession->Parts()->Work());
    NXOpen::Part* displayPart(theSession->Parts()->Display());

    NXOpen::Features::Feature* nullNXOpen_Features_Feature(NULL);

    NXOpen::Features::ExtractFaceBuilder* extractFaceBuilder1;
    extractFaceBuilder1 = workPart->Features()->CreateExtractFaceBuilder(nullNXOpen_Features_Feature);


    extractFaceBuilder1->SetParentPart(NXOpen::Features::ExtractFaceBuilder::ParentPartTypeWorkPart);


    extractFaceBuilder1->SetInheritMaterial(true);

    extractFaceBuilder1->SetAssociative(true);

    extractFaceBuilder1->SetFixAtCurrentTimestamp(false);

    extractFaceBuilder1->SetHideOriginal(false);

    extractFaceBuilder1->SetDeleteHoles(false);

    extractFaceBuilder1->SetInheritDisplayProperties(false);

    extractFaceBuilder1->SetTraverseInteriorEdges(true);

    extractFaceBuilder1->SetType(NXOpen::Features::ExtractFaceBuilder::ExtractTypeRegionOfFaces);

   

    extractFaceBuilder1->SeedFace()->SetValue(intputfaces[0]);

    std::vector<NXOpen::Face*> boundaryfaces = GetBoundaryFaces(intputfaces);

    if (boundaryfaces.size() == 0)
    {
        extractFaceBuilder1->Destroy();
        return intputfaces[0]->GetBody();
    }


    extractFaceBuilder1->BoundaryFaces()->Add(boundaryfaces);

    extractFaceBuilder1->SetAssociative(false);

    NXOpen::NXObject* nXObject1;
    nXObject1 = extractFaceBuilder1->Commit();

    extractFaceBuilder1->Destroy();

    return nXObject1;
}

void RegionBuilder::ClassifyAllFacesbyColor(TaggedObject* selectbody)
{

    NXOpen::Body* body = dynamic_cast<NXOpen::Body*>(selectbody);
    if (body == nullptr) {
        return;
    }
    m_allfaces = body->GetFaces();
    for (size_t i = 0; i < m_allfaces.size(); i++)
    {
        NXOpen::Face* face = m_allfaces[i];

        // ��ȡ�������
        int facecolor = face->Color();
        switch (facecolor) {
        case 211:  // ˮƽ������ɫ
           
            m_faceTypeMap[face] = FaceRegionType::HORIZONTAL;
            break;
        case 186:  // �����ɫ  
            
            m_faceTypeMap[face] = FaceRegionType::SIDEWALL;
            break;
        case 36:   // ͹����ɫ
           
            m_faceTypeMap[face] = FaceRegionType::CONVEX;
            break;
        case 6:    // ������ɫ
            
            m_faceTypeMap[face] = FaceRegionType::CONCAVE;
            break;
        default:   // δ������ɫ(216)��������ɫ
          
            m_faceTypeMap[face] = FaceRegionType::UNSORTED;
            break;
        }

    }
}

void RegionBuilder::Commit(NXOpen::TaggedObject* body)
{
  
    if (!body) {
        return;
    }

    m_allfaces.clear();
    m_faceTypeMap.clear();
    ClassifyAllFacesbyColor(body);

    m_regions.clear();
    m_visited.clear();

    int regionId = 0;

    for (NXOpen::Face* face : m_allfaces) {
        if (!m_visited.contains(face)) {
            FaceRegionType faceType = m_faceTypeMap[face];
            collectRegionBFS(face, faceType, regionId++);
        }
    }

  
    for (auto& reg : m_regions) 
    {
        try {
            reg.regionobj = GetExtractRegion(reg.faces);
            updateRegioninfo(reg);
        }
        catch (const std::exception&) {
            reg.regionobj = nullptr;
        }
    }


}

void RegionBuilder::updateRegioninfo(Region& reg)
{
    try
    {
        NXOpen::Features::Feature* regfeature = dynamic_cast<NXOpen::Features::Feature*>(reg.regionobj);

        reg.body = regfeature->GetBodies()[0];
        reg.faces = reg.body->GetFaces();
        reg.boundaryEdges = calculateBoundaryEdges(reg);
        reg.isRing = isRingRegion(reg, 0.01);
        reg.body->SetBooleanUserAttribute("isRing", -1, reg.isRing, NXOpen::Update::Option::OptionNow);
        reg.body->SetAttribute("RegionType", static_cast<int>(reg.type), NXOpen::Update::Option::OptionNow);

    }
    catch (const std::exception&)
    {
        reg.body = dynamic_cast<NXOpen::Body*>(reg.regionobj);
        reg.faces = reg.body->GetFaces();
        reg.boundaryEdges = calculateBoundaryEdges(reg);
        reg.isRing = isRingRegion(reg, 0.01);
        reg.body->SetBooleanUserAttribute("isRing", -1, reg.isRing, NXOpen::Update::Option::OptionNow);
        reg.body->SetAttribute("RegionType", static_cast<int>(reg.type), NXOpen::Update::Option::OptionNow);
    }


}

vector<Edge*> RegionBuilder::calculateBoundaryEdges(Region& region) 
{
    std::unordered_map<NXOpen::Edge*, int> edgeCount;
    region.boundaryEdges.clear();

    for (NXOpen::Face* face : region.faces) {
        std::vector<NXOpen::Edge*> edges = face->GetEdges();
        for (NXOpen::Edge* edge : edges) {
            edgeCount[edge]++;
        }
    }

    vector<Edge*> edges;
    edges.clear();
    for (const auto& pair : edgeCount) {
        if (pair.second == 1) {
            edges.push_back(pair.first);
        }
    }
    return edges;
}

int RegionBuilder::getOrCreateVertexId(const NXOpen::Point3d& point,std::vector<VertexInfo>& vertices,double tolerance) const

{
    // �����Ƿ���ھ���С����ֵ�Ķ���
    for (const auto& vertex : vertices) {
        if (PointDistance(point, vertex.point) < tolerance) {
            return vertex.id;
        }
    }

    // �����¶���
    VertexInfo newVertex;
    newVertex.point = point;
    newVertex.id = vertices.size();
    vertices.push_back(newVertex);

    return newVertex.id;
}

bool RegionBuilder::isRingRegion(const Region& region, double tolerance) 
{
    if (region.boundaryEdges.empty()) {
        return false;
    }

    int loopCount = countBoundaryLoops(region.boundaryEdges, tolerance);

    // �����2�������ϵĶ����պϱ߽磬˵���ǻ�������
    return loopCount >= 2;
}

int RegionBuilder::countBoundaryLoops(const std::vector<NXOpen::Edge*>& boundaryEdges,double tolerance)
{
    if (boundaryEdges.empty()) {
        return 0;
    }

    // ����1���ռ�����Ψһ���㲢����ID
    std::vector<VertexInfo> vertices;
    std::vector<EdgeInfo> edges;

    for (NXOpen::Edge* edge : boundaryEdges) {
        NXOpen::Point3d v1, v2;
        edge->GetVertices(&v1, &v2);

        EdgeInfo edgeInfo;
        edgeInfo.vertex1Id = getOrCreateVertexId(v1, vertices, tolerance);
        edgeInfo.vertex2Id = getOrCreateVertexId(v2, vertices, tolerance);
        edgeInfo.edge = edge;
        edges.push_back(edgeInfo);
    }

    // ����2�������ڽӱ�������ID -> �����ı�������
    std::unordered_map<int, std::vector<int>> vertexToEdgeIndices;
    for (size_t i = 0; i < edges.size(); ++i) {
        vertexToEdgeIndices[edges[i].vertex1Id].push_back(i);
        vertexToEdgeIndices[edges[i].vertex2Id].push_back(i);
    }

    // ����3��ʹ��DFS�ҳ�������ͨ����
    std::vector<bool> visitedEdges(edges.size(), false);
    int loopCount = 0;

    for (size_t startIdx = 0; startIdx < edges.size(); ++startIdx) {
        if (visitedEdges[startIdx]) {
            continue;
        }

        // DFS�����ҳ�һ����ͨ����
        std::vector<int> component;
        std::stack<int> stack;
        stack.push(startIdx);
        visitedEdges[startIdx] = true;

        while (!stack.empty()) {
            int currentIdx = stack.top();
            stack.pop();
            component.push_back(currentIdx);

            const EdgeInfo& currentEdge = edges[currentIdx];

            // ����ͨ���������ӵ�������
            for (int vertexId : {currentEdge.vertex1Id, currentEdge.vertex2Id}) {
                for (int neighborIdx : vertexToEdgeIndices[vertexId]) {
                    if (!visitedEdges[neighborIdx]) {
                        visitedEdges[neighborIdx] = true;
                        stack.push(neighborIdx);
                    }
                }
            }
        }

        // ����4����������ͨ�����Ƿ��γɱպϻ�·
        std::unordered_map<int, int> vertexDegree;
        for (int edgeIdx : component) {
            vertexDegree[edges[edgeIdx].vertex1Id]++;
            vertexDegree[edges[edgeIdx].vertex2Id]++;
        }

        bool isClosedLoop = true;
        for (const auto& [vertexId, degree] : vertexDegree) {
            if (degree != 2) {
                // �����������2��˵�����Ǳպϻ�·
                //m_countFile << degree << endl;
                isClosedLoop = false;
                break;
            }
        }

        if (isClosedLoop && !component.empty()) {
            loopCount++;
        }
    }

    return loopCount; 

    //tag_t body_tag = boundaryEdges[0]->GetBody()->Tag();
    //int num_boundaries = 0;
    //int* num_edges = nullptr;
    //tag_t* edge_tags = nullptr;

    //// ����UF������ȡ�߽���Ϣ
    //int error = UF_MODL_ask_body_boundaries(
    //    body_tag,
    //    &num_boundaries,
    //    &num_edges,
    //    &edge_tags
    //);

    //UF_free(num_edges);
    //UF_free(edge_tags);

    //return num_boundaries;
}