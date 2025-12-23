#include "FaceProcessor.h"
#include <uf_vec.h>
#include <unordered_set>

// 构造函数
FaceProcessor::FaceProcessor()
{
    // 初始化最小半径阈值（单位：mm，可根据实际需求调整）
    m_minRadiusThreshold = 30;  // 半径小于10mm认为是曲面

    // 初始化Facet参数
    UF_FACET_INIT_PARAMETERS(&m_facetParams);
    UF_FACET_ask_default_parameters(&m_facetParams);



    // 生成带时间戳的日志文件名
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
    string filename = "C:\\shotpeening\\source\\SandSprayProject\\txtfile\\test" + std::string(timestamp) + ".txt";

    m_logfile.open(filename, std::ios::out);

    // 写入日志文件头信息
    if (m_logfile.is_open()) {
        m_logfile << "========================================" << std::endl;
        m_logfile << "Face Processor Log File" << std::endl;
        m_logfile << "Created: " << std::asctime(timeinfo);
        m_logfile << "========================================" << std::endl;
        m_logfile << "Configuration Parameters:" << std::endl;
        m_logfile << "  Minimum Radius Threshold: " << m_minRadiusThreshold << " mm" << std::endl;
       
        m_logfile << "========================================" << std::endl;
        m_logfile << std::endl;
    }
}

// 析构函数
FaceProcessor::~FaceProcessor()
{
    if (m_logfile.is_open()) {
        // 写入结束信息
        std::time_t now = std::time(nullptr);
        m_logfile << std::endl;
        m_logfile << "========================================" << std::endl;
        m_logfile << "Log closed: " << std::asctime(std::localtime(&now));
        m_logfile << "========================================" << std::endl;

        // 关闭文件
        m_logfile.close();
    }
}




void FaceProcessor::classifyFaces(vector<TaggedObject*> selectbody)
{
    if (selectbody.empty()) {
        return;
    }

    try
    {
        NXOpen::NXObject* findattr = dynamic_cast<NXOpen::NXObject*>(selectbody[0]);
        bool isProcess = findattr->GetBooleanUserAttribute("isProcess",-1);
        ClassifyAllFacesbyColor(selectbody);
    }
    catch (const std::exception&)
    {
        ClassifyAllFacesbyType(selectbody);
    }
}

void FaceProcessor::ClassifyAllFacesbyType(vector<TaggedObject*> selectbody)
{
    if (selectbody.empty()) {
        return;
    }
    clearallFacevectors();
    // 检查输入

    NXOpen::Body* body = dynamic_cast<NXOpen::Body*>(selectbody[0]);
    if (body == nullptr) {
        return;
    }
    body->SetBooleanUserAttribute("isProcess", -1, true, NXOpen::Update::Option::OptionNow);
    // 获取body的所有面
    m_all_faces = body->GetFaces();

    // 遍历所有面，根据类型分类
    for (size_t i = 0; i < m_all_faces.size(); i++)
    {
        NXOpen::Face* face = m_all_faces[i];

        // 获取面的类型
        int faceType = face->SolidFaceType();
        FaceRegionType currentregiontype;
        // 分类存储

        if (faceType == NXOpen::Face::FaceType::FaceTypePlanar) {
            // 是平面，放入平面数组
            currentregiontype = classifyPlanarFace(face);
        }

        else {
            // 不是平面
            currentregiontype = classifyCurvedFaceByMinRadius(face);
        }

        switch (currentregiontype) {
        case FaceRegionType::HORIZONTAL:
            m_horizontal_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::HORIZONTAL;
            break;
        case FaceRegionType::SIDEWALL:
            m_sidewall_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::SIDEWALL;
            break;
        case FaceRegionType::CONVEX:
            m_convex_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::CONVEX;
            break;
        case FaceRegionType::CONCAVE:
            m_concave_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::CONCAVE;
            break;
        }
    }
    //所有面分类为平面与非平面 再将平面分成水平面和侧壁面 非平面分为凸面和凹面
}

void FaceProcessor::ClassifyAllFacesbyColor(vector<TaggedObject*> selectbody)
{
    if (selectbody.empty()) {
        return;
    }
    NXOpen::Body* body = dynamic_cast<NXOpen::Body*>(selectbody[0]);
    if (body == nullptr) {
        return;
    }
    m_all_faces = body->GetFaces();
    for (size_t i = 0; i < m_all_faces.size(); i++)
    {
        NXOpen::Face* face = m_all_faces[i];

        // 获取面的类型
        int facecolor = face->Color();
        switch (facecolor) {
        case 211:  // 水平区域颜色
            m_horizontal_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::HORIZONTAL;
            break;
        case 186:  // 侧壁颜色  
            m_sidewall_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::SIDEWALL;
            break;
        case 36:   // 凸面颜色
            m_convex_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::CONVEX;
            break;
        case 6:    // 凹面颜色
            m_concave_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::CONCAVE;
            break;
        default:   // 未分类颜色(216)或其他颜色
            m_unsorted_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::UNSORTED;
            break;
        }

    }
}

void FaceProcessor::clearallFacevectors()
{
    m_all_faces.clear();
    m_concave_faces.clear();
    m_convex_faces.clear();
    m_horizontal_faces.clear();
    m_sidewall_faces.clear();
    m_unsorted_faces.clear();
    m_faceClassificationMap.clear();  // 清空map
}

FaceRegionType FaceProcessor::classifyPlanarFace(NXOpen::Face* face)
{
    // 获取面上的一个点和该点的法向量
    double point[3] = { 0.0, 0.0, 0.0 };
   

    double normal[3] = { 0.0, 0.0, 0.0 };
    double u_deriv[3], v_deriv[3];
    double u_deriv2[3], v_deriv2[3], radii[2];

    tag_t faceTag = face->Tag();
    double uvmm[4];
    UF_MODL_ask_face_uv_minmax(faceTag, uvmm);

    double uv_param[] = { 0.5*(uvmm[0]+uvmm[1]),0.5 * (uvmm[2] + uvmm[3]) };
    UF_MODL_ask_face_props(faceTag, uv_param, point, u_deriv, v_deriv,
        u_deriv2, v_deriv2, normal, radii);

    // 归一化法向量
    double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (length > 0) {
        normal[0] /= length;
        normal[1] /= length;
        normal[2] /= length;
    }

    // 计算法向量与Z轴的夹角
    // Z轴向量为 (0, 0, 1)
    double dotProduct = std::abs(normal[2]);  // 与Z轴的点积，取绝对值

    // 判断阈值（可以调整）
    

    FaceRegionType faceType;
    if (dotProduct > m_horizontalThreshold) {
        faceType = FaceRegionType::HORIZONTAL;
       
    }
    else {
        faceType = FaceRegionType::SIDEWALL;
    }

    return faceType;
}

FaceRegionType FaceProcessor::classifyCurvedFaceByMinRadius(NXOpen::Face* face)
{
    tag_t faceTag = face->Tag();

    // 根据函数签名要求准备固定大小的数组
    int num_radii = 0;
    double radii[2];        // 最多2个曲率半径
    double positions[6];    // 2个位置，每个3个坐标
    double params[4];       // 2个参数位置，每个2个u,v参数

    // 调用 UF 函数获取最小曲率半径
    int error = UF_MODL_ask_face_min_radii(faceTag, &num_radii, radii, positions, params);

    if (error != 0) {
        // 错误时默认归入平面组
        m_horizontal_faces.push_back(face);
        return FaceRegionType::HORIZONTAL;
    }

    // 如果没有找到曲率半径（返回0），说明是平面或大曲率半径面
    if (num_radii == 0) {
        return classifyPlanarFace(face);  // 进一步判断是水平还是侧壁
    }

    // 找出绝对值最小的曲率半径及其对应的原始值
    double minAbsRadius = DBL_MAX;
    double minRadiusOriginal = 0;  // 保存原始值用于判断凹凸

    for (int i = 0; i < num_radii; i++) {
        double absRadius = std::abs(radii[i]);

        m_logfile << "Radius[" << i << "] = " << radii[i]
            << " (abs = " << absRadius << ")"
            << " at position (" << positions[i * 3] << ", "
            << positions[i * 3 + 1] << ", " << positions[i * 3 + 2] << ")" << std::endl;

        if (absRadius < minAbsRadius && absRadius > 1e-6) {
            minAbsRadius = absRadius;
            minRadiusOriginal = radii[i];  // 保存原始值
        }
    }

    // 第一步：判断曲率半径绝对值是否小于阈值
    if (minAbsRadius < m_minRadiusThreshold) {
        // 是曲面（小曲率半径）
        // 第二步：判断凹凸性
        if (minRadiusOriginal > 0) {
            // 正值表示凹面
            return FaceRegionType::CONCAVE;
        }
        else {
            // 负值表示凸面
            return FaceRegionType::CONVEX;
        }
    }
    else {
        // 大曲率半径，当作平面处理
        return classifyPlanarFace(face);  // 进一步判断是水平还是侧壁
    }
}

std::vector<TaggedObject*> FaceProcessor::GetOneRegionFaces(FaceRegionType regiontype) const

{
    std::vector<TaggedObject*> returntag;

    switch (regiontype) {
    case FaceRegionType::HORIZONTAL:
        returntag.insert(returntag.begin(),m_horizontal_faces.begin(), m_horizontal_faces.end());
        return returntag;
        break;
    case FaceRegionType::SIDEWALL:
        returntag.insert(returntag.begin(), m_sidewall_faces.begin(), m_sidewall_faces.end());
        return returntag;
        break;
    case FaceRegionType::CONVEX:
        returntag.insert(returntag.begin(), m_convex_faces.begin(), m_convex_faces.end());
        return returntag;
        break;
    case FaceRegionType::CONCAVE:
        returntag.insert(returntag.begin(), m_concave_faces.begin(), m_concave_faces.end());
        return returntag;
        break;

    }
}

std::vector<Face*>& FaceProcessor::GetRegionFacesRef(FaceRegionType regiontype)
{
    switch (regiontype) 
    {
    case FaceRegionType::HORIZONTAL:
        return m_horizontal_faces;
    case FaceRegionType::SIDEWALL:
        return m_sidewall_faces;
    case FaceRegionType::CONVEX:
        return m_convex_faces;
    case FaceRegionType::CONCAVE:
        return m_concave_faces;
    case FaceRegionType::UNSORTED:
        return m_unsorted_faces;
    default:
        return m_unsorted_faces;  // 默认返回unsorted
    }
}

void FaceProcessor::FastRemoveFromVector(std::vector<Face*>& vec, Face* obj)
{
    auto it = std::find(vec.begin(), vec.end(), obj);
    if (it != vec.end()) {
        // 与最后一个元素交换，然后pop_back
        if (it != vec.end() - 1) {
            std::swap(*it, vec.back());
        }
        vec.pop_back();
    }
}

void FaceProcessor::HandleUserFaceEdits(
    FaceRegionType currentDisplayType,
    const std::vector<TaggedObject*>& userSelectedFaces)
{
    std::unordered_set<TaggedObject*> userSelectedSet(
        userSelectedFaces.begin(), userSelectedFaces.end());

    std::vector<Face*>& currentTypeVector = GetRegionFacesRef(currentDisplayType);

    std::vector<Face*> facesToRemove;
    for (auto* face : currentTypeVector) {
        if (userSelectedSet.find(face) == userSelectedSet.end()) {
            facesToRemove.push_back(face);
            m_unsorted_faces.push_back(face);
            m_faceClassificationMap[face] = FaceRegionType::UNSORTED;
        }
    }

    // 批量删除（从后往前删除避免索引问题）
    for (auto* face : facesToRemove) {
        FastRemoveFromVector(currentTypeVector, face);
    }

    // 4. 检查新添加的面
    for (auto* tagged : userSelectedFaces) {
        NXOpen::Face* face = dynamic_cast<Face*>(tagged);
        auto mapIt = m_faceClassificationMap.find(face);

        if (mapIt != m_faceClassificationMap.end()) {
            if (mapIt->second != currentDisplayType) {
                // 从原分类中移除
                std::vector<Face*>& oldVector = GetRegionFacesRef(mapIt->second);
                FastRemoveFromVector(oldVector, face);

                // 添加到当前分类
                currentTypeVector.push_back(face);
                m_faceClassificationMap[face] = currentDisplayType;
            }
        }
        else {
            currentTypeVector.push_back(face);
            m_faceClassificationMap[face] = currentDisplayType;
        }
    }
    //gengxinyasne
    SetFacesColor();
}

void FaceProcessor::SetFacesColor()
{
    for (const auto& pair : m_faceClassificationMap)
    {
        auto displayobj(dynamic_cast<DisplayableObject*>(pair.first));
        displayobj->SetColor(GetColorByType(pair.second));
        displayobj->RedisplayObject();
    }
}

int FaceProcessor::GetColorByType(FaceRegionType regiontype)
{
    switch (regiontype)
    {
    case FaceRegionType::HORIZONTAL:
        return 211;
        break;
    case FaceRegionType::CONVEX:
        return 36;
        break;
    case FaceRegionType::CONCAVE:
        return 6;
        break;
    case FaceRegionType::SIDEWALL:
        return 186;
        break;
    case FaceRegionType::UNSORTED:
        return 216;
        break;
    default:
        return 216;
        break;
    }
}