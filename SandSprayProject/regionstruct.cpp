#include "regionstruct.h"
#include "HelpFunction.h"
#include <uf_modl.h>
RegionBoundaryEdgeInfo::RegionBoundaryEdgeInfo(Edge* edge)
{
    // 从边创建曲线
    tag_t edge_tag = edge->Tag();
    UF_MODL_create_curve_from_edge(edge_tag, &boundarycurvetag);

    // 获取起点和终点
    double start_param = 0.0;
    double end_param = 1.0;
    double start_pt[3], end_pt[3];

    ask_curve_point(boundarycurvetag, start_param, start_pt);
    ask_curve_point(boundarycurvetag, end_param, end_pt);

    startpoint = Point3d(start_pt[0], start_pt[1], start_pt[2]);
    endpoint = Point3d(end_pt[0], end_pt[1], end_pt[2]);
    
    // 计算起点切向量（使用很近的两点）
    double point[3], tangent[3], p_norm[3], b_norm[3], torsion, rad_of_cur;
    UF_MODL_ask_curve_props(boundarycurvetag, 0, point, tangent, p_norm, b_norm, &torsion, &rad_of_cur);
    start_tangent = Vector3d(tangent[0], tangent[1], tangent[2]);


    // 计算终点切向量
    UF_MODL_ask_curve_props(boundarycurvetag, 1, point, tangent, p_norm, b_norm, &torsion, &rad_of_cur);
    end_tangent = Vector3d(tangent[0], tangent[1], tangent[2]);

   

}

