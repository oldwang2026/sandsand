#include "HelpFunction.h"
#include <uf_modl.h>
#include <cmath>
#include <uf_defs.h>
#include <uf_vec.h>
#include <NXOpen/NXException.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/BasePart.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/Builder.hxx>
#include <NXOpen/CurveDumbRule.hxx>
#include <NXOpen/DisplayableObject.hxx>
#include <NXOpen/Features_CompositeCurveBuilder.hxx>
#include <NXOpen/Features_Feature.hxx>
#include <NXOpen/Features_FeatureCollection.hxx>
#include <NXOpen/IBaseCurve.hxx>
#include <NXOpen/NXObject.hxx>
#include <NXOpen/Part.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Point.hxx>
#include <NXOpen/PreviewBuilder.hxx>
#include <NXOpen/ScCollector.hxx>
#include <NXOpen/ScRuleFactory.hxx>
#include <NXOpen/Section.hxx>
#include <NXOpen/SelectBodyList.hxx>
#include <NXOpen/SelectDatumPlane.hxx>
#include <NXOpen/SelectDisplayableObjectList.hxx>
#include <NXOpen/SelectFace.hxx>
#include <NXOpen/SelectFaceList.hxx>
#include <NXOpen/SelectObject.hxx>
#include <NXOpen/SelectObjectList.hxx>
#include <NXOpen/SelectPointList.hxx>
#include <NXOpen/SelectTaggedObjectList.hxx>
#include <NXOpen/SelectionIntentRule.hxx>
#include <NXOpen/SelectionIntentRuleOptions.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/Spline.hxx>
#include <NXOpen/SplineCollection.hxx>
#include <NXOpen/TaggedObject.hxx>
#include <NXOpen/NXObjectManager.hxx>
#include <uf_defs.h>
#include <uf_ui_types.h>
#include <uf_vec.h>
#include <uf_facet.h>
#include <uf_disp.h>
#include <uf_obj.h>


double PointDistance(const Point3d& p1, const Point3d& p2)
{
    double dx = p1.X - p2.X;
    double dy = p1.Y - p2.Y;
    double dz = p1.Z - p2.Z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double CalculateVectorAngle(const Vector3d& v1, const Vector3d& v2)
{
    // 计算向量长度
    double vec1[3] = { v1.X,v1.Y,v1.Z };
    double vec2[3] = { v2.X,v2.Y,v2.Z };
    double smallangle, largeangle;
    UF_MODL_ask_vector_angle(vec1, vec2, &smallangle, &largeangle);
    return smallangle;
}

int PDCAPP_COM_intialize_point_data
(
    int num_points,
    UF_CURVE_pt_slope_crvatr_t** pt_data
)
{
    int err = 0, i = 0;
    double zero[3] = { 0.0, 0.0, 0.0 };
    UF_CURVE_pt_slope_crvatr_t* point_data;

    if (num_points < 1) return(1);

    point_data = (UF_CURVE_pt_slope_crvatr_t*)UF_allocate_memory(
        num_points * sizeof(UF_CURVE_pt_slope_crvatr_t), &err);
    if (err) return(err);

    for (i = 0; i < num_points; i++)
    {
        UF_VEC3_copy(zero, point_data[i].point);

        point_data[i].slope_type = UF_CURVE_SLOPE_NONE;
        UF_VEC3_copy(zero, point_data[i].slope);

        point_data[i].crvatr_type = UF_CURVE_CRVATR_NONE;
        UF_VEC3_copy(zero, point_data[i].crvatr);
    }

    *pt_data = point_data;
    return(0);
}

int PDCAPP_COM_create_spline_f_pt(
    double* pt,        /* I: array of points */
    int num_points,    /* I: point number */
    int start_tang,    /* I: sign of start */
    double* tangent1,  /* I: start tangent */
    int end_tang,      /* I: sign of end */
    double* tangent2,  /* I: end tangent */
    tag_t* spline_id   /* O: created spline */
)
{
    int degree;
    int periodicity = 0;
    double* parameters = NULL;
    int i, save_def_data = 1;
    UF_CURVE_pt_slope_crvatr_t* point_data;
    int error;
    tag_t sp_id;

    if (num_points < 2)  return(1);

    if (num_points < 4) degree = num_points - 1;
    else degree = 3;

    if (degree < 3)
    {
        if (start_tang) degree++;
        if (end_tang) degree++;
        if (degree > 3) degree = 3;
    }

    PDCAPP_COM_intialize_point_data(num_points, &point_data);

    for (i = 0; i < num_points; i++)
    {
        UF_VEC3_copy(&pt[3 * i], point_data[i].point);
        if (i == 0)
        {
            if (start_tang)
            {
                point_data[i].slope_type = UF_CURVE_SLOPE_DIR;
                UF_VEC3_copy(tangent1, point_data[i].slope);
            }
        }
        else if (i == num_points - 1)
        {
            if (end_tang)
            {
                point_data[i].slope_type = UF_CURVE_SLOPE_DIR;
                UF_VEC3_copy(tangent2, point_data[i].slope);
            }
        }
    }

    error = UF_CURVE_create_spline_thru_pts(degree, periodicity,
        num_points,
        point_data, parameters, save_def_data, &sp_id);
    UF_free(point_data);

    if (!error) *spline_id = sp_id;
    return(error);
}

int PDCAPP_COM_ask_param_to_arclength(
    tag_t curve_id, /* I: id of curve */
    double len,     /* I: length */
    double* param   /* O: parameter */
)
{
    double startp = 0, endp = 1;
    int  status;
    double u1, u2, u0;
    double lin_len, len1;

    status = UF_CURVE_ask_arc_length(curve_id, startp, endp, UF_MODL_UNITS_PART,
        &lin_len);
    if (status) return(1);

    if (len < 0 || len > lin_len) return(1);
    if (len < 10e-6)
    {
        *param = 0;
        return(0);
    }

    u1 = 0.;
    u2 = 1.;
    u0 = 0.5 * (u1 + u2);

    UF_CURVE_ask_arc_length(curve_id, startp, u0, UF_MODL_UNITS_PART,
        &len1);
    while (fabs(len1 - len) > 1e-3)
    {
        if (len > len1)
        {
            u1 = u0;
            u0 = 0.5 * (u1 + u2);
        }
        else
        {
            u2 = u0;
            u0 = 0.5 * (u1 + u2);
        }

        UF_CURVE_ask_arc_length(curve_id, startp, u0, UF_MODL_UNITS_PART,
            &len1);
    }

    *param = u0;

    return(0);
}

extern void ask_curve_point_parm(tag_t curve, double point[3], double& parm)
{
    double p[3];
    UF_MODL_ask_curve_parm(curve, point, &parm, p);

}

void ask_curve_point(tag_t curve, double parm, double(&point)[3])
{
    double tangent[3];
    double a[3];
    double p_norm[3], b_norm[3], torsion, rad;
    UF_MODL_ask_curve_props(curve, parm, point, tangent, p_norm, b_norm, &torsion, &rad);

}

void ask_curve_tangent(tag_t curve, double parm, double(&tan)[3])
{
    double point[3], pn[3], bn[3], torsion, rad;
    UF_MODL_ask_curve_props(curve, parm, point, tan, pn, bn, &torsion, &rad);
}

double ask_curve_length(double param1, double param2, tag_t curve)
{
    double rtr = 0;
    UF_CURVE_ask_arc_length(curve, param1, param2, UF_MODL_MMETER, &rtr);
    return rtr;
}

double ask_min_dis(TaggedObject* obj1, TaggedObject* obj2)
{
    double guess[3];
    double dis;
    double pt1[3], pt2[3];
    UF_MODL_ask_minimum_dist(obj1->Tag(), obj2->Tag(), 0, guess, 0, guess, &dis, pt1, pt2);
    return dis;
}

double ask_min_dis(TaggedObject* obj1, Point3d point)
{
    
    double guess[3];
    double coord[3] = {point.X,point.Y,point.Z};
    double dis;
    double pt1[3], pt2[3];
    int erro = UF_MODL_ask_minimum_dist(obj1->Tag(), NULL_TAG, 0, guess, 1, coord, &dis, pt1, pt2);
    return dis;
}

double ask_min_dis(tag_t& obj1, Point3d point)
{

    double guess[3];
    double coord[3] = { point.X,point.Y,point.Z };
    double dis;
    double pt1[3], pt2[3];
    int erro = UF_MODL_ask_minimum_dist(obj1, NULL_TAG, 0, guess, 1, coord, &dis, pt1, pt2);
    return dis;
}

int FacetBody(tag_t sheet_body, tag_t* facet_model)
{
    UF_FACET_parameters_t faceting_params;

    UF_FACET_INIT_PARAMETERS(&faceting_params);
    UF_FACET_ask_default_parameters(&faceting_params);
    faceting_params.max_facet_edges = 3;
    faceting_params.number_storage_type = 1;
    faceting_params.specify_surface_tolerance = true;
    faceting_params.surface_dist_tolerance = 0.03;//0.03;
    faceting_params.curve_dist_tolerance = 0.03;//0.03;
    faceting_params.surface_angular_tolerance = 0.4;//0.4;
    faceting_params.specify_curve_tolerance = true;
    faceting_params.curve_angular_tolerance = 0.4;//0.4;
    faceting_params.specify_parameters = true;
    faceting_params.specify_view_direction = true;
    faceting_params.silh_view_direction[0] = 0;
    faceting_params.silh_view_direction[1] = 0;
    faceting_params.silh_view_direction[2] = 1;
    faceting_params.silh_chord_tolerance = 0.015;
    faceting_params.curve_max_length = 10;
    faceting_params.specify_max_facet_size = true;
    faceting_params.specify_convex_facets = true;
    faceting_params.max_facet_size = 10;


    tag_t faceted_model = NULL_TAG;
    int err = UF_FACET_facet_solid(sheet_body, &faceting_params, &faceted_model);
    if (err)
    {
        //char msg[133];
        //UF_get_fail_message(err, msg);
        //uc1601(msg,1);
        return err;
    }
    int num_facet;
    UF_FACET_ask_num_faces(faceted_model, &num_facet);

    UF_FACET_disassoc_from_solid(faceted_model);

    *facet_model = faceted_model;

    return 0;
}

void CreateTempLine(double start_point[3], double end_point[3], int color)
{
    UF_OBJ_disp_props_t disp;
    disp.layer = 81;
    disp.color = color;
    disp.blank_status = UF_OBJ_NOT_BLANKED;
    disp.line_width = UF_OBJ_WIDTH_NORMAL;
    disp.font = 0;
    disp.highlight_status = false;

    // 创建临时直线
    UF_DISP_display_temporary_line(NULL, UF_DISP_USE_WORK_VIEW, start_point, end_point, &disp);
}

void SmoothSpline(tag_t& splineTag, int count)
{
    NXOpen::Session* theSession = NXOpen::Session::GetSession();
    NXOpen::Part* workPart(theSession->Parts()->Work());
    NXOpen::Part* displayPart(theSession->Parts()->Display());
    for (size_t i = 0; i < count; i++)
    {

        try
        {
            NXOpen::Spline* spline1(dynamic_cast<NXOpen::Spline*>(NXOpen::NXObjectManager::Get(splineTag)));

            NXOpen::Features::SmoothSpline* nullNXOpen_Features_SmoothSpline(NULL);
            NXOpen::Features::SmoothSplineBuilder* smoothSplineBuilder1;
            smoothSplineBuilder1 = workPart->Features()->CreateSmoothSplineBuilder(nullNXOpen_Features_SmoothSpline);

            smoothSplineBuilder1->SetType(NXOpen::Features::SmoothSplineBuilder::TypesCurvatureVariation);

            smoothSplineBuilder1->CurveRange()->Start()->Update(NXOpen::GeometricUtilities::OnPathDimensionBuilder::UpdateReasonPath);

            smoothSplineBuilder1->CurveRange()->End()->Update(NXOpen::GeometricUtilities::OnPathDimensionBuilder::UpdateReasonPath);

            smoothSplineBuilder1->CurveRange()->Start()->Expression()->SetFormula("0");

            smoothSplineBuilder1->CurveRange()->Center()->Expression()->SetFormula("50");

            smoothSplineBuilder1->CurveRange()->End()->Expression()->SetFormula("100");
            smoothSplineBuilder1->Curve()->SetValue(spline1);

            smoothSplineBuilder1->SetSmoothingFactor(100);

            NXOpen::NXObject* nXObject1;
            nXObject1 = smoothSplineBuilder1->Commit();

            smoothSplineBuilder1->Destroy();

        }
        catch (...)
        {
            return;
        }
    }


}

void RotateVectorAroundAxis(double vector[3], const double axis[3], double angleDeg)
{
    // 角度转弧度

    double angleRad = angleDeg * PI / 180.0;
    double cosTheta = cos(angleRad);
    double sinTheta = sin(angleRad);

    // 罗德里格斯旋转公式：
    // v' = v*cos(θ) + (axis × v)*sin(θ) + axis*(axis·v)*(1-cos(θ))

    // 计算 axis·v（点积）- 使用 UF_VEC3_dot
    double dotProduct;
    UF_VEC3_dot(axis, vector, &dotProduct);  // 通过指针输出

    // 计算 axis × v（叉积）- 使用 UF_VEC3_cross
    double crossProd[3];
    UF_VEC3_cross(axis, vector, crossProd);

    // 应用罗德里格斯公式
    double result[3];
    result[0] = vector[0] * cosTheta +
                crossProd[0] * sinTheta +
                axis[0] * dotProduct * (1.0 - cosTheta);
    result[1] = vector[1] * cosTheta +
                crossProd[1] * sinTheta +
                axis[1] * dotProduct * (1.0 - cosTheta);
    result[2] = vector[2] * cosTheta +
                crossProd[2] * sinTheta +
                axis[2] * dotProduct * (1.0 - cosTheta);

    // 写回原向量 - 使用 UF_VEC3_copy
    UF_VEC3_copy(result, vector);
}

double NormalizeVector(double vector[3])
{
    // 计算向量的模（长度）
    double magnitude = sqrt(vector[0] * vector[0] +
                           vector[1] * vector[1] +
                           vector[2] * vector[2]);

    // 如果向量长度太小（接近零向量），返回0并保持原向量不变
    if (magnitude < 1e-10) {
        return 0.0;
    }

    // 单位化：向量的每个分量除以模
    vector[0] /= magnitude;
    vector[1] /= magnitude;
    vector[2] /= magnitude;

    return magnitude;
}
