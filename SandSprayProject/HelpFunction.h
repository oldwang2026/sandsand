#pragma once

#include <uf.h>
#include <NXOpen/BlockStyler_ListBox.hxx>
#include <NXOpen/Point.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/UI.hxx>
#include <NXOpen/NXMessageBox.hxx>
#include <NXOpen/Callback.hxx>
#include <uf_curve.h>
#include <NXOpen/Spline.hxx>
#include <NXOpen/SplineCollection.hxx>
#include <NXOpen/TaggedObject.hxx>
#include <NXOpen/Features_FeatureCollection.hxx>
#include <NXOpen/Features_SmoothSplineBuilder.hxx>
#include <NXOpen/NXObjectManager.hxx>
#include <NXOpen/SelectCurve.hxx>
using namespace NXOpen;

#define TOLERANCE 0.1;


double PointDistance(const Point3d& p1, const Point3d& p2);

double CalculateVectorAngle(const Vector3d& v1, const Vector3d& v2);

int PDCAPP_COM_intialize_point_data
(
	int num_points,
	UF_CURVE_pt_slope_crvatr_t** pt_data
);

int PDCAPP_COM_create_spline_f_pt(
	double* pt,        /* I: array of points */
	int num_points,    /* I: point number */
	int start_tang,    /* I: sign of start */
	double* tangent1,  /* I: start tangent */
	int end_tang,      /* I: sign of end */
	double* tangent2,  /* I: end tangent */
	tag_t* spline_id   /* O: created spline */
);

int PDCAPP_COM_ask_param_to_arclength(
	tag_t curve_id, /* I: id of curve */
	double len,     /* I: length */
	double* param   /* O: parameter */
);

void ask_curve_point_parm(tag_t curve, double point[3], double& parm);

void ask_curve_point(tag_t curve, double parm, double(&point)[3]);

void ask_curve_tangent(tag_t curve, double parm, double(&tan)[3]);

double ask_curve_length(double param1, double param2, tag_t curve);

double ask_min_dis(TaggedObject* obj1,TaggedObject* obj2);

double ask_min_dis(TaggedObject* obj1, Point3d point);

double ask_min_dis(tag_t& obj1, Point3d point);

int FacetBody(tag_t sheet_body, tag_t* facet_model);

void CreateTempLine(double start_point[3], double end_point[3], int color);

void SmoothSpline(tag_t& splineTag, int count);

/**
 * @brief 绕任意轴旋转向量（罗德里格斯旋转公式）
 * @param vector 待旋转的向量（输入/输出）
 * @param axis 旋转轴向量（必须是单位向量）
 * @param angleDeg 旋转角度（度）
 */
void RotateVectorAroundAxis(double vector[3], const double axis[3], double angleDeg);

/**
 * @brief 向量单位化（归一化）
 * @param vector 待单位化的向量（输入/输出），将被修改为单位向量
 * @return 返回原向量的长度（模），如果向量为零向量则返回0
 */
double NormalizeVector(double vector[3]);

