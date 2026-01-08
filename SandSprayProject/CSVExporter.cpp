#include "CSVExporter.h"
#include "HelpFunction.h"
#include <NXOpen/NXObject.hxx>
#include <NXOpen/Body.hxx>
#include <uf.h>
#include <uf_modl.h>
#include <uf_curve.h>
#include <uf_vec.h>
#include <uf_ui.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <direct.h>
#include <sys/stat.h>
#include <cfloat>
#include <unordered_map>
#include <uf_object_types.h>
#include <uf_obj.h>
//==============================================================================
// Get current timestamp string (format: YYYYMMDD_HHMMSS)
//==============================================================================
std::string CSVExporter::GetCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);

    struct tm timeinfo;
    localtime_s(&timeinfo, &now_time_t);  // Windows safe version

    std::stringstream ss;
    ss << std::setfill('0')
       << std::setw(4) << (timeinfo.tm_year + 1900)
       << std::setw(2) << (timeinfo.tm_mon + 1)
       << std::setw(2) << timeinfo.tm_mday << "_"
       << std::setw(2) << timeinfo.tm_hour
       << std::setw(2) << timeinfo.tm_min
       << std::setw(2) << timeinfo.tm_sec;

    return ss.str();
}

//==============================================================================
// Generate CSV filename (format: spraypath_XXX_timestamp.csv)
//==============================================================================
std::string CSVExporter::GenerateCSVFileName(
    int sequenceNumber,
    const std::string& timestamp)
{
    std::stringstream ss;
    ss << "C:\\txtfile\\spraypath_"
       << std::setfill('0') << std::setw(3) << sequenceNumber
       << "_" << timestamp << ".csv";
    return ss.str();
}

//==============================================================================
// Write CSV file
//==============================================================================
void CSVExporter::WriteCSVFile(
    const std::vector<RobotPathPoint>& points,
    const std::string& filepath)
{
    std::ofstream csvFile(filepath);
    if (!csvFile.is_open()) {
        // Silent failure if file cannot be opened (or optionally throw exception)
        return;
    }

    // Set output format: fixed decimal point, precision 3 digits
    csvFile << std::fixed << std::setprecision(3);

    // Write data rows (no header)
    for (const auto& pt : points) {
        csvFile << pt.x << "," << pt.y << "," << pt.z << ","
                << pt.nx << "," << pt.ny << "," << pt.nz << std::endl;
    }

    csvFile.close();
}

//==============================================================================
// Helper function: Get surface normal at specified point from Region Body
//==============================================================================
bool CSVExporter::GetSurfaceNormalAtPoint(
    tag_t regionBodyTag,
    const double point[3],
    double normal[3])
{
    // 1. Get all faces from body
    int type, subtype;
	UF_OBJ_ask_type_and_subtype(regionBodyTag, &type, &subtype);
    int num_faces = 0;

	Body* bodyPtr = dynamic_cast<Body*>(NXOpen::NXObjectManager::Get(regionBodyTag));
	auto facesVec = bodyPtr->GetFaces();
	num_faces = static_cast<int>(facesVec.size());

    tag_t* faces = new tag_t[num_faces];
    for (size_t i = 0; i < num_faces; i++)  faces[i] = facesVec[i]->Tag();

    // 2. Find closest face
    double min_distance = DBL_MAX;
    tag_t closest_face = NULL_TAG;
    double closest_point[3];

    int status;

    for (int i = 0; i < num_faces; i++) {
        double min_dist;
        double pt_on_face[3];
        double pt_unused[3];

        status = UF_MODL_ask_minimum_dist(
            faces[i],      // face
            NULL_TAG,      // use point coordinates
            0,             // guess1 not provided
            NULL,
            1,             // guess2 provided
            const_cast<double*>(point),  // point on curve
            &min_dist,
            pt_on_face,    // closest point on face
            pt_unused
        );

        if (status == 0 && min_dist < min_distance) {
            min_distance = min_dist;
            closest_face = faces[i];
            UF_VEC3_copy(pt_on_face, closest_point);
        }
    }


    if (closest_face == NULL_TAG) {
        return false;
    }

    // 3. Get UV parameters
    double uv_param[2];
    double face_pnt[3];
    status = UF_MODL_ask_face_parm(
        closest_face,
        const_cast<double*>(point),  // reference point (point on curve)
        uv_param,                    // output: UV parameters
        face_pnt                     // output: point on surface
    );

    if (status != 0) {
        return false;
    }

    // 4. Get normal vector
    double point_out[3], u1[3], v1[3], u2[3], v2[3];
    double unit_norm[3], radii[2];

    status = UF_MODL_ask_face_props(
        closest_face,
        uv_param,      // UV parameters
        point_out,
        u1, v1,
        u2, v2,
        unit_norm,     // unit normal vector <- this is what we need
        radii
    );

    if (status != 0) {
        return false;
    }

    delete[] faces;

    // 5. Output normal vector
    UF_VEC3_copy(unit_norm, normal);
    return true;
}

//==============================================================================
// Preprocessing function: Calculate lateral tilt angle (determine sign based on first point)
//==============================================================================
double CSVExporter::CalculateLateralTiltAngle(
    tag_t regionBodyTag,
    FaceRegionType regionType,
    const double firstTangent[3],
    const double firstPoint[3],
    bool isReversed)
{
    // HORIZONTAL region has no lateral tilt
    if (regionType == FaceRegionType::HORIZONTAL) {
        return 0.0;
    }

    // 1. Calculate along-segment vector T1
    double T1[3];
    if (!isReversed) {
        UF_VEC3_copy(firstTangent, T1);
    } else {
        T1[0] = -firstTangent[0];
        T1[1] = -firstTangent[1];
        T1[2] = -firstTangent[2];
    }

    // 2. Construct dividing plane normal: PlaneNormal = normalize(T1 x [0,0,1])
    double z_axis[3] = {0, 0, 1};
    double plane_normal[3];
    UF_VEC3_cross(T1, z_axis, plane_normal);

    double mag, plane_normal_unit[3];
    UF_VEC3_unitize(plane_normal, 0.0218, &mag, plane_normal_unit);

    // 3. Get surface normal N
    double surface_normal[3];
    if (!GetSurfaceNormalAtPoint(regionBodyTag, firstPoint, surface_normal)) {
        // Return default value 0 (no lateral tilt) on failure
        return 0.0;
    }

    // 4. Correct normal (ensure z > 0, pointing upward)
    if (surface_normal[2] < 0) {
        surface_normal[0] = -surface_normal[0];
        surface_normal[1] = -surface_normal[1];
        surface_normal[2] = -surface_normal[2];
    }

    // 5. Determine lateral tilt angle sign
    double side;
    UF_VEC3_dot(surface_normal, plane_normal_unit, &side);

    // 6. Return lateral tilt angle based on region type
    double lateralTiltAngle;
    if (regionType == FaceRegionType::SIDEWALL) {
        lateralTiltAngle = (side > 0) ? 60.0 : -60.0;
    } else {  // CONCAVE or CONVEX
        lateralTiltAngle = (side > 0) ? 45.0 : -45.0;
    }

    return lateralTiltAngle;
}

//==============================================================================
// Unified tool axis calculation function (applicable to all region types)
//==============================================================================
bool CSVExporter::CalculateToolAxis(
    const double tangent[3],
    bool isReversed,
    double lateralTiltAngle,  // lateral tilt angle from preprocessing
    double sprayAngle,
    double toolAxis[3])
{
    // 1. Initial tool axis (Z-axis)
    double tool_axis[3] = {0, 0, 1};

    // 2. Calculate along-segment axis (X-axis)
    double tangent_vec[3];
    if (!isReversed) {
        UF_VEC3_copy(tangent, tangent_vec);
    } else {
        tangent_vec[0] = -tangent[0];
        tangent_vec[1] = -tangent[1];
        tangent_vec[2] = -tangent[2];
    }

    // 2.1 Normalize tangent_vec to ensure it's a unit vector (required by RotateVectorAroundAxis)
    NormalizeVector(tangent_vec);

    // 3. Rotate lateral tilt angle (around along-segment axis X)
    if (lateralTiltAngle != 0.0) {
        RotateVectorAroundAxis(tool_axis, tangent_vec, lateralTiltAngle);
    }

    // 4. Calculate tilt angle rotation axis (Y-axis = tool axis x along-segment axis)
    double tilt_axis[3];
    UF_VEC3_cross(tool_axis, tangent_vec, tilt_axis);

    double mag, tilt_axis_unit[3];
    UF_VEC3_unitize(tilt_axis, 1e-10, &mag, tilt_axis_unit);

    // 5. Rotate tilt angle (around Y-axis, negative angle)
    if (sprayAngle != 0.0) {
        RotateVectorAroundAxis(tool_axis, tilt_axis_unit, -sprayAngle);
    }

    // 6. Normalize tool axis to ensure unit vector
    NormalizeVector(tool_axis);

    // 7. Output
    UF_VEC3_copy(tool_axis, toolAxis);
    return true;
}

//==============================================================================
// Extract robot path points from curve
//==============================================================================
std::vector<CSVExporter::RobotPathPoint>
CSVExporter::ExtractPointsFromCurve(
    tag_t curveTag,
    double samplingDistance,
    double sprayAngle,
    bool isReversed,
    FaceRegionType regionType,
    tag_t regionBodyTag)
{
    std::vector<RobotPathPoint> pathPoints;

    // 1. Get curve total length
    double totalLength = 0.0;
    int status = UF_CURVE_ask_arc_length(curveTag, 0.0, 1.0,
                                         UF_MODL_UNITS_PART, &totalLength);
    if (status != 0 || totalLength <= 0.0) {
        return pathPoints;  // return empty list
    }

    // 2. Calculate number of sample points (at least 2 points: start and end)
    int numSamples = static_cast<int>(totalLength / samplingDistance) + 1;
    if (numSamples < 2) {
        numSamples = 2;
    }

    // ======== Preprocessing: Calculate lateral tilt angle (based on first point) ========
    double lateralTiltAngle = 0.0;

    if (regionType != FaceRegionType::UNSORTED && regionBodyTag != NULL_TAG) {
        // Get first point position and tangent
        double firstParameter = 0.0;
        double firstArcLength = isReversed ? totalLength : 0.0;
        PDCAPP_COM_ask_param_to_arclength(curveTag, firstArcLength, &firstParameter);

        double firstPoint[3], firstTangent[3], p_norm[3], b_norm[3];
        double torsion, rad_of_cur;
        UF_MODL_ask_curve_props(curveTag, firstParameter,
                                firstPoint, firstTangent, p_norm, b_norm,
                                &torsion, &rad_of_cur);

        // Call preprocessing function to determine lateral tilt angle
        lateralTiltAngle = CalculateLateralTiltAngle(
            regionBodyTag, regionType,
            firstTangent, firstPoint, isReversed
        );
    }

    // 3. Sample uniformly along curve
    for (int i = 0; i < numSamples; i++) {
        // 3.1 Calculate current arc length position (direction determined by isReversed)
        int sampleIndex = isReversed ? (numSamples - 1 - i) : i;
        double arcLength = (sampleIndex * totalLength) / (numSamples - 1);

        // 3.2 Arc length to parameter
        double parameter = 0.0;
        PDCAPP_COM_ask_param_to_arclength(curveTag, arcLength, &parameter);

        // 3.3 Get point coordinates and curve properties (including tangent)
        double point[3], tangent[3], p_norm[3], b_norm[3];
        double torsion, rad_of_cur;
        UF_MODL_ask_curve_props(curveTag, parameter,
                                point, tangent, p_norm, b_norm,
                                &torsion, &rad_of_cur);

        // Construct robot path point
        RobotPathPoint rp;
        rp.x = point[0];
        rp.y = point[1];
        rp.z = point[2];

        // ======== Unified tool axis vector calculation (surface normal) ========
        double toolAxisVec[3] = {0.0, 0.0, 0.0};
        bool success = CalculateToolAxis(
            tangent,             // current point tangent
            isReversed,          // is reversed
            lateralTiltAngle,    // lateral tilt angle from preprocessing
            sprayAngle,          // tilt angle
            toolAxisVec          // output: tool axis vector
        );

        if (!success) {
            // Use default value (vertical upward) on failure
            toolAxisVec[0] = 0.0;
            toolAxisVec[1] = 0.0;
            toolAxisVec[2] = 1.0;
        }

        // Store tool axis vector (surface normal, corresponding to CNC I,J,K)
        rp.nx = toolAxisVec[0];  // I
        rp.ny = toolAxisVec[1];  // J
        rp.nz = toolAxisVec[2];  // K

        pathPoints.push_back(rp);
    }

    return pathPoints;
}

// Main CSV export function
// Main CSV export function
void CSVExporter::ExportCSVFiles(
    const std::vector<NXOpen::TaggedObject*>& csv_curve_select,
    double sprayAngle,
    const SprayPathConnector& sequencer, // <--- 修改 1: 类型更新
    const char* outputDir)
{
    try {

        if (csv_curve_select.empty()) {
            return;
        }


        struct stat info;
        if (stat(outputDir, &info) != 0) {
            // Directory doesn't exist, create it
            _mkdir(outputDir);
        }


        std::string timestamp = GetCurrentTimestamp();


        std::unordered_map<tag_t, bool> curveDirectionMap;

        // 获取优化后的序列
        std::vector<std::pair<int, bool>> sequence = sequencer.getFinalSequence();

        if (!sequence.empty()) {

            for (const auto& item : sequence) {
                int pathIndex = item.first;

                // <--- 修改 2: 逻辑转换开始 --->
                // item.second 是 entryIsStart (true = 从起点进 = 正向)
                // extractPoints 需要 isReversed (true = 反向)
                // 所以逻辑取反：
                bool isReversed = !item.second;
                // <--- 修改 2: 逻辑转换结束 --->

                if (pathIndex >= 0 && pathIndex < csv_curve_select.size()) {
                    NXOpen::NXObject* obj = dynamic_cast<NXOpen::NXObject*>(csv_curve_select[pathIndex]);
                    if (obj != nullptr) {
                        curveDirectionMap[obj->Tag()] = isReversed;
                    }
                }
            }
        }


        int successCount = 0;
        for (size_t i = 0; i < csv_curve_select.size(); i++) {

            NXOpen::NXObject* nxObj = dynamic_cast<NXOpen::NXObject*>(csv_curve_select[i]);
            if (nxObj == nullptr) continue;

            tag_t curveTag = nxObj->Tag();

            FaceRegionType regionType = FaceRegionType::UNSORTED;
            try {
                int typeValue = nxObj->GetIntegerAttribute("RegionType");
                regionType = static_cast<FaceRegionType>(typeValue);
            }
            catch (...) {

            }


            tag_t regionBodyTag = NULL_TAG;
            try {
                regionBodyTag = nxObj->GetIntegerAttribute("RegionTag");
            }
            catch (...) {

            }


            bool isReversed = false;
            auto it = curveDirectionMap.find(curveTag);
            if (it != curveDirectionMap.end()) {
                isReversed = it->second;
            }


            std::vector<RobotPathPoint> points =
                ExtractPointsFromCurve(curveTag, 10.0, sprayAngle, isReversed, regionType, regionBodyTag);

            if (points.empty()) {
                continue;  // skip invalid curve
            }


            std::string filename = GenerateCSVFileName(i + 1, timestamp);

            // Write CSV file
            WriteCSVFile(points, filename);

            successCount++;
        }
    }
    catch (...) {

        return;
    }
}
