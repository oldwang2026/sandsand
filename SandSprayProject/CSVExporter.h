#ifndef CSVEXPORTER_H_INCLUDED
#define CSVEXPORTER_H_INCLUDED


#include <uf_defs.h>
#include <vector>
#include <string>
#include "regionstruct.h"
#include "SprayPathSequencer.h"

//==============================================================================
// CSV Export Tool Class (Static utility class, cannot be instantiated)
//==============================================================================
class CSVExporter
{
private:
    // Private constructors to prevent instantiation
    CSVExporter() = delete;
    ~CSVExporter() = delete;
    CSVExporter(const CSVExporter&) = delete;
    CSVExporter& operator=(const CSVExporter&) = delete;

public:
    //==========================================================================
    // Robot path point data structure
    //==========================================================================
    struct RobotPathPoint {
        double x, y, z;      // Position coordinates (X, Y, Z)
        double nx, ny, nz;   // Tool axis vector (I, J, K) - points toward spray head
    };

    //==========================================================================
    // Main entry function: Export CSV files
    //==========================================================================
    // Parameters:
    //   csv_curve_select - Selected spray curve list
    //   sprayAngle     - Spray tilt angle (degrees)
    //   sequencer      - Path sequencer (get path direction info)
    //   outputDir      - Output directory path
    static void ExportCSVFiles(
        const std::vector<NXOpen::TaggedObject*>& csv_curve_select,
        double sprayAngle,
        const SprayPathSequencer& sequencer,
        const char* outputDir = "E:\\vsproject\\SandSprayProject\\txtfile"
    );

    //==========================================================================
    // Core processing function: Extract robot path points from curve
    //==========================================================================
    static std::vector<RobotPathPoint> ExtractPointsFromCurve(
        tag_t curveTag,
        double samplingDistance,
        double sprayAngle,
        bool isReversed,
        FaceRegionType regionType,
        tag_t regionBodyTag
    );

    //==========================================================================
    // Tool axis calculation functions
    //==========================================================================

    // Get surface normal at specified point from Region Body
    static bool GetSurfaceNormalAtPoint(
        tag_t regionBodyTag,
        const double point[3],
        double normal[3]
    );

    // Calculate lateral tilt angle (based on first point to determine sign)
    // Return: HORIZONTAL=0deg, CONCAVE/CONVEX=±45deg, SIDEWALL=±60deg
    static double CalculateLateralTiltAngle(
        tag_t regionBodyTag,
        FaceRegionType regionType,
        const double firstTangent[3],
        const double firstPoint[3],
        bool isReversed
    );

    // Unified tool axis calculation (applicable to all region types)
    static bool CalculateToolAxis(
        const double tangent[3],
        bool isReversed,
        double lateralTiltAngle,
        double sprayAngle,
        double toolAxis[3]
    );

    //==========================================================================
    // File operation helper functions
    //==========================================================================

    // Generate CSV filename (with sequence number and timestamp)
    static std::string GenerateCSVFileName(
        int sequenceNumber,
        const std::string& timestamp
    );

    // Write CSV file
    static void WriteCSVFile(
        const std::vector<RobotPathPoint>& points,
        const std::string& filepath
    );

    // Get current timestamp string (format: YYYYMMDD_HHMMSS)
    static std::string GetCurrentTimestamp();
};

#endif // CSVEXPORTER_H_INCLUDED
