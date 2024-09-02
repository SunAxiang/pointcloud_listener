#ifndef CUSTOM_POINT_TYPE_H
#define CUSTOM_POINT_TYPE_H

#include <pcl/point_types.h>

// Define a custom point type with all the required fields
struct EIGEN_ALIGN16 CustomPointType
{
    PCL_ADD_POINT4D;     // Add the usual XYZ fields
    float doppler;       // Doppler
    float intensity;     // Intensity
    float range_std;     // Range standard deviation
    float azimuth_std;   // Azimuth standard deviation
    float elevation_std; // Elevation standard deviation
    float doppler_std;   // Doppler standard deviation

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
} EIGEN_ALIGN16;

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, doppler, doppler)(float, intensity, intensity)(float, range_std, range_std)(float, azimuth_std, azimuth_std)(float, elevation_std, elevation_std)(float, doppler_std, doppler_std))

#endif // CUSTOM_POINT_TYPE_H
