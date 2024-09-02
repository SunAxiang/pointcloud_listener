#ifndef CUSTOM_POINT_TYPES_H
#define CUSTOM_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

// Define a custom point type structure with additional fields
struct CustomPointType
{
    PCL_ADD_POINT4D; // Macro to add x, y, z (and padding)
    float doppler;
    float intensity;
    float range_std;
    float azimuth_std;
    float elevation_std;
    float doppler_std;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
} EIGEN_ALIGN16;                    // Ensure 16-byte alignment

// Register the custom point type so that PCL can recognize it
POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, doppler, doppler)(float, intensity, intensity)(float, range_std, range_std)(float, azimuth_std, azimuth_std)(float, elevation_std, elevation_std)(float, doppler_std, doppler_std))

#endif // CUSTOM_POINT_TYPES_H
