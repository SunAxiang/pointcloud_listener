#ifndef CUSTOM_POINT_TYPE_CONVERTER_H
#define CUSTOM_POINT_TYPE_CONVERTER_H

#include <pcl/point_types.h>

// Define a custom point type to match the fields in the PointCloud2 message
struct CustomPointType
{
    PCL_ADD_POINT4D; // Preferred way of adding a XYZ+padding
    float doppler;
    float range;
    float power;
    float alpha;
    float beta;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure proper alignment
} EIGEN_ALIGN16;                    // Force SSE alignment

// Register the point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, doppler, Doppler)(float, range, Range)(float, power, Power)(float, alpha, Alpha)(float, beta, Beta))

#endif // CUSTOM_POINT_TYPE_CONVERTER_H
