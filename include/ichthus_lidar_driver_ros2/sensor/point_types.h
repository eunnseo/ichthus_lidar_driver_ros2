// #define PCL_NO_PRECOMPILE
// #include <pcl/memory.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>

// namespace pcl
// {
//   struct PointXYZI
//   {
//     PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
//     float test;
//     PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
//   } EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

//   POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
//                                     (float, x, x)
//                                     (float, y, y)
//                                     (float, z, z)
//                                     (float, test, test)
// )



#ifndef _POINT_TYPES_H_
#define _POINT_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{
   namespace traits
  {
    // template<> struct asEnum<std::uint64_t> { static const std::uint8_t value = pcl::PCLPointField::UINT32;  };
    // template<> struct asType<pcl::PCLPointField::UINT32>  { using type = std::uint64_t; };
  }

  struct PointXYZICA
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 
    std::uint16_t channel;
    std::uint32_t azimuth;                            
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

  struct PointXYZITCA
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 
    double timestamp;
    std::uint16_t channel;
    std::uint32_t azimuth;                            
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

}


POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZICA,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, channel, channel)
                                  (std::uint32_t, azimuth, azimuth))

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZITCA,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, timestamp, timestamp)
                                  (std::uint16_t, channel, channel)
                                  (std::uint32_t, azimuth, azimuth))

#endif // _POINT_TYPES_H_
