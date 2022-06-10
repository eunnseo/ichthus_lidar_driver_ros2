#ifndef _OS1_64_HPP_
#define _OS1_64_HPP_

#include <iostream>

#include <eigen3/Eigen/Core>

#include <ichthus_lidar_driver_ros2/sensor/lidar_interface.hpp>
#include <ichthus_lidar_driver_ros2/sensor/os1_64/os1_64_packet.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace os1_64
    {
      class OusterI64 : public LiDARInterface
      {
      public:
        OusterI64(double lidar_origin_to_beam_origin_mm,
                  const std::vector<double> &transform,
                  const std::vector<double> &azimuth_angles_deg,
                  const std::vector<double> &altitude_angles_deg);

        void initLookUpTable();

        void initResolution();

        void msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<PointT> &out_cloud);

      private:
        const uint32_t encoder_ticks_per_rev_ = 90112; 
      };
    }
  }
}

#endif // _OS1_64_HPP_