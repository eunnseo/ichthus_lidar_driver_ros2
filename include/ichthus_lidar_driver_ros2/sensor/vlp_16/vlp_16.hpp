#ifndef _VLP_16_HPP_
#define _VLP_16_HPP_

#include <iostream>
#include <string>
#include <sstream>

#include <eigen3/Eigen/Core>

#include <ichthus_lidar_driver_ros2/sensor/lidar_interface.hpp>
#include <ichthus_lidar_driver_ros2/sensor/vlp_16/vlp_16_packet.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace vlp_16
    {
      class Velodyne16 : public LiDARInterface
      {
      public:
        Velodyne16(double lidar_origin_to_beam_origin_mm,
                  const std::vector<double> &transform,
                  const std::vector<double> &azimuth_angles_deg,
                  const std::vector<double> &altitude_angles_deg,
                  const std::vector<int64_t> &used_channels_,
                  const std::vector<int64_t> &used_azimuths_,
                  const std::vector<double> &used_range_);

        void initLookUpTable();

        void initResolution();

        void initUsedPoints();

        // double getRange(const char *dist_val1, const char *dist_val2);

        void msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud);

      private:
        const uint32_t encoder_ticks_per_rev_ = 90112; 
      };
    }
  }
}

#endif // _VLP_16_HPP_