#ifndef _LIDAR_INTERFACE_HPP_
#define _LIDAR_INTERFACE_HPP_

#include <iostream>
#include <queue>
#include <chrono>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <ichthus_lidar_driver_ros2/sensor/point_types.h>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    class LiDARInterface
    {
    public:
#define PointT pcl::PointXYZITCA

      using Points = Eigen::Array<double, Eigen::Dynamic, 3>;
      using Matrix4d = Eigen::Matrix<double, 4, 4, Eigen::DontAlign>;
      using nsec = std::chrono::nanoseconds;

      LiDARInterface(double lidar_origin_to_beam_origin_mm,
                     const std::vector<double> &transform,
                     const std::vector<double> &azimuth_angles_deg,
                     const std::vector<double> &altitude_angles_deg,
                     const std::vector<int64_t> &used_channels,
                     const std::vector<int64_t> &used_azimuths)
          : lidar_origin_to_beam_origin_mm_(lidar_origin_to_beam_origin_mm),
            azimuth_angles_deg_(azimuth_angles_deg), altitude_angles_deg_(altitude_angles_deg),
            used_channels_(used_channels), used_azimuths_(used_azimuths)
      {
        std::cout << "LiDARInterface construction\n";
        lidar_to_sensor_transform_ = Matrix4d::Identity();
        for (uint32_t col = 0; col < lidar_to_sensor_transform_.cols(); col++)
        {
          for (uint32_t row = 0; row < lidar_to_sensor_transform_.rows(); row++)
          {
            lidar_to_sensor_transform_(row, col) = transform[row * lidar_to_sensor_transform_.rows() + col];
          }
        }
      }

      virtual void initLookUpTable() = 0;

      virtual void initResolution() = 0;

      virtual void initUsedPoints() = 0;

      virtual void msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud) = 0;

    protected:
      double lidar_origin_to_beam_origin_mm_;
      Matrix4d lidar_to_sensor_transform_;
      std::vector<double> azimuth_angles_deg_;
      std::vector<double> altitude_angles_deg_;

      std::vector<int64_t> used_channels_;
      std::vector<int64_t> used_azimuths_;
      std::vector<std::vector<bool>> is_used_point_;

      size_t num_azimuth_;  // the number of measurements per scan (e.g. 512, 1024, 2048)
      size_t num_channels_; // the number of channels

      struct LookUpTable
      {
        Points direction;
        Points offset;
      } lut_;
    };
  }
}

#endif