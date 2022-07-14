#ifndef _FRONTEND_HPP_
#define _FRONTEND_HPP_

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <ichthus_lidar_driver_ros2/net/packet_utils.hpp>
#include <ichthus_lidar_driver_ros2/sensor/os1_64/os1_64.hpp>
#include <ichthus_lidar_driver_ros2/sensor/os1_64/os1_64_packet.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace frontend
  {
    class Frontend
    {
    public:
      explicit Frontend()
          : lidar_port_(7502), imu_port_(7503), lidar_sockfd_(-1), imu_sockfd_(-1), lidar_origin_to_beam_origin_mm_(0.0)
      {
      }

      inline void setLiDARModel(const std::string &model)
      {
        model_ = model;
      }

      inline void setLiDARIPAddr(const std::string &ip_addr)
      {
        ip_addr_ = ip_addr;
      }

      inline void setLiDARPort(const long int lidar_port)
      {
        lidar_port_ = lidar_port;
      }
      inline void setIMUPort(long int imu_port)
      {
        imu_port_ = imu_port;
      }

      inline void setBeamAzimuthAngles(const std::vector<double> &beam_azimuth_angles)
      {
        beam_azimuth_angles_ = beam_azimuth_angles;
      }
      inline void setBeamAltitudeAngles(const std::vector<double> &beam_altitude_angles)
      {
        beam_altitude_angles_ = beam_altitude_angles;
      }

      inline void setToSensorTransform(const std::vector<double> &lidar_to_sensor_transform)
      {
        lidar_to_sensor_transform_ = lidar_to_sensor_transform;
      }

      inline void setToBeamOrigin(const double lidar_origin_to_beam_origin_mm)
      {
        lidar_origin_to_beam_origin_mm_ = lidar_origin_to_beam_origin_mm;
      }

      inline void setUsedChannels(const std::vector<int64_t> &used_channels)
      {
        used_channels_ = used_channels;
      }
      inline void setUsedAzimuths(const std::vector<int64_t> &used_azimuths)
      {
        used_azimuths_ = used_azimuths;
      }
      inline void setUsedRange(const std::vector<double> &used_range)
      {
        used_range_ = used_range;
      }

      void init();
      
      net::PacketState poll();
      bool readLiDARPacket(uint8_t *buf, int max_buf_size, std::string &dst_ipaddr, ssize_t &nbytes);

      void msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud);

      void addPacket(std::vector<uint8_t> &pkt_msg_buf);
      void flushPacket(pcl::PointCloud<pcl::PointXYZITCA> &out_cloud);

    private:
      std::string ip_addr_;
      std::string model_;

      long int lidar_port_;
      long int imu_port_;

      SOCKET lidar_sockfd_;
      SOCKET imu_sockfd_;

      std::vector<double> beam_altitude_angles_;
      std::vector<double> beam_azimuth_angles_;
      std::vector<double> lidar_to_sensor_transform_;
      double lidar_origin_to_beam_origin_mm_;

      std::vector<int64_t> used_channels_;
      std::vector<int64_t> used_azimuths_;
      std::vector<double> used_range_;

      std::shared_ptr<sensor::LiDARInterface> lidar_ptr_;

      std::queue<std::vector<uint8_t>> packet_queue_;

    };
  }
}

#endif