#ifndef _FRONTEND_NODE_HPP_
#define _FRONTEND_NODE_HPP_

#include <iostream>
#include <thread>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/create_timer.hpp"

#include <ichthus_lidar_driver_ros2/lib/frontend.hpp>
#include <ichthus_lidar_driver_ros2/msg/packet.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace frontend_node
  {
// #define USE_TIMER

    class FrontendNode : public rclcpp::Node
    {
    public:
      explicit FrontendNode(const rclcpp::NodeOptions &node_options);
      ~FrontendNode();

      int packetLoopThread();

    private:
      const int MAX_BUFFER_SIZE;

      struct Param
      {
        bool use_pcap;
        std::string pcap_file;
        float pcap_wait_factor;

        std::string model;
        std::string ns; // namespace
        std::string frame_id;

        std::string ip_addr;
        long int lidar_port;
        long int imu_port;

        bool use_imu;
        int period_ms;

        std::vector<double> beam_altitude_angles;
        std::vector<double> beam_azimuth_angles;
        std::vector<double> lidar_to_sensor_transform;
        double lidar_origin_to_beam_origin_mm;

        std::vector<int64_t> used_channels;
        std::vector<int64_t> used_azimuths;
        std::vector<int64_t> used_range;

        Param()
            : use_pcap(false), pcap_file(""), pcap_wait_factor(0.6), lidar_port(7502), imu_port(7503), lidar_origin_to_beam_origin_mm(0.0), period_ms(100)
        {
        }
      } param_;

      std::thread poll_thread_;
      frontend::Frontend frontend_;

      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_cloud_;

#ifdef USE_TIMER
      rclcpp::TimerBase::SharedPtr timer_;
      void callbackTimer();
#endif

    };

  } // namespace frontend_node
} // namespace ichthus_lidar_driver_ros2

#endif