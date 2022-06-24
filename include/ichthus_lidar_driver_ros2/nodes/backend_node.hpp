#ifndef _BACKEND_NODE_HPP_
#define _BACKEND_NODE_HPP_

#include <iostream>
#include <chrono>
#include <cinttypes>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <ichthus_lidar_driver_ros2/lib/backend.hpp>
#include <ichthus_lidar_driver_ros2/msg/packet.hpp>
#include <ichthus_lidar_driver_ros2/sensor/point_types.h>

namespace ichthus_lidar_driver_ros2
{
  namespace backend_node
  {
    class BackendNode : public rclcpp::Node
    {
    public:
      explicit BackendNode(const rclcpp::NodeOptions &node_options);
      ~BackendNode();

      inline double deg2Rad(const double deg)
      {
        return M_PI * (deg / 180.0);
      }

    private:
      struct Param
      {
        std::string frame_id;
        int period_ms;
        bool use_deblurring;

        std::vector<std::string> ns; // namespace
        backend::Pose cloud_pose;

        Param()
            : frame_id("base_link"), period_ms(100), use_deblurring(true)
        {
        }
      } param_;

      rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_can_odom_;
      std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_lidar_cloud_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_cloud_;

      rclcpp::TimerBase::SharedPtr timer_;

      std::vector<std::shared_ptr<backend::InputCloud>> in_cloud_arr_;

      void printBackendParams();
      void printPose(const std::string ns, const backend::Pose &pose);
      
      void callbackLiDARCloud(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const int cld_idx);
      void callbackCanOdom(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
      void callbackTimer();

    }; // class BackendNode

  } // namespace backend_node
} // namespace ichthus_lidar_driver_ros2

#endif
