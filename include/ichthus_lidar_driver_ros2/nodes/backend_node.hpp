#ifndef _BACKEND_NODE_HPP_
#define _BACKEND_NODE_HPP_

#include <iostream>
#include <chrono>

#include <cinttypes>
#include <iomanip>

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

        std::vector<std::string> ns; // namespace
        backend::Pose cloud_pose;

        Param()
            : frame_id("base_link"), period_ms(100)
        {
        }
      } param_;

      rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_can_odom_;
      std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_lidar_cloud_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_cloud_;

      rclcpp::TimerBase::SharedPtr timer_;

      std::vector<std::shared_ptr<backend::InputCloud>> in_cloud_arr_;

      void callbackLiDARCloud(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const int cld_idx);
      void callbackOdomCan(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
      void callbackTimer();

      // deblurring
      std::deque<sensor::Velocity> velocity_queue_;
      std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_queue_;
      bool deblurringPointCloud(const tf2::Transform &tf2_base_link_to_sensor, sensor_msgs::msg::PointCloud2 &points);

    }; // class BackendNode

  } // namespace backend_node
} // namespace ichthus_lidar_driver_ros2

#endif
