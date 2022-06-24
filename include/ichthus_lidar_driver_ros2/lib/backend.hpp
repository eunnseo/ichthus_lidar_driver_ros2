#ifndef _BACKEND_HPP_
#define _BACKEND_HPP_

#include <iostream>
#include <queue>
#include <iomanip>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <ichthus_lidar_driver_ros2/sensor/velocity.hpp>
#include <ichthus_lidar_driver_ros2/sensor/point_types.h>

namespace ichthus_lidar_driver_ros2
{
  namespace backend
  {
#define PointT pcl::PointXYZITCA

    struct Pose
    {
      double yaw_; // radian
      double pitch_;
      double roll_;
      double x_; // meter
      double y_;
      double z_;

      Pose()
          : yaw_(0.0), pitch_(0.0), roll_(0.0), x_(0.0), y_(0.0), z_(0.0)
      {
      }
    }; // struct Pose

    class InputCloud
    {
    public:
      explicit InputCloud(const std::string ns, const backend::Pose &pose);
      ~InputCloud();

      void initLiDARTF(const Pose &pose);

      void addCloud(pcl::PointCloud<PointT> &in_cloud);
      void popCloud(pcl::PointCloud<PointT> &out_cloud, const bool use_deblurring);

      // void clearCloud();

      void addVelocity(const sensor::Velocity &vel);

      bool processPointCloud(pcl::PointCloud<PointT> &cloud, const tf2::Transform &tf2_transform);
      // bool deblurringPointCloud(pcl::PointCloud<PointT> &cloud, const tf2::Transform &tf2_base_link_to_sensor_);

    private:
      std::string ns_;
      Pose pose_;
      Eigen::Matrix4f mat_transform_; // base_link to sensor
      tf2::Transform tf2_transform_; // base_link to sensor

      pcl::PointCloud<PointT> out_cloud_;
      pcl::PointCloud<PointT> in_cloud_;

      // TODO: std::deque -> std::list
      std::deque<sensor::Velocity> velocity_queue_;
    }; // class InputCloud

  }
}

#endif // _BACKEND_HPP_