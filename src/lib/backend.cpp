#include <ichthus_lidar_driver_ros2/lib/backend.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace backend
  {
    /*************************************************************************/
    /* class InputCloud                                                      */
    /*************************************************************************/
    InputCloud::InputCloud(const std::string ns, const backend::Pose &pose)
    {
      std::cout << "InputCloud constructor: " << ns << std::endl;

      ns_ = ns;
      pose_ = pose;
      initLiDARTF(pose_);

      std::cout << std::endl;
    }

    void InputCloud::initLiDARTF(const backend::Pose &pose)
    {
      Eigen::Translation3f tl(pose.x_, pose.y_, pose.z_);
      Eigen::AngleAxisf rot_x(pose.roll_, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf rot_y(pose.pitch_, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf rot_z(pose.yaw_, Eigen::Vector3f::UnitZ());

      transform_ = (tl * rot_z * rot_y * rot_x).matrix();

      std::cout << "Transform matrix: " << std::endl;
      std::cout << transform_ << std::endl;

      tf2::Quaternion quat{};
      tf2::Transform tf2_transform{};
      quat.setRPY(pose.roll_, pose.pitch_, pose.yaw_);
      tf2_transform.setOrigin(tf2::Vector3(pose.x_, pose.y_, pose.z_));
      tf2_transform.setRotation(quat);
      tf2_base_link_to_sensor_ = tf2_transform;
    }

    void InputCloud::addCloud(pcl::PointCloud<PointT> &in_cloud)
    {
      // pcl::PointCloud<PointT>::Ptr tf_cloud_ptr(new pcl::PointCloud<PointT>);
      // pcl::transformPointCloud(in_cloud, *tf_cloud_ptr, transform_);

      // TODO: Merge deblurringPointCloud and transformPointCloud
      deblurringPointCloud(in_cloud);

      pcl::PointCloud<PointT> tf_cloud;
      pcl::transformPointCloud(in_cloud, tf_cloud, transform_);

      tf_cloud_ += tf_cloud;
      // tf_cloud_.clear();
    }

    void InputCloud::popCloud(pcl::PointCloud<PointT> &out_cloud)
    {
      if (tf_cloud_.size() == 0)
      {
        return;
      }
      // if (tf_cloud_.size() > 0)
      // {
      //   static int seq = 0;
      //   pcl::io::savePCDFileBinary("/home/eunseo/tmp/" + std::to_string(seq++) + ".pcd", tf_cloud_);
      // }

      out_cloud = tf_cloud_;
      tf_cloud_.clear();
    }

    void InputCloud::addVelocity(const sensor::Velocity &vel)
    {
      velocity_queue_.push_back(vel);

      while (!velocity_queue_.empty())
      {
        // for replay rosbag
        if (
            rclcpp::Time(velocity_queue_.front().header.stamp) >
            rclcpp::Time(vel.header.stamp))
        {
          velocity_queue_.pop_front();
        }
        else if ( // NOLINT
            rclcpp::Time(velocity_queue_.front().header.stamp) <
            rclcpp::Time(vel.header.stamp) - rclcpp::Duration::from_seconds(1.0))
        {
          velocity_queue_.pop_front();
        }
        break;
      }
    }

    bool InputCloud::deblurringPointCloud(pcl::PointCloud<PointT> &cloud)
    {
      if (cloud.empty() || velocity_queue_.empty())
      {
        std::cerr << "cloud or velocity_queue_ is empty." << std::endl;
        return false;
      }

      pcl::PointCloud<PointT>::iterator point_it = cloud.begin();
      float theta{0.0f};
      float x{0.0f};
      float y{0.0f};
      double prev_time_stamp_sec{point_it->timestamp};
      const double first_point_time_stamp_sec{point_it->timestamp};

      auto velocity_it = std::lower_bound(
          std::begin(velocity_queue_), std::end(velocity_queue_),
          first_point_time_stamp_sec,
          [](const sensor::Velocity &vel, const double t)
          {
            return rclcpp::Time(vel.header.stamp).seconds() < t;
          });
      velocity_it = velocity_it == std::end(velocity_queue_)
                        ? std::end(velocity_queue_) - 1
                        : velocity_it;

      const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor_.inverse()};
      for (; point_it != cloud.end(); ++point_it)
      {
        for (;
             (velocity_it != std::end(velocity_queue_) - 1 &&
              point_it->timestamp > rclcpp::Time(velocity_it->header.stamp).seconds());
             ++velocity_it)
        {
        }

        float v_x{static_cast<float>(velocity_it->linear_x)};
        float v_y{static_cast<float>(velocity_it->linear_y)};
        float w{static_cast<float>(velocity_it->angular_z)};

        if (std::abs(point_it->timestamp - rclcpp::Time(velocity_it->header.stamp).seconds()) > 0.1)
        {
          std::cout << "velocity time_stamp is too late. Cloud not interpolate." << std::endl;
          v_x = 0.0f;
          v_y = 0.0f;
          w = 0.0f;
        }

        const float time_offset = static_cast<float>(point_it->timestamp - prev_time_stamp_sec);

        const tf2::Vector3 sensorTF_point{point_it->x, point_it->y, point_it->z};

        const tf2::Vector3 base_linkTF_point{tf2_base_link_to_sensor_inv * sensorTF_point};

        theta += w * time_offset;
        tf2::Quaternion baselink_quat{};
        baselink_quat.setRPY(0.0, 0.0, theta);

        // const float dis = v * time_offset;
        // x += dis * std::cos(theta);
        // y += dis * std::sin(theta);
        x += v_x * time_offset;
        y += v_y * time_offset;

        tf2::Transform baselinkTF_odom{};
        baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0.0));
        baselinkTF_odom.setRotation(baselink_quat);

        const tf2::Vector3 base_linkTF_trans_point{baselinkTF_odom * base_linkTF_point};

        const tf2::Vector3 sensorTF_trans_point{tf2_base_link_to_sensor_ * base_linkTF_trans_point};

        point_it->x = sensorTF_trans_point.getX();
        point_it->y = sensorTF_trans_point.getY();
        point_it->z = sensorTF_trans_point.getZ();
        prev_time_stamp_sec = point_it->timestamp;
      }
      return true;
    }

  }
}
