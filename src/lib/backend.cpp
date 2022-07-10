#include <ichthus_lidar_driver_ros2/lib/backend.hpp>
#include <iomanip> 

namespace ichthus_lidar_driver_ros2
{
  namespace backend
  {
    /*************************************************************************/
    /* class InputCloud                                                      */
    /*************************************************************************/
    InputCloud::InputCloud(const std::string ns, const backend::Pose &pose)
    {
      ns_ = ns;
      pose_ = pose;
      initLiDARTF(pose_);
    }

    void InputCloud::initLiDARTF(const backend::Pose &pose)
    {
      Eigen::Translation3f tl(pose.x_, pose.y_, pose.z_);
      Eigen::AngleAxisf rot_x(pose.roll_, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf rot_y(pose.pitch_, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf rot_z(pose.yaw_, Eigen::Vector3f::UnitZ());

      mat_transform_ = (tl * rot_z * rot_y * rot_x).matrix();

      // std::cout << "[backend] Transform matrix (Eigen matrix): " << std::endl;
      // std::cout << mat_transform_ << std::endl;

      tf2::Quaternion quat{};
      tf2::Transform tf2_transform{};
      quat.setRPY(pose.roll_, pose.pitch_, pose.yaw_);
      tf2_transform.setOrigin(tf2::Vector3(pose.x_, pose.y_, pose.z_));
      tf2_transform.setRotation(quat);

      tf2_transform_ = tf2_transform;

      // std::cout << "[backend] Transform matrix (tf2): " << std::endl;
      // std::cout << "tf2_transform_.getOrigin().getX() = " << tf2_transform_.getOrigin().getX() << std::endl;
      // std::cout << "tf2_transform_.getOrigin().getY() = " << tf2_transform_.getOrigin().getY() << std::endl;
      // std::cout << "tf2_transform_.getOrigin().getZ() = " << tf2_transform_.getOrigin().getZ() << std::endl;
      // std::cout << "tf2_transform_.getRotation().getX() = " << tf2_transform_.getRotation().getX() << std::endl;
      // std::cout << "tf2_transform_.getRotation().getY() = " << tf2_transform_.getRotation().getY() << std::endl;
      // std::cout << "tf2_transform_.getRotation().getZ() = " << tf2_transform_.getRotation().getZ() << std::endl;
      // std::cout << "tf2_transform_.getRotation().getW() = " << tf2_transform_.getRotation().getW() << std::endl;
    }

    void InputCloud::addCloud(pcl::PointCloud<pcl::PointXYZITCA> &in_cloud)
    {
      out_cloud_ += in_cloud;
    }

    void InputCloud::popCloud(pcl::PointCloud<pcl::PointXYZI> &out_cloud, const bool use_deblurring)
    {
      if (out_cloud_.empty())
      {
        // std::cerr << "out_cloud_ is empty." << std::endl;
        return;
      }
      else if (velocity_list_.empty())
      {
        std::cerr << "Velocity List is empty." << std::endl;
      }
      else
      {
        /* do nothing */
      }

      /********** save pcd files **********/
      // pcl::PointCloud<pcl::PointXYZITCA> tf_cloud;
      // pcl::transformPointCloud(out_cloud_, tf_cloud, mat_transform_);
      // if (out_cloud_.size() > 0)
      // {
      //   static int seq1 = 0;
      //   pcl::io::savePCDFileBinary("/root/shared_dir/tmp/not_deblurred/" + std::to_string(seq1++) + ".pcd", tf_cloud);
      // }
      // tf_cloud.clear();

      // processPointCloud(out_cloud_, tf2_transform_);
      // if (out_cloud_.size() > 0)
      // {
      //   static int seq2 = 0;
      //   pcl::io::savePCDFileBinary("/root/shared_dir/tmp/deblurred/" + std::to_string(seq2++) + ".pcd", out_cloud_);
      // }
      // pcl::copyPointCloud(out_cloud_, out_cloud);
      /************************************/

      if (use_deblurring)
      {
        // std::cout << "deblurring on\n";
        processPointCloud(out_cloud_, tf2_transform_);

        pcl::copyPointCloud(out_cloud_, out_cloud);
        // out_cloud = out_cloud_;
      }
      else
      {
        // std::cout << "deblurring off\n";
        pcl::PointCloud<pcl::PointXYZITCA> tf_cloud;
        pcl::transformPointCloud(out_cloud_, tf_cloud, mat_transform_);

        pcl::copyPointCloud(tf_cloud, out_cloud);
        // out_cloud = tf_cloud;
      }

      out_cloud_.clear();
    }

    void InputCloud::addVelocity(const sensor::Velocity &vel)
    {
      velocity_list_.push_back(vel);

      while (!velocity_list_.empty())
      {
        if ( // NOLINT
            rclcpp::Time(velocity_list_.front().header.stamp) <
            rclcpp::Time(vel.header.stamp) - rclcpp::Duration::from_seconds(1.0))
        {
          velocity_list_.pop_front();
        }
        else
        {
          /* do nothing */
        }
        break;
      }
    }

    /* deblurring + transform (base_link to sensor) */
    bool InputCloud::processPointCloud(pcl::PointCloud<pcl::PointXYZITCA> &cloud, const tf2::Transform &tf2_transform)
    {
      if (cloud.empty() || velocity_list_.empty())
      {
        // std::cerr << "cloud or velocity_list_ is empty." << std::endl;
        return false;
      }

      pcl::PointCloud<pcl::PointXYZITCA>::iterator point_it = cloud.end() - 1;
      float theta{0.0f};
      float x{0.0f};
      float y{0.0f};
      const double last_point_time_stamp_sec{point_it->timestamp};
      double next_time_stamp_sec{point_it->timestamp};

      auto velocity_it = std::lower_bound(
          velocity_list_.begin(), velocity_list_.end(),
          last_point_time_stamp_sec,
          [](const sensor::Velocity &vel, const double t)
          {
            return rclcpp::Time(vel.header.stamp).seconds() < t;
          });
      velocity_it = velocity_it == velocity_list_.end()
                        ? std::prev(velocity_list_.end())
                        : velocity_it;

      for (; point_it != cloud.begin(); --point_it)
      {
        for (;
             (velocity_it != std::prev(velocity_list_.end()) &&
              point_it->timestamp > rclcpp::Time(velocity_it->header.stamp).seconds());
             ++velocity_it)
        {
        }

        float v{static_cast<float>(velocity_it->linear_x)};
        float w{static_cast<float>(velocity_it->angular_z)};

        double time_diff = std::abs(point_it->timestamp - rclcpp::Time(velocity_it->header.stamp).seconds());
        if (time_diff > 0.1)
        {
          v = 0.0f;
          w = 0.0f;
        }

        const double time_offset = static_cast<double>(next_time_stamp_sec - point_it->timestamp);
        if (time_offset < 0)
        {
          std::cout << "Time offset < 0" << std::endl;
        }

        const tf2::Vector3 sensorTF_point{point_it->x, point_it->y, point_it->z};

        const tf2::Vector3 baselinkTF_point{tf2_transform * sensorTF_point};

        theta -= w * time_offset;
        tf2::Quaternion baselink_quat{};
        baselink_quat.setRPY(0.0, 0.0, theta);

        const double dis = v * time_offset;
        x -= dis * std::cos(theta);
        y -= dis * std::sin(theta);

        tf2::Transform baselinkTF_odom{};
        baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0.0));
        baselinkTF_odom.setRotation(baselink_quat);

        const tf2::Vector3 baselinkTF_trans_point{baselinkTF_odom * baselinkTF_point};

        point_it->x = baselinkTF_trans_point.getX();
        point_it->y = baselinkTF_trans_point.getY();
        point_it->z = baselinkTF_trans_point.getZ();

        next_time_stamp_sec = point_it->timestamp;
      }
      return true;
    }

    // bool InputCloud::deblurringPointCloud(pcl::PointCloud<pcl::PointXYZITCA> &cloud, const tf2::Transform &tf2_base_link_to_sensor_)
    // {
    //   if (cloud.empty() || velocity_queue_.empty())
    //   {
    //     // std::cerr << "cloud or velocity_queue_ is empty." << std::endl;
    //     return false;
    //   }

    //   pcl::PointCloud<pcl::PointXYZITCA>::iterator point_it = cloud.end() - 1;
    //   float theta{0.0f};
    //   float x{0.0f};
    //   float y{0.0f};
    //   const double last_point_time_stamp_sec{point_it->timestamp};
    //   double next_time_stamp_sec{point_it->timestamp};

    //   auto velocity_it = std::lower_bound(
    //       std::begin(velocity_queue_), std::end(velocity_queue_),
    //       last_point_time_stamp_sec,
    //       [](const sensor::Velocity &vel, const double t)
    //       {
    //         return rclcpp::Time(vel.header.stamp).seconds() < t;
    //       });
    //   velocity_it = velocity_it == std::end(velocity_queue_)
    //                     ? std::end(velocity_queue_) - 1
    //                     : velocity_it;

    //   const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor_.inverse()};

    //   for (; point_it != cloud.begin(); --point_it)
    //   {
    //     for (;
    //          (velocity_it != std::end(velocity_queue_) - 1 &&
    //           point_it->timestamp > rclcpp::Time(velocity_it->header.stamp).seconds());
    //          ++velocity_it)
    //     {
    //     }

    //     float v{static_cast<float>(velocity_it->linear_x)};
    //     float w{static_cast<float>(velocity_it->angular_z)};

    //     double time_diff = std::abs(point_it->timestamp - rclcpp::Time(velocity_it->header.stamp).seconds());
    //     if (time_diff > 0.1)
    //     {
    //       v = 0.0f;
    //       w = 0.0f;
    //     }

    //     const double time_offset = static_cast<double>(next_time_stamp_sec - point_it->timestamp);
    //     if (time_offset < 0)
    //     {
    //       std::cout << "Time offset < 0" << std::endl;
    //     }

    //     const tf2::Vector3 sensorTF_point{point_it->x, point_it->y, point_it->z};

    //     const tf2::Vector3 base_linkTF_point{tf2_base_link_to_sensor_inv * sensorTF_point};

    //     theta -= w * time_offset;
    //     tf2::Quaternion baselink_quat{};
    //     baselink_quat.setRPY(0.0, 0.0, theta);

    //     const double dis = v * time_offset;
    //     x -= dis * std::cos(theta);
    //     y -= dis * std::sin(theta);

    //     tf2::Transform baselinkTF_odom{};
    //     baselinkTF_odom.setOrigin(tf2::Vector3(x, y, 0.0));
    //     baselinkTF_odom.setRotation(baselink_quat);

    //     const tf2::Vector3 base_linkTF_trans_point{baselinkTF_odom * base_linkTF_point};

    //     const tf2::Vector3 sensorTF_trans_point{tf2_base_link_to_sensor_ * base_linkTF_trans_point};

    //     point_it->x = sensorTF_trans_point.getX();
    //     point_it->y = sensorTF_trans_point.getY();
    //     point_it->z = sensorTF_trans_point.getZ();

    //     next_time_stamp_sec = point_it->timestamp;
    //   }
    //   return true;
    // }

  }
}
