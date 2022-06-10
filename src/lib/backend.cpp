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
      tf2_base_link_to_sensor = tf2_transform;
    }

    void InputCloud::addCloud(pcl::PointCloud<PointT> &in_cloud)
    {
      // pcl::PointCloud<PointT>::Ptr tf_cloud_ptr(new pcl::PointCloud<PointT>);
      // pcl::transformPointCloud(in_cloud, *tf_cloud_ptr, transform_);

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
      //   pcl::io::savePCDFileASCII("/home/eunseo/tmp/" + std::to_string(seq++) + ".pcd", tf_cloud_);
      // }

      out_cloud = tf_cloud_;
      tf_cloud_.clear();
    }

  }
}
