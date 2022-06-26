#include <ichthus_lidar_driver_ros2/nodes/backend_node.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace backend_node
  {
    /*************************************************************************/
    /* class BackendNode                                                     */
    /*************************************************************************/
    BackendNode::BackendNode(const rclcpp::NodeOptions &node_options)
        : Node("backend", node_options)
    // : Node("backend", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
      std::cout << "Backend construction " << this->get_name() << std::endl;

      param_.ns = declare_parameter("ns", std::vector<std::string>());
      param_.frame_id = declare_parameter("out_cloud.frame_id", "");
      param_.period_ms = declare_parameter("out_cloud.period_ms", 100);
      param_.use_deblurring = declare_parameter("out_cloud.use_deblurring", true);

      printBackendParams();

      sub_lidar_cloud_.resize(param_.ns.size());
      for (uint32_t ns_i = 0; ns_i < param_.ns.size(); ns_i++)
      {
        std::string key("in_clouds." + param_.ns[ns_i]);
        std::string topic_name("/" + param_.ns[ns_i] + "/lidar_cloud");

        param_.cloud_pose.yaw_ = declare_parameter(key + ".pose_yaw", double());
        param_.cloud_pose.pitch_ = declare_parameter(key + ".pose_pitch", double());
        param_.cloud_pose.roll_ = declare_parameter(key + ".pose_roll", double());
        param_.cloud_pose.x_ = declare_parameter(key + ".pose_x", double());
        param_.cloud_pose.y_ = declare_parameter(key + ".pose_y", double());
        param_.cloud_pose.z_ = declare_parameter(key + ".pose_z", double());

        printPose(param_.ns[ns_i], param_.cloud_pose);

        param_.cloud_pose.yaw_ = deg2Rad(param_.cloud_pose.yaw_);
        param_.cloud_pose.pitch_ = deg2Rad(param_.cloud_pose.pitch_);
        param_.cloud_pose.roll_ = deg2Rad(param_.cloud_pose.roll_);

        in_cloud_arr_.emplace_back(new backend::InputCloud(param_.ns[ns_i], param_.cloud_pose));

        sub_lidar_cloud_[ns_i] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name,
            rclcpp::SensorDataQoS{}.keep_last(1280),
            // 1280,
            [this, ns_i](const sensor_msgs::msg::PointCloud2::UniquePtr msg)
            {
              // this->callbackLiDARCloud(msg, ns_i);

              // TODO: Do not use lambda function
              pcl::PointCloud<pcl::PointXYZITCA> in_cloud;
              pcl::fromROSMsg(*msg, in_cloud);
              in_cloud_arr_[ns_i]->addCloud(in_cloud);
            });
      }

      timer_ = create_wall_timer(
          std::chrono::milliseconds(param_.period_ms),
          std::bind(&BackendNode::callbackTimer, this));

      pub_merged_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/merged_cloud",
          rclcpp::SensorDataQoS{}.keep_last(640));

      sub_can_odom_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/can_odom",
          rclcpp::SensorDataQoS{}.keep_last(20),
          std::bind(&BackendNode::callbackCanOdom, this, std::placeholders::_1));
    }

    BackendNode::~BackendNode()
    {
    }

    void BackendNode::printBackendParams()
    {
      std::cout << "[backend param] ns: ";
      for (size_t i = 0; i < param_.ns.size(); i++)
        std::cout << param_.ns[i] << ", ";
      std::cout << "\n";
      std::cout << "[backend param] frame_id: " << param_.frame_id << std::endl;
      std::cout << "[backend param] period_ms: " << param_.period_ms << std::endl;
      if (param_.use_deblurring)
        std::cout << "[backend param] use_deblurring: true\n";
      else
        std::cout << "[backend param] use_deblurring: false\n";
    }

    void BackendNode::printPose(const std::string ns, const backend::Pose &pose)
    {
      std::cout << "[backend param] " << ns
                << " LiDAR Pose (yaw,pitch,roll,x,y,z) = "
                << pose.yaw_ << " "
                << pose.pitch_ << " "
                << pose.roll_ << " "
                << pose.x_ << " "
                << pose.y_ << " "
                << pose.z_ << std::endl;
    }

    void BackendNode::callbackCanOdom(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
    {
      sensor::Velocity vel{};
      vel.header.stamp = msg->header.stamp;
      vel.linear_x = msg->twist.twist.linear.x;
      vel.angular_z = msg->twist.twist.angular.z;

      for (uint32_t ns_i = 0; ns_i < param_.ns.size(); ns_i++)
      {
        in_cloud_arr_[ns_i]->addVelocity(vel);
      }
    }

    // void BackendNode::callbackLiDARCloud(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const int cld_idx)
    // {
    //   // std::cout << "Received lidar cloud from: " << cld_idx << std::endl;

    //   pcl::PointCloud<pcl::PointXYZITCA> in_cloud;
    //   pcl::fromROSMsg(*msg, in_cloud);
    //   in_cloud_arr_[cld_idx]->addCloud(in_cloud);
    // }

    void BackendNode::callbackTimer()
    {
      pcl::PointCloud<pcl::PointXYZI> out_cloud;

      uint64_t nearest_ts = 0;
      for (uint32_t cld_i = 0; cld_i < in_cloud_arr_.size(); cld_i++)
      {
        pcl::PointCloud<pcl::PointXYZI> tf_cloud;
        in_cloud_arr_[cld_i]->popCloud(tf_cloud, param_.use_deblurring);

        if (nearest_ts < tf_cloud.header.stamp)
        {
          nearest_ts = tf_cloud.header.stamp;
        }

        out_cloud += tf_cloud;
      }
      out_cloud.header.stamp = nearest_ts;

      sensor_msgs::msg::PointCloud2::UniquePtr out_msg(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(out_cloud, *out_msg);
      out_msg->header.frame_id = param_.frame_id;

      // pub_merged_cloud_->publish(std::move(out_msg));
      pub_merged_cloud_->publish(*out_msg);
    }

  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_lidar_driver_ros2::backend_node::BackendNode)