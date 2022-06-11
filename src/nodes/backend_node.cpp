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
      std::cout << "\tBackend contruction " << this->get_name() << std::endl;

      param_.ns = declare_parameter("ns", std::vector<std::string>());
      param_.frame_id = declare_parameter("out_cloud.frame_id", "");
      param_.period_ms = declare_parameter("out_cloud.period_ms", 100);

      sub_lidar_cloud_.resize(param_.ns.size());
      for (uint32_t ns_i = 0; ns_i < param_.ns.size(); ns_i++)
      {
        std::cout << "ns = " << param_.ns[ns_i] << std::endl;
        std::string key("in_clouds." + param_.ns[ns_i]);
        std::string topic_name("/" + param_.ns[ns_i] + "/lidar_cloud");

        param_.cloud_pose.yaw_ = declare_parameter(key + ".pose_yaw", double());
        param_.cloud_pose.pitch_ = declare_parameter(key + ".pose_pitch", double());
        param_.cloud_pose.roll_ = declare_parameter(key + ".pose_roll", double());
        param_.cloud_pose.x_ = declare_parameter(key + ".pose_x", double());
        param_.cloud_pose.y_ = declare_parameter(key + ".pose_y", double());
        param_.cloud_pose.z_ = declare_parameter(key + ".pose_z", double());

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
              // printf(
              //   " Received message with value: %u, and address: 0x%" PRIXPTR "\n", msg->header.stamp.nanosec,
              //   reinterpret_cast<std::uintptr_t>(msg.get()));

              // this->callbackLiDARCloud(msg, ns_i);

              // deblurringPointCloud(in_cloud_arr_[ns_i]->tf2_base_link_to_sensor, *msg);

              // TODO: Do not use lambda function
              pcl::PointCloud<PointT> in_cloud;
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
          rclcpp::SensorDataQoS{}.keep_last(1280),
          std::bind(&BackendNode::callbackOdomCan, this, std::placeholders::_1));
    }

    BackendNode::~BackendNode()
    {
    }

    void BackendNode::callbackOdomCan(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
    {
      std::cout << "Received can_odom data" << std::endl;

      sensor::Velocity vel{};
      vel.header.stamp = msg->header.stamp;
      vel.linear_x = msg->twist.twist.linear.x;
      vel.linear_y = msg->twist.twist.linear.y;
      vel.angular_z = msg->twist.twist.angular.z;

      for (uint32_t ns_i = 0; ns_i < param_.ns.size(); ns_i++)
      {
        in_cloud_arr_[ns_i]->addVelocity(vel);
      }

    }

    // void BackendNode::callbackLiDARCloud(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const int cld_idx)
    // {
    //   // std::cout << "Received lidar cloud from: " << cld_idx << std::endl;

    //   pcl::PointCloud<PointT> in_cloud;
    //   pcl::fromROSMsg(*msg, in_cloud);
    //   in_cloud_arr_[cld_idx]->addCloud(in_cloud);
    // }

    void BackendNode::callbackTimer()
    {
      pcl::PointCloud<PointT> out_cloud;
      uint64_t oldest_ts = 0;
      for (uint32_t cld_i = 0; cld_i < in_cloud_arr_.size(); cld_i++)
      {
        pcl::PointCloud<PointT> tf_cloud;
        in_cloud_arr_[cld_i]->popCloud(tf_cloud);

        if (oldest_ts > tf_cloud.header.stamp || oldest_ts == 0)
        {
          oldest_ts = tf_cloud.header.stamp;
        }

        out_cloud += tf_cloud;
      }
      out_cloud.header.stamp = oldest_ts;
      // std::cout << "Backend merged_cloud timestamp: " << out_cloud.header.stamp << std::endl;

      sensor_msgs::msg::PointCloud2::UniquePtr out_msg(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(out_cloud, *out_msg);
      out_msg->header.frame_id = param_.frame_id;

      pub_merged_cloud_->publish(std::move(out_msg));
    }

  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_lidar_driver_ros2::backend_node::BackendNode)