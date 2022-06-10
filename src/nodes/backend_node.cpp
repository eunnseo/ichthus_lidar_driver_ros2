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

              // std::cout << "before deblurring\n";
              deblurringPointCloud(in_cloud_arr_[ns_i]->tf2_base_link_to_sensor, *msg);
              // std::cout << "after deblurring\n";

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
      // std::cout << "Received can_odom data" << std::endl;

      // sensor::Velocity vel{};
      // vel.header.stamp = msg->header.stamp;
      // vel.linear_x = msg->twist.twist.linear.x;
      // vel.linear_y = msg->twist.twist.linear.y;
      // vel.angular_z = msg->twist.twist.angular.z;

      // std::cout << "velocity" << std::endl;
      // std::cout << "linear_x = " << vel.linear_x
      //           << " linear_y = " << vel.linear_y
      //           << " angular_z = " << vel.angular_z << std::endl;

      // std::cout << "Before push\n";
      twist_queue_.push_back(*msg);
      // std::cout << "After push\n";
      // std::cout << "twist_queue_.size() = " << twist_queue_.size() << std::endl;

      while (!twist_queue_.empty()) {
        // for replay rosbag
        if (
          rclcpp::Time(twist_queue_.front().header.stamp) >
          rclcpp::Time(msg->header.stamp)) {
          twist_queue_.pop_front();
        } else if (  // NOLINT
          rclcpp::Time(twist_queue_.front().header.stamp) <
          rclcpp::Time(msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
          twist_queue_.pop_front();
        }
        break;
      }
    }

    // void BackendNode::callbackLiDARCloud(const sensor_msgs::msg::PointCloud2::UniquePtr msg, const int cld_idx)
    // {
    //   // std::cout << "Received lidar cloud from: " << cld_idx << std::endl;

    //   deblurringPointCloud(in_cloud_arr_[cld_idx]->tf2_base_link_to_sensor, *msg);

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

    bool BackendNode::deblurringPointCloud(
        const tf2::Transform &tf2_base_link_to_sensor, sensor_msgs::msg::PointCloud2 &points)
    {
      ////////////////////////
      // pcl::PointCloud<PointT>::Ptr cld(new pcl::PointCloud<PointT>);
      // for(int i = 0; i < cld->points.size(); i++)
      // {
      // }
      // cld->clear();
      ////////////////////////

      // if (points.data.empty() || twist_queue_.empty())
      // {
      //   std::cerr << "input_pointcloud->points or twist_queue_ is empty." << std::endl;
      //   return false;
      // }
      if (points.data.empty())
      {
        // std::cerr << "input_pointcloud->points is empty." << std::endl;
        return false;
      }
      if (twist_queue_.empty())
      {
        // std::cerr << "twist_queue_ is empty." << std::endl;
        return false;
      }

      auto time_stamp_field_it = std::find_if(
          std::cbegin(points.fields), std::cend(points.fields),
          [this](const sensor_msgs::msg::PointField &field)
          {
            return field.name == "timestamp";
          });
      if (time_stamp_field_it == points.fields.cend())
      {
        std::cerr << "Required field time stamp doesn't exist in the point cloud." << std::endl;
        return false;
      }

      sensor_msgs::PointCloud2Iterator<float> it_x(points, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(points, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(points, "z");
      sensor_msgs::PointCloud2ConstIterator<double> it_time_stamp(points, "timestamp");

      float theta{0.0f};
      float x{0.0f};
      float y{0.0f};
      double prev_time_stamp_sec{*it_time_stamp};
      const double first_point_time_stamp_sec{*it_time_stamp};

      auto velocity_it = std::lower_bound(
          std::begin(twist_queue_), std::end(twist_queue_),
          first_point_time_stamp_sec, [](const geometry_msgs::msg::TwistWithCovarianceStamped &x, const double t)
          { return rclcpp::Time(x.header.stamp).seconds() < t; });
      velocity_it = velocity_it == std::end(twist_queue_)
                        ? std::end(twist_queue_) - 1
                        : velocity_it;

      const tf2::Transform tf2_base_link_to_sensor_inv{tf2_base_link_to_sensor.inverse()};
      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp)
      {
        for (;
             (velocity_it != std::end(twist_queue_) - 1 &&
              *it_time_stamp > rclcpp::Time(velocity_it->header.stamp).seconds());
             ++velocity_it)
        {
        }

        // float v{static_cast<float>(velocity_it->longitudinal_velocity)};
        // float w{static_cast<float>(velocity_it->heading_rate)};
        float v_x{static_cast<float>(velocity_it->twist.twist.linear.x)};
        float v_y{static_cast<float>(velocity_it->twist.twist.linear.y)};
        float w{static_cast<float>(velocity_it->twist.twist.angular.z)};

        if (std::abs(*it_time_stamp - rclcpp::Time(velocity_it->header.stamp).seconds()) > 0.1)
        {
          // std::cout << "velocity time_stamp is too late. Cloud not interpolate." << std::endl;
          v_x = 0.0f;
          v_y = 0.0f;
          w = 0.0f;
        }

        const float time_offset = static_cast<float>(*it_time_stamp - prev_time_stamp_sec);

        const tf2::Vector3 sensorTF_point{*it_x, *it_y, *it_z};

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

        const tf2::Vector3 sensorTF_trans_point{tf2_base_link_to_sensor * base_linkTF_trans_point};

        *it_x = sensorTF_trans_point.getX();
        *it_y = sensorTF_trans_point.getY();
        *it_z = sensorTF_trans_point.getZ();

        prev_time_stamp_sec = *it_time_stamp;
      }
      return true;
    }

  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_lidar_driver_ros2::backend_node::BackendNode)