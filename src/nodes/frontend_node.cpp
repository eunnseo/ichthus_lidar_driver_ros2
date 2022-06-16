#include <ichthus_lidar_driver_ros2/nodes/frontend_node.hpp>

#include <unistd.h>


namespace ichthus_lidar_driver_ros2
{
  namespace frontend_node
  {
    FrontendNode::FrontendNode(const rclcpp::NodeOptions &node_options)
        : Node("frontend", node_options), MAX_BUFFER_SIZE(100000)
        // : Node("frontend", rclcpp::NodeOptions().use_intra_process_comms(true)), MAX_BUFFER_SIZE(100000)
    {
      std::cout << "\tFrontend contruction " << this->get_name() << std::endl;

      param_.pcap_file = declare_parameter("pcap_file", "");
      param_.use_pcap = (param_.pcap_file != "");
      param_.pcap_wait_factor = declare_parameter("pcap_wait_factor", 0.6);
      param_.model = declare_parameter("model", "");
      param_.ns = declare_parameter("ns", "");
      param_.frame_id = declare_parameter("frame_id", "");
      param_.ip_addr = declare_parameter("ip_addr", "");
      param_.lidar_port = declare_parameter("lidar_port", 7502);
      param_.imu_port = declare_parameter("imu_port", 7503);
      param_.use_imu = declare_parameter("use_imu", false);
      param_.period_ms = declare_parameter("period_ms", 100);

      /* lidar calibration parameters */
      param_.beam_azimuth_angles = declare_parameter("beam_azimuth_angles", std::vector<double>());
      param_.beam_altitude_angles = declare_parameter("beam_altitude_angles", std::vector<double>());
      param_.lidar_to_sensor_transform = declare_parameter("lidar_to_sensor_transform", std::vector<double>());
      param_.lidar_origin_to_beam_origin_mm = declare_parameter("lidar_origin_to_beam_origin_mm", 0.0);
      param_.used_channels = declare_parameter("used_channels", std::vector<int64_t>());
      param_.used_azimuths = declare_parameter("used_azimuths", std::vector<int64_t>());
      param_.used_range = declare_parameter("used_range", std::vector<int64_t>());
      /*******************************/

      // for (size_t i = 0; i < param_.used_channels.size(); i++)
      // {
      //   std::cout << param_.used_channels[i] << " ";
      // }
      // std::cout << "\n";
      // for (size_t j = 0; j < param_.used_azimuths.size(); j++)
      // {
      //   std::cout << param_.used_azimuths[j] << " ";
      // }
      // std::cout << "\n";
      
      if (!param_.use_pcap)
      {
        frontend_.setLiDARModel(param_.model);
        frontend_.setLiDARIPAddr(param_.ip_addr);
        frontend_.setLiDARPort(param_.lidar_port);
        frontend_.setIMUPort(param_.imu_port);

        // TODO: 파라미터 값을 frontend_node에 저장할 필요 없이, 바로 frontend에게 넘겨주면 됨
        frontend_.setBeamAzimuthAngles(param_.beam_azimuth_angles);
        frontend_.setBeamAltitudeAngles(param_.beam_altitude_angles);
        frontend_.setToSensorTransform(param_.lidar_to_sensor_transform);
        frontend_.setToBeamOrigin(param_.lidar_origin_to_beam_origin_mm);

        frontend_.setUsedChannels(param_.used_channels);
        frontend_.setUsedAzimuths(param_.used_azimuths);
        frontend_.setUsedRange(param_.used_range);

        frontend_.init();
      }

      pub_lidar_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          param_.ns + "/lidar_cloud",
          // 5);
          rclcpp::SensorDataQoS{}.keep_last(1280));

#ifdef USE_TIMER
      timer_ = create_wall_timer(
          std::chrono::milliseconds(param_.period_ms),
          std::bind(&FrontendNode::callbackTimer, this));
#endif

      poll_thread_ = std::thread(&FrontendNode::packetLoopThread, this);
    }

    FrontendNode::~FrontendNode()
    {
      poll_thread_.join();
    }

    int FrontendNode::packetLoopThread()
    {
      std::cout << "packetLoopThread.." << std::endl;
      int retval;
      long nbytes = -1;
      std::vector<uint8_t> buf(MAX_BUFFER_SIZE + 1); // -> 포인터
      std::string dst_ipaddr;
      net::PacketState state;

      while (rclcpp::ok())
      {
        if (!param_.use_pcap)
        {
          state = frontend_.poll();
          if (state == net::PacketState::EXIT)
          {
            RCLCPP_INFO(this->get_logger(), "poll_client: caught signal, exiting");
            retval = EXIT_SUCCESS;
            break;
          }
          if (state & net::PacketState::PACKET_ERROR)
          {
            std::cout << "poll_client: returned error\n"
                      << std::endl;
            retval = EXIT_FAILURE;
            break;
          }
          if (state & net::PacketState::LIDAR_DATA)
          {
            if (frontend_.readLiDARPacket(buf.data(), buf.size(), dst_ipaddr, nbytes))
            {
              if (param_.ip_addr == dst_ipaddr)
              {
                /* Midend */
#ifdef USE_TIMER
                frontend_.addPacket(buf);
#else
                pcl::PointCloud<PointT> out_cloud;
                frontend_.msg2Cloud(buf, out_cloud);

                sensor_msgs::msg::PointCloud2::UniquePtr out_msg(new sensor_msgs::msg::PointCloud2);
                pcl::toROSMsg(out_cloud, *out_msg);
                out_msg->header.frame_id = param_.frame_id;

                pub_lidar_cloud_->publish(std::move(out_msg));
#endif
                /**********/
              }
            }
            else
            {
              retval = EXIT_FAILURE;
              break;
            }
          }
          if (state & net::PacketState::IMU_DATA)
          {
          }
        }
      }

      std::cout << "Exit packetLoop" << std::endl;

      return retval;
    }

#ifdef USE_TIMER
    void FrontendNode::callbackTimer()
    {
      pcl::PointCloud<PointT> out_cloud;
      frontend_.flushPacket(out_cloud);

      sensor_msgs::msg::PointCloud2::UniquePtr out_msg(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(out_cloud, *out_msg);
      out_msg->header.frame_id = param_.frame_id;

      pub_lidar_cloud_->publish(std::move(out_msg));

      // printf(
      //     "Published message with value: %u, and address: 0x%" PRIXPTR "\n", out_msg->header.stamp.nanosec, reinterpret_cast<std::uintptr_t>(out_msg.get()));
    }
#endif
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ichthus_lidar_driver_ros2::frontend_node::FrontendNode)