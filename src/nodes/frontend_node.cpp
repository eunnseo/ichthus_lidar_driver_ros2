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
      std::cout << "\nFrontend construction " << this->get_name() << std::endl;

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
      std::vector<double> beam_azimuth_angles = declare_parameter("beam_azimuth_angles", std::vector<double>());
      std::vector<double> beam_altitude_angles = declare_parameter("beam_altitude_angles", std::vector<double>());
      std::vector<double> lidar_to_sensor_transform = declare_parameter("lidar_to_sensor_transform", std::vector<double>());
      double lidar_origin_to_beam_origin_mm = declare_parameter("lidar_origin_to_beam_origin_mm", 0.0);
      // std::vector<int64_t> used_channels = declare_parameter("used_channels", std::vector<int64_t>());
      // std::vector<int64_t> used_azimuths = declare_parameter("used_azimuths", std::vector<int64_t>());
      // std::vector<double> used_range = declare_parameter("used_range", std::vector<double>());
      /*******************************/

      param_.used_channels = declare_parameter("used_channels", std::vector<int64_t>());
      param_.used_azimuths = declare_parameter("used_azimuths", std::vector<int64_t>());
      param_.used_range = declare_parameter("used_range", std::vector<double>());

      printFrontendParams();

      if (!param_.use_pcap)
      {
        frontend_.setLiDARModel(param_.model);
        frontend_.setLiDARIPAddr(param_.ip_addr);
        frontend_.setLiDARPort(param_.lidar_port);
        frontend_.setIMUPort(param_.imu_port);

        frontend_.setBeamAzimuthAngles(beam_azimuth_angles);
        frontend_.setBeamAltitudeAngles(beam_altitude_angles);
        frontend_.setToSensorTransform(lidar_to_sensor_transform);
        frontend_.setToBeamOrigin(lidar_origin_to_beam_origin_mm);

        frontend_.setUsedChannels(param_.used_channels);
        frontend_.setUsedAzimuths(param_.used_azimuths);
        frontend_.setUsedRange(param_.used_range);
        
        // frontend_.setUsedChannels(used_channels);
        // frontend_.setUsedAzimuths(used_azimuths);
        // frontend_.setUsedRange(used_range);

        frontend_.init();
      }

      pub_lidar_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          param_.ns + "/lidar_cloud",
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

    void FrontendNode::printFrontendParams()
    {
      if (param_.use_pcap)
      {
        std::cout << "[frontend param] pcap_file: " << param_.pcap_file << std::endl;
        std::cout << "[frontend param] pcap_wait_factor: " << param_.pcap_wait_factor << std::endl;
      }
      std::cout << "[frontend param] model: " << param_.model << std::endl;
      std::cout << "[frontend param] ns: " << param_.ns << std::endl;
      std::cout << "[frontend param] frame_id: " << param_.frame_id << std::endl;
      std::cout << "[frontend param] ip_addr: " << param_.ip_addr << std::endl;
      std::cout << "[frontend param] lidar_port: " << param_.lidar_port << std::endl;

      std::cout << "[frontend param] used_channels num: ";
      size_t min, max;
      size_t used_channels_num = 0;
      for (size_t i = 0; i < param_.used_channels.size(); i += 2)
      {
        min = param_.used_channels[i];
        max = param_.used_channels[i+1];
        used_channels_num += (max - min);
      }
      std::cout << used_channels_num << std::endl;
      
      std::cout << "[frontend param] used_azimuths num: ";
      size_t used_azimuths_num = 0;
      for (size_t i = 0; i < param_.used_azimuths.size(); i += 2)
      {
        min = param_.used_azimuths[i];
        max = param_.used_azimuths[i+1];
        used_azimuths_num += (max - min);
      }
      std::cout << used_azimuths_num << std::endl;
      
      std::cout << "[frontend param] used_range: "
                << param_.used_range[0] << ", "
                << param_.used_range[1] << "\n";

#ifdef USE_TIMER
      std::cout << "[frontend param] period_ms: " << param_.period_ms << std::endl;
#endif
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
          std::cout << "state: " << state << std::endl;
          if (state == net::PacketState::EXIT)
          {
            RCLCPP_INFO(this->get_logger(), "poll_client: caught signal, exiting");
            retval = EXIT_SUCCESS;
            break;
          }
          if (state == net::PacketState::TIMEOUT)
          {
            std::cout << "poll_client: timeout\n";
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
            std::cout << "poll_client: lidar_data\n";
            if (frontend_.readLiDARPacket(buf.data(), buf.size(), dst_ipaddr, nbytes))
            {
              if (param_.ip_addr == dst_ipaddr)
              {
                std::cout << "param_.ip_addr == dst_ipaddr\n";
                /* Midend */
#ifdef USE_TIMER
                frontend_.addPacket(buf);
#else
                pcl::PointCloud<pcl::PointXYZITCA> out_cloud;
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
      pcl::PointCloud<pcl::PointXYZITCA> out_cloud;
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