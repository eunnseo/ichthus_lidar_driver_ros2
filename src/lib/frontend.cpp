#include <ichthus_lidar_driver_ros2/lib/frontend.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace frontend
  {
    void Frontend::init()
    {
      // std::cout << "before lidar_sockfd_: " << lidar_sockfd_ << std::endl;
      // std::cout << "before imu_sockfd_: " << imu_sockfd_ << std::endl;

      lidar_sockfd_ = net::create_udp_socket(lidar_port_);
      imu_sockfd_ = net::create_udp_socket(imu_port_);

      // std::cout << "after lidar_sockfd_: " << lidar_sockfd_ << std::endl;
      // std::cout << "after imu_sockfd_: " << imu_sockfd_ << std::endl;

      if (model_ == "OS1-64")
      {
        std::cout << "model: " << model_ << std::endl;
        lidar_ptr_.reset(new sensor::os1_64::OusterI64(lidar_origin_to_beam_origin_mm_,
                                                       lidar_to_sensor_transform_,
                                                       beam_azimuth_angles_,
                                                       beam_altitude_angles_,
                                                       used_channels_,
                                                       used_azimuths_));
      }
      else if (model_ == "VLP-16")
      {
      }
      else if (model_ == "PDR-64")
      {
      }
      else
      {
      }

      lidar_ptr_->initResolution();
      lidar_ptr_->initLookUpTable();
      lidar_ptr_->initUsedPoints();
    }

    net::PacketState Frontend::poll()
    {
      // std::cout << "Start polling" << std::endl;
      // return (net::poll(lidar_sockfd_, imu_sockfd_));
      return (net::poll(lidar_sockfd_));
    }

    // bool readLiDARPacket(buf.data(), buf.size(), dst_ipaddr)
    bool Frontend::readLiDARPacket(uint8_t *buf, int max_buf_size, std::string &dst_ipaddr, ssize_t &nbytes)
    {
      bool retval;
      struct sockaddr_in sender_addr;
      if ((retval = net::read_packet(lidar_sockfd_, buf, max_buf_size, sender_addr, nbytes)))
      {
        dst_ipaddr = net::ipaddr_from_sockaddr(sender_addr);
      }

      return retval;
    }

    void Frontend::msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<PointT> &out_cloud)
    {
      lidar_ptr_->msg2Cloud(pkt_msg_buf, out_cloud);
    }

    void Frontend::addPacket(std::vector<uint8_t> &pkt_msg_buf)
    {
      packet_queue_.push(pkt_msg_buf);
    }

    void Frontend::flushPacket(pcl::PointCloud<PointT> &out_cloud)
    {
      while (!packet_queue_.empty())
      {
        std::vector<uint8_t> &pkt_msg_buf = packet_queue_.front();
        lidar_ptr_->msg2Cloud(pkt_msg_buf, out_cloud);

        packet_queue_.pop();
      }

      

      // std::cout << "point cloud" << std::endl;
      // std::cout << "x: " << cloud.points[0].x << std::endl;
      // std::cout << "y: " << cloud.points[0].y << std::endl;
      // std::cout << "z: " << cloud.points[0].z << std::endl;
    }

  }
}
