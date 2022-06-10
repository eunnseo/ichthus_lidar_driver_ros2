#include <ichthus_lidar_driver_ros2/sensor/os1_64/os1_64.hpp>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace os1_64
    {
      OusterI64::OusterI64(double lidar_origin_to_beam_origin_mm,
                           const std::vector<double> &transform,
                           const std::vector<double> &azimuth_angles_deg,
                           const std::vector<double> &altitude_angles_deg)
          : LiDARInterface(lidar_origin_to_beam_origin_mm, transform, azimuth_angles_deg, altitude_angles_deg)
      {
      }

      void OusterI64::initLookUpTable()
      {
        if (azimuth_angles_deg_.size() != num_channels_ || altitude_angles_deg_.size() != num_channels_)
          throw std::invalid_argument("unexpected scan dimensions");

        Eigen::ArrayXd encoder(num_azimuth_ * num_channels_);  // theta_e
        Eigen::ArrayXd azimuth(num_azimuth_ * num_channels_);  // theta_a
        Eigen::ArrayXd altitude(num_azimuth_ * num_channels_); // phi

        const double azimuth_radians = M_PI * 2.0 / num_azimuth_;

        // populate angles for each pixel
        for (size_t v = 0; v < num_azimuth_; v++) // 1024
        {
          for (size_t u = 0; u < num_channels_; u++) // 64
          {
            size_t i = u * num_azimuth_ + v;
            encoder(i) = 2.0 * M_PI - (v * azimuth_radians);
            azimuth(i) = -azimuth_angles_deg_[u] * M_PI / 180.0;
            altitude(i) = altitude_angles_deg_[u] * M_PI / 180.0;
          }
        }

        // unit vectors for each pixel
        lut_.direction = LiDARInterface::Points{num_azimuth_ * num_channels_, 3};
        lut_.direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
        lut_.direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
        lut_.direction.col(2) = altitude.sin();

        // offsets due to beam origin
        lut_.offset = LiDARInterface::Points{num_azimuth_ * num_channels_, 3};
        lut_.offset.col(0) = encoder.cos() - lut_.direction.col(0);
        lut_.offset.col(1) = encoder.sin() - lut_.direction.col(1);
        lut_.offset.col(2) = -lut_.direction.col(2);
        lut_.offset *= lidar_origin_to_beam_origin_mm_;

        // apply the supplied transform
        auto rot = lidar_to_sensor_transform_.topLeftCorner(3, 3).transpose();
        auto trans = lidar_to_sensor_transform_.topRightCorner(3, 1).transpose();
        lut_.direction.matrix() *= rot;
        lut_.offset.matrix() *= rot;
        lut_.offset.matrix() += trans.replicate(num_azimuth_ * num_channels_, 1);

        // apply scaling factor
        lut_.direction *= os1_64_packet::range_unit;
        lut_.offset *= os1_64_packet::range_unit;

        // std::cout << "lut_.direction.size(): " << lut_.direction.size() << std::endl;
        // std::cout << "lut_.offset.size(): " << lut_.offset.size() << std::endl;
        // std::cout << "lut_.direction.rows() = " << lut_.direction.rows() << std::endl;
        // std::cout << "lut_.direction.cols() = " << lut_.direction.cols() << std::endl;
        // std::cout << std::endl;

        // std::cout << "lut_.direction.data() = " << *(lut_.direction.data()) << std::endl;
        // std::cout << "lut_.direction(0, 0) = " << lut_.direction(0, 0) << std::endl;
        // std::cout << "lut_.direction(0, 1) = " << lut_.direction(0, 1) << std::endl;
        // std::cout << "lut_.direction(0, 2) = " << lut_.direction(0, 2) << std::endl;
      }

      void OusterI64::initResolution()
      {
        num_azimuth_ = 1024;
        num_channels_ = 64;
      }

      void OusterI64::msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<PointT> &out_cloud)
      {
        // bool first = true;
        os1_64_packet::Packet *pkt_ptr = (os1_64_packet::Packet *)(&pkt_msg_buf[0]);

        for (uint32_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; blk_idx++)
        {
          const os1_64_packet::Header &header = pkt_ptr->blocks[blk_idx].header;

          uint64_t ts = ((uint64_t *)header.timestamp)[0]; // nanosec
          if (out_cloud.empty())
          {
            out_cloud.header.stamp = ts * 0.001; // nanosec -> microsec
          }
          
          uint32_t azimuth_idx = header.encoder_count / (encoder_ticks_per_rev_ / num_azimuth_);

          for (uint32_t chan_idx = 0; chan_idx < NUM_LIDAR_CHANNELS; chan_idx++)
          {
            // if (chan_idx % 2 == 0) // 짝수번째 채널 버림
            //   continue;

            os1_64_packet::Data data = pkt_ptr->blocks[blk_idx].data[chan_idx];

            uint32_t range = data.range_mm & 0x000fffff;
            if (range < 300)
              continue;

            PointT point;
            uint32_t table_idx = chan_idx * num_azimuth_ + azimuth_idx;
            point.x = lut_.direction(table_idx, 0) * range + lut_.offset(table_idx, 0);
            point.y = lut_.direction(table_idx, 1) * range + lut_.offset(table_idx, 1);
            point.z = lut_.direction(table_idx, 2) * range + lut_.offset(table_idx, 2);
            // std::cout << "(x,y,z)(m) = (" << point.x << "," << point.y << "," << point.z << ")" << std::endl;

            // point.intensity = static_cast<float>(data.signal_photons);
            point.intensity = static_cast<float>(data.reflectivity);
            point.timestamp = static_cast<double>(ts) * 1e-9; // nanosec -> sec
            point.channel = static_cast<uint16_t>(chan_idx);
            point.azimuth = azimuth_idx;

            out_cloud.push_back(point);
          }
        }
      }

    }
  }
}