#include <ichthus_lidar_driver_ros2/sensor/os1_64/os1_64.hpp>
#include <iomanip> 

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace os1_64
    {
      OusterI64::OusterI64(double lidar_origin_to_beam_origin_mm,
                           const std::vector<double> &transform,
                           const std::vector<double> &azimuth_angles_deg,
                           const std::vector<double> &altitude_angles_deg,
                           const std::vector<int64_t> &used_channels,
                           const std::vector<int64_t> &used_azimuths,
                           const std::vector<double> &used_range)
          : LiDARInterface(lidar_origin_to_beam_origin_mm, transform, azimuth_angles_deg, altitude_angles_deg, used_channels, used_azimuths, used_range)
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
        lut_.direction *= MM_TO_M;
        lut_.offset *= MM_TO_M;

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

      void OusterI64::initUsedPoints()
      {
        // std::cout << "initUsedPoints..\n";
        // std::cout << "used_azimuths_.size() = " << used_azimuths_.size() << std::endl;
        // std::cout << "used_channels_.size() = " << used_channels_.size() << std::endl;

        for (size_t i = 0; i < num_channels_; i++)
        {
          is_used_point_.push_back(std::vector<bool>());
          for (size_t j = 0; j < num_azimuth_; j++)
          {
            is_used_point_[i].push_back(false);
          }
        }

        size_t min, max;
        std::vector<int64_t> azim_idx_arr;
        std::vector<int64_t> chan_idx_arr;
        for (size_t i = 0; i < used_azimuths_.size(); i += 2)
        {
          min = used_azimuths_[i];
          max = used_azimuths_[i+1];
          for (size_t azim_i = min; azim_i < max; azim_i++)
          {
            azim_idx_arr.push_back(azim_i);
          }
        }
        for (size_t i = 0; i < used_channels_.size(); i += 2)
        {
          min = used_channels_[i];
          max = used_channels_[i+1];
          for (size_t chan_i = min; chan_i < max; chan_i++)
          {
            chan_idx_arr.push_back(chan_i);
          }
        }

        for (size_t i = 0; i < chan_idx_arr.size(); i++)
        {
          for (size_t j = 0; j < azim_idx_arr.size(); j++)
          {
            size_t chan_idx = chan_idx_arr[i];
            size_t azim_idx = azim_idx_arr[j];
            is_used_point_[chan_idx][azim_idx] = true;
          }
        }

        azim_idx_arr.clear();
        chan_idx_arr.clear();
      }

      void OusterI64::msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud)
      {
        double range_min = used_range_[0] * M_TO_MM;
        double range_max = used_range_[1] * M_TO_MM;

        os1_64_packet::Packet *pkt_ptr = (os1_64_packet::Packet *)(&pkt_msg_buf[0]);

        for (uint32_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; blk_idx++)
        {
          const os1_64_packet::Header &header = pkt_ptr->blocks[blk_idx].header;

          uint64_t ts = ((uint64_t *)header.timestamp)[0]; // nanosec
          // if (out_cloud.empty())
          if (blk_idx == (BLOCKS_PER_PACKET - 1))
          {
            out_cloud.header.stamp = ts * 0.001; // nanosec -> microsec
          }
          
          uint32_t azimuth_idx = header.encoder_count / (encoder_ticks_per_rev_ / num_azimuth_);

          for (uint32_t chan_idx = 0; chan_idx < NUM_LIDAR_CHANNELS; chan_idx++)
          {
            if (is_used_point_[chan_idx][azimuth_idx] == 0)
              continue;

            os1_64_packet::Data data = pkt_ptr->blocks[blk_idx].data[chan_idx];

            double range = static_cast<double>(data.range_mm & 0x000fffff);
            if (range < range_min || range > range_max)
              continue;

            pcl::PointXYZITCA point;
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