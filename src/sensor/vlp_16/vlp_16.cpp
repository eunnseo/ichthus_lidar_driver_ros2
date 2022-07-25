#include <ichthus_lidar_driver_ros2/sensor/vlp_16/vlp_16.hpp>
#include <iomanip> 

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace vlp_16
    {
      Velodyne16::Velodyne16(double lidar_origin_to_beam_origin_mm,
                           const std::vector<double> &transform,
                           const std::vector<double> &azimuth_angles_deg,
                           const std::vector<double> &altitude_angles_deg,
                           const std::vector<int64_t> &used_channels,
                           const std::vector<int64_t> &used_azimuths,
                           const std::vector<double> &used_range)
          : LiDARInterface(lidar_origin_to_beam_origin_mm, transform, azimuth_angles_deg, altitude_angles_deg, used_channels, used_azimuths, used_range)
      {
      }

      void Velodyne16::initLookUpTable()
      {
      }

      void Velodyne16::initResolution()
      {
        num_azimuth_ = 1024;
        num_channels_ = 16;
      }

      void Velodyne16::initUsedPoints()
      {
        /* X */
        return;
      }

      double Velodyne16::getRange(const char *dist_val1, const char *dist_val2)
      {
        // TODO
        return 0.0;
      }

      void Velodyne16::msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud)
      {
        std::cout << '1' << std::endl;
        double range_min = used_range_[0] * M_TO_MM;
        double range_max = used_range_[1] * M_TO_MM;

        vlp_16_packet::Packet *pkt_ptr = (vlp_16_packet::Packet *)(&pkt_msg_buf[0]);

        for (uint32_t blk_idx = 0; blk_idx < NUM_BLOCK; blk_idx++) //data block 0~11
        {
          
          uint32_t ts = pkt_ptr->timestamp;
          // if (out_cloud.empty())
          if (blk_idx == (NUM_BLOCK - 1))
          {
            out_cloud.header.stamp = ts;  // microsec
          }
          
          uint16_t azimuth_idx = pkt_ptr->blocks[blk_idx].azimuth;

          for (uint32_t chan_idx = 0; chan_idx < NUM_LIDAR_CHANNELS; chan_idx++) //channel 0 ~ 15 azimoth n
          {
            // if (is_used_point_[chan_idx][azimuth_idx] == 0)
            //   continue;
            
            //js

            int range1_idx = 3 * chan_idx + 1;
            int range2_idx = 3 * chan_idx + 2;
            int reflectivity_idx = 3 * chan_idx + 3;

            char *tmp1 = (char *)&pkt_ptr->blocks[blk_idx].firing1[range2_idx];
            char *tmp2 = (char *)&pkt_ptr->blocks[blk_idx].firing1[range1_idx];

            std::string hex;
            // hex.append(pkt_ptr->blocks[blk_idx].firing1[range2]);
            // hex.append(pkt_ptr->blocks[blk_idx].firing1[range1]);
            // char tmp1_val = (char)0xa1;
            // char tmp2_val = (char)0xb2;

            //tmp1 = &tmp1_val;
            //tmp2 = &tmp2_val;

            hex.append(tmp1);
            hex.append(tmp2);

            //std::cout<< hex << std::endl;

            unsigned int distance_decimal = std::stoul(hex, nullptr, 16);

            double distance2obj = distance_decimal * 2 * MM_TO_M;
            // distance calculation
            
            double intensity = pkt_ptr->blocks[blk_idx].firing1[reflectivity_idx];


            // double range = static_cast<double>(data.range_mm & 0x000fffff);
            // if (range < range_min || range > range_max)
            //   continue;

            // pcl::PointXYZITCA point;
            // uint32_t table_idx = chan_idx * num_azimuth_ + azimuth_idx;
            // point.x = lut_.direction(table_idx, 0) * range + lut_.offset(table_idx, 0);
            // point.y = lut_.direction(table_idx, 1) * range + lut_.offset(table_idx, 1);
            // point.z = lut_.direction(table_idx, 2) * range + lut_.offset(table_idx, 2);
            // // std::cout << "(x,y,z)(m) = (" << point.x << "," << point.y << "," << point.z << ")" << std::endl;

            // // point.intensity = static_cast<float>(data.signal_photons);
            // point.intensity = static_cast<float>(data.reflectivity);
            // point.timestamp = static_cast<double>(ts) * 1e-9; // nanosec -> sec
            // point.channel = static_cast<uint16_t>(chan_idx);
            // point.azimuth = azimuth_idx;

            // out_cloud.push_back(point);
          }

          // for (uint32_t chan_idx = 0; chan_idx < NUM_LIDAR_CHANNELS; chan_idx++) //channel 0 ~ 15 azimoth n+1
          // {
          //   if (is_used_point_[chan_idx][azimuth_idx] == 0)
          //     continue;

          //   vlp_16_packet::Data data = pkt_ptr->blocks[blk_idx].data[chan_idx];

          //   double range = static_cast<double>(data.range_mm & 0x000fffff);
          //   if (range < range_min || range > range_max)
          //     continue;

          //   pcl::PointXYZITCA point;
          //   uint32_t table_idx = chan_idx * num_azimuth_ + azimuth_idx;
          //   point.x = lut_.direction(table_idx, 0) * range + lut_.offset(table_idx, 0);
          //   point.y = lut_.direction(table_idx, 1) * range + lut_.offset(table_idx, 1);
          //   point.z = lut_.direction(table_idx, 2) * range + lut_.offset(table_idx, 2);
          //   // std::cout << "(x,y,z)(m) = (" << point.x << "," << point.y << "," << point.z << ")" << std::endl;

          //   // point.intensity = static_cast<float>(data.signal_photons);
          //   point.intensity = static_cast<float>(data.reflectivity);
          //   point.timestamp = static_cast<double>(ts) * 1e-9; // nanosec -> sec
          //   point.channel = static_cast<uint16_t>(chan_idx);
          //   point.azimuth = azimuth_idx;

          //   out_cloud.push_back(point);
          // }
        }
      }

    }
  }
}