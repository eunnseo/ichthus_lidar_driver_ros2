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

      // double Velodyne16::getRange(const char *dist_val1, const char *dist_val2)
      // {
      //   // TODO
      //   std::string hex;

      //   hex.append(dist_val1);
      //   hex.append(dist_val2);

      //   //std::cout<< hex << std::endl;

      //   unsigned int distance_decimal = std::stoul(hex, nullptr, 16);

      //   double distance2obj = distance_decimal * 2 * MM_TO_M;

      //   return distance2obj;
      // }

      void Velodyne16::msg2Cloud(const std::vector<uint8_t> &pkt_msg_buf, pcl::PointCloud<pcl::PointXYZITCA> &out_cloud)
      {

        std::cout << '1' << std::endl;
        // double range_min = used_range_[0] * M_TO_MM;
        // double range_max = used_range_[1] * M_TO_MM;

        float azimuth;
        float azimuth_diff;
        int raw_azimuth_diff;
        float last_azimuth_diff = 0.0f;
        float azimuth_corrected_f;
        int azimuth_corrected;
        float x, y, z;
        float intensity;

        // vlp_16_packet::Packet *pkt_ptr = (vlp_16_packet::Packet *)(&pkt_msg_buf[0]);

        const vlp_16_packet::RawPacket *pkt_ptr = reinterpret_cast<const vlp_16_packet::RawPacket *>(&pkt_msg_buf[0]);

        // float time_diff_start_to_this_packet =
        //   (rclcpp::Time(pkt_ptr.stamp) - scan_start_time).seconds();

        for (uint32_t blk_idx = 0; blk_idx < NUM_BLOCK; blk_idx++) // data block 0~11
        {

          if (vlp_16_packet::UPPER_BANK != pkt_ptr->blocks[blk_idx].header)
          {
            return; // bad packet: skip the rest
          }

          azimuth = static_cast<float>(pkt_ptr->blocks[blk_idx].rotation);

          if (blk_idx < (vlp_16_packet::BLOCKS_PER_PACKET_VLP16 - 1))
          {
            raw_azimuth_diff = pkt_ptr->blocks[blk_idx + 1].rotation - pkt_ptr->blocks[blk_idx].rotation;
            azimuth_diff = static_cast<float>((36000 + raw_azimuth_diff) % 36000);

            // some packets contain an angle overflow where azimuth_diff < 0
            if (raw_azimuth_diff < 0)
            {
              // raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
              // RCLCPP_WARN(
              //   get_logger(), "Packet containing angle overflow, first angle: %u second angle: %u",
              //   raw->blocks[block].rotation, raw->blocks[block+1].rotation);
              // if last_azimuth_diff was not zero, we can assume that the velodyne's
              // speed did not change very much and use the same difference
              if (last_azimuth_diff > 0)
              {
                azimuth_diff = last_azimuth_diff;
              }
              else
              {
                // otherwise we are not able to use this data
                // TODO(somebody): we might just not use the second 16 firings
                continue;
              }
            }

            last_azimuth_diff = azimuth_diff;
          }
          else
          {
            azimuth_diff = last_azimuth_diff;
          }

          for (int firing = 0, k = 0; firing < vlp_16_packet::VLP16_FIRINGS_PER_BLOCK; firing++)
          {
            for (int chan_idx = 0; chan_idx < vlp_16_packet::VLP16_SCANS_PER_FIRING; chan_idx++, k += vlp_16_packet::RAW_SCAN_SIZE)
            {
              // velodyne_pointcloud::LaserCorrection &corrections = calibration_->laser_corrections[chan_idx];

              /** Position Calculation */
              union vlp_16_packet::TwoBytes dist
              {
              };
              dist.bytes[0] = pkt_ptr->blocks[blk_idx].data[k];
              dist.bytes[1] = pkt_ptr->blocks[blk_idx].data[k + 1];

              /** correct for the laser rotation as a function of timing during the firings **/
              azimuth_corrected_f =
                  azimuth + (azimuth_diff * ((chan_idx * vlp_16_packet::VLP16_DSR_TOFFSET) + (firing * vlp_16_packet::VLP16_FIRING_TOFFSET)) / vlp_16_packet::VLP16_BLOCK_TDURATION);
              azimuth_corrected = (static_cast<int>(std::round(azimuth_corrected_f)) % 36000);

              /*condition added to avoid calculating points which are not
                in the interesting defined area (min_angle < area < max_angle)*/
              int min_angle = 0;
              int max_angle = 36000;

              if ((azimuth_corrected >= min_angle &&
                   azimuth_corrected <= max_angle &&
                   min_angle < max_angle) ||
                  (min_angle > max_angle &&
                   (azimuth_corrected <= max_angle ||
                    azimuth_corrected >= min_angle)))
              {
                // convert polar coordinates to Euclidean XYZ
                float distance = dist.uint * 0.002;
                distance += corrections.dist_correction;

                float cos_vert_angle = corrections.cos_vert_correction;
                float sin_vert_angle = corrections.sin_vert_correction;
                float cos_rot_correction = corrections.cos_rot_correction;
                float sin_rot_correction = corrections.sin_rot_correction;

                // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
                // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
                float cos_rot_angle =
                    vlp_16_packet::cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                    vlp_16_packet::sin_rot_table_[azimuth_corrected] * sin_rot_correction;
                float sin_rot_angle =
                    vlp_16_packet::sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                    vlp_16_packet::cos_rot_table_[azimuth_corrected] * sin_rot_correction;

                float horiz_offset = corrections.horiz_offset_correction;
                float vert_offset = corrections.vert_offset_correction;

                // Compute the distance in the xy plane (w/o accounting for rotation)
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

                // Calculate temporal X, use absolute value.
                float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
                // Calculate temporal Y, use absolute value
                float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

                if (xx < 0.0f)
                {
                  xx = -xx;
                }

                if (yy < 0.0f)
                {
                  yy = -yy;
                }

                // Get 2points calibration values,Linear interpolation to get distance
                // correction for X and Y, that means distance correction use
                // different value at different distance
                float distance_corr_x = 0;
                float distance_corr_y = 0;

                if (corrections.two_pt_correction_available)
                {
                  distance_corr_x =
                      (corrections.dist_correction - corrections.dist_correction_x) *
                          (xx - 2.4f) / (25.04f - 2.4f) +
                      corrections.dist_correction_x;
                  distance_corr_x -= corrections.dist_correction;
                  distance_corr_y =
                      (corrections.dist_correction - corrections.dist_correction_y) *
                          (yy - 1.93f) / (25.04f - 1.93f) +
                      corrections.dist_correction_y;
                  distance_corr_y -= corrections.dist_correction;
                }

                float distance_x = distance + distance_corr_x;
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
                x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

                float distance_y = distance + distance_corr_y;
                /**the new term of 'vert_offset * sin_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
                y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

                // Using distance_y is not symmetric, but the velodyne manual
                // does this.
                /**the new term of 'vert_offset * cos_vert_angle'
                 * was added to the expression due to the mathemathical
                 * model we used.
                 */
                z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

                /** Use standard ROS coordinate system (right-hand rule) */
                float x_coord = y;
                float y_coord = -x;
                float z_coord = z;

                /** Intensity Calculation */
                float min_intensity = corrections.min_intensity;
                float max_intensity = corrections.max_intensity;

                intensity = pkt_ptr->blocks[blk_idx].data[k + 2];

                float focal_offset = 256.0f * vlp_16_packet::square(1.0f - corrections.focal_distance / 13100.0f);
                float focal_slope = corrections.focal_slope;
                intensity += focal_slope *
                             (std::abs(focal_offset - 256.0f * vlp_16_packet::square((1.0f - distance) / 65535.0f)));
                intensity = (intensity < min_intensity) ? min_intensity : intensity;
                intensity = (intensity > max_intensity) ? max_intensity : intensity;

                // float time = 0;
                // if (timing_offsets_.size())
                // {
                //   time = timing_offsets_[blk_idx][firing * 16 + chan_idx] + time_diff_start_to_this_packet;
                // }

                // data.addPoint(
                //     x_coord, y_coord, z_coord, corrections.laser_ring,
                //     distance, intensity, pkt_ptr->timestamp);

                pcl::PointXYZITCA point;
                point.x = x_coord;
                point.y = y_coord;
                point.z = z_coord;

                point.intensity = static_cast<float>(intensity);
                point.timestamp = static_cast<double>(pkt_ptr->timestamp); // TODO: sec
                point.channel = static_cast<uint16_t>(chan_idx);
                point.azimuth = azimuth_corrected;

                out_cloud.push_back(point);
              }
            }

            // data.newLine();
          }
        }
      }

    }
  }
}