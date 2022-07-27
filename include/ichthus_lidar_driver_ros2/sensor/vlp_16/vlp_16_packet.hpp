#ifndef _VLP_16_PACKET_HPP_
#define _VLP_16_PACKET_HPP_

#include <iostream>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace vlp_16_packet
    {
#define MM_TO_M 0.001
#define M_TO_MM 1000

#define FIRING_SIZE 48 // 16 * 3
#define NUM_BLOCK 12
#define NUM_LIDAR_CHANNELS 16

      static const uint16_t UPPER_BANK = 0xeeff;
      static const uint16_t LOWER_BANK = 0xddff;

      static const int SIZE_BLOCK = 100;
      static const int RAW_SCAN_SIZE = 3;
      static const int SCANS_PER_BLOCK = 32;
      static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

      static const int VLP16_FIRINGS_PER_BLOCK = 2;
      static const int VLP16_SCANS_PER_FIRING = 16;
      static const float VLP16_BLOCK_TDURATION = 110.592f; // [µs]
      static const float VLP16_DSR_TOFFSET = 2.304f;       // [µs]
      static const float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]

      struct RawBlock
      {
        uint16_t header;   ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation; ///< 0-35999, divide by 100 to get degrees
        uint8_t data[BLOCK_DATA_SIZE];
      };

      union TwoBytes
      {
        uint16_t uint;
        uint8_t bytes[2];
      };

      static const int PACKET_SIZE = 1206;
      static const int BLOCKS_PER_PACKET = 12;
      static const int PACKET_STATUS_SIZE = 4;
      static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

      struct RawPacket
      {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint16_t revolution;
        uint8_t status[PACKET_STATUS_SIZE];
      };

    }
  }
}

#endif // _VLP_16_PACKET_HPP_