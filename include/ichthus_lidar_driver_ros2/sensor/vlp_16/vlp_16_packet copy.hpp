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

      struct Block
      {
        uint16_t flag;
        uint16_t azimuth;
        uint8_t firing1[FIRING_SIZE];
        uint8_t firing2[FIRING_SIZE];
      }; // 100Bytes

      struct Packet
      {
        Block blocks[NUM_BLOCK];
        uint32_t timestamp;
        uint16_t factory;
      }; // 1206 Bytes

    }
  }
}

#endif // _VLP_16_PACKET_HPP_