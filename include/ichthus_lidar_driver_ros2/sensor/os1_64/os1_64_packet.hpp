#ifndef _OS1_64_PACKET_HPP_
#define _OS1_64_PACKET_HPP_

#include <iostream>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    namespace os1_64_packet
    {
#define MM_TO_M 0.001
#define M_TO_MM 1000

#define NUM_LIDAR_CHANNELS 64
#define BLOCKS_PER_PACKET 16
#define TS_SIZE 2

// const int TS_SIZE = 2; 

      struct Header
      {
        uint32_t timestamp[TS_SIZE];
        uint16_t measurement_id;
        uint16_t frame_id;
        uint32_t encoder_count;
      }; // 16 Bytes

      struct Data
      {
        uint32_t range_mm;
        uint16_t reflectivity;
        uint16_t signal_photons;
        uint32_t near_inrared_photons;
      }; // 12 Bytes

      struct Block
      {
        Header header;
        Data data[NUM_LIDAR_CHANNELS];
        uint32_t status;
      }; // 788 Bytes
      
      struct Packet
      {
        Block blocks[BLOCKS_PER_PACKET];
      }; // 12608 Bytes

    }
  }
}

#endif // _OS1_64_PACKET_HPP_