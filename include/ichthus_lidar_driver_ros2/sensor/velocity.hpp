#ifndef _VELOCITY_HPP_
#define _VELOCITY_HPP_

#include "std_msgs/msg/header.hpp"
#include <iostream>
#include <chrono>

namespace ichthus_lidar_driver_ros2
{
  namespace sensor
  {
    struct Velocity
    {
      std_msgs::msg::Header header;
      float linear_x;
      float angular_z;
    };

  }
}

#endif // _VELOCITY_HPP_
