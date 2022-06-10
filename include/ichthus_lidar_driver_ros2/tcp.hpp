#ifndef _PACKET_HPP_
#define _PACKET_HPP_

#include <iostream>
#include <vector>

namespace ichthus_lidar_driver_ros2
{
  namespace packet
  {
    bool do_tcp_cmd(SOCKET sock_fd, const std::vector<std::string> &cmd_tokens,
                    std::string &res)
    {
      const size_t max_res_len = 16 * 1024;
      auto read_buf = std::unique_ptr<char[]>{new char[max_res_len + 1]};

      std::stringstream ss;
      for (const auto &token : cmd_tokens)
        ss << token << " ";
      ss << "\n";
      std::string cmd = ss.str();

      ssize_t len = send(sock_fd, cmd.c_str(), cmd.length(), 0);
      if (len != (ssize_t)cmd.length())
      {
        return false;
      }

      // need to synchronize with server by reading response
      std::stringstream read_ss;
      do
      {
        len = recv(sock_fd, read_buf.get(), max_res_len, 0);
        if (len < 0)
        {
          std::cerr << "do_tcp_cmd recv(): " << impl::socket_get_error()
                    << std::endl;
          return false;
        }
        read_buf.get()[len] = '\0';
        read_ss << read_buf.get();
      } while (len > 0 && read_buf.get()[len - 1] != '\n');

      res = read_ss.str();
      res.erase(res.find_last_not_of(" \r\n\t") + 1);

      return true;
    }

  }
}
#endif