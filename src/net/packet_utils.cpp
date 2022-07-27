#include <ichthus_lidar_driver_ros2/net/packet_utils.hpp>

#include <string>

#if defined _WIN32

#include <winsock2.h>

#else

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#endif

namespace ichthus_lidar_driver_ros2
{
  namespace net
  {

#ifdef _WIN32
    struct StaticWrapper
    {
      WSADATA wsa_data;

      StaticWrapper() { WSAStartup(MAKEWORD(1, 1), &wsa_data); }

      ~StaticWrapper() { WSACleanup(); }
    };

    static StaticWrapper resources = {};
#endif

    int socket_close(SOCKET sock)
    {
#ifdef _WIN32
      return closesocket(sock);
#else
      return close(sock);
#endif
    }

    std::string socket_get_error()
    {
#ifdef _WIN32
      int errnum = WSAGetLastError();
      char buf[256] = {0};
      if (FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, errnum, 0, buf,
                        sizeof(buf), NULL) != 0)
      {
        return std::string(buf);
      }
      else
      {
        return std::string{"Unknown WSA error "} + std::to_string(errnum);
      }
#else
      return std::strerror(errno);
#endif
    }

    bool socket_valid(SOCKET sock)
    {
#ifdef _WIN32
      return sock != SOCKET_ERROR;
#else
      return sock >= 0;
#endif
    }

    bool socket_exit()
    {
#ifdef _WIN32
      auto result = WSAGetLastError();
      return result == WSAECONNRESET || result == WSAECONNABORTED ||
             result == WSAESHUTDOWN;
#else
      return errno == EINTR;
#endif
    }

    int socket_set_non_blocking(SOCKET value)
    {
#ifdef _WIN32
      u_long non_blocking_mode = 1;
      return ioctlsocket(value, FIONBIO, &non_blocking_mode);
#else
      return fcntl(value, F_SETFL, fcntl(value, F_GETFL, 0) | O_NONBLOCK);
#endif
    }

    int socket_set_reuse(SOCKET value)
    {
      int option = 1;
#ifndef _WIN32
      int res =
          setsockopt(value, SOL_SOCKET, SO_REUSEPORT, &option, sizeof(option));
      if (res != 0)
        return res;
#endif
      return setsockopt(value, SOL_SOCKET, SO_REUSEADDR, (char *)&option,
                        sizeof(option));
    }

    int socket_set_rcvtimeout(SOCKET sock, int timeout_sec)
    {
#ifdef _WIN32
      DWORD timeout_ms = timeout_sec * 1000;
      return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout_ms,
                        sizeof timeout_ms);
#else
      struct timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;
      return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv,
                        sizeof tv);
#endif
    }

    std::string ipaddr_from_sockaddr(struct sockaddr_in &sender_addr)
    {
      return (inet_ntoa(sender_addr.sin_addr));
    }

    SOCKET create_udp_socket(int port)
    {
      struct addrinfo hints, *info_start, *ai;

      memset(&hints, 0, sizeof hints);
      hints.ai_family = AF_UNSPEC;
      hints.ai_socktype = SOCK_DGRAM;
      hints.ai_flags = AI_PASSIVE;

      auto port_s = std::to_string(port);

      int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
      if (ret != 0)
      {
        std::cerr << "udp getaddrinfo(): " << gai_strerror(ret) << std::endl;
        return SOCKET_ERROR;
      }
      if (info_start == NULL)
      {
        std::cerr << "udp getaddrinfo(): empty result" << std::endl;
        return SOCKET_ERROR;
      }

      // try to bind a dual-stack ipv6 socket, but fall back to ipv4 only if that
      // fails (when ipv6 is disabled via kernel parameters). Use two passes to
      // deal with glibc addrinfo ordering:
      // https://sourceware.org/bugzilla/show_bug.cgi?id=9981
      // for (auto preferred_af : {AF_INET6, AF_INET})
      for (auto preferred_af : {AF_INET})
      {
        for (ai = info_start; ai != NULL; ai = ai->ai_next)
        {
          if (ai->ai_family != preferred_af)
            continue;

          // choose first addrinfo where bind() succeeds
          SOCKET sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
          if (!net::socket_valid(sock_fd))
          {
            std::cerr << "udp socket(): " << net::socket_get_error()
                      << std::endl;
            continue;
          }

          int off = 0;
          if (ai->ai_family == AF_INET6 &&
              setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY, (char *)&off,
                         sizeof(off)))
          {
            std::cerr << "udp setsockopt(): " << net::socket_get_error()
                      << std::endl;
            net::socket_close(sock_fd);
            continue;
          }

          if (net::socket_set_reuse(sock_fd))
          {
            std::cerr << "udp socket_set_reuse(): "
                      << net::socket_get_error() << std::endl;
          }

          if (bind(sock_fd, ai->ai_addr, (socklen_t)ai->ai_addrlen))
          {
            std::cerr << "udp bind(): " << net::socket_get_error()
                      << std::endl;
            net::socket_close(sock_fd);
            continue;
          }

          // bind() succeeded; set some options and return
          if (net::socket_set_non_blocking(sock_fd))
          {
            std::cerr << "udp fcntl(): " << net::socket_get_error()
                      << std::endl;
            net::socket_close(sock_fd);
            continue;
          }

          if (setsockopt(sock_fd, SOL_SOCKET, SO_RCVBUF, (char *)&RCVBUF_SIZE,
                         sizeof(RCVBUF_SIZE)))
          {
            std::cerr << "udp setsockopt(): " << net::socket_get_error()
                      << std::endl;
            net::socket_close(sock_fd);
            continue;
          }

          freeaddrinfo(info_start);
          return sock_fd;
        }
      }

      // could not bind() a UDP server socket
      freeaddrinfo(info_start);
      return SOCKET_ERROR;
    }

    PacketState poll(SOCKET lidar_fd, SOCKET imu_fd, const int timeout_sec)
    {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(lidar_fd, &rfds);
      FD_SET(imu_fd, &rfds);

      timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;

      SOCKET max_fd = std::max(lidar_fd, imu_fd);

      int retval = select((int)max_fd + 1, &rfds, NULL, NULL, &tv);

      PacketState res = PacketState(0);

      if (!net::socket_valid(retval) && net::socket_exit())
      {
        res = EXIT;
      }
      else if (!net::socket_valid(retval))
      {
        std::cerr << "select: " << net::socket_get_error() << std::endl;
        res = PacketState(res | PACKET_ERROR);
      }
      else if (retval)
      {
        if (FD_ISSET(lidar_fd, &rfds))
          res = PacketState(res | LIDAR_DATA);
        if (FD_ISSET(imu_fd, &rfds))
          res = PacketState(res | IMU_DATA);
      }

      return res;
    }

    PacketState poll(SOCKET lidar_fd, const int timeout_sec)
    {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(lidar_fd, &rfds);

      timeval tv;
      tv.tv_sec = timeout_sec;
      tv.tv_usec = 0;

      int retval = select((int)lidar_fd + 1, &rfds, NULL, NULL, &tv);

      PacketState res = PacketState(0);

      if (!net::socket_valid(retval) && net::socket_exit())
      {
        res = EXIT;
      }
      else if (!net::socket_valid(retval))
      {
        std::cerr << "select: " << net::socket_get_error() << std::endl;
        res = PacketState(res | PACKET_ERROR);
      }
      else if (retval)
      {
        if (FD_ISSET(lidar_fd, &rfds))
          res = PacketState(res | LIDAR_DATA);
      }

      return res;
    }

    // PacketState poll(SOCKET lidar_fd, SOCKET imu_fd, const int timeout_sec)
    // {
    //   struct pollfd fds[2];
    //   fds[0].fd = lidar_fd;
    //   fds[0].events = POLLIN;

    //   fds[1].fd = imu_fd;
    //   fds[1].events = POLLIN;

    // }

    bool read_packet(SOCKET sockfd, uint8_t *buf, int max_buf_size, struct sockaddr_in &sender_addr, ssize_t &nbytes)
    {
      bool retval;
      socklen_t sender_addr_len = sizeof(sender_addr);
      nbytes = recvfrom(sockfd, buf, max_buf_size, 0, (sockaddr *)&sender_addr, &sender_addr_len);
      // std::cout << "recvfrom: " << ipaddr_from_sockaddr(sender_addr)
        // << " " << sender_addr.sin_port << " " << nbytes << " " << max_buf_size << std::endl;

      if (nbytes < 0)
      {
        if (errno != EWOULDBLOCK)
        {
          perror("recvfail");
          retval = false;
        }
      }
      else
      {
        retval = true;
      }

      return retval;
    }

  } // namespace net

} // namespace ichthus_lidar_driver_ros2
