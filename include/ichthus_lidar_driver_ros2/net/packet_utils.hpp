/**
 * @file
 * @brief Compatibility with windows (unsupported)
 */

#pragma once

#include <iostream>
#include <string>

#if defined _WIN32 // --------- On Windows ---------

// Try and limit the stuff windows brings in
#include <winsock2.h>
#include <ws2tcpip.h>

// ssize_t is available in vs
#ifdef _MSC_VER
#include <BaseTsd.h>
#define ssize_t SSIZE_T
#endif

#else // --------- Compiling on *nix ---------

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <pcap/pcap.h>  //pcap_open_offline()
#include <poll.h>       //pollfd

#include <cstdint>
#include <memory>
#include <cstring>

// Define windows types
#ifndef SOCKET
#define SOCKET int
#endif

#ifndef FDSET
#define FDSET fd_set
#endif

#define SOCKET_ERROR -1

#endif // --------- End Platform Differentiation Block ---------

namespace ichthus_lidar_driver_ros2
{
  namespace net
  {
    #define POLL_TIMEOUT 0
    #define POLL_ERROR -1

    enum PacketState
    {
      TIMEOUT = 0,
      PACKET_ERROR = 1,
      LIDAR_DATA = 2,
      IMU_DATA = 4,
      EXIT = 8
    };

    // default udp receive buffer size on windows is very low -- use 256K
    const int RCVBUF_SIZE = 256 * 1024;

    // timeout for reading from a TCP socket during config
    const int RCVTIMEOUT_SEC = 10;

    /**
     * Close a specified socket
     * @param sock The socket file descriptor to close
     * @return success
     */
    int socket_close(SOCKET sock);

    /**
     * Get the error message for socket errors
     * @return The socket error message
     */
    std::string socket_get_error();

    /**
     * Check if a socket file descriptor is valid
     * @param sock The socket file descriptor to check
     * @return The validity of the socket file descriptor
     */
    bool socket_valid(SOCKET value);

    /**
     * Check if the last error was a socket exit event
     * @return If the socket has exited
     */
    bool socket_exit();

    /**
     * Set a specified socket to non-blocking
     * @param sock The socket file descriptor to set non-blocking
     * @return success
     */
    int socket_set_non_blocking(SOCKET value);

    /**
     * Set a specified socket to reuse
     * @param sock The socket file descriptor to set reuse
     * @return success
     */
    int socket_set_reuse(SOCKET value);

    /**
     * Set SO_RCVTIMEO on the specified socket
     * @param sock The socket file descriptor
     * @return success
     */
    int socket_set_rcvtimeout(SOCKET sock, int timeout_sec);

    std::string ipaddr_from_sockaddr(struct sockaddr_in &sender_addr);
    
    SOCKET create_udp_socket(int port);
    
    PacketState poll(SOCKET lidar_fd, SOCKET imu_fd, const int timeout_sec = 1);

    PacketState poll(SOCKET lidar_fd, const int timeout_sec = 1);

    bool read_packet(SOCKET sockfd, uint8_t *buf, int max_buf_size, struct sockaddr_in &sender_addr, ssize_t &nbytes);
  } // namespace ichthus_lidar_driver_ros2
} // namespace socket
