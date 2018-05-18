/////////////////////////////////////////////////////////////////////////////
//
//   Copyright 2017 Open Source Robotics Foundation, Inc.
//   Copyright 2017-2018 Right Hand Robotics
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
/////////////////////////////////////////////////////////////////////////////

#include "reflex_hand.h"
#include <ros/console.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ifaddrs.h>
#include <netdb.h>
using namespace reflex_hand;

ReflexHandState::ReflexHandState()
{
  systime_us_ = 0;
  for (int i = 0; i < NUM_TACTILE; i++)
    tactile_pressures_[i] = tactile_temperatures_[i] = 0;
}


ReflexHand::ReflexHand(const std::string &interface)
: happy_(true)
{
  ROS_INFO("ReflexHand constructor");
  ROS_INFO("Ethernet: %s", interface.c_str());
  std::string multicast_address_string = "224.0.0.124";
  ROS_INFO("Multicast address: %s", multicast_address_string.c_str());
  
  // TODO: Parameterize
  const char *mcast_addr_str = multicast_address_string.c_str(); 
  
  tx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  ROS_FATAL_COND(tx_sock_ < 0, "Couldn't create socket");
  ROS_FATAL_COND(rx_sock_ < 0, "Couldn't create socket");
  memset(&mcast_addr_, 0, sizeof(mcast_addr_));
  mcast_addr_.sin_family = AF_INET;
  mcast_addr_.sin_addr.s_addr = inet_addr(mcast_addr_str);

  ifaddrs *ifaddr;
  if (getifaddrs(&ifaddr) == -1) {
    ROS_FATAL("Couldn't get ipv4 address of interface %s", interface.c_str());
    return;
  }

  std::string tx_iface_addr;
  bool found_interface = false;

  for (ifaddrs *ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;

    int family = ifa->ifa_addr->sa_family;

    if (family != AF_INET)
      continue;

    char host[NI_MAXHOST];
    if (getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                    host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST))
      continue;

    ROS_INFO("Found address %s on interface %s",
             host, ifa->ifa_name);

    if (std::string(ifa->ifa_name) == interface) {
      ROS_INFO("using %s as the TX interface for IPv4 UDP multicast", host);
      tx_iface_addr = host;
      found_interface = true;
      break;
    }
  }

  freeifaddrs(ifaddr);

  if (!found_interface) {
    ROS_FATAL("Unable to find IPv4 address of interface %s. Perhaps it "
              "needs to be set to a static address?", interface.c_str());
    happy_ = false;
    return;
  }

  in_addr local_addr;
  local_addr.s_addr = inet_addr(tx_iface_addr.c_str());
  int result = 0, loopback = 0;
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_IF,
                      (char *)&local_addr, sizeof(local_addr));
  ROS_FATAL_COND(result < 0, "Couldn't set local interface for udp tx sock," 
                             "fails with errno: %d", errno);
  result = setsockopt(tx_sock_, IPPROTO_IP, IP_MULTICAST_LOOP, 
                      &loopback, sizeof(loopback));
  ROS_FATAL_COND(result < 0, "Couldn't turn off outgoing multicast loopback," 
                             "fails with errno: %d", errno);

  // Set up RX side 
  int reuseaddr = 1;
  result = setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR,
                      &reuseaddr, sizeof(reuseaddr));
  ROS_FATAL_COND(result < 0, 
                 "couldn't set SO_REUSEADDR on UDP RX socket, "
                 "fails with errno: %d", errno);
  sockaddr_in rx_bind_addr;
  memset(&rx_bind_addr, 0, sizeof(rx_bind_addr));
  rx_bind_addr.sin_family = AF_INET;
  rx_bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  rx_bind_addr.sin_port = htons(PORT_BASE);
  result = bind(rx_sock_, (sockaddr *)&rx_bind_addr, sizeof(rx_bind_addr));
  ROS_FATAL_COND(result < 0, 
                 "couldn't bind rx socket to port %d, fails with errno: %d", 
                 PORT_BASE, errno);
  ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr_str);
  mreq.imr_interface.s_addr = inet_addr(tx_iface_addr.c_str());
  result = setsockopt(rx_sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                      &mreq, sizeof(mreq));
  ROS_FATAL_COND(result < 0, 
                 "couldn't add to multicast group, fails with errno: %d", 
                 errno);
  ROS_INFO("constructor complete");
}


ReflexHand::~ReflexHand() 
{
}


void ReflexHand::tx(const uint8_t *msg, const uint16_t msg_len, 
                    const uint16_t port)
{
  // ROS_INFO("ReflexHand::tx %d bytes to port %d", msg_len, port);
  mcast_addr_.sin_port = htons(port);
  int nsent = sendto(tx_sock_, msg, msg_len, 0,
                     (sockaddr *)&mcast_addr_, sizeof(mcast_addr_));
  ROS_ERROR_COND(nsent < 0, "woah. sendto() returned %d", nsent);
}

/////////////////////////////////////////////////////////////// calls rx()
/*
  what does this do?
*/
bool ReflexHand::listen(const double max_seconds)
{
  static uint8_t rxbuf[2000] = {0};
  fd_set rdset;
  FD_ZERO(&rdset);
  FD_SET(rx_sock_, &rdset);
  timeval timeout;
  timeout.tv_sec = (time_t)trunc(max_seconds);
  timeout.tv_usec = (suseconds_t)((max_seconds - timeout.tv_sec) * 1e6);
  
  int rv = select(rx_sock_ + 1, &rdset, NULL, NULL, &timeout);

  if (rv > 0 && FD_ISSET(rx_sock_, &rdset)) {
    ///////////////////////////////////////////////////////////////////////////
    int nbytes = recvfrom(rx_sock_, rxbuf, sizeof(rxbuf), 0, NULL, NULL);
    rx(rxbuf, nbytes); // What is rxbuf? What is nbytes?
  }

  return true;
}


/*
    initIMUCal:
      --- Sends UDP message to firmware (enet.c) 
*/
void ReflexHand::initIMUCal() {
  uint8_t msg[1]; 
  msg[0] = 3;               // Set first thing in array CommandPacket enum
  tx(msg, sizeof(msg), PORT_BASE);
}


/*
    loadIMUCalData: 
      -- Appends CommandPacket enum to beginning of array
      -- Transmits appended array to firmware (enet.c)
    Takes in: uint8_t array 
    Returns: Nothing
*/
void ReflexHand::loadIMUCalData(uint8_t data[88]) { //22 Registers * 4 IMUs
  uint8_t msg[89];
  msg[0] = 4; //Cmd byte on firmware to set calibration data

  for (int i = 0; i < 88; i++){
    msg[i + 1] = data[i];
  }

  tx(msg, sizeof(msg), PORT_BASE); 
}

/*
    refreshIMUCalData: 
      -- Appends CommandPacket enum to beginning of array
      -- Transmits appended array to firmware (enet.c)
    Takes in: Nothing
    Returns: Nothing
*/
void ReflexHand::refreshIMUCalData() {
  uint8_t msg[1];
  msg[0] = 5;

  tx(msg, sizeof(msg), PORT_BASE);
}


void ReflexHand::setServoTargets(const uint16_t *targets)
{
  // NUM_SERVOS set to 4 in reflex_hand.h, 
  uint8_t msg[1 + 2 * NUM_SERVOS]; 
  
  /*
  In reflex_hand.h, Command

    enum CommandPacket { CP_SET_SERVO_MODE = 1,C
                         CP_SET_SERVO_TARGET = 2 };

  */

  // Set first entry in array to CommandPacket enum
  msg[0] = CP_SET_SERVO_TARGET;               
   
  // Populate msg array 
  for (int i = 0; i < NUM_SERVOS; i++) {        
    msg[1 + 2 * i] = (targets[i] >> 8) & 0xff;
    msg[2 + 2 * i] = targets[i] & 0xff;
  }

  tx(msg, sizeof(msg), PORT_BASE);
}


void ReflexHand::setServoControlModes(const ControlMode *modes)
{
  uint8_t msg[NUM_SERVOS + 1];
  msg[0] = CP_SET_SERVO_MODE; 

  for (int i = 0; i < NUM_SERVOS; i++)
    msg[i + 1] = (uint8_t)modes[i];

  tx(msg, sizeof(msg), PORT_BASE);
}


void ReflexHand::setServoControlModes(const ControlMode mode)
{
  const ControlMode modes[4] = { mode, mode, mode, mode };
  setServoControlModes(modes);
}


// Struct used in void ReflexHand::rx() only
typedef struct 
{
  uint8_t  header[6];
  uint32_t systime;
  uint16_t tactile_pressures[ReflexHandState::NUM_TACTILE];
  uint16_t tactile_temperatures[ReflexHandState::NUM_TACTILE];
  uint16_t encoders[ReflexHandState::NUM_FINGERS];
  uint8_t  dynamixel_error_states[4];
  uint16_t dynamixel_angles[4];
  uint16_t dynamixel_speeds[4];
  uint16_t dynamixel_loads[4];
  uint8_t  dynamixel_voltages[4];
  uint8_t  dynamixel_temperatures[4];
  uint16_t imus[ReflexHandState::NUM_IMUS * 4];
  int8_t  imu_calibration_status[ReflexHandState::NUM_IMUS];
  uint16_t imu_calibration_data[ReflexHandState::NUM_IMUS * 11];

} __attribute__((packed)) mcu_state_format_1_t;


/*
  TODO: 
    CHECK IF PACKET LENGTH IS WHAT WE WANT
    Add helper function. This is too long (<40 lines)
    Understand what const does
    Ask John to explain passing arrays by reference and UNDERSTAND
*/
void ReflexHand::rx(const uint8_t *msg, const uint16_t msg_len) 
{
  // First, check packet format "magic byte" and the length
  if (msg[0] != 1) {
    ROS_ERROR("Unexpected magic byte received on UDP multicast port: 0x%02x", 
              msg[0]);
    return;
  }

  if (msg_len != sizeof(mcu_state_format_1_t)) {  // Leftover palm data adds 44 bytes
    ROS_INFO("msg_len: %d", msg_len);
    ROS_ERROR("EXPOCTED packet length %d, but saw %d instead",
              (int)sizeof(mcu_state_format_1_t), msg_len - 44);
    return;
  }

  // Create instance of struct, set pointer to msg
  mcu_state_format_1_t *rx_state_msg = (mcu_state_format_1_t *)msg; 
  rx_state_.systime_us_                  = rx_state_msg->systime;
  
  for (int i = 0; i < ReflexHandState::NUM_FINGERS; i++)
    rx_state_.encoders_[i]               = rx_state_msg->encoders[i];
  
  for (int i = 0; i < ReflexHandState::NUM_TACTILE; i++) {
    rx_state_.tactile_pressures_[i]      = rx_state_msg->tactile_pressures[i];
    rx_state_.tactile_temperatures_[i]   = rx_state_msg->tactile_temperatures[i];
  }

  for (int i = 0; i < 4; i++) {
    rx_state_.dynamixel_error_states_[i] = rx_state_msg->dynamixel_error_states[i];
    rx_state_.dynamixel_angles_[i]       = rx_state_msg->dynamixel_angles[i];
    rx_state_.dynamixel_speeds_[i]       = rx_state_msg->dynamixel_speeds[i];
    rx_state_.dynamixel_loads_[i]        = rx_state_msg->dynamixel_loads[i];
    rx_state_.dynamixel_voltages_[i]     = rx_state_msg->dynamixel_voltages[i];
    rx_state_.dynamixel_temperatures_[i] = rx_state_msg->dynamixel_temperatures[i];
    rx_state_.imu_calibration_status[i]  = rx_state_msg->imu_calibration_status[i];
  }
  
  for (int i = 0; i < ReflexHandState::NUM_IMUS*4; i++)
    rx_state_.imus[i]                    = rx_state_msg->imus[i];

///////////////////////////////////////////////////////////////////////////////

  // TODO(LANCE): Understand what this does. I believe this is untested
  // I think this is wrong. 
  // imu_calibration_data[] should be uint16, thus there should only be 11 entries
  // TODO(LANCE)(Monday October 2, 2017): Comment this out and observe effects

  for (int i = 0; i < ReflexHandState::NUM_IMUS * 22; i++) // Loop 22 x per IMU, 88 total.
    rx_state_.imu_calibration_data[i]    = rx_state_msg->imu_calibration_data[i];

///////////////////////////////////////////////////////////////////////////////      

  // Now that the rx_state_ struct is populated, fire off callback
  // TODO(LANCE): Understand what the state_cb_ callback does
  if (state_cb_)
    state_cb_(&rx_state_);
}


void ReflexHand::setStateCallback(StateCallback callback)
{
  state_cb_ = callback;
}