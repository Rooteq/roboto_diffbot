#ifndef DIFFDRIVE_ARDUINO_CAN_COMMS_HPP
#define DIFFDRIVE_ARDUINO_CAN_COMMS_HPP

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>
#include <unistd.h>
#include <iostream>


struct Config
{
    std::string left_wheel_name = "";
    std::string right_wheel_name = "";
    float loop_rate = 0.0;
    std::string can_interface = "";
    int enc_counts_per_rev = 0;
};


// CAN IDs for different message types
constexpr canid_t MOTOR_COMMAND_ID = 0x101;  // ID for motor commands
constexpr canid_t ENCODER_REQUEST_ID = 0x102; // ID for encoder request
constexpr canid_t ENCODER_RESPONSE_ID = 0x103; // ID for encoder response

class ArduinoComms
{
public:
    ArduinoComms() : socket_fd_(-1) {}

    void connect(const std::string &can_interface)
    {
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            throw std::runtime_error("Error creating CAN socket");
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface.c_str());
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Error getting CAN interface index");
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Error binding CAN socket");
        }

        // Set up filter to only receive encoder response messages
        struct can_filter filter;
        filter.can_id = ENCODER_RESPONSE_ID;
        filter.can_mask = CAN_SFF_MASK;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
            close(socket_fd_);
            throw std::runtime_error("Error setting CAN filter");
        }
    }

    void disconnect()
    {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    bool connected() const
    {
        return socket_fd_ >= 0;
    }

    void read_encoder_values(int32_t &val_1, int32_t &val_2)
    {
        can_frame frame;
        frame.can_id = ENCODER_REQUEST_ID;
        frame.can_dlc = 0;  // Empty frame for request
        
        if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::runtime_error("Error sending can motor request");
        }

        if (read(socket_fd_, &frame, sizeof(frame)) < 0) {
            throw std::runtime_error("Error reading can motor response");
        }

        if (frame.can_id != ENCODER_RESPONSE_ID) {
            throw std::runtime_error("Unexpected CAN ID");
        }

        val_1 = ((int32_t)frame.data[0] << 24) | 
                ((int32_t)frame.data[1] << 16) | 
                ((int32_t)frame.data[2] << 8)  | 
                (int32_t)frame.data[3];

        val_2 = ((int32_t)frame.data[4] << 24) | 
                ((int32_t)frame.data[5] << 16) | 
                ((int32_t)frame.data[6] << 8)  | 
                (int32_t)frame.data[7];
    }

    void set_motor_values(int val_1, int val_2)
    {
        struct can_frame frame;
        frame.can_id = MOTOR_COMMAND_ID;
        frame.can_dlc = 4;

        val_1 = std::max(std::min(val_1, 32767), -32768);
        val_2 = std::max(std::min(val_2, 32767), -32768);

        frame.data[0] = (val_1 >> 8) & 0xFF;
        frame.data[1] = val_1 & 0xFF;
        frame.data[2] = (val_2 >> 8) & 0xFF;
        frame.data[3] = val_2 & 0xFF;

        if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
            throw std::runtime_error("Error sending motor values");
        }
    }

private:
    int socket_fd_;
};

#endif // DIFFDRIVE_ARDUINO_CAN_COMMS_HPP