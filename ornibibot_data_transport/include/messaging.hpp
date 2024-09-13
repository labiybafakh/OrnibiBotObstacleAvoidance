#ifndef MESSAGING_HPP
#define MESSAGING_HPP

#include <Arduino.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "credentials.hpp"


struct ornibibot_data{
    uint32_t timestamp;
    int16_t roll, pitch, yaw;
    int8_t turning;
    float temperature;
    float pressure;
    float altitude;
    float linear_accel_x, linear_accel_y, linear_accel_z;
    float angular_x, angular_y, angular_z;
};

class messaging{
    public:
        messaging(const ornibibot_data &data, WiFiUDP* udp);
        ~messaging();
        bool SendUDP();
    private:
        std::vector<uint8_t> SerializeData();
        volatile ornibibot_data ornibibot_data_;
        WiFiUDP* udp_;
        const char* server_ip_;
        const int udp_port_;
};

#endif