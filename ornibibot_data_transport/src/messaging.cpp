#include "messaging.hpp"


messaging::messaging(const ornibibot_data &data, WiFiUDP* udp): 
    ornibibot_data_(data), udp_(udp), server_ip_(SERVER_IP), udp_port_(UDP_PORT){

}

messaging::~messaging(){
    
}

bool messaging::SendUDP(){
    std::vector<uint8_t> buffer_(sizeof(ornibibot_data), 0);
    buffer_.assign(messaging::SerializeData().begin(), messaging::SerializeData().end());

    udp_->beginPacket(server_ip_, udp_port_);
    udp_->write((uint8_t*)&ornibibot_data_, sizeof(ornibibot_data_));
    return udp_->endPacket();
}
