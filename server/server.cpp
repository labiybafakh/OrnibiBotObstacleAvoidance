#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <atomic>
#include <fstream>
#include <chrono>
#include <ncurses.h>
#include <iomanip>
#include <sstream>



#define PORT 8080
#define BUFFER_SIZE 1024

std::atomic<uint8_t> flag_fly{0};  // Initialize with 's' (stopped)
std::atomic<bool> is_recording{false};
std::atomic<bool> should_exit{false};
std::ofstream csv_file;

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

std::string generate_filename() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "recording_";
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    ss << ".csv";

    return ss.str();
}

void input_thread() {
    char ch;
    while (true) {
        if(std::cin.get(ch)){
            if (ch == 'a' || ch == 'A') {
                if (!is_recording) {
                    is_recording = true;
                    auto now = std::chrono::system_clock::now();
                    auto in_time_t = std::chrono::system_clock::to_time_t(now);

                    csv_file.open(generate_filename());
                    if (csv_file.is_open()) {
                        csv_file << "ornibibot_data_->timestamp" << ",";
                        csv_file << "ornibibot_data_->turning" << ",";
                        csv_file << "ornibibot_data_->linear_accel_x" << ",";
                        csv_file << "ornibibot_data_->linear_accel_y" << ",";
                        csv_file << "ornibibot_data_->linear_accel_z" << ",";
                        csv_file << "ornibibot_data_->angular_x" << ",";
                        csv_file << "ornibibot_data_->angular_y" << ",";
                        csv_file << "ornibibot_data_->angular_z" << ",";
                        csv_file << "ornibibot_data_->roll" << ",";
                        csv_file << "ornibibot_data_->pitch" << ",";
                        csv_file << "ornibibot_data_->yaw" << ",";
                        csv_file << "ornibibot_data_->temperature" << ",";
                        csv_file << "ornibibot_data_->pressure" << ",";
                        csv_file << "ornibibot_data_->altitude\n";
                        std::cout << "Started recording to CSV." << std::endl;
                    } else {
                        std::cerr << "Failed to open CSV file" << std::endl;
                        is_recording = false;
                    }
                }
            } else if (ch == 's' || ch == 'S') {
                if (is_recording) {
                    is_recording = false;
                    csv_file.close();
                    std::cout << "Stopped recording and saved CSV." << std::endl;
                }
            } else if (ch == 'q' || ch == 'Q') {
                if (is_recording) {
                    is_recording = false;
                    csv_file.close();
                    std::cout << "Stopped recording and saved CSV." << std::endl;
                }
                should_exit = true;
                break;
            }

            else if(ch == 'z' || ch =='Z'){
                flag_fly = 1;
                std::cout << "Robot is flapping" << std::endl;
            }
            else if(ch == 'x' || ch =='X'){
                flag_fly = 0;
                std::cout << "Robot stop to flap" << std::endl;
            }
        }      
        

    }
}


int main() {
    int sockfd;
    char buffer[BUFFER_SIZE];
    struct sockaddr_in servaddr, cliaddr;
    
    // Creating socket file descriptor
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
    
    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
    
    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
    
    // Bind the socket with the server address
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    
    std::cout << "UDP Server is listening on port " << PORT << std::endl;

    std::thread input(input_thread);

    while(true) {
    ssize_t bytes_sent = sendto(sockfd, &flag_fly, sizeof(flag_fly), 0,
                                (struct sockaddr *) &cliaddr, sizeof(cliaddr));


        socklen_t len = sizeof(cliaddr);  // Use socklen_t for address length
        
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, 0, 
                    (struct sockaddr *) &cliaddr, &len);
        
        if (n == sizeof(ornibibot_data)) {
            ornibibot_data* ornibibot_data_ = (ornibibot_data*)buffer;
            
            if(is_recording){
                    csv_file << ornibibot_data_->timestamp << ",";
                    csv_file << ornibibot_data_->turning << ",";
                    csv_file << ornibibot_data_->linear_accel_x << ",";
                    csv_file << ornibibot_data_->linear_accel_y << ",";
                    csv_file << ornibibot_data_->linear_accel_z << ",";
                    csv_file << ornibibot_data_->angular_x << ",";
                    csv_file << ornibibot_data_->angular_y << ",";
                    csv_file << ornibibot_data_->angular_z << ",";
                    csv_file << ornibibot_data_->roll << ",";
                    csv_file << ornibibot_data_->pitch << ",";
                    csv_file << ornibibot_data_->yaw << ",";
                    csv_file << ornibibot_data_->temperature << ",";
                    csv_file << ornibibot_data_->pressure << ",";
                    csv_file << ornibibot_data_->altitude << "\n";

            }
            // std::cout << "Received ornibibot_data from " << inet_ntoa(cliaddr.sin_addr) << ":" << ntohs(cliaddr.sin_port) << std::endl;
            // std::cout << "Timestamp: " << ornibibot_data_->timestamp << std::endl;
            // std::cout << "Roll: " << ornibibot_data_->roll << std::endl;
            // std::cout << "Pitch: " << ornibibot_data_->pitch << std::endl;
            // std::cout << "Yaw: " << ornibibot_data_->yaw << std::endl;
            // std::cout << "Lin-x: " << ornibibot_data_->linear_accel_x << std::endl;
            // std::cout << "Lin-y: " << ornibibot_data_->linear_accel_y << std::endl;
            // std::cout << "Lin-z: " << ornibibot_data_->linear_accel_z << std::endl;
            // std::cout << "Ang-x: " << ornibibot_data_->linear_accel_x << std::endl;
            // std::cout << "Ang-y: " << ornibibot_data_->linear_accel_y << std::endl;
            // std::cout << "Ang-z: " << ornibibot_data_->linear_accel_z << std::endl;
            // std::cout << "Turning: " << ornibibot_data_->turning << std::endl;
            // std::cout << "Altitude: " << ornibibot_data_->altitude << std::endl;
            // std::cout << "Pressure: " << ornibibot_data_->pressure << std::endl;
            // std::cout << "Temparature: " << ornibibot_data_->temperature << std::endl;
        }
    }

    input.join();
    
    close(sockfd);
    return 0;
}