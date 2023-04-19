#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include <math.h>
#include <fstream>

#define MAXSIZE 1024
serial::Serial ser;

#define PI 3.1415926
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define DEG_TO_RAD 0.017453292519943295

uint8_t IMU_Data[64];
uint8_t AHRS_Data[56];
bool Data_of_IMU = 0;
bool Data_of_AHRS = 0;

using namespace std;
using namespace std::chrono_literals;


class imu_pub : public rclcpp::Node
{
    public:
    imu_pub()
    : Node("imu_node")
    {
        pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("imu_raw_data", 1000);

        try
        {
            ser.setPort("/dev/ttyUSB0");
            ser.setBaudrate(921600);
            // ser.setFlowcontrol(serial::flowcontrol_none);
            // ser.setParity(serial::parity_none);
            // ser.setStopbits(serial::stopbits_one);
            // ser.setBytesize(serial::eightbits);
            serial::Timeout to=serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port");
            exit(0);
        }

        if (ser.isOpen())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port Initialized");
        }

        else
        {
            exit(0);
        }
        rclcpp::Rate loop_rate(100);
        while (rclcpp::ok())
        {
            size_t num = ser.available();
            auto ros_buffer = std_msgs::msg::UInt8MultiArray();
            if (num!=0)
            {
                ser.read(ros_buffer.data, num);
                for(int i =0; i<ros_buffer.data.size(); i++)
                {
                    std::cout<<std::hex<<(ros_buffer.data[i]&0xff)<<" ";
                }
                std::cout<<std::endl;
            }
            pub->publish(ros_buffer);
            loop_rate.sleep();
        }
    }

    private:
    bool if_debug_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv),
    rclcpp::spin(std::make_shared<imu_pub>());
    rclcpp::shutdown();
    return 0;
}
