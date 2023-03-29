#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <serial/serial.h>
#include <math.h>
#include <fstream>
#include <imu_n100/data_struct.h>
#include <imu_n100/crc_table.h>

// ROS2 messages
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <boost/thread.hpp>
#include <string>

using namespace std;
using namespace std::chrono_literals;

namespace FDILink
{
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56
// #define IMU_LEN  0x24   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x54 //84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295

class ahrsBringup : public rclcpp::Node
{
    public:
    ahrsBringup();
    ~ahrsBringup();
    void imu_callback();
    bool checkCS8(int len);
    bool checkCS16(int len);
    void checkSN(int type);
    void magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz); 

    private:
    bool if_debug_;

    // Sum info
    int sn_lost_ = 0;
    int crc_error_ = 0;
    uint8_t read_sn_ = 0;
    bool first_sn_;
    int device_type_ = 1;

    // Serial
    serial::Serial serial_;
    std::string serial_port_;
    int serial_baud_;
    int serial_timeout_;

    // DATA
    FDILink::imu_frame_read imu_frame_;
    FDILink::ahrs_frame_read ahrs_frame_;
    FDILink::insgps_frame_read insgps_frame_;

    // Frame name
    string imu_frame_id_;

    // Topic
    string imu_topic_, mag_pose_2d_topic_;
    float w1;
    float x1;
    float y1;
    float z1;
    float data_store[4];
    int i = 0;
    float yaw;
    float store_yaw;
    float roll = 0.0;
    float pitch = 0.0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_pub;

};
}
#endif