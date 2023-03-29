#include <imu_n100/imu_driver.h>
#include <Eigen/Eigen>
#include <cstdlib>

namespace FDILink
{
    ahrsBringup::ahrsBringup() : Node("imu_node"), first_sn_(false), serial_timeout_(20)
    {
        this->get_parameter_or("debug", if_debug_, false);
        this->get_parameter_or("device_type", device_type_, 0);
        this->get_parameter_or("imu_topic", imu_topic_, std::string("/imu/data2"));
        this->get_parameter_or("imu_frame", imu_frame_id_, std::string("imu"));
        this->get_parameter_or("mag_pose_2d_topic", mag_pose_2d_topic_, std::string("/mag_pose_2d"));

        // Serial
        this->get_parameter_or("port", serial_port_, std::string("/dev/ttyUSB0"));
        this->get_parameter_or("baud", serial_baud_, 921600);

        // Publisher
        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_.c_str(), 10);
        mag_pose_pub = this->create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_.c_str(), 10);
        try
        {
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baud_);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            serial_.setParity(serial::parity_none);
            serial_.setStopbits(serial::stopbits_one);
            serial_.setBytesize(serial::eightbits);
            serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
            serial_.setTimeout(time_out);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to open port ");
            exit(0);
        }
        if (serial_.isOpen())
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Serial Port initialized");
        }
        else
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to initial Serial port ");
            exit(0);
        }
        imu_callback();
    }

    ahrsBringup::~ahrsBringup()
    {
        if (serial_.isOpen())
            serial_.close();
    }

    void ahrsBringup::imu_callback()
    {   
        RCLCPP_INFO(this->get_logger(), "ahrsBringup::processLoop: start");
        while (rclcpp::ok())
        {
         
            
            if (!serial_.isOpen())
            {
                RCLCPP_WARN(this->get_logger(), "serial unopen");
            }
            // check head start
            uint8_t check_head[1] = {0xff};
            size_t head_s = serial_.read(check_head, 1);
            if (if_debug_)
            {
                if (head_s != 1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Read serial port time out! can't read pack head.");
                }
                std::cout << std::endl;
                std::cout << "check_head: " << std::hex << (int)check_head[0] << std::dec << std::endl;
            }
            if (check_head[0] != FRAME_HEAD)
            {
                continue;
            }
            // check head type
            uint8_t head_type[1] = {0xff};
            size_t type_s = serial_.read(head_type, 1);
            if (if_debug_)
            {
                std::cout << "head_type:  " << std::hex << (int)head_type[0] << std::dec << std::endl;
            }
            if (head_type[0] != TYPE_IMU && head_type[0] != TYPE_AHRS && head_type[0] != TYPE_INSGPS && head_type[0] != 0x50 && head_type[0] != TYPE_GROUND)
            {
                RCLCPP_WARN(this->get_logger(), "head_type error: %02X", head_type[0]);
                continue;
            }
            // check head length
            uint8_t check_len[1] = {0xff};
            size_t len_s = serial_.read(check_len, 1);
            if (if_debug_)
            {
                std::cout << "check_len: " << std::dec << (int)check_len[0] << std::endl;
            }
            if (head_type[0] == TYPE_IMU && check_len[0] != IMU_LEN)
            {
                RCLCPP_WARN(this->get_logger(), "head_len error (imu)");
                continue;
            }
            else if (head_type[0] == TYPE_AHRS && check_len[0] != AHRS_LEN)
            {
                RCLCPP_WARN(this->get_logger(), "head_len error (ahrs)");
                continue;
            }
            else if (head_type[0] == TYPE_INSGPS && check_len[0] != INSGPS_LEN)
            {
                RCLCPP_WARN(this->get_logger(), "head_len error (insgps)");
                continue;
            }
            else if (head_type[0] == TYPE_GROUND || head_type[0] == 0x50)
            {
                uint8_t ground_sn[1];
                size_t ground_sn_s = serial_.read(ground_sn, 1);
                if (++read_sn_ != ground_sn[0])
                {
                    if (ground_sn[0] < read_sn_)
                    {
                        if (if_debug_)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                        sn_lost_ += 256 - (int)(read_sn_ - ground_sn[0]);
                        read_sn_ = ground_sn[0];
                        // continue;
                    }
                    else
                    {
                        if (if_debug_)
                        {
                            RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                        }
                        sn_lost_ += (int)(ground_sn[0] - read_sn_);
                        read_sn_ = ground_sn[0];
                        // continue;
                    }
                }
                uint8_t ground_ignore[500];
                size_t ground_ignore_s = serial_.read(ground_ignore, (check_len[0] + 4));
                continue;
            }
            // read head sn
            uint8_t check_sn[1] = {0xff};
            size_t sn_s = serial_.read(check_sn, 1);
            uint8_t head_crc8[1] = {0xff};
            size_t crc8_s = serial_.read(head_crc8, 1);
            uint8_t head_crc16_H[1] = {0xff};
            uint8_t head_crc16_L[1] = {0xff};
            size_t crc16_H_s = serial_.read(head_crc16_H, 1);
            size_t crc16_L_s = serial_.read(head_crc16_L, 1);
            if (if_debug_)
            {
                std::cout << "check_sn: " << std::hex << (int)check_sn[0] << std::dec << std::endl;
                std::cout << "head_crc8: " << std::hex << (int)head_crc8[0] << std::dec << std::endl;
                std::cout << "head_crc16_H: " << std::hex << (int)head_crc16_H[0] << std::dec << std::endl;
                std::cout << "head_crc16_L: " << std::hex << (int)head_crc16_L[0] << std::dec << std::endl;
            }
            // put header & check crc8 & count sn lost
            if (head_type[0] == TYPE_IMU)
            {
                imu_frame_.frame.header.header_start = check_head[0];
                imu_frame_.frame.header.data_type = head_type[0];
                imu_frame_.frame.header.data_size = check_len[0];
                imu_frame_.frame.header.serial_num = check_sn[0];
                imu_frame_.frame.header.header_crc8 = head_crc8[0];
                imu_frame_.frame.header.header_crc16_h = head_crc16_H[0];
                imu_frame_.frame.header.header_crc16_l = head_crc16_L[0];
                uint8_t CRC8 = CRC8_Table(imu_frame_.read_buf.frame_header, 4);
                if (CRC8 != imu_frame_.frame.header.header_crc8)
                {
                    RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                    continue;
                }
                if (!first_sn_)
                {
                    read_sn_ = imu_frame_.frame.header.serial_num - 1;
                    first_sn_ = true;
                }
                // check sn
                ahrsBringup::checkSN(TYPE_IMU);
            }
            else if (head_type[0] == TYPE_AHRS)
            {
                ahrs_frame_.frame.header.header_start = check_head[0];
                ahrs_frame_.frame.header.data_type = head_type[0];
                ahrs_frame_.frame.header.data_size = check_len[0];
                ahrs_frame_.frame.header.serial_num = check_sn[0];
                ahrs_frame_.frame.header.header_crc8 = head_crc8[0];
                ahrs_frame_.frame.header.header_crc16_h = head_crc16_H[0];
                ahrs_frame_.frame.header.header_crc16_l = head_crc16_L[0];
                uint8_t CRC8 = CRC8_Table(ahrs_frame_.read_buf.frame_header, 4);
                if (CRC8 != ahrs_frame_.frame.header.header_crc8)
                {
                    RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                    continue;
                }
                if (!first_sn_)
                {
                    read_sn_ = ahrs_frame_.frame.header.serial_num - 1;
                    first_sn_ = true;
                }
                // check sn
                ahrsBringup::checkSN(TYPE_AHRS);
            }
            else if (head_type[0] == TYPE_INSGPS)
            {
                insgps_frame_.frame.header.header_start = check_head[0];
                insgps_frame_.frame.header.data_type = head_type[0];
                insgps_frame_.frame.header.data_size = check_len[0];
                insgps_frame_.frame.header.serial_num = check_sn[0];
                insgps_frame_.frame.header.header_crc8 = head_crc8[0];
                insgps_frame_.frame.header.header_crc16_h = head_crc16_H[0];
                insgps_frame_.frame.header.header_crc16_l = head_crc16_L[0];
                uint8_t CRC8 = CRC8_Table(insgps_frame_.read_buf.frame_header, 4);
                if (CRC8 != insgps_frame_.frame.header.header_crc8)
                {
                    RCLCPP_WARN(this->get_logger(), "header_crc8 error");
                    continue;
                }
                else if (if_debug_)
                {
                    std::cout << "header_crc8 matched." << std::endl;
                }

                ahrsBringup::checkSN(TYPE_INSGPS);
            }
            if (head_type[0] == TYPE_IMU)
            {
                uint16_t head_crc16_l = imu_frame_.frame.header.header_crc16_l;
                uint16_t head_crc16_h = imu_frame_.frame.header.header_crc16_h;
                uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
                size_t data_s = serial_.read(imu_frame_.read_buf.read_msg, (IMU_LEN + 1)); // 48+1
                //==============================
                // if (if_debug_){
                //   for (size_t i = 0; i < (IMU_LEN + 1); i++)
                //   {
                //     std::cout << std::hex << (int)imu_frame_.read_buf.read_msg[i] << " ";
                //   }
                //   std::cout << std::dec << std::endl;
                // }
                //================================
                uint16_t CRC16 = CRC16_Table(imu_frame_.frame.data.data_buff, IMU_LEN);
                if (if_debug_)
                {
                    std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
                    std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                    std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                    std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                    bool if_right = ((int)head_crc16 == (int)CRC16);
                    std::cout << "if_right: " << if_right << std::endl;
                }

                if (head_crc16 != CRC16)
                {
                    RCLCPP_WARN(this->get_logger(), "check crc16 faild(imu).");
                    continue;
                }
                else if (imu_frame_.frame.frame_end != FRAME_END)
                {
                    RCLCPP_WARN(this->get_logger(), "check frame end.");
                    continue;
                }
            }
            // Information
            else if (head_type[0] == TYPE_AHRS)
            {
                uint16_t head_crc16_l = ahrs_frame_.frame.header.header_crc16_l;
                uint16_t head_crc16_h = ahrs_frame_.frame.header.header_crc16_h;
                uint16_t head_crc16 = head_crc16_l + (head_crc16_h << 8);
                size_t data_s = serial_.read(ahrs_frame_.read_buf.read_msg, (AHRS_LEN + 1)); // 48+1
                //================================================
                // if (if_debug_){
                //   for (size_t i = 0; i < (AHRS_LEN + 1); i++)
                //   {
                //     std::cout << std::hex << (int)ahrs_frame_.read_buf.read_msg[i] << " ";
                //   }
                //   std::cout << std::dec << std::endl;
                // }
                //================================================
                uint16_t CRC16 = CRC16_Table(ahrs_frame_.frame.data.data_buff, AHRS_LEN);
                if (if_debug_)
                {
                    std::cout << "CRC16:        " << std::hex << (int)CRC16 << std::dec << std::endl;
                    std::cout << "head_crc16:   " << std::hex << (int)head_crc16 << std::dec << std::endl;
                    std::cout << "head_crc16_h: " << std::hex << (int)head_crc16_h << std::dec << std::endl;
                    std::cout << "head_crc16_l: " << std::hex << (int)head_crc16_l << std::dec << std::endl;
                    bool if_right = ((int)head_crc16 == (int)CRC16);
                    std::cout << "if_right: " << if_right << std::endl;
                }

                if (head_crc16 != CRC16)
                {
                    RCLCPP_WARN(this->get_logger(), "check crc16 faild(ahrs).");
                    continue;
                }
                else if (ahrs_frame_.frame.frame_end != FRAME_END)
                {
                    RCLCPP_WARN(this->get_logger(), "check frame end.");
                    continue;
                }
            }
            else if (head_type[0] == TYPE_INSGPS)
            {
                uint16_t head_crc16 = insgps_frame_.frame.header.header_crc16_l + ((uint16_t)insgps_frame_.frame.header.header_crc16_h << 8);
                size_t data_s = serial_.read(insgps_frame_.read_buf.read_msg, (INSGPS_LEN + 1)); // 48+1
                uint16_t CRC16 = CRC16_Table(insgps_frame_.frame.data.data_buff, INSGPS_LEN);
                if (head_crc16 != CRC16)
                {
                    RCLCPP_WARN(this->get_logger(), "check crc16 faild(insgps).");
                    continue;
                }
                else if (insgps_frame_.frame.frame_end != FRAME_END)
                {
                    RCLCPP_WARN(this->get_logger(), "check frame end.");
                    continue;
                }
            }
            if (head_type[0] == TYPE_IMU)
            {
                // publish imu topic
                auto imu_data = sensor_msgs::msg::Imu();
                // imu_data.header.stamp = rclcpp::Time::now();
                //imu_data.header.stamp = rclcpp::Time::now();
                imu_data.header.frame_id = imu_frame_id_.c_str();
                Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                                          ahrs_frame_.frame.data.data_pack.Qx,
                                          ahrs_frame_.frame.data.data_pack.Qy,
                                          ahrs_frame_.frame.data.data_pack.Qz);

                Eigen::Quaterniond q_r =
                    Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q_rr =
                    Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitX());
                Eigen::Quaterniond q_xiao_rr =
                    Eigen::AngleAxisd(3.14159 / 2, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitX());
                    
                if (device_type_ == 0)
                {
                    data_store[0] = ahrs_frame_.frame.data.data_pack.Qx;
                    data_store[1] = ahrs_frame_.frame.data.data_pack.Qy;
                    data_store[2] = ahrs_frame_.frame.data.data_pack.Qz;
                    data_store[3] = ahrs_frame_.frame.data.data_pack.Qw;
                    yaw = -atan2(2*(data_store[3]*data_store[2] + data_store[1]*data_store[0]), 1 - 2*(data_store[1]*data_store[1] + data_store[2]*data_store[2]));
                    
                    while (i <= 6)
                    {
                        store_yaw = yaw;
                        i += 1;
                        break;
                    }
                    if (store_yaw >0)
                    {
                        if(yaw>=store_yaw && yaw <=3.14)
                        {
                            yaw = yaw - store_yaw;
                        }
                        else if(yaw <store_yaw && yaw>-3.139)
                        {
                            yaw =6.28-store_yaw + yaw;
                        }
                        if (yaw>3.14 && yaw <6.28)
                        {
                            yaw = yaw - 6.28;
                        }
                    }
                    else if(store_yaw <0)
                    {
                        if(yaw<=3.14 && yaw >=store_yaw)
                        {
                            yaw = yaw - store_yaw;
                        }
                        else if(yaw >-3.139 && yaw<store_yaw)
                        {
                            yaw = 6.28-store_yaw + yaw;
                        }
                        if (yaw>3.14 && yaw <6.28)
                        {
                            yaw = yaw - 6.28;
                        }
                    }
                    
                    yaw = int(yaw*1000 + .5);
                    yaw = (float)(yaw/1000);
                    std::cout << "yaw: "  << (float) yaw<< std::endl;
                    //std::cout << "yaw: "  << (float)(store_yaw)<< std::endl;
                    // imu_data.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
                    // imu_data.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
                    // imu_data.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
                    // imu_data.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
                    imu_data.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
                    imu_data.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
                    imu_data.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
                    imu_data.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
                    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
                    imu_data.angular_velocity.y = ahrs_frame_.frame.data.data_pack.PitchSpeed;
                    imu_data.angular_velocity.z = ahrs_frame_.frame.data.data_pack.HeadingSpeed;
                    imu_data.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
                    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
                    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
                }
                else if (device_type_ == 1)
                {
                    Eigen::Quaterniond q_out = q_r * q_ahrs * q_rr;
                    // Eigen::Quaterniond q_out = q_ahrs;
                    imu_data.orientation.w = q_out.w();
                    imu_data.orientation.x = q_out.x();
                    imu_data.orientation.y = q_out.y();
                    imu_data.orientation.z = q_out.z();
                    imu_data.angular_velocity.x = ahrs_frame_.frame.data.data_pack.RollSpeed;
                    imu_data.angular_velocity.y = -ahrs_frame_.frame.data.data_pack.PitchSpeed;
                    imu_data.angular_velocity.z = -ahrs_frame_.frame.data.data_pack.HeadingSpeed;
                    imu_data.linear_acceleration.x = -imu_frame_.frame.data.data_pack.accelerometer_x;
                    imu_data.linear_acceleration.y = imu_frame_.frame.data.data_pack.accelerometer_y;
                    imu_data.linear_acceleration.z = imu_frame_.frame.data.data_pack.accelerometer_z;
                }
                imu_pub->publish(imu_data);
                Eigen::Quaterniond rpy_q(imu_data.orientation.w,
                                         imu_data.orientation.x,
                                         imu_data.orientation.y,
                                         imu_data.orientation.z);

                auto pose_2d = geometry_msgs::msg::Pose2D();
                double magx, magy, magz, roll, pitch;
                if (device_type_ == 0)
                {   
                    magx = imu_frame_.frame.data.data_pack.magnetometer_x;
                    magy = imu_frame_.frame.data.data_pack.magnetometer_y;
                    magz = imu_frame_.frame.data.data_pack.magnetometer_z;
                    roll = ahrs_frame_.frame.data.data_pack.Roll;
                    pitch = ahrs_frame_.frame.data.data_pack.Pitch;
                }
                else if (device_type_ == 1)
                {
                    magx = -imu_frame_.frame.data.data_pack.magnetometer_x;
                    magy = imu_frame_.frame.data.data_pack.magnetometer_y;
                    magz = imu_frame_.frame.data.data_pack.magnetometer_z;

                    Eigen::Vector3d EulerAngle = rpy_q.matrix().eulerAngles(2, 1, 0);
                    roll = EulerAngle[2];
                    pitch = EulerAngle[1];
                }
                double magyaw;
                magCalculateYaw(roll, pitch, magyaw, magx, magy, magz);
                pose_2d.theta = magyaw;
                mag_pose_pub->publish(pose_2d);
            }
        }
    }

    void ahrsBringup::magCalculateYaw(double roll, double pitch, double &magyaw, double magx, double magy, double magz)
    {
        double temp1 = magy * cos(roll) + magz * sin(roll);
        double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
        magyaw = atan2(-temp1, temp2);
        if (magyaw < 0)
        {
            magyaw = magyaw + 2 * PI;
        }
        // return magyaw;
    }

    void ahrsBringup::checkSN(int type)
    {
        switch (type)
        {
        case TYPE_IMU:
            if (++read_sn_ != imu_frame_.frame.header.serial_num)
            {
                if (imu_frame_.frame.header.serial_num < read_sn_)
                {
                    sn_lost_ += 256 - (int)(read_sn_ - imu_frame_.frame.header.serial_num);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
                else
                {
                    sn_lost_ += (int)(imu_frame_.frame.header.serial_num - read_sn_);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
            }
            read_sn_ = imu_frame_.frame.header.serial_num;
            break;

        case TYPE_AHRS:
            if (++read_sn_ != ahrs_frame_.frame.header.serial_num)
            {
                if (ahrs_frame_.frame.header.serial_num < read_sn_)
                {
                    sn_lost_ += 256 - (int)(read_sn_ - ahrs_frame_.frame.header.serial_num);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
                else
                {
                    sn_lost_ += (int)(ahrs_frame_.frame.header.serial_num - read_sn_);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
            }
            read_sn_ = ahrs_frame_.frame.header.serial_num;
            break;

        case TYPE_INSGPS:
            if (++read_sn_ != insgps_frame_.frame.header.serial_num)
            {
                if (insgps_frame_.frame.header.serial_num < read_sn_)
                {
                    sn_lost_ += 256 - (int)(read_sn_ - insgps_frame_.frame.header.serial_num);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
                else
                {
                    sn_lost_ += (int)(insgps_frame_.frame.header.serial_num - read_sn_);
                    if (if_debug_)
                    {
                        RCLCPP_WARN(this->get_logger(), "detected sn lost.");
                    }
                }
            }
            read_sn_ = insgps_frame_.frame.header.serial_num;
            break;

        default:
            break;
        }
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FDILink::ahrsBringup>());
    FDILink::ahrsBringup bp;
    rclcpp::shutdown();
    return 0;
}
