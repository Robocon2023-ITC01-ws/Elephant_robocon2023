#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from ros_imu_bno055.imu_bno055_api import *
import os

# Ros2 message
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

import numpy as np
import math

# Sensor IMU
class IMU_Node(Node):
    def __init__(self):
        super(IMU_Node, self).__init__('imu_node')

        # Get node name
        self.node_name = self.get_name()

        # Get ros params
        self.get_ros_params()

        # Create an IMU instant
        self.bno055 = BoschIMU(port = self.serial_port)

        # Internal variables
        self.imu_data_seq_counter = 0
        self.stop_request = False 

        # Publisher in Vector3 message
        self.imu_pub = self.create_publisher(Vector3, "/imu/data", 1)
        self.timer_data = self.create_timer(0.001, self.imu_publisher)
        # Publisher in Imu message
        self.imu_pub2 = self.create_publisher(Imu, "/imu/data2", 1)
        self.timer_data2 = self.create_timer(0.001, self.imu_publisher2)
        self.set_imu_configuration()


    def get_ros_params(self):
        self.declare_parameter('/serial_port', '/dev/ttyUSB0')
        self.declare_parameter('/frame_id','imu_link')
        self.declare_parameter('/operation_mode','IMU')
        self.declare_parameter('/oscillator','INTERNAL')
        self.declare_parameter('/reset_orientation',True)
        self.declare_parameter('/frequency',0.05)
        self.serial_port = self.get_parameter('/serial_port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('/frame_id').get_parameter_value().string_value
        self.operation_mode_str = self.get_parameter('/operation_mode').get_parameter_value().string_value
        self.oscillator_str = self.get_parameter('/oscillator').get_parameter_value().string_value
        self.reset_orientation = self.get_parameter('/reset_orientation').get_parameter_value().string_value
        self.frequency = self.get_parameter('/frequency')
        switcher = {
            'IMU' : IMU,
            'COMPASS' : COMPASS,
            'M4G' : M4G,
            'NDOF_FMC_OFF' : NDOF_FMC_OFF,
            'NDOF' : NDOF,
        }

        self.operation_mode = switcher.get(self.operation_mode_str, 'IMU')

        switcher = {
            'EXTERNAL' : EXTERNAL_OSCILLATOR,
            'INTERNAL' : INTERNAL_OSCILLATOR
        }

        self.oscillator = switcher.get(self.oscillator_str, 'INTERNAL')

    def set_imu_configuration(self):

        # IMU configuration: required everytime the IMU is turned on or reset

        # Enable IMU configuration
        status_1 = self.bno055.enable_imu_configuration()

        if status_1 == RESPONSE_OK:
            self.get_logger().info("Configuration mode activated")
        else:
            self.get_logger().error("Unable to activate configuration mode")
            
        # set IMU units
        status_2 = self.bno055.set_imu_units(acceleration_units = METERS_PER_SECOND,
        angular_velocity_units = RAD_PER_SECOND,
        euler_orientation_units = RAD,
        temperature_units = CELSIUS,
        orientation_mode = WINDOWS_ORIENTATION
        )

        if status_2 == RESPONSE_OK:
            self.get_logger().info("Units configured successfully")
        else:
            self.get_logger().warn("Unable to configure units")

        # Set imu axis

        status_3 = self.bno055.set_imu_axis(axis_placement = P1)
        if status_3 == RESPONSE_OK:
            self.get_logger().info("Axis configured successfully")
        else:
            self.get_logger().warn("Unable to configure axis")


        status_oscillator = self.bno055.set_oscillator(oscillator_type = self.oscillator)
        if status_oscillator == RESPONSE_OK:
            self.get_logger().info("oscillator configured successfully")
        else:
            self.get_logger().info("Unable to configure oscillator")

        # Set operation mode. Exit configuration mode and activate IMU to work
        status_4 = self.bno055.set_imu_operation_mode(operation_mode = self.operation_mode)
        if status_4 == RESPONSE_OK:
            self.get_logger().info("Operation mode configured successfully")
        else: 
            self.get_logger().error("Unable to configure operation mode")
        
        # Check all status
        if(status_1 == RESPONSE_OK and status_2 == RESPONSE_OK and status_2 == RESPONSE_OK and status_4 == RESPONSE_OK):
            self.get_logger().info("IMU is working now in mode!")
        else:
            self.get_logger().warn("The IMU was not configured correctly. It may not work")
            

    def reset_imu(self):
        status = self.bno055.reset_imu()
        if status == RESPONSE_OK:
            self.get_logger().info("IMU successfully reset")
        else:
            self.get_logger().warn("Reset IMU failed")


    def euler_from_quaternion(self, x, y, z, w):
        t0 = 2 * (w * x + y * z)
        t1 = 1 - 2 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = 2 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2 * (w * z + x * y)
        t4 = 1 - 2 * (y * y + z * z)
        yaw = -math.atan2(t3, t4)
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        q0 = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        q1 = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        q2 = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        q3 = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return q0, q1, q2, q3

    def imu_publisher(self):
        imu_data = Vector3()
        euler = self.bno055.get_euler_orientation()
        imu_data.x = euler[1]
        imu_data.y = euler[2]
        imu_data.z = euler[0]
        if self.reset_orientation ==True:
            self.reset_imu()
        # Configuration is necessary everytinr the IMU is turned on or reset
        #self.set_imu_configuration()
        if self.stop_request == False:
                self.bno055.update_imu_data()
                self.imu_pub.publish(imu_data)

    def imu_publisher2(self):
        linear_acceleration = self.bno055.get_linear_acceleration()
        gyroscope = self.bno055.get_gyroscope()
        euler = self.bno055.get_euler_orientation()
        imu_data2 = Imu()
        imu_data2.header.frame_id = self.frame_id
        #imu_data2.header.seq = self.imu_data_seq_counter

        # Quaternion from euler
        q0, q1, q2, q3 = self.quaternion_from_euler(euler[1], euler[2], euler[0])
        # Orientation
        imu_data2.orientation.x = q0
        imu_data2.orientation.y = q1
        imu_data2.orientation.z = q2
        imu_data2.orientation.w = q3

        # Linear acceleration
        imu_data2.linear_acceleration.x = linear_acceleration[0]
        imu_data2.linear_acceleration.y = linear_acceleration[1]
        imu_data2.linear_acceleration.z = linear_acceleration[2]

        # Angular velocity
        imu_data2.angular_velocity.x = gyroscope[0]
        imu_data2.angular_velocity.y = gyroscope[1]
        imu_data2.angular_velocity.z = gyroscope[2]

        imu_data2.orientation_covariance[0] = -1
        imu_data2.linear_acceleration_covariance[0] = -1
        imu_data2.angular_velocity_covariance[0] = -1

        #self.imu_data_seq_counter =+ 1
        self.imu_pub2.publish(imu_data2)

def main(args=None):
    rclpy.init(args=args)
    pub_node = IMU_Node()
    rclpy.spin(pub_node)
    pub_node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()


