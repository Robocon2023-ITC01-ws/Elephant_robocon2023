#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_imu_bno055.imu_bno055_api import * 
import time
import os

NOT_CALIBRATED = 0x00
FULL_CALIBRATION = 0x01

class CalibrationIMU(Node):

    def __init__(self):


        # Init node
        super(CalibrationIMU, self).__init__('ros_imu_bno055_calibration_node')

        # timer 
        self.timer_run = self.create_timer(0.001, self.run)
        # Get node name
        self.node_name = self.get_name()

        # Get ros params
        self.get_ros_params()

        # Create an IMU instance
        self.bno055 = BoschIMU(port = self.serial_port)

        self.calibration_full_counter = 0

        # self.init_calibration()

    def get_ros_params(self):
        self.declare_parameter('/serial_port','/dev/ttyUSB0')
        self.declare_parameter('/operation_mode', 'IMU')
        self.serial_port = self.get_parameter('/serial_port').get_parameter_value().string_value
        self.operation_mode_str = self.get_parameter('/operation_mode').get_parameter_value().string_value

        switcher = {

            'IMU': IMU,
            'COMPASS': COMPASS,
            'M4G': M4G,
            'NDOF_FMC_OFF': NDOF_FMC_OFF,
            'NDOF': NDOF,
        }

        self.operation_mode = switcher.get(self.operation_mode_str, 'IMU')


    def init_calibration(self):

        print("=============================================================")
        self.get_logger().info("The IMU will be calibrated to work in %s mode", self.operation_mode_str)
        print("=============================================================")



    def calibrate_imu(self):

        is_imu_calibrated = NOT_CALIBRATED

        calibration_status, status = self.bno055.calibrate_imu(self.operation_mode)
        
        if status == RESPONSE_OK:

            system_calibration_status = calibration_status[0]
            gyroscope_calibration_status = calibration_status[1]
            accelerometer_calibration_status = calibration_status[2]
            magnetometer_calibration_status = calibration_status[3]
            

            # Calibration for NDOF_FMC_OFF and NDOF             
            if self.operation_mode == NDOF_FMC_OFF or self.operation_mode == NDOF:

                print("[System: " + str(system_calibration_status) + "]", end = '')
                print(" [Gyroscope: " + str(gyroscope_calibration_status) + "]", end = '' ) 
                print(" [Accelerometer: " + str(accelerometer_calibration_status) + "]", end = '')
                print(" [Magnetometer: " + str(magnetometer_calibration_status) + "]" )

                if (system_calibration_status == 3 and gyroscope_calibration_status == 3
                and accelerometer_calibration_status == 3 and magnetometer_calibration_status == 3) :

                    self.calibration_full_counter+=1


            # Calibration for IMU
            if self.operation_mode == IMU:

                print(" [Gyroscope: " + str(gyroscope_calibration_status) + "]", end = '' ) 
                print(" [Accelerometer: " + str(accelerometer_calibration_status) + "]")

                if (gyroscope_calibration_status == 3 and accelerometer_calibration_status == 3) :
                    self.calibration_full_counter+=1
            

            # Calibration for COMPASS  and M4G
            if self.operation_mode == COMPASS or self.operation_mode == M4G:
                
                print(" [Accelerometer: " + str(accelerometer_calibration_status) + "]", end = '')
                print(" [Magnetometer: " + str(magnetometer_calibration_status) + "]" )

                if(accelerometer_calibration_status == 3 and magnetometer_calibration_status == 3) :
                    self.calibration_full_counter+=1


            # When the calibration is FULL three consecutive times, the calibration is assumed to be good
            if self.calibration_full_counter >= 3:

                is_imu_calibrated = FULL_CALIBRATION
                self.get_logger().info("IMU successfully calibrated!")


            time.sleep(1)

        return is_imu_calibrated


    def read_calibration(self):

        calibration, status = self.bno055.get_calibration()

        if status == RESPONSE_OK:
            #rospy.loginfo("The obtained calibration is: ")
            #print(calibration)
            pass
        else:
            self.get_logger().error("Unable to read IMU calibration")

        return calibration


    def write_calibration(self, calibration):


        status = self.bno055.set_calibration(calibration)
    
        if status == RESPONSE_OK:
            self.get_logger().info("Calibration successfully written to the IMU")
            self.save_calibration_in_file(calibration)

        else:
            self.get_logger().info("Unable to calibrate the IMU")


    def save_calibration_in_file(self, calibration):

        # Path to this file
        dir_path = os.path.dirname(os.path.realpath(__file__))

        try: 

            binary_file = open(str(dir_path) + "/" + str(self.operation_mode_str) + "_calibration", "wb")
            binary_file.write(calibration)
            binary_file.close()

            self.get_logger().info("Calibration successfully saved in binary file 'calibration'")

        except: 

            self.get_logger().error("Error while saving calibration in file 'calibration'")
        
        

    def read_calibration_from_file(self):

        # Path to this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        
        try:
            binary_file = open(str(dir_path) + "/calibration", "rb")
            calibration_data = binary_file.read()
            binary_file.close()

            self.get_logger().info("The calibration read from the 'calibration' file is: ")
            print(calibration_data)
        
        except:
            calibration_data = 0
            self.get_logger().error("The file does not exist or the file cannot be read")

        return calibration_data


    def run(self):

        # IMU calibration
        status = self.calibrate_imu()

        if status == FULL_CALIBRATION:

            # Read calibration and show it by the terminal
            calibration_data = self.read_calibration()

            # Write the calibration to the IMU and save it to a binary file
            self.write_calibration(calibration_data)

            self.get_logger().info("Calibration has finished successfully. Closing node...")
            # self.signal_shutdown("")

def main(args=None):
    rclpy.init(args=args)
    imu_calibration = CalibrationIMU()
    rclpy.spin(imu_calibration)
    imu_calibration.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()