#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time, Duration
import sys
from sensor_msgs.msg import Imu, MagneticField



import logging
import datetime
import pathlib
import os
import threading
import math
from .aceinna.tools import OpenIMU

# For odometry message
from transforms3d.euler import quat2euler, euler2quat

convert_rads = math.pi /180
convert_tesla = 1/10000
frame_id = 'fsds/FSCar'#'OpenIMU'

LOGGER = logging.getLogger(__name__)

class OpenIMUros(Node):
    def __init__(self):
        super().__init__("ros_openimu")
        self.openimudev = OpenIMU()
        self.openimudev.startup()
        self.pub_imu: Publisher = self.create_publisher(Imu, 'sr_imu/imu_acc_ar', 1)
        self.pub_mag: Publisher = self.create_publisher(MagneticField, 'sr_imu/imu_mag', 1)
        #read the data - call the get imu measurement data
        self.packetType = 'a2'                       # z1, s1, a1, a2, e1, e2
        

    def close(self):
        self.openimudev.close()

    def read_z1(self, readback):
        imu_msg = Imu()             # IMU data
        mag_msg = MagneticField()   # Magnetometer data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[1]
        imu_msg.linear_acceleration.y = readback[2]
        imu_msg.linear_acceleration.z = readback[3]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[4] * convert_rads
        imu_msg.angular_velocity.y = readback[5] * convert_rads
        imu_msg.angular_velocity.z = readback[6] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data - convert Gauss to Tesla
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x = readback[7] * convert_tesla
        mag_msg.magnetic_field.y = readback[8] * convert_tesla
        mag_msg.magnetic_field.z = readback[9] * convert_tesla
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        self.pub_mag.publish(mag_msg)

    def read_s1(self, readback):
        imu_msg = Imu()             # IMU data
        mag_msg = MagneticField()   # Magnetometer data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[2]
        imu_msg.linear_acceleration.y = readback[3]
        imu_msg.linear_acceleration.z = readback[4]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[5] * convert_rads
        imu_msg.angular_velocity.y = readback[6] * convert_rads
        imu_msg.angular_velocity.z = readback[7] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data - convert Gauss to Tesla
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x = readback[8] * convert_tesla
        mag_msg.magnetic_field.y = readback[9] * convert_tesla
        mag_msg.magnetic_field.z = readback[10] * convert_tesla
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        self.pub_mag.publish(mag_msg)

    def read_a1(self, readback):
        imu_msg = Imu()             # IMU data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[7]
        imu_msg.linear_acceleration.y = readback[8]
        imu_msg.linear_acceleration.z = readback[9]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[4] * convert_rads
        imu_msg.angular_velocity.y = readback[5] * convert_rads
        imu_msg.angular_velocity.z = readback[6] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

    def read_a2(self, readback):
        imu_msg = Imu()             # IMU data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        roll, pitch, yaw = readback[2], readback[3], readback[4]
        (w, x, y, z) = euler2quat(roll, pitch, yaw)
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[8]
        imu_msg.linear_acceleration.y = readback[9]
        imu_msg.linear_acceleration.z = readback[10]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[5] * convert_rads
        imu_msg.angular_velocity.y = readback[6] * convert_rads
        imu_msg.angular_velocity.z = readback[7] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.orientation.w = w
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z
        imu_msg.orientation_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

    def read_e1(self, readback):
        imu_msg = Imu()             # IMU data
        mag_msg = MagneticField()   # Magnetometer data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[5]
        imu_msg.linear_acceleration.y = readback[6]
        imu_msg.linear_acceleration.z = readback[7]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[8] * convert_rads
        imu_msg.angular_velocity.y = readback[9] * convert_rads
        imu_msg.angular_velocity.z = readback[10] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data - convert Gauss to Tesla
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x = readback[14] * convert_tesla
        mag_msg.magnetic_field.y = readback[15] * convert_tesla
        mag_msg.magnetic_field.z = readback[16] * convert_tesla
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        self.pub_mag.publish(mag_msg)

    def read_e2(self, readback):
        imu_msg = Imu()             # IMU data
        mag_msg = MagneticField()   # Magnetometer data
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = frame_id
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[5]
        imu_msg.linear_acceleration.y = readback[6]
        imu_msg.linear_acceleration.z = readback[7]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[11] * convert_rads
        imu_msg.angular_velocity.y = readback[12] * convert_rads
        imu_msg.angular_velocity.z = readback[13] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data - convert Gauss to Tesla
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x = readback[20] * convert_tesla
        mag_msg.magnetic_field.y = readback[21] * convert_tesla
        mag_msg.magnetic_field.z = readback[22] * convert_tesla
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        self.pub_mag.publish(mag_msg)

    def pub_sensors(self, packetType, readback):
        if packetType == 'z1':
            self.read_z1(readback)
        elif packetType == 's1':
            self.read_s1(readback)
        elif packetType == 'a1':
            self.read_a1(readback)
        elif packetType == 'a2':
            self.read_a2(readback)
        elif packetType == 'e1':
            self.read_e1(readback)
        elif packetType == 'e2':
            self.read_e2(readback)
        else :
            print("unknown packet type")

    def readimu(self, pktType):
        try:
            readback = self.openimudev.getdata(pktType)
            return readback
        except KeyboardInterrupt:
            exit()
        except:
            print("oopsie")
            return None

    def pub_data(self):
        readback = self.readimu(self.packetType)
        if readback is not None:
            self.pub_sensors(self.packetType, readback)

def main(args=sys.argv[1:]):
    # defaults args
    loglevel = 'info'
    print_logs = False

    numeric_level = getattr(logging, loglevel.upper(), None)

    # setting up logging
    path = str(pathlib.Path(__file__).parent.resolve())
    if not os.path.isdir(path + '/logs'):
        os.mkdir(path + '/logs')

    date = datetime.datetime.now().strftime('%d_%m_%Y_%H_%M_%S')
    logging.basicConfig(
        filename=f'{path}/logs/{date}.log',
        filemode='w',
        format='%(asctime)s | %(levelname)s:%(name)s: %(message)s',
        datefmt='%I:%M:%S %p',
        # encoding='utf-8',
        level=numeric_level,
    )
    
    # begin ros node
    rclpy.init(args=args)    

    node = OpenIMUros()
    LOGGER.info("OpenIMU driver initialized.")

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(100)

    try:
        while rclpy.ok():
            node.pub_data()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()

    rclpy.shutdown()
    thread.join()



if __name__ == '__main__':
    main(sys.argv[1:])