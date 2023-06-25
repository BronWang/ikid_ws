#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf.transformations import quaternion_from_euler



def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# Parsing serial Port Data
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    angle_flag=False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    # According to the judgment of the data length bit, the corresponding length data can be obtained
    if key < 11:
        return
    else:
        data_buff = list(buff.values())  # Get dictionary ownership value
        if buff[1] == 0x51 :
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 Check failure')

        elif buff[1] == 0x52:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

            else:
                print('0x52 Check failure')

        elif buff[1] == 0x53:
            if checkSum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                angle_flag = True
            else:
                print('0x53 Check failure')
        elif buff[1] == 0x54:
            if checkSum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
            else:
                print('0x54 Check failure')
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag:                
            stamp = rospy.get_rostime()

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_link"

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = "base_link"

            angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            imu_msg.orientation.x = qua[0]
            imu_msg.orientation.y = qua[1]
            imu_msg.orientation.z = qua[2]
            imu_msg.orientation.w = qua[3]

            imu_msg.angular_velocity.x = angularVelocity[0]
            imu_msg.angular_velocity.y = angularVelocity[1]
            imu_msg.angular_velocity.z = angularVelocity[2]

            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]

            mag_msg.magnetic_field.x = magnetometer[0]
            mag_msg.magnetic_field.y = magnetometer[1]
            mag_msg.magnetic_field.z = magnetometer[2]

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)


key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    rospy.init_node("wit_normal")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 115200)
    print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32mSerial port opened successfully...\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32mSerial port opened successfully...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31mSerial port opening failure\033[0m")
        exit(0)
    else:
        imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=100)
        mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=100)

        while not rospy.is_shutdown():
            try:
                buff_count = wt_imu.inWaiting()
            except Exception as e:
                print("exception:" + str(e))
                print("imu disconnect")
                exit(0)
            else:
                if buff_count > 0:
                    buff_data = wt_imu.read(buff_count)
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])

