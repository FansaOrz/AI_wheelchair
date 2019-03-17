#!/usr/bin/env python
import rospy
import serial
import string
import math
from time import time, sleep
from sensor_msgs.msg import Imu
import tf
import io

grad2rad = 3.141592/180.0

rospy.init_node("imu_node")
pub = rospy.Publisher('imu_data', Imu, queue_size=10)
#print pubRaw

imuMsg = Imu()
imuMsg.orientation_covariance = [999999 , 0 , 0,
                                0, 9999999, 0,
                                0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,
                                      0 , 99999, 0,
                                      0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
                                        0 , 0.2, 0,
                                        0 , 0 , 0.2]
roll=0
pitch=0
yaw=0
num = 0
ser = serial.Serial('/dev/ttyUSB0', 9600)
line = ser.readline(300)
while line:
    if len(line) > 15:
        print len(line)
        print line

        '''
        line = line.replace("#YPR=","")
        line = line.replace("#YPRAMG=","")   # Delete "#YPR="
        '''
        words = string.split(line,",")    # Fields split
        if len(words) > 2:
            try:
                yaw = float(words[0]) * grad2rad
                pitch = -float(words[1]) * grad2rad
                roll = -float(words[2]) * grad2rad

                # Publish message
                imuMsg.linear_acceleration.x = float(words[3]) # tripe axis accelerator meter
                imuMsg.linear_acceleration.y = float(words[4])
                imuMsg.linear_acceleration.z = float(words[5])

                imuMsg.angular_velocity.x = float(words[9]) #gyro
                imuMsg.angular_velocity.y = float(words[10])
                imuMsg.angular_velocity.z = float(words[11])
            except Exception as e:
                num = 1

            q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
            imuMsg.orientation.x = q[0] #magnetometer
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]

            imuMsg.header.stamp= rospy.Time.now()
            imuMsg.header.frame_id = 'base_link'
            pub.publish(imuMsg)
            line = ser.readline(300)
    else:
        ser = serial.Serial('/dev/ttyUSB0', 9600)
        line = ser.readline(300)
ser.close()
#f.close
