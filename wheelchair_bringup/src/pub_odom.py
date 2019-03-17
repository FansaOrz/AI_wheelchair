#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

'''
左轮99 直径0.54
右轮103
轮距 0.6
'''
import serial
import time
import rospy
from rosserial_arduino.msg import command
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.broadcaster import TransformBroadcaster
from math import cos, sin
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class pub_odom:
    def __init__(self):
        self.left_if_front = True
        self.right_if_front = True
        self.width = 0.58
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th_tol = 0
        self.f = open("./aaa.txt", 'w')
        self.left_num = 0
        self.right_num = 0
        self.left_old = 0
        self.right_old = 0
        self.if_first = True
        self.dura = time.time() + 0.2
        self.current_pose = PoseWithCovarianceStamped()
        rospy.init_node("odom_pub")
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint') # the name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()
        rospy.Subscriber('/command_velocity', command, self.command_callback)
        rospy.Subscriber('/encoder', String, self.encoder_callback)

    def command_callback(self, msg):
        if abs(msg.lwheel_vtarget) > abs(msg.rwheel_vtarget):
            if msg.lwheel_vtarget < 0:
                self.left_if_front = True
                self.right_if_front = True
            elif msg.lwheel_vtarget > 0:
                self.left_if_front = False
                self.right_if_front = False
        else:
            if msg.rwheel_vtarget > 0:
                self.left_if_front = False
                self.right_if_front = True
            elif msg.rwheel_vtarget < 0:
                self.left_if_front = True
                self.right_if_front = False

    def encoder_callback(self, msg):
        line = msg.data
        #print line
        if self.if_first:
            self.left_old = line[0]
            self.right_old = line[1]
            self.if_first = False
            return
        if line[0] != self.left_old:
            self.left_num += 1
            self.left_old = line[0]
        if line[1] != self.right_old:
            self.right_num += 1
            self.right_old = line[1]
        if time.time() > self.dura:
            #left
            ''''
            left_vel = self.left_num * ((3.14 * 0.54) / 100) * 10
            if not self.left_if_front:
                left_vel = -left_vel
            right_vel = self.right_num * ((3.14 * 0.540) / 104) * 10
            if not self.right_if_front:
                right_vel = -right_vel
            x_vel = (left_vel + right_vel) / 2
            ang = (right_vel - left_vel) / (2*self.width)
            self.f.write(str(left_vel) + ' ' + str(right_vel))
            #角度 = 角速度×时间
            self.th = ang * 0.1
            self.th_tol += self.th
            print "left_vel:=", left_vel, "right_vel:=", right_vel, "ang:=", ang, "th_tol:=", self.th_tol
            if (x_vel != 0):
                # calculate distance traveled in x and y
                x = cos(self.th) * (x_vel * 0.1)
                y = sin(self.th) * (x_vel * 0.1)
                # calculate the final position of the robot
                self.x = self.x + x
                self.y = self.y + y
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th_tol / 2)
            quaternion.w = cos(self.th_tol / 2)

            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )
            # 发布odom topic信息
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = x_vel
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = ang
            '''
            dleft = self.left_num * ((3.14*0.54) / 100) / 2
            dright = self.right_num * ((3.14*0.54) / 104) / 2
            if not self.left_if_front:
                dleft = -dleft
            if not self.right_if_front:
                dright = -dright
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / (2*0.6)
            dt = 0.2
            vxy = dxy_ave / dt
            vth = dth / dt
            if dxy_ave != 0:
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th_tol) * dx - sin(self.th_tol) * dy)
                self.y += (sin(self.th_tol) * dx + cos(self.th_tol) * dy)
            if dth != 0:
                self.th_tol += dth
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th_tol / 2)
            quaternion.w = cos(self.th_tol / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )
            print "left_vel:=", dleft, "right_vel:=", dright, "ang:=", vth, "th_tol:=", self.th_tol
            # 发布odom topic信息
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth
            '''
            odom.pose.covariance[0] = 1e-3
            odom.pose.covariance[7] = 1e-3
            odom.pose.covariance[14] = 1e6
            odom.pose.covariance[21] = 1e6
            odom.pose.covariance[28] = 1e6
            odom.pose.covariance[35] = 1e3
            odom.twist.covariance[0] =1e-3
            odom.twist.covariance[7] =1e-3
            odom.twist.covariance[14] =1e6
            odom.twist.covariance[21] =1e6
            odom.twist.covariance[28] =1e6
            odom.twist.covariance[35] =1e3
            '''
            self.odom_pub.publish(odom)
            self.left_num = 0
            self.right_num = 0
            self.dura = time.time() + .2

if __name__ == "__main__":
    ss = pub_odom()
    rospy.spin()
