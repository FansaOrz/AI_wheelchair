#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
#roslib.load_manifest('turtlebot_teleop') # No longer needed in catkin!
import rospy
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import sys, select, termios, tty



class joy_con():
    def __init__(self):
        self.moveBindings = {
            'i':(1,0),
            'k':(0, 0),
            'o':(1,-1),
            'j':(0,1),
            'l':(0,-1),
            'u':(1,1),
            ',':(-1,0)}
        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9)}
        self.key = ''
        self.speed = 1
        self.turn = 1
        rospy.init_node('wheelchair_joy')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0
        self.msg = """
        Control Your Wheelchair!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
                ,    

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        space key, k : force stop
        anything else : stop smoothly

        CTRL-C to quit
        """
        self.pub_joy()

    def joyCallback(self, msg):
        if msg.buttons[11] == 1.0:
            self.key = 'b'
            return
        if msg.buttons[10] == 1.0:
            self.key = 's'
            return 
        if msg.axes[5] == 1.0:
            if msg.axes[4] == -1.0:
                self.key = 'o'
            elif msg.axes[4] == 1.0:
                self.key = 'u'
            elif msg.axes[4] == 0.0:
                self.key = 'i'
        elif msg.axes[5] == -1.0 and msg.axes[4] == 0.0:
            self.key = ','
        elif msg.axes[5] == 0.0:
            if msg.axes[4] == 1.0:
                self.key = 'j'
            elif msg.axes[4] == -1.0:
                self.key = 'l'
            elif msg.axes[4] == 0.0:
                self.key = 'k'
        print self.key

    def pub_joy(self):
        while True:
            if self.key == 'b':
                print "break"
                twist = Twist()
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
                self.pub.publish(twist)
                exit()
            if self.key == '':
                print "continue"
                continue
            if self.key in self.moveBindings.keys():
                self.x = self.moveBindings[self.key][0]
                self.th = self.moveBindings[self.key][1]
                self.count = 0
            elif self.key == ' ' or self.key == 's':
                self.x = 0
                self.th = 0
                self.control_speed = 0
                self.control_turn = 0
            else:
                self.count = self.count + 1
                if self.count > 4:
                    self.x = 0
                    self.th = 0
                if (self.key == '\x03'):
                    break
            self.target_speed = self.speed * self.x
            self.target_turn = self.turn * self.th

            if self.target_speed > self.control_speed:
                self.control_speed = min(self.target_speed, self.control_speed + 0.06)
            elif self.target_speed < self.control_speed:
                self.control_speed = max(self.target_speed, self.control_speed - 0.06)
            else:
                self.control_speed = self.target_speed

            if self.target_turn > self.control_turn:
                self.control_turn = min(self.target_turn, self.control_turn + 0.1)
            elif self.target_turn < self.control_turn:
                self.control_turn = max(self.target_turn, self.control_turn - 0.1)
            else:
                self.control_turn = self.target_turn

            twist = Twist()
            twist.linear.x = self.control_speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.control_turn
            self.pub.publish(twist)
            sleep(.1)
            # print("loop: {0}".format(count))
            # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))


#speed = .2


if __name__=="__main__":
    joy = joy_con()
    rospy.spin()
    '''
    loop_receive()
    try:
        print msg
        print vels(speed,turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print vels(speed,turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    '''

