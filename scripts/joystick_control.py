#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_SPEED = 600
ANGLE_FACTOR = 90
epsilon = 0.12

class TeleOpJoy:
    def __init__(self):
        self.sub = rospy.Subscriber('/joy', Joy, self.callback_func)
        self.modesub = rospy.Subscriber('/joy', Joy, self.select_mode)
        self.step_pub = rospy.Publisher('/stepper', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo', Float64, queue_size=1)
        self.angle = 0
        self.vel = 0
        self.mode = 0
        #rospy.Rate(100)
    
    def select_mode(self, msg):
        if msg.buttons[0] == 1:
            self.mode = 1
        elif msg.buttons[1] == 1:
            self.mode = 2
        

    def callback_func(self, msg):

        if self.mode == 1:
            if abs(msg.axes[4]) > epsilon:
                self.vel = round(msg.axes[4], 2) * MAX_SPEED
            else:
                self.vel = 0

            self.angle = (1 + round(msg.axes[1], 1)) * ANGLE_FACTOR
            servo_msg, stepper_msg = Float64(), Float64()
            servo_msg.data = self.angle
            stepper_msg.data = self.vel
            self.vels_pub(servo_msg, stepper_msg)

        rospy.loginfo('Angle is {} and vel is {}'.format(self.angle, self.vel))
    
    def vels_pub(self, msg_servo, msg_stepper):
        rospy.sleep(0.001)
        self.step_pub.publish(msg_stepper)
        self.servo_pub.publish(msg_servo)



if __name__ == "__main__":
    rospy.init_node("py_teleop_joy_subpub")
    tele = TeleOpJoy()
    rospy.spin()