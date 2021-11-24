#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

SPEED_FACTOR = 200
ANGLE_FACTOR = 1

class TeleOp:
    def __init__(self):
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback_func)
        self.step_pub = rospy.Publisher('/stepper', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('/servo', Float64, queue_size=1)
        self.angle = 0
        self.vel = 0
        #rospy.Rate(100)
    
    def callback_func(self, msg):
        self.angle += msg.angular.z * ANGLE_FACTOR
        self.vel = msg.linear.x * SPEED_FACTOR
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
    rospy.init_node("py_teleop_subpub")
    tele = TeleOp()
    rospy.spin()
