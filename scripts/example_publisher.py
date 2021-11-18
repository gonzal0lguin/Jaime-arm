
#!/usr/bin/env python

#rostopic pub /stepper std_msgs/Float64 "data: 0.0"
#rosrun rosserial_python serial_node.py /dev/ttyUSB0


import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool

class NumberCounter:
    def __init__(self):
        self.counter = 0
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=1)
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_number)
        self.reset_service = rospy.Service("/reset_counter", SetBool, self.callback_reset_counter)
    def callback_number(self, msg):
        print(msg)
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        rospy.sleep(0.3)
        self.pub.publish(new_msg)
    def callback_reset_counter(self, req):
        if req.data:
            self.counter = 0
            return True, "Counter has been successfully reset"
        return False, "Counter has not been reset"

if __name__ == '__main__':
    rospy.init_node('number_counter')
    yay = NumberCounter()
    new_msg = Int64()
    new_msg.data = 10000
    # print(new_msg)
    rospy.sleep(0.3)
    #yay.pub.publish(new_msg)
    # yay.callback_number(new_msg)
    new_pub = rospy.Publisher("/stepper", Int64, queue_size=1)
    rospy.sleep(0.5)
    new_pub.publish(new_msg)    
    rospy.spin()



