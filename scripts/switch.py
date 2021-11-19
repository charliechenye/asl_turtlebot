#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class KeyBoardSwitch:
    def __init__(self):
        rospy.init_node('keyboard_switch_control')
        self.tel_flg = rospy.get_param("/switch/kbd_flg", False)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.tel_sub = rospy.Subscriber('/cmd_vel_tel', Twist, self.tel_callback)
        self.nav_sub = rospy.Subscriber('/cmd_vel_nav', Twist, self.nav_callback)
        self.switch_sub = rospy.Subscriber('/switch_input', Bool, self.switch_callback)

    def tel_callback(self, msg):
        if self.tel_flg:
            self.pub.publish(msg)

    def nav_callback(self, msg):
        if not self.tel_flg:
            self.pub.publish(msg)

    def switch_sub(self, msg):
        self.tel_flg = msg.data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    gr = KeyBoardSwitch()
    gr.run()
