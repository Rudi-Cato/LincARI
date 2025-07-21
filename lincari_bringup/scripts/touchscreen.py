#!/usr/bin/env python

import rospy
from pal_web_msgs.msg import WebGoTo

if __name__ == "__main__":
    rospy.init_node("web_go_to_sample_publisher")

    pub = rospy.Publisher("/web/go_to", WebGoTo, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    msg = WebGoTo()
    msg.type = WebGoTo.TOUCH_PAGE  # or just use `4`, same thing
    msg.value = "FirstPage"   # or any other target page like "Audio_consent"

    rospy.loginfo(f"Publishing WebGoTo: type={msg.type}, value='{msg.value}'")

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
