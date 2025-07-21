#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

def send_motion_goal(motion_name, skip_planning=False, priority=0):
    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

    rospy.loginfo("Waiting for /play_motion action server...")
    client.wait_for_server()
    rospy.loginfo("/play_motion server available!")

    goal = PlayMotionGoal()
    goal.motion_name = motion_name
    goal.skip_planning = skip_planning
    goal.priority = priority

    rospy.loginfo(f"Sending motion: {motion_name}")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"Motion '{motion_name}' completed with result: {result}")

def main():
    rospy.init_node('play_motion_keyboard_sender')

    while not rospy.is_shutdown():
        try:
            motion_name = raw_input("Enter motion name to play (or 'q' to quit): ").strip()
        except NameError:
            motion_name = input("Enter motion name to play (or 'q' to quit): ").strip()

        if motion_name.lower() == 'q':
            break

        if motion_name == '':
            rospy.logwarn("Motion name cannot be empty.")
            continue

        send_motion_goal(motion_name)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass