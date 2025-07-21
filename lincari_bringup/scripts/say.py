#!/usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import sys
import random
import os

# ROS imports
import rospy

# Actions
from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal

def connection():
    tts_client = SimpleActionClient('/tts', TtsAction)
    tts_client.wait_for_server()
    return tts_client

def create_goal(text, lang_id='en_GB'):
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = lang_id
    return goal

def log_text(text, log_file='spoken_log.txt'):
    with open(log_file, 'a', encoding='utf-8') as f:
        f.write(text + '\n')

if __name__ == '__main__':
    try:
        rospy.init_node('speak')
        print("connecting to TTS server")
        tts_client = connection()
        print("connected !")

        while not rospy.is_shutdown():
            text = input("what would you like to say? ")
            if not text.strip():
                rospy.loginfo("no text entered")
                continue

            log_text(text)  # Log the input text to a file

            goal = create_goal(text)
            print("sending goal to TTS server")
            tts_client.send_goal_and_wait(goal)
            print("Finished")

    except rospy.ROSInterruptException:
        print("Abruptly finished!")
