#!/usr/bin/env python3

import rospy
from hri_msgs.msg import LiveSpeech
import os

class SpeechLogger:
    def __init__(self, topic_name, log_file_path):
        rospy.init_node('anonymous_speaker_speech_logger', anonymous=True)

        self.log_file = log_file_path
        
        # Ensure the directory exists
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        rospy.loginfo(f"Opening log file: {self.log_file}")
        self.file = open(self.log_file, 'a')  # append mode

        self.sub = rospy.Subscriber(topic_name, LiveSpeech, self.callback, queue_size=10)
        rospy.loginfo(f"Subscribed to topic: {topic_name}")

    def callback(self, msg):
        rospy.loginfo(f"Received incremental: '{msg.incremental}' final: '{msg.final}' confidence: {msg.confidence}")

        if msg.incremental.strip():
            self.file.write(msg.incremental + '\n')
            self.file.flush()  # ensure data is written immediately
            rospy.loginfo(f"Logged incremental speech: {msg.incremental}")

    def spin(self):
        rospy.spin()
        self.file.close()

if __name__ == '__main__':
    # Parameters
    topic = rospy.get_param('~topic', '/humans/voices/anonymous_speaker/speech')
    log_path = rospy.get_param('~log_file', '/workspaces/LincARI/lincari_logging/audio_recordings/speech_log.txt')

    logger = SpeechLogger(topic, log_path)
    logger.spin()
