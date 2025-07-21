import rospy
from audio_common_msgs.msg import AudioData  
import pyaudio
import wave

class AudioPlayerNode:
    def __init__(self, topic_name, rate=16000, device_name_substr='Jabra', filename="output.wav"):
        rospy.init_node('audio_player_node', anonymous=True)

        self.rate = rate
        self.filename = filename

        self.p = pyaudio.PyAudio()

        # Find audio output device index matching a substring in device name
        self.device_index = None
        for i in range(self.p.get_device_count()):
            dev_info = self.p.get_device_info_by_index(i)
            dev_name = dev_info['name']
            rospy.loginfo(f"Audio device {i}: {dev_name}")
            if device_name_substr in dev_name:
                self.device_index = i
                rospy.loginfo(f"Selected audio output device #{i}: {dev_name}")
                break

        if self.device_index is None:
            rospy.logerr(f"Could not find audio output device matching '{device_name_substr}'")
            exit(1)

        # Open audio output stream
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=self.rate,
                                  output=True,
                                  output_device_index=self.device_index,
                                  frames_per_buffer=1024)

        # Prepare WAV file for writing
        self.wav_file = wave.open(self.filename, 'wb')
        self.wav_file.setnchannels(1)
        self.wav_file.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
        self.wav_file.setframerate(self.rate)

        # Subscribe to the ROS audio topic
        self.sub = rospy.Subscriber(topic_name, AudioData, self.audio_callback, queue_size=10)

        rospy.loginfo(f"Subscribed to {topic_name} and ready to play audio. Saving to {self.filename}")

    def audio_callback(self, msg):
        # Play audio
        self.stream.write(msg.data)

        # Write to WAV file
        self.wav_file.writeframes(msg.data)

    def spin(self):
        rospy.spin()

        # Cleanup
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

        self.wav_file.close()
        rospy.loginfo(f"Saved audio to {self.filename}")

if __name__ == '__main__':
    topic = rospy.get_param('~topic', '/humans/voices/anonymous_speaker/audio')
    player = AudioPlayerNode(topic)
    player.spin()
