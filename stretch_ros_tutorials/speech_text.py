#!/usr/bin/env python3

# Import modules
import rclpy
import os
from rclpy.node import Node

# Import SpeechRecognitionCandidates from the speech_recognition_msgs package
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class SpeechText(Node):
    """
    A class that saves the interpreted speech from the ReSpeaker Microphone Array to a text file.
    """
    def __init__(self):
        """
        Initialize subscriber and directory to save speech to text file.
        :param self: The self reference.
        """
        super().__init__('stretch_speech_text')
        # Initialize subscriber
        self.sub = self.create_subscription(SpeechRecognitionCandidates, "speech_to_text", self.callback, 1)

        # Create path to save captured images to the stored data folder
        self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'

        # Create log message
        self.get_logger().info("Listening to speech")

    def callback(self,msg):
        """
        A callback function that receives the speech transcript and appends the
        transcript to a text file.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        # Take all items in the iterable list and join them into a single string
        transcript = ' '.join(map(str,msg.transcript))

        # Define the file name and create a complete path name
        file_name = 'speech.txt'
        completeName = os.path.join(self.save_path, file_name)

        # Append 'hello' at the end of file
        with open(completeName, "a+") as file_object:
            file_object.write("\n")
            file_object.write(transcript)

def main(args=None):
    # Initialize the node and name it speech_text
    rclpy.init(args=args)

    # Instantiate the SpeechText class
    speech_txt = SpeechText()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rclpy.spin(speech_txt)

if __name__ == '__main__':
    main()