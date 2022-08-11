#!/usr/bin/env python3

# Import modules
import rospy
import os

# Import SpeechRecognitionCandidates from the speech_recognition_msgs package
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechText:
    """
    A class that saves the interpreted speech from the ReSpeaker Microphone Array to a text file.
    """
    def __init__(self):
        """
        Initialize subscriber and directory to save speech to text file.
        """
        # Initialize subscriber
        self.sub = rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates, self.callback)

        # Create path to save captured images to the stored data folder
        self.save_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'

        # Create log message
        rospy.loginfo("Listening to speech.")

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


if __name__ == '__main__':
    # Initialize the node and name it speech_text
    rospy.init_node('speech_text')

    # Instantiate the SpeechText class
    SpeechText()

    # Give control to ROS.  This will allow the callback to be called whenever new
    # messages come in.  If we don't put this line in, then the node will not work,
    # and ROS will not process any messages
    rospy.spin()
