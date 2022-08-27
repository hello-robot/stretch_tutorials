#!/usr/bin/env python3

# Import modules
import math
import rospy
import sys

# We're going to subscribe to 64-bit integers, so we need to import the definition
# for them
from sensor_msgs.msg import JointState

# Import Int32 message typs from the std_msgs package
from std_msgs.msg import Int32

# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

# Import SpeechRecognitionCandidates from the speech_recognition_msgs package
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class GetVoiceCommands:
    """
    A class that subscribes to the speech to text recognition messages, prints
    a voice command menu, and defines step size for translational and rotational
    mobile base motion.
    """
    def __init__(self):
        """
        A function that initializes subscribers and defines the three different
        step sizes.
        :param self: The self reference.
        """
        # Set step size as medium by default
        self.step_size = 'medium'
        self.rad_per_deg = math.pi/180.0

        # Small step size parameters
        self.small_deg = 5.0
        self.small_rad = self.rad_per_deg * self.small_deg
        self.small_translate = 0.025

        # Medium step size parameters
        self.medium_deg = 10.0
        self.medium_rad = self.rad_per_deg * self.medium_deg
        self.medium_translate = 0.05

        # Big step size parameters
        self.big_deg = 20.0
        self.big_rad = self.rad_per_deg * self.big_deg
        self.big_translate = 0.1

        # Initialize the voice command
        self.voice_command = None

        # Initialize the sound direction
        self.sound_direction = 0

        # Initialize subscribers
        self.speech_to_text_sub  = rospy.Subscriber("/speech_to_text",  SpeechRecognitionCandidates, self.callback_speech)
        self.sound_direction_sub = rospy.Subscriber("/sound_direction", Int32,                       self.callback_direction)

    def callback_direction(self, msg):
        """
        A callback function that converts the sound direction from degrees to radians.
        :param self: The self reference.
        :param msg: The Int32 message type.
        """
        self.sound_direction = msg.data * -self.rad_per_deg

    def callback_speech(self,msg):
        """
        A callback function that takes all items in the iterable list and join
        them into a single string.
        :param self: The self reference.
        :param msg: The SpeechRecognitionCandidates message type.
        """
        self.voice_command = ' '.join(map(str,msg.transcript))

    def get_inc(self):
        """
        A function that sets the increment size for translational and rotational
        base motion.
        :param self:The self reference.

        :returns inc: A dictionary type.
        """
        if self.step_size == 'small':
            inc = {'rad': self.small_rad, 'translate': self.small_translate}
        if self.step_size == 'medium':
            inc = {'rad': self.medium_rad, 'translate': self.medium_translate}
        if self.step_size == 'big':
            inc = {'rad': self.big_rad, 'translate': self.big_translate}
        return inc

    def print_commands(self):
        """
        A function that prints the voice teleoperation menu.
        :param self: The self reference.
        """
        print('                                           ')
        print('------------ VOICE TELEOP MENU ------------')
        print('                                           ')
        print('               VOICE COMMANDS              ')
        print(' "forward": BASE FORWARD                   ')
        print(' "back"   : BASE BACK                      ')
        print(' "left"   : BASE ROTATE LEFT               ')
        print(' "right"  : BASE ROTATE RIGHT              ')
        print(' "stretch": BASE ROTATES TOWARDS SOUND     ')
        print('                                           ')
        print('                 STEP SIZE                 ')
        print(' "big"    : BIG                            ')
        print(' "medium" : MEDIUM                         ')
        print(' "small"  : SMALL                          ')
        print('                                           ')
        print('                                           ')
        print(' "quit"   : QUIT AND CLOSE NODE            ')
        print('                                           ')
        print('-------------------------------------------')

    def get_command(self):
        """
        A function that defines the teleoperation command based on the voice command.
        :param self: The self reference.

        :returns command: A dictionary type.
        """
        command = None
        # Move base forward command
        if self.voice_command == 'forward':
            command = {'joint': 'translate_mobile_base', 'inc': self.get_inc()['translate']}

        # Move base back command
        if self.voice_command == 'back':
            command = {'joint': 'translate_mobile_base', 'inc': -self.get_inc()['translate']}

        # Rotate base left command
        if self.voice_command == 'left':
            command = {'joint': 'rotate_mobile_base', 'inc': self.get_inc()['rad']}

        # Rotate base right command
        if self.voice_command == 'right':
            command = {'joint': 'rotate_mobile_base', 'inc': -self.get_inc()['rad']}

        # Move base to sound direction command
        if self.voice_command == 'stretch':
            command = {'joint': 'rotate_mobile_base', 'inc': self.sound_direction}

        # Set the step size of translational and rotational base motions
        if (self.voice_command == "small") or (self.voice_command == "medium") or (self.voice_command == "big"):
            self.step_size = self.voice_command
            rospy.loginfo('Step size = {0}'.format(self.step_size))

        if self.voice_command == 'quit':
            # Sends a signal to rospy to shutdown the ROS interfaces
            rospy.signal_shutdown("done")

            # Exit the Python interpreter
            sys.exit(0)

        # Reset voice command to None
        self.voice_command = None

        # return the updated command
        return command


class VoiceTeleopNode(hm.HelloNode):
    """
    A class that inherits the HelloNode class from hm and sends joint trajectory
    commands.
    """
    def __init__(self):
        """
        A function that declares object from the GetVoiceCommands class, instantiates
        the HelloNode class, and set the publishing rate.
        """
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_state = None
        self.speech = GetVoiceCommands()


    def joint_states_callback(self, msg):
        """
        A callback function that stores Stretch's joint states.
        :param self: The self reference.
        :param msg: The JointState message type.
        """
        self.joint_state = msg

    def send_command(self, command):
        """
        Function that makes an action call and sends base joint trajectory goals.
        :param self: The self reference.
        :param command: A dictionary type.
        """
        joint_state = self.joint_state
        # Conditional statement to send  joint trajectory goals
        if (joint_state is not None) and (command is not None):
            # Assign point as a JointTrajectoryPoint message type
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)

            # Assign trajectory_goal as a FollowJointTrajectoryGoal message type
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

            # Extract the joint name from the command dictionary
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            # Extract the increment type from the command dictionary
            inc = command['inc']
            rospy.loginfo('inc = {0}'.format(inc))
            new_value = inc

            # Assign the new_value position to the trajectory goal message type
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            rospy.loginfo('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))

            # Make the action call and send goal of the new joint position
            self.trajectory_client.send_goal(trajectory_goal)
            rospy.loginfo('Done sending command.')

            # Reprint the voice command menu
            self.speech.print_commands()

    def main(self):
        """
        The main function that instantiates the HelloNode class, initializes the subscriber,
        and call other methods in both the VoiceTeleopNode and GetVoiceCommands classes.
        :param self: The self reference.
        """
        hm.HelloNode.main(self, 'voice_teleop', 'voice_teleop', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rate = rospy.Rate(self.rate)
        self.speech.print_commands()

        while not rospy.is_shutdown():
            # Get voice command
            command = self.speech.get_command()

            # Send voice command for joint trajectory goals
            self.send_command(command)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Instanstiate a `VoiceTeleopNode()` object and execute the main() method
        node = VoiceTeleopNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
