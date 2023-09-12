##!/usr/bin/env python3

import rclpy
import hello_helpers.hello_misc as hm
import os
import csv
import time
import pandas as pd
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
from matplotlib import animation
from datetime import datetime
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointActuatorEffortSensor(hm.HelloNode):
    """
    A class that sends multiple joint trajectory goals to a single joint.
    """
    def __init__(self, export_data=True, animate=True):
        """
        Function that initializes the subscriber,and other features.
        :param self: The self reference.
        :param export_data: A boolean message type.
        """
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'issue_command', 'issue_command', wait_for_first_pointcloud=False)
        self.joints = ['joint_lift']
        self.joint_effort = []
        self.save_path = '/home/hello-robot/ament_ws/src/stretch_tutorials/stored_data'
        self.export_data = export_data
        self.result = False
        self.file_name = datetime.now().strftime("effort_sensing_tutorial_%Y%m%d%I")
        

    def issue_command(self):
        """
        Function that makes an action call and sends joint trajectory goals
        to a single joint.
        :param self: The self reference.
        """
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = self.joints

        point0 = JointTrajectoryPoint()
        point0.positions = [0.9]

        trajectory_goal.trajectory.points = [point0]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        while self.joint_state is None:
            time.sleep(0.1)
        self._send_goal_future = self.trajectory_client.send_goal_async(trajectory_goal, self.feedback_callback)
        self.get_logger().info('Sent position goal = {0}'.format(trajectory_goal))
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Failed')
            return

        self.get_logger().info('Succeded')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result = future.result().result
        self.get_logger().info('Sent position goal = {0}'.format(self.result))

    def feedback_callback(self, feedback_msg):
        """
        The feedback_callback function deals with the incoming feedback messages
        from the trajectory_client. Although, in this function, we do not use the
        feedback information.
        :param self: The self reference.
        :param feedback: FollowJointTrajectoryActionFeedback message.
        """
        if 'wrist_extension' in self.joints:
            self.joints.remove('wrist_extension')
            self.joints.append('joint_arm_l0')

        current_effort = []
        for joint in self.joints:
            index = self.joint_state.name.index(joint)
            current_effort.append(self.joint_state.effort[index])

        if not self.export_data:
            print("name: " + str(self.joints))
            print("effort: " + str(current_effort))
        else:
            self.joint_effort.append(current_effort)
        
        if self.export_data:
            file_name = self.file_name
            completeName = os.path.join(self.save_path, file_name)
            with open(completeName, "w") as f:
                writer = csv.writer(f)
                writer.writerow(self.joints)
                writer.writerows(self.joint_effort)
    
    def plot_data(self, animate = True):
        while not self.result:
            time.sleep(0.1)
        file_name = self.file_name
        self.completeName = os.path.join(self.save_path, file_name)
        self.data = pd.read_csv(self.completeName)
        self.y_anim = []
        self.animate = animate

        for joint in self.data.columns:

            # Create figure, labels, and title
            fig = plt.figure()
            plt.title(joint + ' Effort Sensing')
            plt.ylabel('Effort')
            plt.xlabel('Data Points')

            # Conditional statement for animation plotting
            if self.animate:
                self.effort = self.data[joint]
                frames = len(self.effort)-1
                anim = animation.FuncAnimation(fig=fig,func=self.plot_animate, repeat=False,blit=False,frames=frames, interval =75)
                plt.show()

                ## If you want to save a video, make sure to comment out plt.show(),
                ## right before this comment.
                # save_path = str(self.completeName + '.mp4')
                # anim.save(save_path, writer=animation.FFMpegWriter(fps=10))

                # Reset y_anim for the next joint effort animation
                del self.y_anim[:]

            # Conditional statement for regular plotting (No animation)
            else:
                self.data[joint].plot(kind='line')
                # save_path = str(self.completeName + '.png')
                # plt.savefig(save_path, bbox_inches='tight')
                plt.show()
    
    def plot_animate(self,i):
        """
        Function that plots every increment of the dataframe.
        :param self: The self reference.
        :param i: index value.
        """
        # Append self.effort values for given joint
        self.y_anim.append(self.effort.values[i])
        plt.plot(self.y_anim, color='blue')

    def main(self):
        """
        Function that initiates the issue_command function.
        :param self: The self reference.
        """
        self.get_logger().info('issuing command')
        self.issue_command()

def main():
    try:
        node = JointActuatorEffortSensor(export_data=True, animate=True)
        node.main()
        node.plot_data()
        node.new_thread.join()
        
        
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
