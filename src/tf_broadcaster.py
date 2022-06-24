from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations

# This node publishes three child static frames in reference to their parent frames as below:
# parent -> link_mast            child -> fk_link_mast
# parent -> link_lift            child -> fk_link_lift
# parent -> link_wrist_yaw       child -> fk_link_wrist_yaw
# Tf frames are not defined according to the DH convention by default 
# The new frames have been declared to make the computation of 
# forward and inverse kinematics easier by defining the new frames
# according to the DH convention
class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('stretch_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.mast = TransformStamped()
        self.mast.header.stamp = self.get_clock().now().to_msg()
        self.mast.header.frame_id = 'link_mast'
        self.mast.child_frame_id = 'fk_link_mast'
        self.mast.transform.translation.x = 0.0
        self.mast.transform.translation.y = 0.0
        self.mast.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.mast.transform.rotation.x = q[0]
        self.mast.transform.rotation.y = q[1]
        self.mast.transform.rotation.z = q[2]
        self.mast.transform.rotation.w = q[3]

    def broadcast_timer_callback(self):
        self.br.sendTransform(self.mast)

        lift = TransformStamped()
        lift.header.stamp = self.get_clock().now().to_msg()
        lift.header.frame_id = 'link_lift'
        lift.child_frame_id = 'fk_link_lift'
        lift.transform.translation.x = 0.0
        lift.transform.translation.y = 2.0
        lift.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        lift.transform.rotation.x = q[0]
        lift.transform.rotation.y = q[1]
        lift.transform.rotation.z = q[2]
        lift.transform.rotation.w = q[3]
        self.br.sendTransform(lift)

        wrist = TransformStamped()
        wrist.header.stamp = self.get_clock().now().to_msg()
        wrist.header.frame_id = 'link_wrist_yaw'
        wrist.child_frame_id = 'fk_link_wrist_yaw'
        wrist.transform.translation.x = 0.0
        wrist.transform.translation.y = 2.0
        wrist.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        wrist.transform.rotation.x = q[0]
        wrist.transform.rotation.y = q[1]
        wrist.transform.rotation.z = q[2]
        wrist.transform.rotation.w = q[3]
        self.br.sendTransform(wrist)

def main():
   rclpy.init()
   node = FixedFrameBroadcaster()
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()