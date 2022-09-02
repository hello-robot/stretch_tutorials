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
        time_period = 0.1 # seconds
        self.timer = self.create_timer(time_period, self.broadcast_timer_callback)

        self.mast = TransformStamped()
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

        self.lift = TransformStamped()
        self.lift.header.stamp = self.get_clock().now().to_msg()
        self.lift.header.frame_id = 'link_lift'
        self.lift.child_frame_id = 'fk_link_lift'
        self.lift.transform.translation.x = 0.0
        self.lift.transform.translation.y = 2.0
        self.lift.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.lift.transform.rotation.x = q[0]
        self.lift.transform.rotation.y = q[1]
        self.lift.transform.rotation.z = q[2]
        self.lift.transform.rotation.w = q[3]
        self.br.sendTransform(self.lift)

        self.wrist = TransformStamped()
        self.wrist.header.stamp = self.get_clock().now().to_msg()
        self.wrist.header.frame_id = 'link_wrist_yaw'
        self.wrist.child_frame_id = 'fk_link_wrist_yaw'
        self.wrist.transform.translation.x = 0.0
        self.wrist.transform.translation.y = 2.0
        self.wrist.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(1.5707, 0, -1.5707)
        self.wrist.transform.rotation.x = q[0]
        self.wrist.transform.rotation.y = q[1]
        self.wrist.transform.rotation.z = q[2]
        self.wrist.transform.rotation.w = q[3]
        self.br.sendTransform(self.wrist)

        self.get_logger().info("Publishing Tf frames. Use RViz to visualize.")

    def broadcast_timer_callback(self):
        self.mast.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.mast)

        self.lift.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.lift)

        self.wrist.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.wrist)


def main(args=None):
    rclpy.init(args=args)
    tf_broadcaster = FixedFrameBroadcaster()

    rclpy.spin(tf_broadcaster)

    tf_broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()