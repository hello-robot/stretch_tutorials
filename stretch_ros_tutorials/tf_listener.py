import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# This node prints the transformation between the fk_link_mast frame and a given target frame
# The default target frame is the link_grasp_center frame
# To change the target frame just pass it as an argument to the node, for ex. "target_frame:='fk_link_lift'"
class FrameListener(Node):

    def __init__(self):
        super().__init__('stretch_tf_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'link_grasp_center')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        # Start a Tf buffer that will store the tf information for a few seconds
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        time_period = 1.0 # seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'fk_link_mast'

        # Look up the transformation between target_frame and link_mast frames
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        self.get_logger().info(
                        f'the pose of target frame {from_frame_rel} with reference to {to_frame_rel} is: {trans}')


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()