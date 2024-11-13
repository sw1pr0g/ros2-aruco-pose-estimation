import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

class RobotPosition(Node):
    def __init__(self):
        super().__init__('robot_position')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(10.0, self.get_robot_position)
    def get_robot_position(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            self.publish_pose(transform)
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')
    def publish_pose(self, transform: TransformStamped):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = transform.transform.translation.x
        pose_msg.pose.pose.position.y = transform.transform.translation.y
        pose_msg.pose.pose.position.z = transform.transform.translation.z
        pose_msg.pose.pose.orientation.x = transform.transform.rotation.x
        pose_msg.pose.pose.orientation.y = transform.transform.rotation.y
        pose_msg.pose.pose.orientation.z = transform.transform.rotation.z
        pose_msg.pose.pose.orientation.w = transform.transform.rotation.w
        pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing pose: '
                               f'x: {pose_msg.pose.pose.position.x}, '
                               f'y: {pose_msg.pose.pose.position.y}, '
                               f'z: {pose_msg.pose.pose.position.z}, '
                               f'orientation: [{pose_msg.pose.pose.orientation.x}, '
                               f'{pose_msg.pose.pose.orientation.y}, '
                               f'{pose_msg.pose.pose.orientation.z}, '
                               f'{pose_msg.pose.pose.orientation.w}]')
def main(args=None):
    rclpy.init(args=args)
    node = RobotPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
