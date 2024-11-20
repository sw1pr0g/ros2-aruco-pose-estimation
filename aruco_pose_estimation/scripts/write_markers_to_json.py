import rclpy, json, os

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer, TransformException
from aruco_interfaces.msg import ArucoMarkers


class WriteMarkersToJson(Node):
    def __init__(self):
        super().__init__('write_markers_to_json_node')

        self.aruco_markers_subscription = self.create_subscription(
            ArucoMarkers,
            'aruco/markers',
            self.listener_callback_aruco_markers,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.json_file_path = os.path.join(os.getcwd(), 'src/ros2-aruco-pose-estimation/aruco_pose_estimation/data/aruco_markers.json')
        self.markers = {}

    def listener_callback_aruco_markers(self, msg):
        timestamp = rclpy.time.Time().from_msg(msg.header.stamp)
        self.get_logger().info(f'Detected marker at time {timestamp.nanoseconds}')

        for i, pose in enumerate(msg.poses):
            marker_id = msg.marker_ids[i]

            pose_stamped = PoseStamped(
                header=Header(
                    frame_id='camera_color_optical_frame',
                    stamp=rclpy.time.Time()
                ),
                pose=pose
            )

            try:
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, "map", rclpy.duration.Duration(seconds=1.0))

                marker_data = {
                    "marker_id": marker_id,
                    "position": {
                        "x": output_pose_stamped.pose.position.x,
                        "y": output_pose_stamped.pose.position.y,
                        "z": output_pose_stamped.pose.position.z
                    },
                    "orientation": {
                        "x": output_pose_stamped.pose.orientation.x,
                        "y": output_pose_stamped.pose.orientation.y,
                        "z": output_pose_stamped.pose.orientation.z,
                        "w": output_pose_stamped.pose.orientation.w
                    },
                    "timestamp": timestamp.nanoseconds
                }

                self.get_logger().info(f'Marker ID {marker_id}: {marker_data}')
                self.markers[marker_id] = marker_data

            except TransformException as e:
                self.get_logger().info(f"Transform failed for marker {marker_id}: {str(e)}")
                continue

        self.save_markers()
        
    def save_markers(self):
        os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
        with open(self.json_file_path, 'w') as file:
            json.dump(list(self.markers.values()), file, indent=4)
            self.get_logger().info(f'Marker data saved to {self.json_file_path}')

    def destroy_node(self):
        self.get_logger().info("Shutting down node and saving markers.")
        self.save_markers(self)
        super().destroy_node()


import rclpy, json, os

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformListener, Buffer, TransformException
from aruco_interfaces.msg import ArucoMarkers


class WriteMarkersToJson(Node):
    def __init__(self):
        super().__init__('write_markers_to_json_node')

        self.aruco_markers_subscription = self.create_subscription(
            ArucoMarkers,
            'aruco/markers',
            self.listener_callback_aruco_markers,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.json_file_path = os.path.join(os.getcwd(), 'src/ros2-aruco-pose-estimation/aruco_pose_estimation/data/aruco_markers.json')
        self.markers = {}

    def listener_callback_aruco_markers(self, msg):
        timestamp = rclpy.time.Time().from_msg(msg.header.stamp)
        self.get_logger().info(f'Detected marker at time {timestamp.nanoseconds}')

        for i, pose in enumerate(msg.poses):
            marker_id = msg.marker_ids[i]

            pose_stamped = PoseStamped(
                header=Header(
                    frame_id='camera_color_optical_frame',
                    stamp=rclpy.time.Time()
                ),
                pose=pose
            )

            try:
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, "map", rclpy.duration.Duration(seconds=1.0))

                marker_data = {
                    "marker_id": marker_id,
                    "position": {
                        "x": output_pose_stamped.pose.position.x,
                        "y": output_pose_stamped.pose.position.y,
                        "z": output_pose_stamped.pose.position.z
                    },
                    "orientation": {
                        "x": output_pose_stamped.pose.orientation.x,
                        "y": output_pose_stamped.pose.orientation.y,
                        "z": output_pose_stamped.pose.orientation.z,
                        "w": output_pose_stamped.pose.orientation.w
                    },
                    "timestamp": timestamp.nanoseconds
                }

                self.get_logger().info(f'Marker ID {marker_id}: {marker_data}')
                self.markers[marker_id] = marker_data

            except TransformException as e:
                self.get_logger().info(f"Transform failed for marker {marker_id}: {str(e)}")
                continue

        self.save_markers()
        
    def save_markers(self):
        os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
        with open(self.json_file_path, 'w') as file:
            json.dump(list(self.markers.values()), file, indent=4)
            self.get_logger().info(f'Marker data saved to {self.json_file_path}')

    def destroy_node(self):
        self.get_logger().info("Shutting down node and saving markers.")
        self.save_markers()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WriteMarkersToJson()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
