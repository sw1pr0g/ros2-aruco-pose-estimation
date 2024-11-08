import rclpy
from rclpy.node import Node
import json
import os
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
from aruco_interfaces.msg import ArucoMarkers


class ArucoToJson(Node):
    def __init__(self):
        super().__init__('aruco_to_json')
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco/markers',
            self.listener_callback,
            10
        )

        self.json_file_path = os.path.join(os.getcwd(), 'src/ros2-aruco-pose-estimation/aruco_pose_estimation/data'
                                                        '/aruco_markers.json')
        self.markers_data = {}

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        frame_id = msg.header.frame_id

        self.get_logger().info(f'Received message with frame_id: {frame_id} at time {timestamp}')

        for i, pose in enumerate(msg.poses):
            marker_id = msg.marker_ids[i]

            marker_data = {
                "marker_id": marker_id,
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z
                },
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w
                }
            }

            self.get_logger().info(f'Marker ID {marker_id}: {marker_data}')

            self.markers_data[marker_id] = marker_data

        os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
        with open(self.json_file_path, 'w') as f:
            json.dump(list(self.markers_data.values()), f, indent=4)
            self.get_logger().info(f'Marker data saved to {self.json_file_path}')


def main(args=None):
    rclpy.init(args=args)
    aruco_to_json = ArucoToJson()
    rclpy.spin(aruco_to_json)
    aruco_to_json.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()