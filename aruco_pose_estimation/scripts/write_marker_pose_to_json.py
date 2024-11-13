import rclpy
import json
import os

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix, translation_from_matrix, \
    concatenate_matrices
from aruco_interfaces.msg import ArucoMarkers


class MarkerPoseCalculator(Node):
    def __init__(self):
        super().__init__('marker_pose_calculator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(ArucoMarkers, '/aruco/markers', self.marker_pose_callback, 10)
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('global_frame', 'map')
        self.marker_data = self.load_existing_data(os.path.join(os.getcwd(), 'src/ros2-aruco-pose-estimation/aruco_pose_estimation/data',
                                       'aruco_markers.json'))

    def load_existing_data(self, filename):
        if os.path.exists(filename):
            with open(filename, 'r') as file:
                return json.load(file)
        return []

    def marker_pose_callback(self, msg):
        print(msg)
        camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        try:
            transform = self.tf_buffer.lookup_transform(global_frame, camera_frame, rclpy.time.Time())
            for i, relative_pose in enumerate(msg.poses):
                marker_pose = self.calculate_absolute_marker_pose(transform, relative_pose, msg.marker_ids[i])
                self.update_marker_data(marker_pose)
            self.save_pose_to_file(os.path.join(os.getcwd(), 'src/ros2-aruco-pose-estimation/aruco_pose_estimation/data',
                                       'aruco_markers.json'), self.marker_data)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def calculate_absolute_marker_pose(self, transform, relative_pose, marker_id):
        camera_matrix = quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        camera_matrix[0:3, 3] = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
        marker_matrix = quaternion_matrix([
            relative_pose.orientation.x,
            relative_pose.orientation.y,
            relative_pose.orientation.z,
            relative_pose.orientation.w
        ])
        marker_matrix[0:3, 3] = [
            relative_pose.position.x,
            relative_pose.position.y,
            relative_pose.position.z
        ]
        absolute_marker_matrix = concatenate_matrices(camera_matrix, marker_matrix)
        absolute_marker_position = translation_from_matrix(absolute_marker_matrix)
        absolute_marker_orientation = quaternion_from_matrix(absolute_marker_matrix)
        return {
            'marker_id': marker_id,
            'position': {
                'x': absolute_marker_position[0],
                'y': absolute_marker_position[1],
                'z': absolute_marker_position[2]
            },
            'orientation': {
                'x': absolute_marker_orientation[0],
                'y': absolute_marker_orientation[1],
                'z': absolute_marker_orientation[2],
                'w': absolute_marker_orientation[3]
            },
            'timestamp': self.get_clock().now().to_msg().sec
        }

    def update_marker_data(self, new_marker_pose):
        for marker in self.marker_data:
            if marker['marker_id'] == new_marker_pose['marker_id']:
                marker.update(new_marker_pose)
                return
        self.marker_data.append(new_marker_pose)

    def save_pose_to_file(self, filename, marker_data):
        with open(filename, 'w') as file:
            json.dump(marker_data, file, indent=4)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPoseCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
