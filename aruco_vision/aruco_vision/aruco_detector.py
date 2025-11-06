#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Bridge to convert ROS <-> OpenCV
        self.br = CvBridge()

        # Subscribe to the raw camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detected marker pose
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_marker_pose', 10)

        # Load ArUco dictionary (DICT_4X4_50 is a common choice)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Camera intrinsics (temporary placeholder)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # assume no distortion for now

        self.marker_length = 0.05  # marker side length in meters (5 cm)

        self.get_logger().info(" ArUco detector node initialized")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids):
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # Draw coordinate axes
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)

                # Publish marker pose
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'camera_link'
                pose.pose.position.x = float(tvec[0])
                pose.pose.position.y = float(tvec[1])
                pose.pose.position.z = float(tvec[2])
                self.pose_pub.publish(pose)

                self.get_logger().info(f"Marker {marker_id[0]}: "
                                       f"x={tvec[0]:.2f}, y={tvec[1]:.2f}, z={tvec[2]:.2f}")

        # Show image in an OpenCV window
        cv2.imshow("ArUco Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
