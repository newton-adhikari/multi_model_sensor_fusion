#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import csv

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from AdaptiveEFK import AdaptiveEKF
from message_filters import Subscriber, ApproximateTimeSynchronizer
from datetime import datetime
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class SensorSubscribers(Node):
    def __init__(self):
        super().__init__('sensor_subscribers')

        self.lidar_sub = Subscriber(self, PointCloud2, '/scan')
        self.camera_sub = Subscriber(self, Image, '/camera_sensor/image_raw')
        self.imu_sub = Subscriber(self, Imu, '/imu/data')

        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.camera_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.fusion_callback)

        self.bridge = CvBridge()
        self.ekf = AdaptiveEKF()

        self.T_lidar_base = np.eye(4)
        self.T_lidar_base[:3, 3] = [0.2, 0.0, 0.1]

        self.T_cam_base = np.eye(4)
        self.T_cam_base[:3, 3] = [0.0, 0.1, 0.2]

        self.T_imu_base = np.eye(4)
        self.T_imu_base[:3, 3] = [0.0, 0.0, 0.05]

        # Camera intrinsics with mock values
        # later replce with original
        self.K = np.array([
            [525.0, 0.0, 319.5],
            [0.0, 525.0, 239.5],
            [0.0, 0.0, 1.0]
        ])

        self.csv_file = open('fusion_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'qx', 'qy', 'qz', 'qw'])

        self.fused_pose_pub = self.create_publisher(PoseStamped, '/fused_pose', 10)
        self.imu_pose_pub = self.create_publisher(PoseStamped, '/imu_pose', 10)
        self.path_pub = self.create_publisher(Path, '/fused_path', 10)
        self.fused_path = Path()
        self.fused_path.header.frame_id = 'base_link'



    def fusion_callback(self, cloud: PointCloud2, image: Image, imu: Imu):
        gen = iter(point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
        first_point = next(gen, (0.0, 0.0, 0.0))
        p_lidar = np.array([*first_point, 1.0])
        p_base = self.T_lidar_base @ p_lidar

        R_imu = quaternion_to_rotation_matrix(imu.orientation)
        p_rotated = R_imu @ p_base[:3]

        # Project LiDAR point into camera image
        T_cam_inv = np.linalg.inv(self.T_cam_base)
        p_cam = T_cam_inv @ np.append(p_rotated, 1.0)
        x, y, z = p_cam[:3]

        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        if z > 0:
            uv = self.K @ np.array([x, y, z])
            u, v = int(uv[0] / z), int(uv[1] / z)
            if 0 <= u < image.width and 0 <= v < image.height:
                cv2.circle(cv_image, (u, v), 5, (0, 255, 0), -1)

        acc = [
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z
        ]

        gyro = [
            imu.angular_velocity.x,
            imu.angular_velocity.y,
            imu.angular_velocity.z
        ]

        self.ekf.predict(acc, gyro)
        self.ekf.update_lidar(p_base[:3])
        fused = self.ekf.get_state()

        imu_pose = PoseStamped()
        imu_pose.header.stamp = self.get_clock().now().to_msg()
        imu_pose.header.frame_id = 'base_link'
        imu_pose.pose.orientation = imu.orientation
        self.imu_pose_pub.publish(imu_pose)

        fused_pose = PoseStamped()
        fused_pose.header.stamp = self.get_clock().now().to_msg()
        fused_pose.header.frame_id = 'base_link'
        fused_pose.pose.position.x = fused[0]
        fused_pose.pose.position.y = fused[1]
        fused_pose.pose.position.z = fused[2]
        fused_pose.pose.orientation = imu.orientation  # reuse IMU orientation
        self.fused_pose_pub.publish(fused_pose)

        self.fused_path.header.stamp = self.get_clock().now().to_msg()
        self.fused_path.poses.append(fused_pose)
        self.path_pub.publish(self.fused_path)



        self.get_logger().info(
            f'Fused EKF -> pos=({fused[0]:.2f}, {fused[1]:.2f}, {fused[2]:.2f}) | '
            f'vel=({fused[3]:.2f}, {fused[4]:.2f}, {fused[5]:.2f}) | '
            f'IMU quaternion=({imu.orientation.x:.2f}, {imu.orientation.y:.2f}, {imu.orientation.z:.2f}, {imu.orientation.w:.2f})'
        )

        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)


        timestamp = datetime.now().isoformat()
        self.csv_writer.writerow([
            timestamp,
            fused[0], fused[1], fused[2],  # position
            fused[3], fused[4], fused[5],  # velocity
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w  # orientation
        ])



def quaternion_to_rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    return R


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscribers()
    rclpy.spin(node)
    node.destroy_node()
    node.csv_file.close()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
