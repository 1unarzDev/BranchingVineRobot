#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge
import cv2 as cv
import struct

from branching_vine_robot.utils.helper import get_angle_2d
from branching_vine_robot.config import *
# from interfaces import Cluster

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster_node")

        # Publisher to publish cluster centroids
        self.cluster_publisher = self.create_publisher(
            PointCloud2, "/cluster/clusters", 10
        )
    
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )

        self.info_subscriber = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.camera_info_callback, 10
        )
        
        self.bridge = CvBridge()

        self.fx = self.fy = self.cx = self.cy = None  # Camera intrinsics
    
    def camera_info_callback(self, msg):
        """ Get camera intrinsics from CameraInfo topic. """
        self.fx = msg.k[0]  # Focal length in x
        self.fy = msg.k[4]  # Focal length in y
        self.cx = msg.k[2]  # Optical center x
        self.cy = msg.k[5]  # Optical center y

    def depth_callback(self, msg):
        """ Convert depth image to a 3D point cloud. """
        if self.fx is None:
            self.get_logger().warn("Camera info not received yet")
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        height, width = depth_image.shape

        # Generate pixel grid
        u, v = np.meshgrid(np.arange(width), np.arange(height))

        # Convert depth to 3D coordinates
        z = depth_image / 1000.0  # Convert mm to meters
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # Stack into Nx3 format (filter out invalid points)
        mask = (z > 0) & (z < DIST_THRESHOLD)  # Remove points too close/far
        points = np.column_stack((x[mask], y[mask], z[mask]))

        # Convert to PointCloud2 message
        point_cloud = self.arr_to_point_cloud(points)
        self.cluster_publisher.publish(point_cloud)
        self.get_logger().info(f"Published {len(points)} points")

        # # Apply DBSCAN clustering
        # clustering = DBSCAN(eps=5, min_samples=10).fit(self.points)

        # # Find cluster centroids
        # unique_labels = set(clustering.labels_)
        # self.centroids = []
        # for label in unique_labels:
        #     if label == -1:
        #         continue
        #     cluster_points = self.points[clustering.labels_ == label]
        #     centroid = np.mean(cluster_points, axis=0)
        #     self.centroids.append(centroid)
        #     
        # for centroid in self.centroids:
        #     point_msg = Point()
        #     point_msg.x = centroid[0]
        #     point_msg.y = centroid[1]
        #     point_msg.z = centroid[2]
        #     self.cluster_publisher.publish(point_msg)

    def arr_to_point_cloud(self, points):
        """ Convert numpy array to PointCloud2 message. """
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link" 

        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.point_step = 12  # 4 bytes per float32 x, y, z
        msg.row_step = msg.point_step * len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary format
        msg.data = b''.join([struct.pack('fff', *p) for p in points])

        return msg

def main(args=None):
    rclpy.init(args=args)
    cluster_node = ClusterNode()

    try:
        rclpy.spin(cluster_node)
    except KeyboardInterrupt:
        cluster_node.get_logger().info("Shutting down")
    finally:
        cluster_node.destroy_node()

if __name__ == "__main__":
    main()