#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point
import numpy as np
import hdbscan
from concurrent.futures import ThreadPoolExecutor
from cv_bridge import CvBridge

from branching_vine_robot.utils.helper import get_angle_2d
from branching_vine_robot.config import *
from interfaces.msg import Cluster

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster_node")

        self.point_publisher = self.create_publisher(
            PointCloud2, "/depth/points", 10
        )

        self.cluster_publisher = self.create_publisher(
            Point, "/depth/clusters", 10
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

    def cluster(self):
        clustering = hdbscan.HDBSCAN(min_cluster_size=1000).fit(self.points)
        unique_labels = np.unique(clustering.labels_)
        centroids = np.array([self.points[clustering.labels_ == lbl].mean(axis=0) for lbl in unique_labels if lbl != -1])
    
        return centroids

    def depth_callback(self, msg):
        """ Convert depth image to a 3D point cloud. """
        if self.fx is None:
            self.get_logger().warn("Camera info not received yet")
            return

        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Generate pixel grid
        v, u = np.indices(depth_image.shape, dtype=np.float32)

        # Convert depth to 3D coordinates
        z = depth_image / 1000.0  # Convert mm to meters
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        # Stack into Nx3 format (filter out invalid points)
        valid_indices = np.nonzero((z > 0) & (z < DIST_THRESHOLD))
        self.points = np.column_stack((x[valid_indices], y[valid_indices], z[valid_indices]))

        # Convert to PointCloud2 message
        point_cloud = self.arr_to_point_cloud(self.points)
        self.point_publisher.publish(point_cloud)
        
        self.get_logger().info(f"Published {len(self.points)} points")

        # Run clustering in parallel
        with ThreadPoolExecutor() as executor:
            future = executor.submit(self.cluster)
            centroids = future.result()
    
        # Publish centroids
        for centroid in centroids:
            point_msg = Point()
            point_msg.x, point_msg.y, point_msg.z = centroid
            self.cluster_publisher.publish(point_msg)
            
            # cluster_msg = Cluster()
            # cluster_msg.center = point_msg
            # cluster_msg.stamp = self.get_clock().now().nanoseconds / (10 ** 9)
            # cluster_msg.size = len(cluster_points)

            self.cluster_publisher.publish(point_msg)

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
        msg.data = np.array(points, dtype=np.float32).tobytes()

        return msg

def main(args=None):
    rclpy.init(args=args)
    cluster_node = ClusterNode()

    try:
        rclpy.spin(cluster_node)
    except KeyboardInterrupt:
        cluster_node.get_logger().info("Cluster node shutting down")
    finally:
        cluster_node.destroy_node()

if __name__ == "__main__":
    main()