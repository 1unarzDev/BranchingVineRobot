#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import numpy as np
import hdbscan
from concurrent.futures import ThreadPoolExecutor
from cv_bridge import CvBridge

from branching_vine_robot.config import *
from interfaces.msg import Clusters

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster_node")
        self.cluster_publisher = self.create_publisher(
            Clusters, "/depth/clusters", 10
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
        
        centroids = []
        sizes = []
        
        for lbl in unique_labels:
            if lbl != -1:  # Ignore noise points
                cluster_points = self.points[clustering.labels_ == lbl]
                centroids.append(cluster_points.mean(axis=0))  # Compute centroid
                sizes.append(len(cluster_points))  # Compute cluster size

        return np.array(centroids, dtype=np.float32), sizes

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

        # Run clustering in parallel
        with ThreadPoolExecutor() as executor:
            future = executor.submit(self.cluster)
            centroids, sizes = future.result()
    
        self.get_logger().info(f"{centroids[:, 0]}")

        # Publish centroids and sizes
        clusters_msg = Clusters()
        clusters_msg.x = centroids[:, 0].tolist()
        clusters_msg.y = centroids[:, 1].tolist()
        clusters_msg.z = centroids[:, 2].tolist()
        clusters_msg.sizes = sizes
        
        self.cluster_publisher.publish(clusters_msg)

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