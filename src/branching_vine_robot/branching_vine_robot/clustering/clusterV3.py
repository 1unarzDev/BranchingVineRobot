#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        
        self.bridge = CvBridge()
    
    def cluster(self):
        """ Perform clustering on the depth image indices and depth values. """
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
        """ Process depth image and cluster using pixel indices and depth values. """
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Generate pixel grid
        v, u = np.indices(depth_image.shape, dtype=np.float32)

        # Convert depth to meters
        z = (depth_image / 1000.0).astype(np.float32)  # Convert mm to meters

        # Filter out invalid points (z > 0 and within threshold)
        valid_indices = np.nonzero((z > 0) & (z < DIST_THRESHOLD))
        u_valid = u[valid_indices]
        v_valid = v[valid_indices]
        z_valid = z[valid_indices]

        # Stack into Nx3 format: (u, v, z)
        self.points = np.column_stack((u_valid, v_valid, z_valid))

        # Run clustering in parallel
        with ThreadPoolExecutor() as executor:
            future = executor.submit(self.cluster)
            centroids, sizes = future.result()
    
        self.get_logger().info(f"Cluster centroids: {centroids}")

        # Publish centroids and sizes
        clusters_msg = Clusters()
        clusters_msg.x = centroids[:, 0].tolist()  # u (pixel x-coordinate)
        clusters_msg.y = centroids[:, 1].tolist()  # v (pixel y-coordinate)
        clusters_msg.z = centroids[:, 2].tolist()  # z (depth in meters)
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