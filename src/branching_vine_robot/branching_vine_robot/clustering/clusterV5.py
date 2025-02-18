import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Clusters
from cv_bridge import CvBridge

from branching_vine_robot.config import MIN_DEPTH, MAX_DEPTH

import numpy as np
from sklearn.cluster import DBSCAN

class Cluster(Node):
    def __init__(self):
        super().__init__('cluster_node')
        
        self.cluster_publisher = self.create_publisher(
            Clusters, "depth/clusters", 10
        )
    
        self.depth_subscriber = self.create_subscription(
            Image, 'camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        
        self.bridge = CvBridge()
        
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Convert image from array of depths to flat array of coordinates
        h, w = depth_image.shape
        x_coords, y_coords = np.meshgrid(np.arange(w), np.arange(h))
        points = np.column_stack((x_coords.ravel(), y_coords.ravel(), depth_image.ravel()))
        
        # Filter invalid depth values
        valid_mask = (~np.isnan(points[:, 2])) & (points[:, 2] > MIN_DEPTH)
        points = points[valid_mask]
    
        # Use only (x, y) for DBSCAN clustering
        xy_points = points[:, :2]  
    
        # Run DBSCAN
        dbscan = DBSCAN(eps=30, min_samples=100)
        labels = dbscan.fit_predict(xy_points)
        
        # Extract unique cluster labels (ignore noise, labeled as -1)
        unique_labels = set(labels) - {-1}
    
        # Compute centroids with (x, y) means and average depth (z)
        centroids = np.array([
            np.append(points[labels == label, :2].mean(axis=0),  # Mean (x, y)
                      points[labels == label, 2].mean())  # Mean (z)
            for label in unique_labels
        ])
    
        # Publish results
        clusters_msg = Clusters()
        
        if centroids.size > 0:  # Ensure centroids array is not empty
            clusters_msg.x, clusters_msg.y, clusters_msg.z = centroids.T.astype(np.float32).tolist()
            clusters_msg.sizes = np.array([np.sum(labels == label) for label in unique_labels], dtype=np.uint32).tolist()
            self.cluster_publisher.publish(clusters_msg)
            self.get_logger().info(f"Published points - x: {clusters_msg.x}, y: {clusters_msg.y}, z: {clusters_msg.z}, sizes: {clusters_msg.sizes}")
        else:
            self.get_logger().info("No clusters found")

def main(args=None):
    rclpy.init(args=args)
    cluster_node = Cluster()
    
    try:
        rclpy.spin(cluster_node)
    except KeyboardInterrupt:
        cluster_node.get_logger().info("Cluster node shutting down")
    finally:
        cluster_node.destroy_node()
    
if __name__ == "__main__":
    main()