import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Clusters
from cv_bridge import CvBridge

from branching_vine_robot.config import MIN_DEPTH, MAX_DEPTH

import numpy as np
from sklearn.cluster import KMeans

class Cluster(Node):
    def __init__(self):
        super().__init__('cluster_node')
        
        self.cluster_publisher = self.create_publisher(
            Clusters, "/depth/clusters", 10
        )
    
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        
        self.bridge = CvBridge()
        
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
        # Downsample for speed
        stride = 2
        h, w = depth_image.shape
        x_coords, y_coords = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
        points = np.column_stack((x_coords.ravel(), y_coords.ravel(), depth_image[::stride, ::stride].ravel()))
    
        # Filter only valid depth values
        valid_mask = (MIN_DEPTH <= points[:, 2]) & (points[:, 2] <= MAX_DEPTH)
        filtered_points = points[valid_mask]
    
        # Ensure we have valid points
        if len(filtered_points) == 0:
            self.get_logger().warn("No valid points for clustering.")
            return
    
        # Use only (x, y) for clustering
        xy_points = filtered_points[:, :2]
    
        n_clusters = 10
    
        kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
        kmeans.fit(xy_points)
    
        labels = kmeans.labels_
    
        # Compute mean depth per cluster (vectorized)
        cluster_counts = np.bincount(labels, minlength=n_clusters)
        mean_depths = np.bincount(labels, weights=filtered_points[:, 2], minlength=n_clusters) / np.maximum(cluster_counts, 1)
    
        # Replace NaN values in depth with MIN_DEPTH
        mean_depths = np.nan_to_num(mean_depths, nan=MIN_DEPTH)
    
        # Combine cluster centers with mean depths
        centroids = np.column_stack((kmeans.cluster_centers_, mean_depths))
    
        # Prepare and publish message
        clusters_msg = Clusters()
        clusters_msg.x, clusters_msg.y, clusters_msg.z = centroids.T.astype(np.float32).tolist()
        clusters_msg.sizes = cluster_counts.tolist()
    
        self.cluster_publisher.publish(clusters_msg)
        self.get_logger().info(f"Published points - x: {clusters_msg.x}, y: {clusters_msg.y}, z: {clusters_msg.z}, sizes: {clusters_msg.sizes}")

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