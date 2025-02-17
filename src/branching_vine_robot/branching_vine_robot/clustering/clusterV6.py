import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Clusters
from cv_bridge import CvBridge

from branching_vine_robot.config import DIST_THRESHOLD

import numpy as np
from hdbscan import HDBSCAN

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
        
        # Convert image from array of depths to flat array of coordinattes
        h, w = depth_image.shape
        x_coords, y_coords = np.meshgrid(np.arange(w), np.arange(h))
        points = np.column_stack((x_coords.ravel(), y_coords.ravel(), depth_image.ravel()))
        
        # Filter values
        points = points[(~np.isnan(points[:, 2]))]
        points = np.clip(points, 0, DIST_THRESHOLD)

        hdbscan = HDBSCAN(min_cluster_size=500, min_samples=10)  # Adjust for density & size
        labels = hdbscan.fit_predict(points)

        # Extract largest clusters only
        unique_labels, counts = np.unique(labels, return_counts=True)

        centroids = np.array([points[labels == label].mean(axis=0) for label in unique_labels])

        clusters_msg = Clusters()
        clusters_msg.x, clusters_msg.y, clusters_msg.z = centroids.T.astype(np.float32).tolist()
        clusters_msg.sizes = counts.tolist()
        
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