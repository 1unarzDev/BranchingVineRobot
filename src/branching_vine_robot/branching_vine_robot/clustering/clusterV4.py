import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from interfaces.msg import Clusters
from cv_bridge import CvBridge

from branching_vine_robot.config import DIST_THRESHOLD

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
        
        # Convert image from array of depths to flat array of coordinattes
        h, w = depth_image.shape
        x_coords, y_coords = np.meshgrid(np.arange(w), np.arange(h))
        points = np.column_stack((x_coords.ravel(), y_coords.ravel(), depth_image.ravel()))
        
        # Filter values
        valid_points = (~np.isnan(points[:, 2])) & (points[:, 2] > 0) & (points[:, 2] < DIST_THRESHOLD)
        
        kmeans = KMeans(n_clusters=3, random_state=42, n_init=10)
        kmeans.fit(valid_points)
        
        clusters_msg = Clusters()
        clusters_msg.x, clusters_msg.y, clusters_msg.z = kmeans.cluster_centers_.T
        clusters_msg.sizes = np.bincount(kmeans.labels_)
        
        self.cluster_publisher.publish(clusters_msg)

def main(args=None):
    rclpy.init(args=args)
    cluster_node = Cluster()
    
    try:
        rclpy.spin(cluster_node)
    except KeyboardInterrupt:
        cluster_node.get_logger().info("State machine node shutting down")
    finally:
        cluster_node.destroy_node()
    
if __name__ == "__main__":
    main()