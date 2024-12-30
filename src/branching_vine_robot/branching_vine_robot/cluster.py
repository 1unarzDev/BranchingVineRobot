import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import numpy as np
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge
import cv2 as cv

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster_node")

        # Publisher to publish cluster centroids
        self.cluster_publisher = self.create_publisher(
            Point, "cluster", 10
        )
    
        # Subscription to depth image
        self.subscription = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10                
        )
        
        self.bridge = CvBridge()
    
    def depth_callback(self, msg):
        # Convert ROS image message to np arr
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        # Flatten depth image to process with DBSCAN
        y_indices, x_indices = np.indices(depth_image.shape)
        points = np.column_stack((x_indices.ravel(), y_indices.ravel(), depth_image.ravel()))
        points = points[~np.isnan(points[:, 2]) & (points[:, 2] > 0)]
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=5, min_samples=10).fit(points)

        # Find cluster centroids
        unique_labels = set(clustering.labels_)
        centroids = []
        for label in unique_labels:
            if label == -1:
                continue
            cluster_points = points[clustering.labels_ == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)
            
        for centroid in centroids:
            point_msg = Point()
            point_msg.x = centroid[0]
            point_msg.y = centroid[1]
            point_msg.z = centroid[2]
            self.cluster_publisher.publish(point_msg)

def main():
    rclpy.init()
    node = ClusterNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()