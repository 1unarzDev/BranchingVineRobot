import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from interfaces.msg import DepthClustered
from sensor_msgs.msg import Image

import numpy as np
from scipy.spatial import distance
from collections import deque
from branching_vine_robot.config import DIST_THRESHOLD

class ClusterNode(Node):
    def __init__(self):
        super().__init__("cluster_node")

        self.cluster_publisher = self.create_publisher(
            DepthClustered, "/depth/clusters", 10
        )
    
        self.depth_subscriber = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        
        self.bridge = CvBridge()
        self.clusters = {}

    def depth_callback(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") / 1000.0
        
        self.depth_map[(self.depth_map <= 0) | (self.depth_map > DIST_THRESHOLD)] = 0

        self.labels, self.clusters = grid_dbscan_tracker(self.depth_map, self.clusters)

        cluster_msg = DepthClustered()
        cluster_msg.height, cluster_msg.width = self.depth_map.shape
        cluster_msg.depths = self.depth_map.flatten().tolist()
        cluster_msg.labels = self.labels.flatten().tolist()
        cluster_msg.centroid_x = [int(x) for x, y in self.clusters.values()]
        cluster_msg.centroid_y = [int(y) for x, y in self.clusters.values()]

        self.cluster_publisher.publish(cluster_msg)

def grid_dbscan_tracker(depth_map, prev_clusters, eps=0.1, min_samples=50, max_movement=5):
    """
    Perform Grid-DBSCAN clustering on a depth map and track clusters across frames.
    
    Args:
        depth_map (np.array): 2D array representing the depth values.
        prev_clusters (dict): {cluster_id: (centroid_x, centroid_y)} from the previous frame.
        eps (float): Threshold for depth similarity.
        min_samples (int): Minimum number of points to form a cluster.
        max_movement (float): Max allowed centroid movement to keep the same cluster ID.

    Returns:
        labels (np.array): 2D array with cluster IDs assigned.
        updated_clusters (dict): Updated {cluster_id: (centroid_x, centroid_y)} for tracking.
    """
    height, width = depth_map.shape
    labels = -np.ones_like(depth_map, dtype=int)  # -1 = unclassified
    cluster_id = 0

    def get_neighbors(i, j):
        """Get 4-way grid neighbors"""
        neighbors = []
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ni, nj = i + di, j + dj
            if 0 <= ni < height and 0 <= nj < width:
                neighbors.append((ni, nj))
        return neighbors

    def expand_cluster(i, j, cluster_id):
        """Flood-fill to expand a cluster"""
        queue = [(i, j)]
        cluster_points = [(i, j)]
        labels[i, j] = cluster_id  # Mark as part of cluster

        while queue:
            ci, cj = queue.pop()
            for ni, nj in get_neighbors(ci, cj):
                if labels[ni, nj] == -1 and abs(depth_map[ni, nj] - depth_map[ci, cj]) < eps:
                    labels[ni, nj] = cluster_id
                    cluster_points.append((ni, nj))
                    queue.append((ni, nj))

        return cluster_points

    # Run DBSCAN-like clustering
    for i in range(height):
        for j in range(width):
            if labels[i, j] == -1:  # Unclassified point
                cluster_points = expand_cluster(i, j, cluster_id)
                if len(cluster_points) < min_samples:
                    for pi, pj in cluster_points:
                        labels[pi, pj] = -2  # Mark as noise
                else:
                    cluster_id += 1  # Increment for valid clusters

    # Compute centroids of detected clusters
    def compute_centroids():
        """Compute centroids of clusters"""
        cluster_sums = {}
        cluster_counts = {}

        for i in range(height):
            for j in range(width):
                label = labels[i, j]
                if label >= 0:  # Valid cluster
                    if label not in cluster_sums:
                        cluster_sums[label] = np.array([0, 0])
                        cluster_counts[label] = 0
                    cluster_sums[label] += np.array([i, j])
                    cluster_counts[label] += 1

        return {label: (sums / cluster_counts[label]) for label, sums in cluster_sums.items()}

    new_centroids = compute_centroids()

    # Match new centroids to previous centroids
    updated_clusters = {}
    used_old_ids = set()
    used_new_ids = set()

    for new_id, new_centroid in new_centroids.items():
        best_match = None
        best_distance = float("inf")

        for old_id, old_centroid in prev_clusters.items():
            if old_id in used_old_ids:
                continue  # Skip if already matched

            dist = distance.euclidean(new_centroid, old_centroid)
            if dist < max_movement and dist < best_distance:
                best_match = old_id
                best_distance = dist

        if best_match is not None:
            updated_clusters[best_match] = new_centroid
            used_old_ids.add(best_match)
            used_new_ids.add(new_id)
        else:
            updated_clusters[new_id] = new_centroid  # Assign new ID

    # Create a mapping from old cluster IDs to new ones
    id_map = {}
    
    # Assign old IDs to new clusters where possible
    for new_id, centroid in updated_clusters.items():
        if new_id in prev_clusters:
            id_map[new_id] = new_id  # Keep the same ID
        else:
            id_map[new_id] = max(prev_clusters.keys(), default=0) + 1  # Assign new unique ID
    
    # Update labels based on the corrected id_map
    for i in range(height):
        for j in range(width):
            label = labels[i, j]
            if label in id_map:
                labels[i, j] = id_map[label]

    return labels, updated_clusters

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