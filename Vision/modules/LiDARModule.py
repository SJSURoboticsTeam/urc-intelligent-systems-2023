import matplotlib.pyplot as plt
import numpy as np
from rplidar import RPLidar, RPLidarException
from sklearn.cluster import KMeans

class LiDARModule:
    def __init__(self, port_name):
        self.lidar = RPLidar(port_name)

    def find_closest_cluster(self, X, cluster_centers):
        closest_cluster = None
        min_distance = float('inf')
        for i, center in enumerate(cluster_centers):
            distance = np.linalg.norm(X - center)
            if distance < min_distance:
                min_distance = distance
                closest_cluster = i
        return closest_cluster

    def get_clusters(self, X):
        # Compute DBSCAN
        self.dbscan.fit(X)

        labels = self.dbscan.labels_
        
        # Find the closest points in each cluster
        cluster_distances = {'Left': float('inf'), 'Center': float('inf'), 'Right': float('inf')}
        for i in range(len(X)):
            label = labels[i]
            if label != -1:  # Ignore noise points
                cluster_label = self.cluster_names[label]
                point = X[i]
                distance = np.linalg.norm(point)  # Euclidean distance to the origin
                if distance < cluster_distances[cluster_label]:
                    cluster_distances[cluster_label] = distance
        
        # Find the cluster with the minimum distance
        closest_cluster = min(cluster_distances, key=cluster_distances.get)
        closest_distance = cluster_distances[closest_cluster]

        return closest_cluster, closest_distance

