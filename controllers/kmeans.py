import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt


class Kmeans:
    def __init__(self):
        pass

    def Kmens(self ,x,y,k):

        points = np.column_stack((x, y))


        n_clusters = k


        kmeans = KMeans(n_clusters=n_clusters)
        kmeans.fit(points)
        labels = kmeans.labels_


        centers = kmeans.cluster_centers_


        plt.figure(figsize=(8, 6))
        colors = ['r', 'g', 'b', 'y', 'c', 'm']
        clusters=[]
        for i in range(n_clusters):
            data = points[labels == i]
            clusters.append(data)
            plt.scatter(data[:, 0], data[:, 1], s=50, c=colors[i], label=f'Cluster {i + 1}')
            plt.scatter(centers[i, 0], centers[i, 1], s=200, c='black', marker='X')

        plt.title('Clusterização K-means')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.show()

        return clusters