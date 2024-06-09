import math
import numpy as np
from matplotlib import pyplot as plt

from skimage.measure import LineModelND, ransac


def ransac_fit_line(readings):
    data = readings
    print(len(data))
    model1, inliers_bools1 = ransac(data, LineModelND, min_samples=2, residual_threshold=1, max_trials=10000)

    outliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if not inlier_bool])

    inliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if inlier_bool])

    x, y = inliers1[:, 0], inliers1[:, 1]
    print(f"Length inliers: {len(inliers1)}")
    plt.figure()

    # Plote os pontos no gráfico 2D
    plt.scatter(x, y, s=1)

    # Configure os rótulos dos eixos
    plt.xlabel('X')
    plt.ylabel('Y')
    # Exiba o gráfico
    plt.show()

    print(len(inliers1))
    print(f"Line point: {model1.params[0]}, direction vector: {model1.params[1]}")
    return list(inliers1), outliers1, model1
