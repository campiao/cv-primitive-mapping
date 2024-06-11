import math
import numpy as np
from matplotlib import pyplot as plt

from skimage.measure import LineModelND, ransac


class RansacPrimitiveClassifier:
    def __init__(self):
        pass

    def fit_line(self, readings):
        data = readings
        print(len(data))
        model1, inliers_bools1 = ransac(data, LineModelND, min_samples=2, residual_threshold=1, max_trials=10000)

        outliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if not inlier_bool])

        inliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if inlier_bool])

        x, y = inliers1[:, 0], inliers1[:, 1]
        print(f"Length inliers: {len(inliers1)}")

        self.plot_line(x,y)

        print(len(inliers1))
        print(f"Line point: {model1.params[0]}, direction vector: {model1.params[1]}")
        return list(inliers1), outliers1, model1

    def solve_shape(self, readings, total_percentage):
        inliers, outliers, line_data = self.fit_line(readings)
        x = [point[0] for point in inliers]
        y = [point[1] for point in inliers]
        lines = [line_data]
        ransac_count = 1

        threshold_outliers = math.floor(len(readings) * total_percentage)
        print("total: ", len(readings))
        print("min_out: ", threshold_outliers)
        while len(outliers) > threshold_outliers:
            inliers, outliers, line_data = self.fit_line(outliers)
            for line in lines:
                if line_data.params[0][0] - line.params[0][0] == 0:
                    declive = 0
                else:
                    declive = (line_data.params[0][1] - line.params[0][1]) / (
                                line_data.params[0][0] - line.params[0][0])
                if line.params[1][0] == 0:
                    stored_slope = 0
                else:
                    stored_slope = line.params[1][1] / line.params[1][0]
                if stored_slope == 0:
                    print("skipped, count", ransac_count)
                    continue
                if declive - stored_slope < 0.01:
                    print("skipped, count", ransac_count)
                    continue
            lines.append(line_data)
            for point in inliers:
                x.append(point[0])
                y.append(point[1])
            ransac_count += 1
            print("not skipped, count", ransac_count)
        return lines, ransac_count, x, y

    def plot_line(self, x, y):
        plt.figure()
        # Plote os pontos no gráfico 2D
        plt.scatter(x, y, s=1)

        # Configure os rótulos dos eixos
        plt.xlabel('X')
        plt.ylabel('Y')
        # Exiba o gráfico
        plt.show()




