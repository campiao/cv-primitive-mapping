from matplotlib import pyplot as plt
from skimage.measure import LineModelND, ransac
import pyransac3d as pyrsc
from deterministic_occupancy_grid import *

from utils import plot_line
from constants import GRID_RESOLUTION


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

        plot_line(x, y)

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

    def square_measures(self, x, y):

        ponto_sup_esq = (min(x), max(y))

        ponto_inf_esq = (min(x), min(y))

        ponto_sup_dir = (max(x), max(y))

        largura = np.abs(ponto_sup_dir[0] - ponto_inf_esq[0])
        altura = np.abs(ponto_sup_dir[1] - ponto_inf_esq[1])

        centro_x = (ponto_sup_dir[0] + ponto_inf_esq[0]) / 2
        centro_y = (ponto_sup_esq[1] + ponto_inf_esq[1]) / 2

        plt.figure()
        plt.scatter(x, y, s=1)
        plt.scatter([ponto_sup_esq[0], ponto_inf_esq[0], ponto_sup_dir[0], centro_x],
                    [ponto_sup_esq[1], ponto_inf_esq[1], ponto_sup_dir[1], centro_y], color='red')  # Pontos extremos
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Pontos Extremos e Centro do Retângulo')
        plt.annotate('Centro', (centro_x, centro_y), textcoords="offset points", xytext=(0, 10), ha='center',
                     color='green')
        plt.show()

        print(f"Ponto Superior Esquerdo: {ponto_sup_esq}")
        print(f"Ponto Inferior Esquerdo: {ponto_inf_esq}")
        print(f"Ponto Superior Direito: {ponto_sup_dir}")
        print(f"Largura: {largura:.2f}")
        print(f"Altura: {altura:.2f}")
        print(f"Centro: ({centro_x:.2f}, {centro_y:.2f})")

    def triangle_measures(self, x, y):

        ponto_inf_dir = (min(x), min(y))

        ponto_inf_esq = (min(x), max(y))

        topo = (max(x), np.abs((ponto_inf_esq[1] + ponto_inf_dir[1]) / 2))

        altura = np.abs(max(y) - min(y))
        base = np.abs(ponto_inf_dir[1] - ponto_inf_esq[1])

        centro_x = np.abs((ponto_inf_esq[0] + topo[0]) / 2)

        centro_y = np.abs((ponto_inf_dir[1] + ponto_inf_esq[1]) / 2)

        plt.figure()
        plt.scatter(x, y, s=1)
        plt.scatter([ponto_inf_dir[0], ponto_inf_esq[0], topo[0], centro_x],
                    [ponto_inf_dir[1], ponto_inf_esq[1], topo[1], centro_y], color='red')  # Pontos extremos
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Pontos Extremos e Centro do Triângulo')
        plt.annotate('Centro', (centro_x, centro_y), textcoords="offset points", xytext=(0, 10), ha='center',
                     color='green')
        plt.show()

        print(f"Ponto Inferior Direito: {ponto_inf_dir}")
        print(f"Ponto Inferior Esquerdo: {ponto_inf_esq}")
        print(f"Topo: {topo}")
        print(f"Altura: {altura:.2f}")
        print(f"Base: {base:.2f}")
        print(f"Centro: ({centro_x:.2f}, {centro_y:.2f})")

    def circle(self, lidar_data_processed, grid):
        sph = pyrsc.Circle()
        center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.05, maxIteration=1000)
        print(f"center: {center}, radius: {radius}")
        print(f"adjusted center: {grid.grid_to_real_coords(center)}, radius: {radius * GRID_RESOLUTION}")

    def pentagon_measures(self, x, y):

        min_y_point = (x[np.argmin(y)], min(y))

        max_x_point = (max(x), y[np.argmax(x)])

        centroid_x = np.mean(x)
        centroid_y = np.mean(y)

        plt.figure()
        plt.scatter(x, y, s=1)
        plt.scatter([max_x_point[0], min_y_point[0], centroid_x],
                    [max_x_point[1], min_y_point[1], centroid_y], color='red')  # Pontos extremos
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Pontos Extremos e Centro do Pentágono')
        plt.annotate('Centro', (centroid_x, centroid_y), textcoords="offset points", xytext=(0, 10), ha='center',
                     color='green')
        plt.show()

        print(f"Centroide: ({centroid_x:.2f}, {centroid_y:.2f})")

        # Calcular a distância entre min_y_point e max_x_point
        distance = np.sqrt((min_y_point[0] - max_x_point[0]) ** 2 + (min_y_point[1] - max_x_point[1]) ** 2)
        print(f"Distância entre o ponto com menor Y e o ponto com maior X: {distance:.2f}")
