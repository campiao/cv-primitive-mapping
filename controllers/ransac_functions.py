from matplotlib import pyplot as plt
from skimage.measure import LineModelND, ransac
import pyransac3d as pyrsc
from deterministic_occupancy_grid import *

from utils import plot_line, grid_to_real_coords
from constants import GRID_RESOLUTION


class RansacPrimitiveClassifier:
    def __init__(self):
        self.function_map = {
            1: self.circle,
            3: self.triangle_measures,
            4: self.square_measures,
            5: self.pentagon_measures
        }

    def fit_line(self, readings):
        data = readings
        model1, inliers_bools1 = ransac(data, LineModelND, min_samples=2, residual_threshold=5, max_trials=10000)

        outliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if not inlier_bool])

        inliers1: np.array = np.array([point for (point, inlier_bool) in zip(data, inliers_bools1) if inlier_bool])

        x, y = inliers1[:, 0], inliers1[:, 1]

        return list(inliers1), outliers1, model1

    def solve_shape(self, readings, total_percentage, verbose):
        inliers, outliers, line_data = self.fit_line(readings)
        x = [point[0] for point in inliers]
        y = [point[1] for point in inliers]
        lines = [line_data]
        ransac_count = 1

        threshold_outliers = math.floor(len(readings) * total_percentage)
        if verbose:
            print(f"Number of points: {len(readings)}")
            print(f"Threshold outliers: {threshold_outliers}")
        while len(outliers) > threshold_outliers:
            inliers, outliers, line_data = self.fit_line(outliers)
            if verbose:
                print(f"Inliers lenght: {len(inliers)}")
                print(f"Outliers lenght: {len(outliers)}")
            lines.append(line_data)
            for point in inliers:
                x.append(point[0])
                y.append(point[1])
            ransac_count += 1

        return ransac_count, x, y

    def get_shape_measures(self, num_lines, *args):
        if num_lines in self.function_map:
            return self.function_map[num_lines](*args)
        else:
            if num_lines > 5:
                return self.function_map[1](*args)
            print(f"No function defined for shape with {num_lines} sides.")

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
        print("Resultados em coordenadas da grid:")
        print(f"\tLargura: {largura:.2f}")
        print(f"\tAltura: {altura:.2f}")
        print(f"\tCentro: ({centro_x:.2f}, {centro_y:.2f})")

        largura = round(largura * GRID_RESOLUTION, 2)
        altura = round(altura * GRID_RESOLUTION, 2)

        centro = [centro_x, centro_y]
        centro = grid_to_real_coords(centro)
        x = centro[0]
        y = centro[1]
        centro = [round(x, 2), round(y, 2)]

        return [centro, largura, altura]

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
        print("Resultados em coordenadas da grid:")
        print(f"\tAltura: {altura:.2f}")
        print(f"\tBase: {base:.2f}")
        print(f"\tCentro: ({centro_x:.2f}, {centro_y:.2f})")

        altura = round(altura * GRID_RESOLUTION, 2)
        base = round(base * GRID_RESOLUTION, 2)

        centro = [centro_x, centro_y]
        centro = grid_to_real_coords(centro)
        x = centro[0]
        y = centro[1]
        centro = [round(x, 2), round(y, 2)]

        return [centro, base, altura]

    def circle(self, x, y):
        lidar_data_processed = np.array([[x[i], y[i], 0] for i in range(len(x))])
        sph = pyrsc.Circle()
        center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.05, maxIteration=1000)

        plt.figure()
        plt.scatter(x, y, s=1)
        plt.scatter([center[0]],
                    [center[1]], color='red')  # Pontos extremos
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.annotate('Centro', (center[0], center[1]), textcoords="offset points", xytext=(0, 10), ha='center',
                     color='green')
        plt.show()

        print("Resultados em coordenadas da grid:")
        print(f"\tCentro: {center}")
        print(f"\tRaio: {radius}")
        center = grid_to_real_coords(center)
        x = center[0]
        y = center[1]
        center = [round(x, 2), round(y, 2)]
        radius = round(radius * GRID_RESOLUTION, 2)
        return [center, radius]

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
        print("Resultados em coordenadas da grid:")
        print(f"\tCentroide: ({centroid_x:.2f}, {centroid_y:.2f})")

        # Calcular a distância entre min_y_point e max_x_point
        distance = np.sqrt((min_y_point[0] - max_x_point[0]) ** 2 + (min_y_point[1] - max_x_point[1]) ** 2)
        print(f"Distância entre o ponto com menor Y e o ponto com maior X: {distance:.2f}")

        center = grid_to_real_coords([centroid_x, centroid_y])
        x = center[0]
        y = center[1]
        center = [round(x, 2), round(y, 2)]
        radius = round(distance * GRID_RESOLUTION, 2)
        return [center, radius, radius]
