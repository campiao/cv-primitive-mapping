import math
import csv

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controllers.transformations import create_tf_matrix, get_translation, get_rotation

def read_coordinates_from_csv(file_path):
    coordinates = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) >= 2:
                x, y = map(float, row[:2])  # Assuming first two columns are x, y coordinates
                coordinates.append([x, y])
    return np.array(coordinates)

def find_possible_poses_from_csv(coordinates: np.ndarray, num_lines: int) -> ([(float, float)], [float]):
    # Structure the coordinates for input to RANSAC
    data = array(coordinates)

    max_iterations = num_lines  # Define the number of iterations

    models = []
    inliers_bools = []
    outliers = []
    original_data = data.copy()  # Preserve the original data

    for i in range(max_iterations):
        # Find the line
        model, inliers_bool = ransac(data, LineModelND, min_samples=2,
                                     residual_threshold=0.007, max_trials=10000)
        models.append(model)
        inliers_bools.append(inliers_bool)

        # Retrieve the outliers
        outliers_data = array([point for (point, inlier_bool) in zip(data, inliers_bool) if not inlier_bool])
        outliers.append(outliers_data)

        # Update data for the next iteration
        data = outliers_data

    # Compute the inliers for each iteration
    inliers = [array([point for (point, inlier_bool) in zip(original_data, inliers_bool) if inlier_bool]) for
               original_data, inliers_bool in zip([original_data] + outliers, inliers_bools)]

    # Run the final function with the collected data
    draw_walls(original_data, models, inliers, outliers[-1])

def draw_walls(data: array, models: array(LineModelND), inliers: array([array]), outliers: array) -> None:
    # Unpack the points into separate lists of x and y coordinates
    data_x, data_y = zip(*data)

    inliers_x = []
    inliers_y = []

    for inlier in inliers:
        inlier_x, inlier_y = zip(*inlier)
        inliers_x.append(inlier_x)
        inliers_y.append(inlier_y)

    outliers_x = []
    outliers_y = []
    if len(outliers) > 0:
        outliers_x, outliers_y = zip(*outliers)

    # Plot the data points with different colors
    for i, (model, inlier_x, inlier_y) in enumerate(zip(models, inliers_x, inliers_y)):
        plt.scatter(inlier_x, inlier_y, color='blue', label=f'Inliers{i + 1}', marker='x')

        # Add the computed line to the plot
        line_x_values, line_y_values = get_model_line_points(inlier_x, inlier_y, model.params[0], model.params[1])
        plt.plot(line_x_values, line_y_values, color='blue', label=f'Line{i + 1}')

    # Add outliers to the plot
    if len(outliers) > 0:
        plt.scatter(outliers_x, outliers_y, color='red', label='Outliers', marker='x')

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LiDAR points and lines relative to the robot')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.gca().set_aspect('equal')
    plt.xlim(min(*data_x, 0) - 0.1, max(*data_x, 0) + 0.1)
    plt.ylim(min(*data_y, 0) - 0.1, max(*data_y, 0) + 0.1)

    # Show the plot
    plt.grid(True)
    plt.show()

def get_model_line_points(points_x_coords: [float], points_y_coords: [float], origin: (float, float),
                          direction: (float, float)) -> (np.ndarray, np.ndarray):
    if direction[0] == 0:  # vertical line
        line_min_y: float = min(points_y_coords)
        line_max_y: float = max(points_y_coords)
        line_y_values: np.ndarray[np.dtype:float] = np.linspace(line_min_y - 0.1, line_max_y + 0.1, 100)

        line_x_values: np.ndarray[np.dtype:float] = np.full(shape=len(line_y_values), fill_value=points_x_coords[0],
                                                            dtype=np.float64)
    else:
        line_min_x: float = min(points_x_coords)
        line_max_x: float = max(points_x_coords)
        line_x_values: np.ndarray[np.dtype:float] = np.linspace(line_min_x - 0.1, line_max_x + 0.1, 100)

        slope: float = direction[1] / direction[0]
        intercept: float = origin[1] - slope * origin[0]
        line_y_values: np.ndarray[np.dtype:float] = slope * line_x_values + intercept
    return line_x_values, line_y_values

def main() -> None:
    # Load coordinates from CSV
    csv_file_path = r'C:\Users\pedro\Desktop\Codigo\Robotica\IRI_public_TP_classes-master\IRI_public_TP_classes\worlds\custom_maps\mapa1_points.csv'
    coordinates = read_coordinates_from_csv(csv_file_path)

    data = array(coordinates)
    original_data = data.copy()

    find_possible_poses_from_csv(coordinates, 8)

    plt.show()

if __name__ == '__main__':
    main()
