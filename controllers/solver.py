import numpy as np

from controllers.constants import PERCENT_OF_TOTAL
from controllers.kmeans import Kmeans
from controllers.ransac_functions import RansacPrimitiveClassifier
from controllers.results import print_results, get_formated_results_data, read_shape_data_from_file, \
    compare_result_and_annotations
from controllers.utils import plot_line, save_results


def solve_shapes_problem(grid, map_name, num_shapes):
    x, y = grid.get_x_y_coord()
    # Load your point cloud as a numpy array (N, 3)
    readings = np.array([[x[i], y[i]] for i in range(len(x))])
    x = readings[:, 0]
    y = readings[:, 1]

    kmeans = Kmeans()
    shapes = kmeans.Kmeans(x, y, num_shapes)
    print(len(shapes))

    ransac = RansacPrimitiveClassifier()
    results = []
    for shape in shapes:
        ransac_count, x, y = ransac.solve_shape(shape, PERCENT_OF_TOTAL, False)
        print(f"Num of ransac runs: {ransac_count}")
        plot_line(x, y)
        shape_measures = ransac.get_shape_measures(ransac_count, x, y)
        if ransac_count > 5:
            ransac_count = 1
        shape_result = [ransac_count, shape_measures]
        results.append(shape_result)

    print_results(results)
    result_to_compare = get_formated_results_data(results)
    annotations = read_shape_data_from_file(f"{map_name}.json")
    total_acc, type_acc, center_acc, measures_acc = compare_result_and_annotations(result_to_compare,
                                                                                   annotations, 0.001)
    return [total_acc, type_acc, center_acc, measures_acc]