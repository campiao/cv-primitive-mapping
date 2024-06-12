import json
import os
from operator import itemgetter

from utils import get_shape_type_by_num_lines, shapes

annotations_dir = "..\\worlds\\annotations"


def read_shape_data_from_file(filename):
    path = os.path.join(annotations_dir, filename)
    with open(path, "r") as f:
        data = json.load(f)
        data = process_annotation_data(data)
        return data


def process_annotation_data(data):
    print(data)
    data_processed = []
    for item in data:
        item.pop("name")
        print(f"Value: {item}")
        data_processed.append(item)
    data_processed = sorted(data_processed, key=itemgetter("type", "x"))
    return data_processed


def get_formated_results_data(results):
    returns = []
    for shape_result in results:
        shape_type = get_shape_type_by_num_lines(shape_result[0])
        measures = shape_result[-1]
        formated_result = {"type": shape_type, "x": measures[0][0], "y": measures[0][1]}
        formated_result = format_shape_result[shape_type](formated_result, measures[1:])
        returns.append(formated_result)
    returns = sorted(returns, key=itemgetter("type", "x"))
    return returns


def compare_result_and_annotations(results, annotations, error_margin):
    total_types = 0
    total_center = 0
    total_measures = 0

    type_true_count = 0
    center_true_count = 0
    measures_true_count = 0
    if len(results) != len(annotations):
        print(f"Number of detected shapes is different from actual shapes: {len(results)},{len(annotations)}")
    for i in range(len(results)):
        shape_result = results[i]
        annotations_result = annotations[i]

        total_types += 1
        total_center += 2

        if shape_result["type"] == annotations_result["type"]:
            type_true_count += 1
        if shape_result["x"] - annotations_result["x"] < error_margin:
            center_true_count += 1
        if shape_result["y"] - annotations_result["y"] < error_margin:
            center_true_count += 1
        keys_to_remove = ['type', 'x', 'y']
        original_shape_result = shape_result
        original_annotations_result = annotations_result
        for key in keys_to_remove:
            shape_result.pop(key)
            annotations_result.pop(key)
        for item, value in shape_result.items():
            total_measures += 1
            if value - annotations_result[item] < error_margin:
                measures_true_count += 1

    type_accuracy = type_true_count / total_types
    center_accuracy = center_true_count / total_center
    measures_accuracy = measures_true_count / total_measures

    total_accuracy = (type_accuracy*0.4 + center_accuracy*0.3 + measures_accuracy*0.3)

    print(f"=====Accuracy of RANSAC algorithm=====")
    print(f"Shape type accuracy: {type_accuracy}")
    print(f"Center position accuracy: {center_accuracy}")
    print(f"Measurements accuracy: {measures_accuracy}")
    print(f"Total accuracy (weighted sum of accuracies): {total_accuracy}")

    return total_accuracy, type_accuracy, center_accuracy, measures_accuracy


def format_circle_result(formated_result_dic, result):
    formated_result_dic["radius"] = result[0]
    return formated_result_dic


def format_rectangle_result(formated_result_dic, result):
    formated_result_dic["width"] = result[0]
    formated_result_dic["height"] = result[1]
    return formated_result_dic


def format_triangle_result(formated_result_dic, result):
    formated_result_dic["base"] = result[0]
    return formated_result_dic


def format_pentagon_result(formated_result_dic, result):
    formated_result_dic["width"] = result[0]
    return formated_result_dic


format_shape_result = {
    "circle": format_circle_result,
    "rectangle": format_rectangle_result,
    "triangle": format_triangle_result,
    "pentagon": format_pentagon_result
}


def print_results(results):
    print("=====Resultados ajustados às coordenadas do Webots=====")
    for result in results:
        print(f"Number of ransac runs(sides): {result[0]}")
        args = result[-1]
        print_shape_results[result[0]](shapes[result[0]], args)
        print()


def print_circle_results(shape, results):
    print(f"Shape: {shape}")
    print(f"Center: {results[0]}")
    print(f"Radius: {results[1]}")


def print_triangle_results(shape, results):
    print(f"Shape: {shape}")
    print(f"Center: {results[0]}")
    print(f"Base: {results[1]}")


def print_square_results(shape, results):
    print(f"Shape: {shape}")
    print(f"Center: {results[0]}")
    print(f"Width: {results[1]}")
    print(f"Height: {results[2]}")


def print_pentagon_results(shape, results):
    print(f"Shape: {shape}")
    print(f"Center: {results[0]}")
    print(f"Width: {results[1]}")


print_shape_results = {
    1: print_circle_results,
    3: print_triangle_results,
    4: print_square_results,
    5: print_pentagon_results
}
