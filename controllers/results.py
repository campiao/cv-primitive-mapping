import json
import os

from utils import get_shape_type_by_num_lines

annotations_dir = "..\\worlds\\annotations\\"


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
    return data_processed


def get_formated_results_data(results):
    returns = []
    for shape_result in results:
        shape_type = get_shape_type_by_num_lines(shape_result[0])
        measures = shape_result[-1]
        formated_result = {"type": shape_type, "x": measures[0][0], "y": measures[0][1]}
        formated_result = format_shape_result[shape_type](formated_result, measures[1:])
        returns.append(formated_result)
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
        shape_result: dict = shape_result.pop("type").pop("x").pop("y")
        annotations_result: dict = annotations_result.pop("type").pop("x").pop("y")
        for item, value in shape_result.items():
            total_measures += 1
            if value - annotations_result[item] < error_margin:
                measures_true_count += 1

    type_accuracy = type_true_count / total_types
    center_accuracy = center_true_count / total_center
    measures_accuracy = measures_true_count / total_measures

    print(f"Shape type accuracy: {type_accuracy}")
    print(f"Center position accuracy: {center_accuracy}")
    print(f"Measurements accuracy: {measures_accuracy}")

    return type_accuracy, center_accuracy, measures_accuracy


def format_circle_result(formated_result_dic, result):
    formated_result_dic["radius"] = result[0]
    return formated_result_dic


def format_rectangle_result(formated_result_dic, result):
    formated_result_dic["width"] = result[0]
    formated_result_dic["height"] = result[1]
    return formated_result_dic


def format_triangle_result(formated_result_dic, result):
    formated_result_dic["base"] = result[0]
    formated_result_dic["height"] = result[1]
    return formated_result_dic


def format_pentagon_result(formated_result_dic, result):
    formated_result_dic["radius"] = result[0]
    formated_result_dic["height"] = result[1]
    return formated_result_dic


format_shape_result = {
    "circle": format_circle_result,
    "rectangle": format_rectangle_result,
    "triangle": format_triangle_result,
    "pentagon": format_pentagon_result
}
