
import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS, Keyboard
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation
from utils import cmd_vel







def main() -> None:
    lidar_readings = []
    scan_count=0
    robot: Robot = Robot()
    timestep: int=100
    kb: Keyboard = Keyboard()
    kb.enable(timestep)

    keyboard_linear_vel: float = 0.3
    keyboard_angular_vel: float = 1.5

    min_x: float = -0.5
    min_y: float = -0.5
    max_x: float = 0.5
    max_y: float = 0.5

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(int(robot.getBasicTimeStep()))
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)
    robot.step()

    # Read the ground-truth (correct robot pose)
    gps_readings: [float] = gps.getValues()
    actual_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    actual_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

    robot_tf: np.ndarray = create_tf_matrix((actual_position[0], actual_position[1], 0.0), actual_orientation)

    # Draw a point cloud for a square map
    num_divisions: int = 100
    fixed_points: [(float, float, float)] = []
    for i in range(num_divisions):
        x: float = min_x + (max_x - min_x) * (i / float(num_divisions - 1))
        fixed_points.append([x, min_y, 0.0])
        fixed_points.append([x, max_y, 0.0])

        y: float = min_y + (max_y - min_y) * (i / float(num_divisions - 1))
        fixed_points.append([min_x, y, 0.0])
        fixed_points.append([max_x, y, 0.0])
    fixed_cloud: np.ndarray = np.asarray(fixed_points)

    #estimated_translations, estimated_rotations = find_possible_poses(robot_tf, lidar.getPointCloud(), min_x, max_x, min_y, max_y)
    #draw_real_vs_estimated_localization(fixed_cloud,
    #                                   actual_position, actual_orientation,
    #                                    estimated_translations, estimated_rotations)

    while robot.step() != -1:
        lidar_data=[]
        lidar_data = lidar.getPointCloud()
        # Armazena as leituras na lista
        lidar_readings.append(lidar_data)
        key: int = kb.getKey()
        if key == ord('W'):
            cmd_vel(robot, keyboard_linear_vel, 0)
        elif key == ord('S'):
            cmd_vel(robot, -keyboard_linear_vel, 0)
        elif key == ord('A'):
            cmd_vel(robot, 0, keyboard_angular_vel)
        elif key == ord('D'):
            cmd_vel(robot, 0, -keyboard_angular_vel)
        else:  # Not a movement key
            cmd_vel(robot, 0, 0)
            if key == ord(' '):
                scan_count += 1
                # Estruturação dos dados em uma matriz de coordenadas (x, y, z)
                lidar_data_processed = []
                for data_point in lidar_data:
                    # Acessa diretamente os atributos x, y e z de cada ponto do LiDAR
                    x = data_point.x
                    y = data_point.y
                    z = data_point.z
                    # Verifica se as coordenadas são finitas antes de adicionar à matriz
                    if math.isfinite(x) and math.isfinite(y):
                        lidar_data_processed.append([x, y])
                lidar_data_processed = np.array(lidar_data_processed)


                # Aplicação do RANSAC para ajustar um modelo de linha aos pontos
                model, inliers = ransac(lidar_data_processed, LineModelND, min_samples=2,
                                        residual_threshold=0.005, max_trials=1000)

                # Obtenção dos inliers (pontos que melhor se ajustam ao modelo)
                inliers_points = lidar_data_processed[inliers]

                # Plotar os pontos inliers
                plt.scatter(lidar_data_processed[:, 0], lidar_data_processed[:, 1], color='blue', label='Inliers')

                # Plotar o modelo de linha ajustado
                plt.plot(inliers_points[:, 0], model.predict(inliers_points[:, 0]), color='red',
                         label='Modelo de Linha')

                plt.xlabel('Coordenada X')
                plt.ylabel('Coordenada Y')
                plt.title('Modelo de Linha Ajustado aos Pontos Inliers')
                plt.legend()
                plt.show()


if __name__ == '__main__':
    main()
