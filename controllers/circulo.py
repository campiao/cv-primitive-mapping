
import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS, Keyboard
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation
from utils import cmd_vel

import pyransac3d as pyrsc
import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS, Keyboard
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation
from utils import cmd_vel

import pyransac3d as pyrsc





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
        #lidar_data=lidar.getPointCloud()
        # Armazena as leituras na lista
        #print(f"adicionei leituras:{len(lidar_readings)}")
        #lidar_readings.append(lidar_data)
        key: int = kb.getKey()
        if key == ord('W'):
            cmd_vel(robot, keyboard_linear_vel, 0)
        elif key == ord('S'):
            cmd_vel(robot, -keyboard_linear_vel, 0)
        elif key == ord('A'):
            cmd_vel(robot, 0, keyboard_angular_vel)
        elif key == ord('D'):
            cmd_vel(robot, 0, -keyboard_angular_vel)
        elif key==ord('K'):
            lidar_data = lidar.getPointCloud()
            print(f"adicionei leituras:{len(lidar_readings)}")
            lidar_readings.append(lidar_data)
        else:  # Not a movement key
            cmd_vel(robot, 0, 0)
            if key == ord(' '):
                lidar_data_processed = []
                for data in lidar_readings:
                    for data_point in data:
                        x = data_point.x
                        y = data_point.y
                        z = data_point.z
                        # Verifica se as coordenadas são finitas antes de adicionar à matriz
                        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                            lidar_data_processed.append([x, y, z])
                print(lidar_data_processed)
                lidar_data_processed = np.array(lidar_data_processed)

                from mpl_toolkits.mplot3d import Axes3D

                # Supondo que 'lidar_points' é a matriz numpy contendo os pontos do lidar
                lidar_points=lidar_data_processed
                # Crie uma nova figura e um subplot 3D
                x = lidar_points[:, 0]
                y = lidar_points[:, 1]

                # Crie um novo gráfico 2D
                plt.figure()

                # Plote os pontos no gráfico 2D
                plt.scatter(x, y, s=1)

                # Configure os rótulos dos eixos
                plt.xlabel('X')
                plt.ylabel('Y')

                # Exiba o gráfico
                plt.show()

                # Load your point cloud as a numpy array (N, 3)

                sph = pyrsc.Circle()
                center,axis, radius, inliers = sph.fit(lidar_data_processed,thresh=0.2,maxIteration=1000)
                print(center)
                print(radius)
                break

if __name__ == '__main__':
    main()
