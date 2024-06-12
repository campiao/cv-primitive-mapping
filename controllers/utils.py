
import math
from typing import List
import csv

import numpy as np

from controller import Robot, Motor, Supervisor, Node, Field, Lidar, GPS, Compass
from controller.device import Device
from matplotlib import pyplot as plt

from constants import LIDAR_SCAN_UPDATES, GRID_RESOLUTION, GRID_ORIGIN, ROBOT_TIMESTEP, RESULTS_FILE_NAME
from controllers.transformations import create_tf_matrix

shapes = {
    1: "circle",
    3: "triangle",
    4: "rectangle",
    5: "pentagon"
}


def save_results(filename, results, num_shapes):
    map_name = f"{str(num_shapes)}_{filename}"
    data = [map_name] + results
    results_file = f"..\\results\\{str(num_shapes)}_{RESULTS_FILE_NAME}"

    with open(results_file, "a", newline="\n") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)

    print(f"Map {map_name} saved to {results_file}.")


def initialize_webots_features():
    robot: Supervisor = Supervisor()
    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(ROBOT_TIMESTEP)
    lidar.enablePointCloud()
    compass: Compass = robot.getDevice('compass')
    compass.enable(ROBOT_TIMESTEP)
    gps: GPS = robot.getDevice('gps')
    gps.enable(ROBOT_TIMESTEP)

    return robot, lidar, compass, gps


def get_shape_type_by_num_lines(ransac_count: int) -> str:
    return shapes[ransac_count]


def plot_line(x, y):
    # Crie um novo gráfico 2D
    plt.figure()

    # Plote os pontos no gráfico 2D
    plt.scatter(x, y, s=1)

    # Configure os rótulos dos eixos
    plt.xlabel('X')
    plt.ylabel('Y')
    # Exiba o gráfico
    plt.show()


def create_waypoints():
    waypoints = []
    coordx = -0.5
    coordy = -0.5

    while coordy < 0.6:
        waypoints.append((coordx, coordy))
        coordx += 0.15
        if coordx > 0.5:
            coordx = -0.5
            coordy += 0.1

    return waypoints


def teletransporte(robot, scan_count, gps, compass, lidar, map, current_count, timestep):
    waypoints = create_waypoints()
    for waypoint in waypoints:
        warp_robot(robot, "EPUCK", waypoint)
        robot.step(timestep)

        if record_lidar_scan(current_count, gps, compass, lidar, map):
            current_count = 0
            scan_count += 1
            print("scan count: ", scan_count)
        current_count += 1
    return scan_count, current_count


def record_lidar_scan(current_count, gps, compass, lidar, map):
    if current_count < LIDAR_SCAN_UPDATES:
        return False

    # Read the robot's pose
    gps_readings: [float] = gps.getValues()
    robot_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])
    robot_tf: np.ndarray = create_tf_matrix((robot_position[0], robot_position[1], 0.0), robot_orientation)

    # Read the LiDAR and update the map
    x, y = map.update_map(robot_tf, lidar.getPointCloud())

    return True


def grid_to_real_coords(coords: [float, float]) -> (int, int):
    return tuple((GRID_RESOLUTION * coords[i]) + GRID_ORIGIN[i] for i in [1, 0])


# Prints the type of all the devices in a scene with a single robot.
def print_devices() -> None:
    supervisor: Supervisor = Supervisor()
    num_devices: int = supervisor.getNumberOfDevices()
    for i in range(num_devices):
        device: Device = supervisor.getDeviceByIndex(i)
        print(device.getName(), '   - NodeType:',
              list(Node.__dict__.keys())[list(Node.__dict__.values()).index(device.getNodeType())])


# This function uses odometry math to translate the linear and angular velocities
# to the left and right motor speeds.
# Note: the robot may take some time to reach the target speeds, since the motors
# can't instantly start rotating at the target motor speeds.
# Made for the epuck robot.
# https://cyberbotics.com/doc/guide/epuck?version=R2021a
AXLE_LENGTH: float = 0.057  # obtained with manual calibration. It should be 0.052 m according to the documentation.
WHEEL_RADIUS: float = 0.0205
MAX_SPEED: float = 6.28

# tangential/linear speed in m/s.
# tangential speed = angular speed * wheel radius
TANGENTIAL_SPEED: float = MAX_SPEED * WHEEL_RADIUS



def cmd_vel(robot: Robot, linear_vel: float, angular_vel: float) -> None:
    r_omega: float = (linear_vel + angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS
    l_omega: float = (linear_vel - angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS

    # Get a handler to the motors and set target position to infinity (speed control)
    left_motor: Motor = robot.getDevice('left wheel motor')
    right_motor: Motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Set up the motor speeds.
    left_motor.setVelocity(l_omega)
    right_motor.setVelocity(r_omega)


def warp_robot(supervisor: Supervisor, robot_def_name: str, new_position: (float, float)) -> None:
    robot_node = supervisor.getFromDef(robot_def_name)
    trans_field: Field = robot_node.getField("translation")
    translation: List[float] = [new_position[0], new_position[1], 0]
    trans_field.setSFVec3f(translation)
    robot_node.resetPhysics()
