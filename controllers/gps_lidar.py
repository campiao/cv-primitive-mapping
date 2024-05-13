"""
IRI - TP4 - Ex 1
By: Gonçalo Leão
"""
import math

from matplotlib import pyplot as plt
import pyransac3d as pyrsc
from controller import Robot, Lidar, LidarPoint, Compass, GPS, Keyboard
import numpy as np

from occupancy_grid import OccupancyGrid
from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, bresenham


class DeterministicOccupancyGrid(OccupancyGrid):
    def __init__(self, origin: (float, float), dimensions: (int, int), resolution: float):
        super().__init__(origin, dimensions, resolution)

        # Initialize the grid
        self.x=[]
        self.y=[]
        self.occupancy_grid: np.ndarray = np.full(dimensions, 0.5, dtype=np.float32)

    def update_map(self, robot_tf: np.ndarray, lidar_points: [LidarPoint]):
        # Get the grid coord for the robot pose
        robot_coord: (int, int) = self.real_to_grid_coords(get_translation(robot_tf)[0:2])

        # Get the grid coords for the lidar points
        grid_lidar_coords: [(int, int)] = []
        for point in lidar_points:
            if math.isfinite(point.x) and math.isfinite(point.y):
                coord: (int, int) = self.real_to_grid_coords(np.dot(robot_tf, [point.x, point.y, 0.0, 1.0])[0:2])
                grid_lidar_coords.append(coord)

        # Set as free the cell of the robot's position
        self.update_cell(robot_coord, False)

        # Set as free the cells leading up to the lidar points
        for coord in grid_lidar_coords:
            for mid_coord in bresenham(robot_coord, coord)[1:-1]:
                self.update_cell(mid_coord, False)

        # Set as occupied the cells for the lidar points
        for coord in grid_lidar_coords:
            self.x.append(coord[0])
            self.y.append(coord[1])
            self.update_cell(coord, True)

        return self.x,self.y



    def update_cell(self, coords: (int, int), is_occupied: bool) -> None:
        if self.are_grid_coords_in_bounds(coords):
            # Update the grid cell
            self.occupancy_grid[coords] = 1 if is_occupied else 0


def main() -> None:
    robot: Robot = Robot()
    timestep: int = 100  # in ms

    kb: Keyboard = Keyboard()
    kb.disable()
    kb.enable(timestep)

    keyboard_linear_vel: float = 0.3
    keyboard_angular_vel: float = 1.5

    map: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], 0.01)

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)

    scan_count: int = 0
    while robot.step(timestep) != -1:
        key: int = kb.getKey()
        if key == ord('W'):
            cmd_vel(robot, keyboard_linear_vel, 0)
        elif key == ord('S'):
            cmd_vel(robot, -keyboard_linear_vel, 0)
        elif key == ord('A'):
            cmd_vel(robot, 0, keyboard_angular_vel)
        elif key == ord('D'):
            cmd_vel(robot, 0, -keyboard_angular_vel)
        elif key == ord('P'):
            # Crie um novo gráfico 2D
            plt.figure()

            # Plote os pontos no gráfico 2D
            plt.scatter(x,y, s=1)

            # Configure os rótulos dos eixos
            plt.xlabel('X')
            plt.ylabel('Y')
            # Exiba o gráfico
            plt.show()
        elif key == ord('L'):
            nova_lista = []
            for i in range(len(x)):
                par = [x[i], y[i], 1]  # Cria uma lista com os números emparelhados e adiciona 1
                nova_lista.append(par)
            lidar_data_processed = np.array(nova_lista)
            # Load your point cloud as a numpy array (N, 3)

            sph = pyrsc.Circle()
            center, axis, radius, inliers = sph.fit(lidar_data_processed, thresh=0.2, maxIteration=1000)
            print(center)
            print(radius)



        else:  # Not a movement key
            cmd_vel(robot, 0, 0)
            if key == ord(' '):  # ord('Q'):
                scan_count += 1
                print('scan count: ', scan_count)

                # Read the robot's pose
                gps_readings: [float] = gps.getValues()
                robot_position: (float, float) = (gps_readings[0], gps_readings[1])
                compass_readings: [float] = compass.getValues()
                robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])
                robot_tf: np.ndarray = create_tf_matrix((robot_position[0], robot_position[1], 0.0), robot_orientation)

                # Read the LiDAR and update the map
                x,y=map.update_map(robot_tf, lidar.getPointCloud())




if __name__ == '__main__':
    main()
