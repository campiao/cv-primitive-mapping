from controller import Robot
from controllers.utils import cmd_vel

robot: Robot = Robot()

while robot.step() != 1:
    cmd_vel(robot, 0.1, 0.1)
