from controller import Robot
import sys
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# We can't use GPS if it's not enabled, but we can check if it exists or just use a Supervisor
# Wait, a normal controller can't access Supervisor unless it's a supervisor.
