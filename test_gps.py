from controller import Robot
robot = Robot()
gps = robot.getDevice('gps')
if gps:
    print("GPS FOUND!")
else:
    print("NO GPS")
