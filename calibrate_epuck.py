import math

# We know that the robot turned "less" than expected.
# If math turned 80 deg, and physical turned 70 deg.
# physical_angle = math_angle * (math_axle / phys_axle)
# 70 = 80 * (0.052 / phys_axle)
# phys_axle = 0.052 * 80 / 70 = 0.0594

# Let's search the standard Epuck axle length!
# Webots E-puck axle length is officially 0.052 for Epuck 1, and 0.053 for Epuck 2?
