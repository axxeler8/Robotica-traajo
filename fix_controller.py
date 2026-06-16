import re

with open('controllers/proyecto_final_controller/proyecto_final_controller.py', 'r') as f:
    code = f.read()

# Update thresholds
code = re.sub(r'DETECT_THRESH\s*=\s*80', 'DETECT_THRESH = 100', code)
code = re.sub(r'CRITICAL_THRESH\s*=\s*150', 'CRITICAL_THRESH = 250', code)
code = re.sub(r'DANGER_THRESH\s*=\s*300', 'DANGER_THRESH = 500', code)

# Fix compute method
old_avoid = """        if self.avoid_counter > 0:
            self.avoid_counter -= 1
            left_sum  = sv[5] + sv[6] + sv[7]   # sensores izquierdos
            right_sum = sv[0] + sv[1] + sv[2]   # sensores derechos
            if left_sum > right_sum:
                return SLOW_SPEED, -SLOW_SPEED, "AVOID_RIGHT"
            else:
                return -SLOW_SPEED, SLOW_SPEED, "AVOID_LEFT"
"""

new_avoid = """        if self.avoid_counter > 0:
            self.avoid_counter -= 1
            left_sum  = sv[5] + sv[6] + sv[7]   # sensores izquierdos
            right_sum = sv[0] + sv[1] + sv[2]   # sensores derechos
            if front_max > CRITICAL_THRESH:
                if left_sum > right_sum:
                    return SLOW_SPEED, -SLOW_SPEED, "AVOID_RIGHT"
                else:
                    return -SLOW_SPEED, SLOW_SPEED, "AVOID_LEFT"
            else:
                if left_sum > right_sum:
                    return SLOW_SPEED, 0.0, "AVOID_RIGHT"
                else:
                    return 0.0, SLOW_SPEED, "AVOID_LEFT"
"""

if old_avoid in code:
    code = code.replace(old_avoid, new_avoid)
    print("Avoidance logic replaced successfully!")
else:
    print("Could not find old avoidance logic to replace.")

# Fix side_max in conditions
old_cond = """        elif front_max > DETECT_THRESH:
            self.avoid_counter = max(self.avoid_counter, AVOID_STEPS // 2)"""

new_cond = """        elif front_max > DETECT_THRESH or side_max > CRITICAL_THRESH:
            self.avoid_counter = max(self.avoid_counter, AVOID_STEPS // 2)"""

if old_cond in code:
    code = code.replace(old_cond, new_cond)
    print("Conditions updated!")

with open('controllers/proyecto_final_controller/proyecto_final_controller.py', 'w') as f:
    f.write(code)

