import math
import random
from controller import Robot

TIME_STEP = 16        
MAX_SPEED = 6.28      

WHEEL_RADIUS = 0.0205   
AXLE_LENGTH = 0.052     


def run_phase(robot, left_motor, right_motor, vl, vr, duration_s, description):
    print(f"\n>>> {description}")

    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

    steps = int(duration_s * 1000 / TIME_STEP)
    for _ in range(steps):
        robot.step(TIME_STEP)

def run_phase_with_noise(robot, left_motor, right_motor, vl, vr, duration_s, description):
    print(f"\n>>> {description}")

    steps = int(duration_s * 1000 / TIME_STEP)
    noise_l = 0.0
    noise_r = 0.0
    for _ in range(steps):
      
        noise_l += random.uniform(-0.03, 0.03)
        noise_r += random.uniform(-0.03, 0.03)
        
    
        noise_l = max(-0.8, min(0.8, noise_l))
        noise_r = max(-0.8, min(0.8, noise_r))
            
        left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vl + noise_l)))
        right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, vr + noise_r)))
        robot.step(TIME_STEP)


def pause(robot, left_motor, right_motor, duration_s=1.0):
    run_phase(robot, left_motor, right_motor, 0, 0, duration_s, "⏸  Pausa")


def main():
    robot = Robot()

    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    run_phase(robot, left_motor, right_motor,
              vl=3.0, vr=3.0, duration_s=6.0,
              description="EXPERIMENTO 1: Movimiento recto (vr = vl = 3.0)")
    
    pause(robot, left_motor, right_motor)

    run_phase(robot, left_motor, right_motor,
              vl=2.0, vr=4.0, duration_s=4.0,
              description="EXPERIMENTO 2: Trayectoria curva (vl=2.0, vr=4.0)")

    pause(robot, left_motor, right_motor)

    run_phase(robot, left_motor, right_motor,
              vl=-3.0, vr=3.0, duration_s=3.0,
              description="EXPERIMENTO 3: Rotación en el lugar (vl=-3.0, vr=3.0)")

    pause(robot, left_motor, right_motor)

    run_phase_with_noise(robot, left_motor, right_motor,
                        vl=3.0, vr=3.0, duration_s=6.0,
                        description="EXTENSIÓN: Movimiento recto con perturbaciones")

    pause(robot, left_motor, right_motor)

    VL_CIR = 2.0
    VR_CIR = 4.0
    omega_cir = (VR_CIR - VL_CIR) * WHEEL_RADIUS / AXLE_LENGTH
    
    CORRECTION_CIR = 1.14 
    FULL_CIRCLE_DURATION = ((2 * math.pi) / omega_cir) * CORRECTION_CIR

    run_phase(robot, left_motor, right_motor,
              vl=VL_CIR, vr=VR_CIR, duration_s=FULL_CIRCLE_DURATION,
              description="DESAFÍO 1: Dibujar un círculo")

    pause(robot, left_motor, right_motor)


    BASE_SPEED = 2.0
    AMPLITUDE = 2.0
    FREQUENCY = 0.6

    PERIOD_8 = 2 * math.pi / FREQUENCY

    steps_8 = int(PERIOD_8 * 1000 / TIME_STEP)
    dt = TIME_STEP / 1000.0

    print(f"\n>>> DESAFÍO 2: Figura en 8")

    t = 0.0
    for i in range(steps_8):
        left_speed = BASE_SPEED - AMPLITUDE * math.sin(FREQUENCY * t)
        right_speed = BASE_SPEED + AMPLITUDE * math.sin(FREQUENCY * t)

        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        robot.step(TIME_STEP)
        t += dt

    pause(robot, left_motor, right_motor)

    print("\n" + "=" * 55)
    print("Todos los experimentos finalizados")
    print("=" * 55)

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    while robot.step(TIME_STEP) != -1:
        pass


if __name__ == "__main__":
    main()
