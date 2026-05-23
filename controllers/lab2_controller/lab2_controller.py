"""
Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores
ICI 4150 - Robótica y Sistemas Autónomos

Implementa:
  - Lectura de sensores de distancia (2 frontales, 1 izquierdo, 1 derecho)
  - Lectura de encoders de rueda
  - Filtro simple (exponencial) sobre sensores frontales
  - Filtro de Kalman escalar para estimar distancia frontal
  - Navegación reactiva usando la estimación fusionada
  - Registro de datos para análisis comparativo
"""

import math
import csv
import os
from controller import Robot

# ---------------------------------------------------------------------------
# Parámetros de simulación
# ---------------------------------------------------------------------------
TIME_STEP = 32          # Paso de simulación en ms
Ts = TIME_STEP / 1000.0  # Tiempo de muestreo en segundos
fs = 1.0 / Ts            # Frecuencia de muestreo en Hz

MAX_SPEED = 6.28         # Velocidad máxima de las ruedas (rad/s)

# Parámetros físicos del e-puck
WHEEL_RADIUS = 0.0205    # Radio de la rueda en m
AXLE_LENGTH = 0.052      # Distancia entre ruedas en m

# ---------------------------------------------------------------------------
# Parámetros de navegación
# ---------------------------------------------------------------------------
FORWARD_SPEED = 3.0      # Velocidad de avance (rad/s)
TURN_SPEED = 2.5         # Velocidad de giro (rad/s)
SAFE_DISTANCE = 0.08     # Umbral de seguridad en m (bajo porque los sensores
                          # IR del e-puck tienen rango maximo ~5 cm)
CRITICAL_DISTANCE = 0.03 # Distancia critica: si el sensor crudo detecta
                          # obstaculo mas cerca que esto, se fuerza giro
MAX_DISTANCE = 0.30      # Distancia máxima considerada (saturación)

# ---------------------------------------------------------------------------
# Parámetros del filtro simple (exponencial)
# ---------------------------------------------------------------------------
ALPHA_FILTER = 0.3       # Factor de suavizado (0 < alpha <= 1)

# ---------------------------------------------------------------------------
# Parámetros del filtro de Kalman escalar
# ---------------------------------------------------------------------------
R = 0.001                # Varianza de la medición (ruido del sensor)
Q = 0.0001               # Varianza del proceso (incertidumbre del modelo)

# ---------------------------------------------------------------------------
# Duración total de la simulación (en segundos) - limitada para recolección
# ---------------------------------------------------------------------------
SIMULATION_DURATION = 120.0  # 2 minutos


def get_sensor_distance(sensor):
    """
    Retorna la distancia en metros medida por el sensor.
    Los sensores de distancia del e-puck en Webots (DistanceSensor)
    devuelven el valor directamente en metros (0 a maxRange).
    Cuando no hay obstáculo, el sensor retorna su valor máximo (maxRange).
    En ese caso, retornamos MAX_DISTANCE para indicar "vía libre".
    """
    value = sensor.getValue()
    max_val = sensor.getMaxValue()

    # Si la lectura está en el máximo (o muy cerca), no hay obstáculo
    if value >= max_val * 0.95:
        return MAX_DISTANCE

    return max(0.002, min(MAX_DISTANCE, value))


def main():
    robot = Robot()

    # -----------------------------------------------------------------------
    # Configuración de motores
    # -----------------------------------------------------------------------
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

    # -----------------------------------------------------------------------
    # Configuración de sensores de distancia
    #   ps0 y ps7: frontales
    #   ps5: lateral izquierdo
    #   ps2: lateral derecho
    # -----------------------------------------------------------------------
    frontal_left_sensor = robot.getDevice('ps7')   # Frontal izquierdo
    frontal_right_sensor = robot.getDevice('ps0')  # Frontal derecho
    left_sensor = robot.getDevice('ps5')           # Lateral izquierdo
    right_sensor = robot.getDevice('ps2')          # Lateral derecho

    # Habilitar sensores
    frontal_left_sensor.enable(TIME_STEP)
    frontal_right_sensor.enable(TIME_STEP)
    left_sensor.enable(TIME_STEP)
    right_sensor.enable(TIME_STEP)

    # Mostrar rangos máximos de los sensores (para depuración)
    print(f"Sensor frontal izq (ps7): maxRange = {frontal_left_sensor.getMaxValue():.4f} m")
    print(f"Sensor frontal der (ps0): maxRange = {frontal_right_sensor.getMaxValue():.4f} m")
    print(f"Sensor lateral izq (ps5): maxRange = {left_sensor.getMaxValue():.4f} m")
    print(f"Sensor lateral der (ps2): maxRange = {right_sensor.getMaxValue():.4f} m")

    # -----------------------------------------------------------------------
    # Configuración de encoders (sensores de posición de las ruedas)
    # -----------------------------------------------------------------------
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(TIME_STEP)
    right_encoder.enable(TIME_STEP)

    # -----------------------------------------------------------------------
    # Variables de estado para el filtro de Kalman
    # -----------------------------------------------------------------------
    # Estado inicial: distancia frontal desconocida, se asume máxima
    d_est = MAX_DISTANCE       # d̂_k (estimación actual)
    P = 0.01                   # Covarianza de la estimación

    # -----------------------------------------------------------------------
    # Variables para el filtro exponencial
    # -----------------------------------------------------------------------
    filtered_frontal = MAX_DISTANCE  # Valor inicial del filtro simple

    # -----------------------------------------------------------------------
    # Variables para encoder (para calcular desplazamiento)
    # -----------------------------------------------------------------------
    prev_left_pos = left_encoder.getValue()
    prev_right_pos = right_encoder.getValue()

    # -----------------------------------------------------------------------
    # Registro de datos (en memoria)
    # -----------------------------------------------------------------------
    data_log = []  # Lista de diccionarios con todas las señales

    # -----------------------------------------------------------------------
    # Bucle principal
    # -----------------------------------------------------------------------
    scenario = robot.getCustomData() or "default"
    print("=" * 60)
    print("LABORATORIO 2: Navegación Reactiva con Kalman")
    print(f"Escenario: {scenario.upper()}")
    print(f"Ts = {Ts:.3f} s,  fs = {fs:.1f} Hz,  TIME_STEP = {TIME_STEP} ms")
    print(f"Safe distance = {SAFE_DISTANCE:.3f} m")
    print(f"Kalman: R = {R:.6f}, Q = {Q:.6f}")
    print("=" * 60)

    step_count = 0
    max_steps = int(SIMULATION_DURATION * 1000 / TIME_STEP)

    while robot.step(TIME_STEP) != -1 and step_count < max_steps:
        step_count += 1
        t = step_count * Ts

        # -------------------------------------------------------------------
        # 1. Lectura de sensores de distancia (crudos)
        # -------------------------------------------------------------------
        raw_frontal_left = get_sensor_distance(frontal_left_sensor)
        raw_frontal_right = get_sensor_distance(frontal_right_sensor)
        raw_left = get_sensor_distance(left_sensor)
        raw_right = get_sensor_distance(right_sensor)

        # Medición frontal: el mínimo entre los dos sensores frontales
        z_frontal = min(raw_frontal_left, raw_frontal_right)

        # -------------------------------------------------------------------
        # 2. Lectura de encoders
        # -------------------------------------------------------------------
        curr_left_pos = left_encoder.getValue()
        curr_right_pos = right_encoder.getValue()

        # Diferencia angular desde el paso anterior (en radianes)
        delta_theta_L = curr_left_pos - prev_left_pos
        delta_theta_R = curr_right_pos - prev_right_pos

        # Desplazamiento lineal de cada rueda
        delta_s_L = WHEEL_RADIUS * delta_theta_L
        delta_s_R = WHEEL_RADIUS * delta_theta_R

        # Avance lineal promedio del robot en este paso
        delta_s = (delta_s_L + delta_s_R) / 2.0

        prev_left_pos = curr_left_pos
        prev_right_pos = curr_right_pos

        # -------------------------------------------------------------------
        # 3. Filtro simple (exponencial) sobre medición frontal
        # -------------------------------------------------------------------
        filtered_frontal = (ALPHA_FILTER * z_frontal +
                            (1.0 - ALPHA_FILTER) * filtered_frontal)

        # -------------------------------------------------------------------
        # 4. Filtro de Kalman para estimar distancia frontal
        # -------------------------------------------------------------------
        # Etapa de PREDICCIÓN
        # La distancia frontal disminuye según el avance del robot
        d_pred = d_est - delta_s
        # La covarianza aumenta por el ruido del proceso
        P_pred = P + Q

        # Etapa de CORRECCIÓN
        # Ganancia de Kalman
        K = P_pred / (P_pred + R)

        # Corrección con la medición de los sensores frontales
        d_est = d_pred + K * (z_frontal - d_pred)

        # Actualización de la covarianza
        P = (1.0 - K) * P_pred

        # Limitar la estimación al rango físico
        d_est = max(0.002, min(MAX_DISTANCE, d_est))

        # -------------------------------------------------------------------
        # 5. Decisión de navegación reactiva
        # -------------------------------------------------------------------
        # Chequeo de emergencia: si el sensor crudo detecta un obstáculo
        # muy cercano, se fuerza el giro inmediatamente sin esperar al Kalman
        if z_frontal < CRITICAL_DISTANCE:
            if raw_left < raw_right:
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)
                action = "TURN_RIGHT"
            else:
                left_motor.setVelocity(-TURN_SPEED)
                right_motor.setVelocity(TURN_SPEED)
                action = "TURN_LEFT"
        elif d_est > SAFE_DISTANCE:
            # Avanzar recto
            left_motor.setVelocity(FORWARD_SPEED)
            right_motor.setVelocity(FORWARD_SPEED)
            action = "FORWARD"
        else:
            # Obstáculo detectado por Kalman: decidir hacia dónde girar
            if raw_left < raw_right:
                # Obstáculo más cercano a la izquierda → girar a la derecha
                left_motor.setVelocity(TURN_SPEED)
                right_motor.setVelocity(-TURN_SPEED)
                action = "TURN_RIGHT"
            else:
                # Obstáculo más cercano a la derecha (o igual) → girar a la izquierda
                left_motor.setVelocity(-TURN_SPEED)
                right_motor.setVelocity(TURN_SPEED)
                action = "TURN_LEFT"

        # -------------------------------------------------------------------
        # 6. Registrar datos
        # -------------------------------------------------------------------
        data_log.append({
            'step': step_count,
            'time': round(t, 4),
            'raw_frontal': round(z_frontal, 6),
            'filtered_frontal': round(filtered_frontal, 6),
            'kalman_estimate': round(d_est, 6),
            'raw_left': round(raw_left, 6),
            'raw_right': round(raw_right, 6),
            'encoder_L': round(curr_left_pos, 6),
            'encoder_R': round(curr_right_pos, 6),
            'delta_s': round(delta_s, 6),
            'kalman_gain': round(K, 6),
            'action': action
        })

        # Log periódico en consola
        if step_count % 50 == 0:
            print(f"[t={t:6.2f}s] raw={z_frontal:.4f}m | "
                  f"filt={filtered_frontal:.4f}m | "
                  f"kalman={d_est:.4f}m | K={K:.4f} | "
                  f"d_s={delta_s:.4f}m | accion={action}")

    # -----------------------------------------------------------------------
    # Guardar datos en CSV (nombre según escenario)
    # -----------------------------------------------------------------------
    csv_filename = f"lab2_data_{scenario}.csv"
    csv_path = os.path.join(os.path.dirname(__file__), csv_filename)
    if data_log:
        keys = data_log[0].keys()
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(data_log)
        print(f"\nDatos guardados en: {csv_path}")
        print(f"Muestras registradas: {len(data_log)}")
        print(f"Tiempo total simulado: {data_log[-1]['time']:.2f} s")

    # -----------------------------------------------------------------------
    # Resumen estadístico
    # -----------------------------------------------------------------------
    if data_log:
        raws = [d['raw_frontal'] for d in data_log]
        filts = [d['filtered_frontal'] for d in data_log]
        kalms = [d['kalman_estimate'] for d in data_log]

        print("\n--- RESUMEN ESTADÍSTICO ---")
        print(f"Señal cruda frontal   - media: {sum(raws)/len(raws):.4f} m, "
              f"min: {min(raws):.4f} m, max: {max(raws):.4f} m")
        print(f"Señal filtrada frontal - media: {sum(filts)/len(filts):.4f} m, "
              f"min: {min(filts):.4f} m, max: {max(filts):.4f} m")
        print(f"Estimación Kalman      - media: {sum(kalms)/len(kalms):.4f} m, "
              f"min: {min(kalms):.4f} m, max: {max(kalms):.4f} m")

        # Contar acciones
        actions = [d['action'] for d in data_log]
        print(f"\nAcciones: FORWARD={actions.count('FORWARD')}, "
              f"TURN_LEFT={actions.count('TURN_LEFT')}, "
              f"TURN_RIGHT={actions.count('TURN_RIGHT')}")

    print("\nSimulación finalizada.")

    # Detener motores
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


if __name__ == "__main__":
    main()
