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
FORWARD_SPEED = 1.2      # Velocidad de avance (rad/s) - mas lenta para evitar choques
TURN_SPEED = 2.5         # Velocidad de giro (rad/s)
SAFE_DISTANCE = 0.040    # Umbral de seguridad en m
CLEAR_DISTANCE = 0.050   # Distancia para considerar via libre (histeresis)
CRITICAL_DISTANCE = 0.020 # Distancia critica para giro de emergencia (m)
MAX_DISTANCE = 0.30      # Distancia máxima considerada (sin obstáculo)
STEER_GAIN = 3.0         # Ganancia de corrección lateral al avanzar
STEER_MAX = 1.2          # Limite de corrección lateral (rad/s)
AVOID_TURN_STEPS = int(0.30 / Ts)  # Duracion de giro al evitar obstaculo
INNOVATION_THRESHOLD = 0.10  # Si la diferencia entre medición y predicción
                              # supera este umbral, se resetea el Kalman

# ---------------------------------------------------------------------------
# Parámetros del filtro simple (exponencial)
# ---------------------------------------------------------------------------
ALPHA_FILTER = 0.3       # Factor de suavizado (0 < alpha <= 1)

# ---------------------------------------------------------------------------
# Parámetros del filtro de Kalman escalar
# ---------------------------------------------------------------------------
R = 0.0005               # Varianza de la medición (ruido del sensor)
Q = 0.0001               # Varianza del proceso (incertidumbre del modelo)

# ---------------------------------------------------------------------------
# Duración total de la simulación (en segundos) - limitada para recolección
# ---------------------------------------------------------------------------
SIMULATION_DURATION = 120.0  # 2 minutos


# ---------------------------------------------------------------------------
# Conversion de sensores IR del e-puck usando lookup table
# ---------------------------------------------------------------------------
DEFAULT_SENSOR_RANGE = 0.05
DEFAULT_SENSOR_MAX_VALUE = 1024.0
DEFAULT_NOISE_THRESHOLD = 80.0


def build_lookup(sensor):
    """Construye la tabla de lookup (distancia, valor, ruido)."""
    table = sensor.getLookupTable()
    entries = []
    for i in range(0, len(table), 3):
        entries.append((table[i], table[i + 1], table[i + 2]))
    return entries


def lookup_distance(value, entries):
    """Interpola distancia desde un valor usando la tabla lookup."""
    if not entries:
        return None

    # Se asume que la tabla esta ordenada por distancia
    v0 = entries[0][1]
    v_last = entries[-1][1]

    # Fuera de rango
    if (v0 >= v_last and value >= v0) or (v0 <= v_last and value <= v0):
        return entries[0][0]
    if (v0 >= v_last and value <= v_last) or (v0 <= v_last and value >= v_last):
        return entries[-1][0]

    # Buscar segmento
    for i in range(len(entries) - 1):
        d1, v1, _ = entries[i]
        d2, v2, _ = entries[i + 1]
        if (v1 >= value >= v2) or (v1 <= value <= v2):
            if v1 == v2:
                return 0.5 * (d1 + d2)
            t = (value - v1) / (v2 - v1)
            return d1 + t * (d2 - d1)

    return entries[-1][0]


def get_sensor_distance(sensor, lookup_info):
    """
    Retorna distancia en metros a partir del valor crudo del sensor.
    Si la lectura esta en el maximo rango, se mapea a MAX_DISTANCE.
    """
    value = sensor.getValue()
    entries = lookup_info['table']
    max_range = lookup_info['max_range']

    distance = lookup_distance(value, entries)
    if distance is None:
        # Fallback simple si no hay lookup table
        if value < DEFAULT_NOISE_THRESHOLD:
            return MAX_DISTANCE
        distance = DEFAULT_SENSOR_RANGE * (1.0 - value / DEFAULT_SENSOR_MAX_VALUE)

    if distance >= max_range * 0.98:
        return MAX_DISTANCE

    return max(0.001, min(MAX_DISTANCE, distance))


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

    # Construir lookup tables para cada sensor
    def make_lookup_info(sensor):
        entries = build_lookup(sensor)
        max_range = entries[-1][0] if entries else DEFAULT_SENSOR_RANGE
        return {'table': entries, 'max_range': max_range}

    lookup_info = {
        'FL': make_lookup_info(frontal_left_sensor),
        'FR': make_lookup_info(frontal_right_sensor),
        'L':  make_lookup_info(left_sensor),
        'R':  make_lookup_info(right_sensor),
    }

    print("--- Diagnostico de sensores ---")
    for name, key, s in [
        ('ps7 FL', 'FL', frontal_left_sensor),
        ('ps0 FR', 'FR', frontal_right_sensor),
        ('ps5 L', 'L', left_sensor),
        ('ps2 R', 'R', right_sensor)
    ]:
        mv = s.getMaxValue()
        mr = lookup_info[key]['max_range']
        table_len = len(lookup_info[key]['table'])
        print(f"  {name}: maxValue={mv:.1f}, maxRange={mr:.3f}m, lookup={table_len} pts")
    print(f"  Saturacion: distancia >= 0.98*maxRange → MAX_DISTANCE={MAX_DISTANCE}m")
    print("-------------------------------")

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

    # Estado de evitacion (mantener giro por algunos pasos)
    avoid_steps = 0

    # -----------------------------------------------------------------------
    # Variables para encoder (para calcular desplazamiento)
    # Se inicializan en None; se asignan tras el primer robot.step()
    # para evitar NaN (encoders no tienen valor válido antes del primer step)
    # -----------------------------------------------------------------------
    prev_left_pos = None
    prev_right_pos = None

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
        # 1. Lectura de sensores de distancia
        # -------------------------------------------------------------------
        # Valor crudo del sensor (proximidad: mayor = más cerca, NO en metros)
        raw_val_fl = frontal_left_sensor.getValue()
        raw_val_fr = frontal_right_sensor.getValue()
        raw_val_l = left_sensor.getValue()
        raw_val_r = right_sensor.getValue()

        # Distancia procesada (MAX_DISTANCE si no hay obstaculo)
        raw_frontal_left = get_sensor_distance(frontal_left_sensor, lookup_info['FL'])
        raw_frontal_right = get_sensor_distance(frontal_right_sensor, lookup_info['FR'])
        raw_left = get_sensor_distance(left_sensor, lookup_info['L'])
        raw_right = get_sensor_distance(right_sensor, lookup_info['R'])

        # Medición frontal: el mínimo entre los dos sensores frontales
        z_frontal = min(raw_frontal_left, raw_frontal_right)

        # -------------------------------------------------------------------
        # 2. Lectura de encoders
        # -------------------------------------------------------------------
        curr_left_pos = left_encoder.getValue()
        curr_right_pos = right_encoder.getValue()

        # En el primer paso, inicializar posiciones previas (evita NaN)
        if prev_left_pos is None:
            prev_left_pos = curr_left_pos
            prev_right_pos = curr_right_pos

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

        # Innovación: diferencia entre medición real y predicción
        innovation = z_frontal - d_pred

        # Si la innovación es muy grande (obstáculo aparece de repente),
        # se resetea el filtro para confiar directamente en la medición.
        # Esto evita el retardo del Kalman ante cambios bruscos.
        if abs(innovation) > INNOVATION_THRESHOLD:
            d_est = z_frontal
            P = R  # Resetear covarianza al nivel de ruido del sensor
        else:
            # Corrección normal de Kalman
            d_est = d_pred + K * innovation
            P = (1.0 - K) * P_pred

        # Limitar la estimación al rango físico
        d_est = max(0.002, min(MAX_DISTANCE, d_est))

        # -------------------------------------------------------------------
        # 5. Decisión de navegación reactiva (anti-colisión)
        # -------------------------------------------------------------------
        # Usar la medida mas conservadora entre Kalman y medicion directa
        front_dist = min(d_est, z_frontal)

        # Direccion de giro: si el obstaculo esta a la izquierda, girar a la derecha
        turn_dir = 1 if raw_left < raw_right else -1  # 1=right, -1=left

        # Si estamos demasiado cerca, activar secuencia de evitacion
        if front_dist < CRITICAL_DISTANCE:
            avoid_steps = max(avoid_steps, AVOID_TURN_STEPS * 2)
        elif front_dist < SAFE_DISTANCE:
            avoid_steps = max(avoid_steps, AVOID_TURN_STEPS)

        if avoid_steps > 0:
            avoid_steps -= 1
            left_motor.setVelocity(TURN_SPEED * turn_dir)
            right_motor.setVelocity(-TURN_SPEED * turn_dir)
            action = "TURN_RIGHT" if turn_dir == 1 else "TURN_LEFT"
        else:
            # Avance con correccion lateral suave
            speed_scale = min(1.0, front_dist / CLEAR_DISTANCE) if CLEAR_DISTANCE > 0 else 1.0
            base_speed = FORWARD_SPEED * speed_scale
            steer = (raw_right - raw_left) * STEER_GAIN
            steer = max(-STEER_MAX, min(STEER_MAX, steer))

            left_speed = base_speed + steer
            right_speed = base_speed - steer

            left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, left_speed)))
            right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, right_speed)))
            action = "FORWARD"

        # -------------------------------------------------------------------
        # 6. Registrar datos
        # -------------------------------------------------------------------
        data_log.append({
            'step': step_count,
            'time': round(t, 4),
            'sensor_raw_FL': round(raw_val_fl, 6),
            'sensor_raw_FR': round(raw_val_fr, 6),
            'raw_frontal': round(z_frontal, 6),
            'front_dist': round(front_dist, 6),
            'filtered_frontal': round(filtered_frontal, 6),
            'kalman_estimate': round(d_est, 6),
            'raw_left': round(raw_left, 6),
            'raw_right': round(raw_right, 6),
            'sensor_raw_L': round(raw_val_l, 6),
            'sensor_raw_R': round(raw_val_r, 6),
            'encoder_L': round(curr_left_pos, 6),
            'encoder_R': round(curr_right_pos, 6),
            'delta_s': round(delta_s, 6),
            'kalman_gain': round(K, 6),
            'action': action
        })

        # Log periódico en consola
        if step_count <= 5 or step_count % 50 == 0:
            print(f"[t={t:6.2f}s] raw_val=({raw_val_fl:.4f},{raw_val_fr:.4f}) "
                  f"conv=({raw_frontal_left:.4f},{raw_frontal_right:.4f}) "
                  f"z={z_frontal:.4f} front={front_dist:.4f} | "
                  f"kalman={d_est:.4f} K={K:.4f} | "
                  f"avoid={avoid_steps} accion={action}")

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
