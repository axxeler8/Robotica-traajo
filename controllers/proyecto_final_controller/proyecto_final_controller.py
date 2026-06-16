"""
Proyecto Final: Navegación Autónoma con Planificación de Rutas (A*)
Asignatura: ICI 4150 — Robótica y Sistemas Autónomos 2026-01
Integrantes: Martín Cevallos, Carlos Abarza, Matías Vergara

Robot: e-puck diferencial
Sensores: 8 sensores IR de proximidad (ps0-ps7) + encoders de rueda
Algoritmo: A* sobre grilla de ocupación 2D con navegación reactiva
"""

import math
import heapq
import os
import csv
from controller import Robot

# ════════════════════════════════════════════════════════════════════
#  CONSTANTES DEL ROBOT E-PUCK
# ════════════════════════════════════════════════════════════════════
TIME_STEP = 32            # ms
MAX_SPEED = 3.14          # velocidad máxima (rad/s)
WHEEL_RADIUS = 0.0205     # m      (radio de cada rueda)
AXLE_LENGTH = 0.052       # m      (distancia entre ruedas)

# ════════════════════════════════════════════════════════════════════
#  PARÁMETROS DE LA GRILLA DE OCUPACIÓN
# ════════════════════════════════════════════════════════════════════
GRID_SIZE = 40            # 40×40 celdas
CELL_SIZE = 0.05          # 2.0 m / 40 = 0.05 m por celda
ARENA_HALF = 1.0          # la arena mide 2 m × 2 m, centrada en (0, 0)
INFLATION = 1             # inflar obstáculos 1 celda (≈5 cm) para evitar roces físicos
                           # compensar el radio del robot (~3.7 cm) + margen

# ════════════════════════════════════════════════════════════════════
#  PARÁMETROS DE NAVEGACIÓN
# ════════════════════════════════════════════════════════════════════
WAYPOINT_THRESH = 0.08    # distancia para dar un waypoint por alcanzado (m)
GOAL_THRESH = 0.04        # distancia para dar la meta por alcanzada (m)
KP_HEADING = 3.0          # ganancia proporcional para control de heading
BASE_SPEED = 3.14         # velocidad de crucero (rad/s)
SLOW_SPEED = 1.0          # velocidad reducida cerca de obstáculos

# ════════════════════════════════════════════════════════════════════
#  PARÁMETROS DE EVITACIÓN REACTIVA
# ════════════════════════════════════════════════════════════════════
DETECT_THRESH = 100.0      # valor crudo del sensor → obstáculo detectado
CRITICAL_THRESH = 250.0   # valor crudo → obstáculo cercano
DANGER_THRESH = 500.0     # valor crudo → peligro inminente
AVOID_STEPS = 30          # pasos de giro durante la evitación (~1 s)
AVOID_EMERGENCY = 50      # pasos para evasión de emergencia (~1.6 s)

# ════════════════════════════════════════════════════════════════════
#  SENSORES DE PROXIMIDAD DEL E-PUCK
# ════════════════════════════════════════════════════════════════════
#   ps0  ─ frontal-derecho   (~10° derecha)
#   ps1  ─ derecho-frontal   (~45° derecha)
#   ps2  ─ derecho            (~90° derecha)
#   ps3  ─ derecho-trasero   (~135° derecha)
#   ps4  ─ izquierdo-trasero (~225°)
#   ps5  ─ izquierdo          (~270°)
#   ps6  ─ izquierdo-frontal (~315°)
#   ps7  ─ frontal-izquierdo (~350°)
SENSOR_NAMES = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

# ════════════════════════════════════════════════════════════════════
#  DEFINICIÓN DE ESCENARIOS
#  Cada obstáculo: (centro_x, centro_z, ancho_x, ancho_z)
#  Coordenadas en el plano XZ de Webots.
# ════════════════════════════════════════════════════════════════════
SCENARIOS = {
    'simple': {
        'start_x': -0.75, 'start_y': -0.75,
        'goal_x':   0.75, 'goal_y':   0.75,
        'initial_heading': math.pi / 2.0,
        'obstacles': [
            # 5 bloques dispersos
            ( 0.00,  0.00, 0.25, 0.25),   # bloque central grande (size 0.25)
            (-0.35,  0.40, 0.20, 0.20),   # superior-izquierda (size 0.2)
            ( 0.40, -0.35, 0.20, 0.20),   # inferior-derecha (size 0.2)
            ( 0.35,  0.55, 0.20, 0.15),   # zona superior (size 0.2x0.15)
            (-0.55, -0.35, 0.15, 0.15),   # inferior-izquierda (size 0.15)
        ],
    },
    'complejo': {
        'start_x': -0.75, 'start_y': -0.75,
        'goal_x':   0.75, 'goal_y':   0.75,
        'initial_heading': math.pi / 2.0,
        'obstacles': [
            # ── 4 muros horizontales que fuerzan un zigzag ─────────
            (-0.25, -0.30, 0.50, 0.05),   # muro 1 (abajo, gap derecho)
            ( 0.25,  0.05, 0.50, 0.05),   # muro 2 (medio, gap izquierdo)
            (-0.25,  0.40, 0.50, 0.05),   # muro 3 (arriba, gap derecho)
            ( 0.25,  0.70, 0.50, 0.05),   # muro 4 (tope, gap izquierdo)
            # ── Bloques que añaden complejidad ─────────────────────
            ( 0.55, -0.55, 0.12, 0.12),   # inferior-derecha
            (-0.55,  0.20, 0.12, 0.12),   # centro-izquierda
            ( 0.60,  0.55, 0.12, 0.12),   # superior-derecha
            (-0.55, -0.10, 0.12, 0.12),   # izquierda-medio
            # ── Bloques adicionales ────────────────────────────────
            ( 0.00, -0.65, 0.15, 0.15),   # centro-abajo
            (-0.65,  0.65, 0.12, 0.12),   # esquina sup-izquierda
            ( 0.65, -0.25, 0.12, 0.12),   # derecha-medio
            (-0.30, -0.65, 0.12, 0.12),   # inferior-izquierda
        ],
    },
}


# ════════════════════════════════════════════════════════════════════
#  FILTRO DE SENSORES
# ════════════════════════════════════════════════════════════════════

class SensorFilter:
    """Filtro de media móvil exponencial (EMA) para suavizar lecturas
    de sensores y reducir ruido sin introducir retardo excesivo."""
    def __init__(self, n_sensors, alpha=0.4):
        self.alpha = alpha
        self.filtered = [0.0] * n_sensors
        self.initialized = False

    def update(self, raw):
        if not self.initialized:
            self.filtered = list(raw)
            self.initialized = True
            return self.filtered
        for i in range(len(raw)):
            self.filtered[i] = self.alpha * raw[i] + (1.0 - self.alpha) * self.filtered[i]
        return self.filtered


# ════════════════════════════════════════════════════════════════════
#                       CLASES PRINCIPALES
# ════════════════════════════════════════════════════════════════════

class OccupancyGrid:
    """Grilla de ocupación 2D que representa el entorno."""

    def __init__(self):
        self.grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]

    # ── Conversión coordenadas ────────────────────────────────────
    def world_to_grid(self, wx, wy):
        """Convierte coordenadas del mundo (x, z) a índices (fila, col)."""
        EPS = 1e-9
        col = int((wx + ARENA_HALF) / CELL_SIZE + EPS)
        row = int((wy + ARENA_HALF) / CELL_SIZE + EPS)
        col = max(0, min(GRID_SIZE - 1, col))
        row = max(0, min(GRID_SIZE - 1, row))
        return (row, col)

    def grid_to_world(self, row, col):
        """Convierte índices (fila, col) a coordenadas del mundo (x, z)."""
        wx = -ARENA_HALF + (col + 0.5) * CELL_SIZE
        wy = -ARENA_HALF + (row + 0.5) * CELL_SIZE
        return (wx, wy)

    # ── Agregar obstáculo ─────────────────────────────────────────
    def add_obstacle(self, cx, cy, sx, sy, inflate=0):
        """Marca celdas ocupadas por un obstáculo rectangular.
        inflate: celdas adicionales de inflación (para radio del robot)."""
        half_x = sx / 2.0 + inflate * CELL_SIZE
        half_y = sy / 2.0 + inflate * CELL_SIZE
        EPS = 1e-9
        c_min = int((cx - half_x + ARENA_HALF) / CELL_SIZE)
        c_max = int((cx + half_x + ARENA_HALF) / CELL_SIZE + EPS)
        r_min = int((cy - half_y + ARENA_HALF) / CELL_SIZE)
        r_max = int((cy + half_y + ARENA_HALF) / CELL_SIZE + EPS)
        for r in range(max(0, r_min), min(GRID_SIZE, r_max + 1)):
            for c in range(max(0, c_min), min(GRID_SIZE, c_max + 1)):
                self.grid[r][c] = 1

    def is_free(self, row, col):
        if 0 <= row < GRID_SIZE and 0 <= col < GRID_SIZE:
            return self.grid[row][col] == 0
        return False

    # ── Imprimir grilla en consola ────────────────────────────────
    def print_grid(self, path=None, start=None, goal=None):
        path_set = set(path) if path else set()
        for r in range(GRID_SIZE - 1, -1, -1):
            line = ""
            for c in range(GRID_SIZE):
                if start and (r, c) == start:
                    line += "S "
                elif goal and (r, c) == goal:
                    line += "G "
                elif (r, c) in path_set:
                    line += "· "
                elif self.grid[r][c] == 1:
                    line += "█ "
                else:
                    line += "  "
            print(line.rstrip())


# ──────────────────────────────────────────────────────────────────
class AStarPlanner:
    """Planificador de rutas A* sobre grilla de ocupación 2D."""

    # 8 direcciones: cardinales (costo 1) + diagonales (costo √2)
    DIRS = [
        (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
    ]

    @staticmethod
    def heuristic(a, b):
        """Distancia euclídea como heurística admisible."""
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    @staticmethod
    def find_path(grid, start, goal):
        """Busca la ruta óptima de start a goal usando A*."""
        open_set = [(0.0, start)]
        came_from = {}
        g = {start: 0.0}
        closed = set()

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                path = []
                while cur in came_from:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                path.reverse()
                return path
            if cur in closed:
                continue
            closed.add(cur)

            for dr, dc, cost in AStarPlanner.DIRS:
                nb = (cur[0] + dr, cur[1] + dc)
                if not grid.is_free(nb[0], nb[1]) or nb in closed:
                    continue
                # No cortar esquinas en diagonal
                if dr != 0 and dc != 0:
                    if not grid.is_free(cur[0] + dr, cur[1]):
                        continue
                    if not grid.is_free(cur[0], cur[1] + dc):
                        continue
                ng = g[cur] + cost
                if ng < g.get(nb, float('inf')):
                    came_from[nb] = cur
                    g[nb] = ng
                    f = ng + AStarPlanner.heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return None  # no se encontró ruta

    @staticmethod
    def smooth_path(path, grid):
        """Reduce waypoints manteniendo línea de visión libre."""
        if len(path) <= 2:
            return list(path)
        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if AStarPlanner._los(grid, path[i], path[j]):
                    break
                j -= 1
            smoothed.append(path[j])
            i = j
        return smoothed

    @staticmethod
    def _los(grid, a, b):
        """Bresenham: verifica línea de visión entre dos celdas."""
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if not grid.is_free(r, c):
                return False
            if r == r1 and c == c1:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc


# ──────────────────────────────────────────────────────────────────
class OdometryEstimator:
    """Estimador de posición por odometría diferencial (encoders).

    Ecuaciones cinemáticas (Lab 1):
        Δs_r = r · Δθ_r         Δs_l = r · Δθ_l
        Δs   = (Δs_r + Δs_l) / 2
        Δφ   = (Δs_r − Δs_l) / L
        x_k  = x_{k-1} + Δs · cos(φ_{k-1} + Δφ/2)
        z_k  = z_{k-1} − Δs · sin(φ_{k-1} + Δφ/2)
        φ_k  = φ_{k-1} + Δφ
    """

    def __init__(self, x0, y0, theta0):
        self.x = x0
        self.y = y0
        self.theta = theta0          # heading (rad) — 0 = +X
        self.prev_left = None
        self.prev_right = None
        self.total_distance = 0.0

    def update(self, left_enc, right_enc):
        if self.prev_left is None:
            self.prev_left = left_enc
            self.prev_right = right_enc
            return

        dl = left_enc  - self.prev_left
        dr = right_enc - self.prev_right
        ds_l = WHEEL_RADIUS * dl
        ds_r = WHEEL_RADIUS * dr

        ds     = (ds_r + ds_l) / 2.0
        dtheta = (ds_r - ds_l) / AXLE_LENGTH

        mid = self.theta + dtheta / 2.0
        self.x     += ds * math.cos(mid)
        self.y     += ds * math.sin(mid)      # +sin para plano XY
        self.theta += dtheta
        # Normalizar a [−π, π]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        self.total_distance += abs(ds)
        self.prev_left  = left_enc
        self.prev_right = right_enc


# ──────────────────────────────────────────────────────────────────
class NavigationController:
    """Seguimiento de waypoints con evitación reactiva de obstáculos."""

    def __init__(self, waypoints):
        self.waypoints = waypoints      # lista de (x, z) en coord. mundo
        self.wp_idx = 0
        self.avoid_counter = 0
        self.reached_goal = False

    def compute(self, x, z, theta, sv):
        """Devuelve (vel_izq, vel_der, acción) para los motores.
        sv = lista de 8 valores crudos de sensores ps0..ps7."""
        if self.reached_goal:
            return 0.0, 0.0, "GOAL_REACHED"

        if self.wp_idx >= len(self.waypoints):
            self.reached_goal = True
            return 0.0, 0.0, "GOAL_REACHED"

        # ── Evitación reactiva (prioridad alta) ──────────────────
        # Clasificación de sensores por zona:
        #   Derecha:     ps0(+10°), ps1(+45°), ps2(+90°), ps3(+135°)
        #   Izquierda:   ps4(-135°), ps5(-90°), ps6(-45°), ps7(-10°)
        front_narrow = max(sv[0], sv[7])
        front_wide   = max(sv[0], sv[1], sv[6], sv[7])
        left_all     = sv[4] + sv[5] + sv[6] + sv[7]
        right_all    = sv[0] + sv[1] + sv[2] + sv[3]
        overall_max  = max(sv)

        # ── Asignar nivel de amenaza actual ──────────────────────
        if overall_max > DANGER_THRESH:
            threat = 3
        elif front_wide > CRITICAL_THRESH or overall_max > CRITICAL_THRESH:
            threat = 2
        elif front_narrow > DETECT_THRESH or front_wide > DETECT_THRESH * 1.5:
            threat = 1
        else:
            threat = 0

        # ── Activar/actualizar contador de evasión ───────────────
        #    Solo se extiende si la amenaza empeora (no reseteo infinito)
        if threat >= 3:
            self.avoid_counter = max(self.avoid_counter, AVOID_EMERGENCY)
        elif threat >= 2:
            self.avoid_counter = max(self.avoid_counter, AVOID_STEPS)
        elif threat >= 1:
            self.avoid_counter = max(self.avoid_counter, AVOID_STEPS // 2)

        if self.avoid_counter > 0:
            self.avoid_counter -= 1
            # Decidir dirección de giro: alejarse del lado con más obstáculos
            turn_right = left_all > right_all

            if threat >= 3:
                # EMERGENCIA: retroceder girando para alejarse del obstáculo
                if turn_right:
                    return SLOW_SPEED * 0.3, -SLOW_SPEED * 0.7, "AVOID_RIGHT"
                else:
                    return -SLOW_SPEED * 0.7, SLOW_SPEED * 0.3, "AVOID_LEFT"
            elif threat >= 2:
                # CRÍTICO: girar en el lugar
                if turn_right:
                    return SLOW_SPEED * 0.6, -SLOW_SPEED * 0.6, "AVOID_RIGHT"
                else:
                    return -SLOW_SPEED * 0.6, SLOW_SPEED * 0.6, "AVOID_LEFT"
            else:
                # DETECCIÓN: arco suave avanzando (no detenerse)
                if turn_right:
                    return SLOW_SPEED * 0.8, SLOW_SPEED * 0.15, "AVOID_RIGHT"
                else:
                    return SLOW_SPEED * 0.15, SLOW_SPEED * 0.8, "AVOID_LEFT"

        # ── Seguimiento de waypoints ─────────────────────────────
        wp_x, wp_y = self.waypoints[self.wp_idx]
        dx = wp_x - x
        dy = wp_y - z
        dist = math.sqrt(dx * dx + dy * dy)

        thr = GOAL_THRESH if self.wp_idx == len(self.waypoints) - 1 else WAYPOINT_THRESH
        if dist < thr:
            self.wp_idx += 1
            if self.wp_idx >= len(self.waypoints):
                self.reached_goal = True
                return 0.0, 0.0, "GOAL_REACHED"
            wp_x, wp_y = self.waypoints[self.wp_idx]
            dx = wp_x - x
            dy = wp_y - z
            dist = math.sqrt(dx * dx + dy * dy)

        # Heading deseado y error
        target = math.atan2(dy, dx)
        err = target - theta
        err = (err + math.pi) % (2 * math.pi) - math.pi   # normalizar

        # Control proporcional
        omega = KP_HEADING * err
        speed_factor = max(0.3, 1.0 - abs(err) / (math.pi / 2.0))
        speed = BASE_SPEED * speed_factor

        # Reducir velocidad cerca del waypoint
        if dist < 0.15:
            speed = min(speed, SLOW_SPEED + (BASE_SPEED - SLOW_SPEED) * dist / 0.15)

        half_diff = omega * AXLE_LENGTH / (2.0 * WHEEL_RADIUS)
        vl = speed - half_diff
        vr = speed + half_diff

        vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
        vr = max(-MAX_SPEED, min(MAX_SPEED, vr))

        return vl, vr, "FOLLOWING"


# ════════════════════════════════════════════════════════════════════
#                      GENERACIÓN DE GRÁFICOS
# ════════════════════════════════════════════════════════════════════

def generate_plots(base_dir, scenario_name, grid, raw_path,
                   waypoints_world, trajectory, data_log,
                   start_cell, goal_cell):
    """Genera gráficos PNG con matplotlib (backend Agg)."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  ⚠ matplotlib no disponible — ejecute plot_results.py aparte.")
        return

    print("\n  Generando gráficos...")

    # ── 1. Grilla + ruta A* ──────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid.grid[r][c]:
                wx, wy = grid.grid_to_world(r, c)
                ax.add_patch(plt.Rectangle(
                    (wx - CELL_SIZE / 2, wy - CELL_SIZE / 2),
                    CELL_SIZE, CELL_SIZE,
                    facecolor='#2d2d2d', edgecolor='none'))
    if raw_path:
        px = [grid.grid_to_world(r, c)[0] for r, c in raw_path]
        py = [grid.grid_to_world(r, c)[1] for r, c in raw_path]
        ax.plot(px, py, 'c-', lw=1, alpha=0.4, label='Ruta A* (celdas)')
    wpx = [w[0] for w in waypoints_world]
    wpy = [w[1] for w in waypoints_world]
    ax.plot(wpx, wpy, 'b-o', lw=2, ms=4, label='Waypoints suavizados')
    ax.plot(wpx[0], wpy[0], 'gs', ms=12, label='Inicio')
    ax.plot(wpx[-1], wpy[-1], 'r*', ms=15, label='Meta')
    ax.set(xlim=(-1.05, 1.05), ylim=(-1.05, 1.05), aspect='equal',
           xlabel='X (m)', ylabel='Z (m)',
           title=f'Grilla de Ocupación y Ruta A* — {scenario_name.capitalize()}')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)
    p = os.path.join(base_dir, f'grid_path_{scenario_name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f"    ✓ {p}")

    # ── 2. Ruta planificada vs trayectoria real ──────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            if grid.grid[r][c]:
                wx, wy = grid.grid_to_world(r, c)
                ax.add_patch(plt.Rectangle(
                    (wx - CELL_SIZE / 2, wy - CELL_SIZE / 2),
                    CELL_SIZE, CELL_SIZE,
                    facecolor='#e0e0e0', edgecolor='none'))
    ax.plot(wpx, wpy, 'b--', lw=2, label='Ruta planificada')
    tx = [t[0] for t in trajectory]
    ty = [t[1] for t in trajectory]
    ax.plot(tx, ty, 'r-', lw=1, alpha=0.7, label='Trayectoria real')
    ax.plot(tx[0], ty[0], 'gs', ms=12, label='Inicio')
    ax.plot(tx[-1], ty[-1], 'r*', ms=15, label='Posición final')
    ax.set(xlim=(-1.05, 1.05), ylim=(-1.05, 1.05), aspect='equal',
           xlabel='X (m)', ylabel='Z (m)',
           title=f'Planificada vs Real — {scenario_name.capitalize()}')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3)
    p = os.path.join(base_dir, f'trajectory_{scenario_name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f"    ✓ {p}")

    # ── 3. Sensores en el tiempo ─────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ts = [d['time'] for d in data_log]
    axes[0].plot(ts, [d['ps7'] for d in data_log], alpha=.7, label='ps7 front-izq')
    axes[0].plot(ts, [d['ps0'] for d in data_log], alpha=.7, label='ps0 front-der')
    axes[0].axhline(DETECT_THRESH, color='orange', ls='--', alpha=.5, label='Detección')
    axes[0].axhline(CRITICAL_THRESH, color='red', ls='--', alpha=.5, label='Crítico')
    axes[0].set(ylabel='Valor crudo', title='Sensores Frontales')
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=.3)

    axes[1].plot(ts, [d['ps6'] for d in data_log], alpha=.7, label='ps6 izq-front')
    axes[1].plot(ts, [d['ps1'] for d in data_log], alpha=.7, label='ps1 der-front')
    axes[1].plot(ts, [d['ps5'] for d in data_log], alpha=.7, label='ps5 izquierdo')
    axes[1].plot(ts, [d['ps2'] for d in data_log], alpha=.7, label='ps2 derecho')
    axes[1].set(ylabel='Valor crudo', xlabel='Tiempo (s)', title='Sensores Laterales')
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=.3)
    fig.suptitle(f'Lecturas de Sensores — {scenario_name.capitalize()}', fontsize=14)
    fig.tight_layout()
    p = os.path.join(base_dir, f'sensors_{scenario_name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f"    ✓ {p}")

    # ── 4. Heading + acciones ────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(ts, [math.degrees(d['theta']) for d in data_log], 'g-', alpha=.7)
    axes[0].set(ylabel='Heading (°)', title='Orientación del Robot')
    axes[0].grid(True, alpha=.3)

    amap = {'FOLLOWING': 0, 'AVOID_LEFT': 1, 'AVOID_RIGHT': 2, 'GOAL_REACHED': 3}
    axes[1].plot(ts, [amap.get(d['action'], -1) for d in data_log], alpha=.7)
    axes[1].set_yticks([0, 1, 2, 3])
    axes[1].set_yticklabels(['Siguiendo', 'Evitando (izq)', 'Evitando (der)', 'Meta'])
    axes[1].set(ylabel='Acción', xlabel='Tiempo (s)', title='Acciones de Navegación')
    axes[1].grid(True, alpha=.3)
    fig.suptitle(f'Heading y Acciones — {scenario_name.capitalize()}', fontsize=14)
    fig.tight_layout()
    p = os.path.join(base_dir, f'heading_{scenario_name}.png')
    fig.savefig(p, dpi=150, bbox_inches='tight'); plt.close(fig)
    print(f"    ✓ {p}")


# ════════════════════════════════════════════════════════════════════
#                         FUNCIÓN PRINCIPAL
# ════════════════════════════════════════════════════════════════════

def main():
    robot = Robot()

    # ── Determinar escenario ─────────────────────────────────────
    scenario_name = (robot.getCustomData() or 'simple').strip().lower()
    if scenario_name not in SCENARIOS:
        print(f"⚠ Escenario '{scenario_name}' no reconocido, usando 'simple'")
        scenario_name = 'simple'
    sc = SCENARIOS[scenario_name]

    print("=" * 60)
    print("PROYECTO FINAL — Navegación Autónoma con A*")
    print(f"Escenario: {scenario_name.upper()}")
    print("Integrantes: Martín Cevallos, Carlos Abarza, Matías Vergara")
    print("=" * 60)

    # ── Inicializar motores ──────────────────────────────────────
    lm = robot.getDevice('left wheel motor')
    rm = robot.getDevice('right wheel motor')
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0); rm.setVelocity(0)

    # ── Inicializar sensores de proximidad ───────────────────────
    sensors = []
    for name in SENSOR_NAMES:
        s = robot.getDevice(name)
        s.enable(TIME_STEP)
        sensors.append(s)

    # ── Inicializar encoders ─────────────────────────────────────
    le = robot.getDevice('left wheel sensor')
    re = robot.getDevice('right wheel sensor')
    le.enable(TIME_STEP)
    re.enable(TIME_STEP)

    # ══════════════════════════════════════════════════════════════
    #  FASE 1 — Construir grilla de ocupación
    # ══════════════════════════════════════════════════════════════
    print("\n[1/4] Construyendo grilla de ocupación…")
    grid = OccupancyGrid()
    display_grid = OccupancyGrid()
    for obs in sc['obstacles']:
        grid.add_obstacle(*obs, inflate=INFLATION)
        display_grid.add_obstacle(*obs, inflate=0)
    # Agregar bordes de la arena como obstáculos virtuales para no chocar
    for g, inf in [(grid, INFLATION), (display_grid, 0.5)]:
        g.add_obstacle(0.0, 1.0, 2.0, 0.0, inflate=inf)   # Muro Superior
        g.add_obstacle(0.0, -1.0, 2.0, 0.0, inflate=inf)  # Muro Inferior
        g.add_obstacle(1.0, 0.0, 0.0, 2.0, inflate=inf)   # Muro Derecho
        g.add_obstacle(-1.0, 0.0, 0.0, 2.0, inflate=inf)  # Muro Izquierdo


    start_cell = grid.world_to_grid(sc['start_x'], sc['start_y'])
    goal_cell  = grid.world_to_grid(sc['goal_x'],  sc['goal_y'])
    print(f"  Inicio: ({sc['start_x']}, {sc['start_y']}) → celda {start_cell}")
    print(f"  Meta:   ({sc['goal_x']}, {sc['goal_y']})  → celda {goal_cell}")

    # Verificar que inicio y meta estén libres
    for label, cell_ref in [("inicio", start_cell), ("meta", goal_cell)]:
        if not grid.is_free(*cell_ref):
            print(f"  ⚠ Celda de {label} ocupada, buscando cercana…")
            found = False
            for radius in range(1, 5):
                for dr in range(-radius, radius + 1):
                    for dc in range(-radius, radius + 1):
                        nc = (cell_ref[0] + dr, cell_ref[1] + dc)
                        if grid.is_free(*nc):
                            if label == "inicio":
                                start_cell = nc
                            else:
                                goal_cell = nc
                            found = True
                            break
                    if found:
                        break
                if found:
                    break

    # ══════════════════════════════════════════════════════════════
    #  FASE 2 — Planificación A*
    # ══════════════════════════════════════════════════════════════
    print("\n[2/4] Ejecutando algoritmo A*…")
    raw_path = AStarPlanner.find_path(grid, start_cell, goal_cell)
    if raw_path is None or len(raw_path) == 0:
        print("  ✗ No se encontró ruta. Verifique los obstáculos.")
        grid.print_grid(start=start_cell, goal=goal_cell)
        while robot.step(TIME_STEP) != -1:
            pass
        return

    print(f"  ✓ Ruta encontrada: {len(raw_path)} celdas")
    smoothed = AStarPlanner.smooth_path(raw_path, grid)
    print(f"  ✓ Ruta suavizada:  {len(smoothed)} waypoints")

    # Usar las coordenadas exactas para inicio y fin, no los centros de celda
    wp_world = [grid.grid_to_world(r, c) for r, c in smoothed]
    wp_world[0] = (sc['start_x'], sc['start_y'])
    wp_world[-1] = (sc['goal_x'], sc['goal_y'])

    print("\n  Waypoints:")
    for i, (wx, wy) in enumerate(wp_world):
        tag = " (INICIO)" if i == 0 else (" (META)" if i == len(wp_world) - 1 else "")
        print(f"    [{i:2d}] ({wx:+.3f}, {wy:+.3f}){tag}")

    print("\n  Mapa (S=inicio, G=meta, ·=ruta, █=obstáculo):")
    display_grid.print_grid(path=raw_path, start=start_cell, goal=goal_cell)

    planned_len = sum(
        math.sqrt((wp_world[i][0] - wp_world[i - 1][0]) ** 2 +
                  (wp_world[i][1] - wp_world[i - 1][1]) ** 2)
        for i in range(1, len(wp_world)))
    print(f"\n  Longitud de ruta planificada: {planned_len:.3f} m")

    # ══════════════════════════════════════════════════════════════
    #  FASE 3 — Navegación
    # ══════════════════════════════════════════════════════════════
    odom = OdometryEstimator(sc['start_x'], sc['start_y'],
                             sc.get('initial_heading', 0.0))
    nav = NavigationController(wp_world)

    data_log = []
    trajectory = []
    near_collisions = 0

    print(f"\n[3/4] Iniciando navegación…")
    print(f"  Vel. base = {BASE_SPEED} rad/s | KP = {KP_HEADING} | "
          f"Umbral wp = {WAYPOINT_THRESH} m")

    sensor_filter = SensorFilter(len(SENSOR_NAMES), alpha=0.4)

    robot.step(TIME_STEP)                         # paso inicial para sensores
    step = 0
    MAX_STEPS = 10000                             # ~320 s de seguridad

    while robot.step(TIME_STEP) != -1 and step < MAX_STEPS:
        step += 1
        t = step * TIME_STEP / 1000.0

        # Encoders → odometría
        odom.update(le.getValue(), re.getValue())

        # Sensores
        sv_raw = [s.getValue() for s in sensors]
        sv = sensor_filter.update(sv_raw)
        if max(sv_raw) > DANGER_THRESH:
            near_collisions += 1

        # Calcular velocidades (con valores filtrados)
        vl, vr, action = nav.compute(odom.x, odom.y, odom.theta, sv)
        lm.setVelocity(vl)
        rm.setVelocity(vr)

        # Log
        trajectory.append((odom.x, odom.y))
        data_log.append({
            'step': step, 'time': round(t, 4),
            'x': round(odom.x, 6), 'y': round(odom.y, 6),
            'theta': round(odom.theta, 6),
            'wp_idx': nav.wp_idx, 'action': action,
            'ps0': round(sv[0], 2), 'ps1': round(sv[1], 2),
            'ps2': round(sv[2], 2), 'ps3': round(sv[3], 2),
            'ps4': round(sv[4], 2), 'ps5': round(sv[5], 2),
            'ps6': round(sv[6], 2), 'ps7': round(sv[7], 2),
            'ps0_raw': round(sv_raw[0], 2), 'ps1_raw': round(sv_raw[1], 2),
            'ps2_raw': round(sv_raw[2], 2), 'ps3_raw': round(sv_raw[3], 2),
            'ps4_raw': round(sv_raw[4], 2), 'ps5_raw': round(sv_raw[5], 2),
            'ps6_raw': round(sv_raw[6], 2), 'ps7_raw': round(sv_raw[7], 2),
            'vl': round(vl, 4), 'vr': round(vr, 4),
        })

        if step % 100 == 0 or action == "GOAL_REACHED":
            print(f"  [t={t:6.1f}s] pos=({odom.x:+.3f}, {odom.y:+.3f}) "
                  f"θ={math.degrees(odom.theta):+6.1f}° "
                  f"wp={nav.wp_idx}/{len(wp_world)} {action}")

        if nav.reached_goal:
            print(f"\n  ✓ ¡META ALCANZADA en {t:.1f} s!")
            break

    lm.setVelocity(0); rm.setVelocity(0)
    robot.step(TIME_STEP)

    if not nav.reached_goal:
        print(f"\n  ✗ Tiempo agotado ({step * TIME_STEP / 1000:.1f} s)")

    # ══════════════════════════════════════════════════════════════
    #  FASE 4 — Métricas y resultados
    # ══════════════════════════════════════════════════════════════
    print("\n[4/4] Calculando métricas…")
    print("=" * 60)

    total_time = step * TIME_STEP / 1000.0
    traj_len = sum(
        math.sqrt((trajectory[i][0] - trajectory[i - 1][0]) ** 2 +
                  (trajectory[i][1] - trajectory[i - 1][1]) ** 2)
        for i in range(1, len(trajectory)))
    final_err = math.sqrt((odom.x - sc['goal_x']) ** 2 +
                          (odom.y - sc['goal_y']) ** 2)

    actions = [d['action'] for d in data_log]
    acounts = {}
    for a in actions:
        acounts[a] = acounts.get(a, 0) + 1

    diff_pct = (abs(traj_len - planned_len) / planned_len * 100
                if planned_len > 0 else 0)

    print(f"\n  RESULTADOS — Escenario {scenario_name.upper()}")
    print(f"  {'─' * 50}")
    print(f"  Meta alcanzada:          {'Sí' if nav.reached_goal else 'No'}")
    print(f"  Tiempo total:            {total_time:.1f} s")
    print(f"  Ruta planificada:        {planned_len:.3f} m")
    print(f"  Trayectoria ejecutada:   {traj_len:.3f} m")
    print(f"  Distancia odométrica:    {odom.total_distance:.3f} m")
    print(f"  Diferencia plan/real:    {abs(traj_len - planned_len):.3f} m ({diff_pct:.1f}%)")
    print(f"  Error final posición:    {final_err:.4f} m")
    print(f"  Casi-colisiones:         {near_collisions}")
    print(f"  Waypoints alcanzados:    {nav.wp_idx}/{len(wp_world)}")
    print(f"\n  Acciones:")
    for an, cnt in sorted(acounts.items()):
        print(f"    {an}: {cnt} ({cnt / len(actions) * 100:.1f}%)")

    # ── Guardar datos ────────────────────────────────────────────
    base = os.path.dirname(os.path.abspath(__file__))

    csv_path = os.path.join(base, f'data_{scenario_name}.csv')
    if data_log:
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=data_log[0].keys())
            w.writeheader(); w.writerows(data_log)
        print(f"\n  Datos → {csv_path}")

    # Ruta planificada + trayectoria real
    pp = os.path.join(base, f'path_{scenario_name}.csv')
    with open(pp, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['type', 'x', 'y'])
        for wx, wy in wp_world:
            w.writerow(['planned', round(wx, 6), round(wy, 6)])
        s = max(1, len(trajectory) // 500)
        for i in range(0, len(trajectory), s):
            w.writerow(['actual', round(trajectory[i][0], 6),
                        round(trajectory[i][1], 6)])
    print(f"  Ruta → {pp}")

    # Grilla
    gp = os.path.join(base, f'grid_{scenario_name}.csv')
    with open(gp, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['row', 'col'])
        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
                if grid.grid[r][c]:
                    w.writerow([r, c])
    print(f"  Grilla → {gp}")

    # Métricas
    mp = os.path.join(base, f'metrics_{scenario_name}.txt')
    with open(mp, 'w') as f:
        f.write(f"escenario: {scenario_name}\n")
        f.write(f"meta_alcanzada: {'si' if nav.reached_goal else 'no'}\n")
        f.write(f"tiempo_total_s: {total_time:.1f}\n")
        f.write(f"ruta_planificada_m: {planned_len:.3f}\n")
        f.write(f"trayectoria_real_m: {traj_len:.3f}\n")
        f.write(f"error_final_m: {final_err:.4f}\n")
        f.write(f"casi_colisiones: {near_collisions}\n")
        f.write(f"waypoints: {nav.wp_idx}/{len(wp_world)}\n")
    print(f"  Métricas → {mp}")

    # ── Gráficos ─────────────────────────────────────────────────
    generate_plots(base, scenario_name, grid, raw_path,
                   wp_world, trajectory, data_log, start_cell, goal_cell)

    print("\n" + "=" * 60)
    print("Simulación finalizada.")
    print("=" * 60)

    while robot.step(TIME_STEP) != -1:
        pass


if __name__ == "__main__":
    main()
