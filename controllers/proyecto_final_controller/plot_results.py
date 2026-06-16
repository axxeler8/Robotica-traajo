#!/usr/bin/env python3
"""
Generador de gráficos para el Proyecto Final.
Uso:  python3 plot_results.py [simple|complejo]

Lee los archivos CSV generados por el controlador y produce gráficos PNG.
"""

import os
import sys
import csv
import math

def load_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))

def main():
    scenario = sys.argv[1] if len(sys.argv) > 1 else 'simple'
    base = os.path.dirname(os.path.abspath(__file__))

    data_file = os.path.join(base, f'data_{scenario}.csv')
    path_file = os.path.join(base, f'path_{scenario}.csv')
    grid_file = os.path.join(base, f'grid_{scenario}.csv')

    if not os.path.exists(data_file):
        print(f"No se encontró {data_file}. Ejecute primero la simulación.")
        return

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = load_csv(data_file)
    path_data = load_csv(path_file) if os.path.exists(path_file) else []
    grid_data = load_csv(grid_file) if os.path.exists(grid_file) else []

    # Reconstruir grilla
    occupied = set()
    for row in grid_data:
        occupied.add((int(row['row']), int(row['col'])))

    GRID_SIZE = 40
    CELL_SIZE = 0.05
    ARENA_HALF = 1.0

    def g2w(r, c):
        return (-ARENA_HALF + (c + 0.5) * CELL_SIZE,
                -ARENA_HALF + (r + 0.5) * CELL_SIZE)

    planned = [(float(r['x']), float(r['y'])) for r in path_data if r['type'] == 'planned']
    actual  = [(float(r['x']), float(r['y'])) for r in path_data if r['type'] == 'actual']

    ts  = [float(d['time']) for d in data]
    xs  = [float(d['x']) for d in data]
    ys  = [float(d['y']) for d in data]

    # ── 1. Ruta planificada vs trayectoria ───────────────────────
    fig, ax = plt.subplots(figsize=(8, 8))
    for r, c in occupied:
        wx, wy = g2w(r, c)
        ax.add_patch(plt.Rectangle(
            (wx - CELL_SIZE / 2, wy - CELL_SIZE / 2),
            CELL_SIZE, CELL_SIZE,
            facecolor='#cccccc', edgecolor='none'))
    if planned:
        ax.plot([p[0] for p in planned], [p[1] for p in planned],
                'b--o', lw=2, ms=4, label='Planificada')
    if actual:
        ax.plot([a[0] for a in actual], [a[1] for a in actual],
                'r-', lw=1, alpha=0.7, label='Real')
    ax.set(xlim=(-1.05, 1.05), ylim=(-1.05, 1.05), aspect='equal',
           xlabel='X (m)', ylabel='Y (m)',
           title=f'Planificada vs Real — {scenario.capitalize()}')
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(base, f'trajectory_{scenario}.png'),
                dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✓ trajectory_{scenario}.png")

    # ── 2. Sensores frontales ────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(ts, [float(d['ps7']) for d in data], alpha=.7, label='ps7')
    axes[0].plot(ts, [float(d['ps0']) for d in data], alpha=.7, label='ps0')
    axes[0].axhline(100, color='orange', ls='--', alpha=.5, label='Detección')
    axes[0].axhline(250, color='red', ls='--', alpha=.5, label='Crítico')
    axes[0].set(ylabel='Valor crudo', title='Sensores Frontales')
    axes[0].legend(fontsize=8); axes[0].grid(True, alpha=.3)

    axes[1].plot(ts, [float(d['ps6']) for d in data], alpha=.7, label='ps6')
    axes[1].plot(ts, [float(d['ps1']) for d in data], alpha=.7, label='ps1')
    axes[1].set(ylabel='Valor crudo', xlabel='Tiempo (s)', title='Sensores Laterales')
    axes[1].legend(fontsize=8); axes[1].grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(base, f'sensors_{scenario}.png'),
                dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✓ sensors_{scenario}.png")

    # ── 3. Heading + acciones ────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    axes[0].plot(ts, [math.degrees(float(d['theta'])) for d in data], 'g-', alpha=.7)
    axes[0].set(ylabel='Heading (°)', title='Orientación')
    axes[0].grid(True, alpha=.3)

    amap = {'FOLLOWING': 0, 'AVOID_LEFT': 1, 'AVOID_RIGHT': 2, 'GOAL_REACHED': 3}
    axes[1].plot(ts, [amap.get(d['action'], -1) for d in data], alpha=.7)
    axes[1].set_yticks([0, 1, 2, 3])
    axes[1].set_yticklabels(['Siguiendo', 'Evit. izq', 'Evit. der', 'Meta'])
    axes[1].set(xlabel='Tiempo (s)', title='Acciones')
    axes[1].grid(True, alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(base, f'heading_{scenario}.png'),
                dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✓ heading_{scenario}.png")

    print(f"\nTodos los gráficos generados para escenario '{scenario}'.")


if __name__ == '__main__':
    main()
