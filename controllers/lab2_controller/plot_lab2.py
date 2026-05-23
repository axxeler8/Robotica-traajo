"""
Script de análisis y visualización para el Laboratorio 2.
Lee el archivo CSV generado por lab2_controller.py y genera gráficos
comparativos de las señales cruda, filtrada y estimada por Kalman.

Uso:
    python3 plot_lab2.py [ruta/al/lab2_data.csv]

Si no se especifica ruta, busca lab2_data.csv en el mismo directorio.
"""

import csv
import sys
import os

import matplotlib
matplotlib.use('Agg')  # Modo sin pantalla (genera archivos)
import matplotlib.pyplot as plt
import numpy as np


def load_data(csv_path):
    """Carga los datos del CSV y retorna listas por columna."""
    data = {
        'time': [], 'raw_frontal': [], 'filtered_frontal': [],
        'kalman_estimate': [], 'raw_left': [], 'raw_right': [],
        'kalman_gain': [], 'delta_s': [], 'action': []
    }
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['time'].append(float(row['time']))
            data['raw_frontal'].append(float(row['raw_frontal']))
            data['filtered_frontal'].append(float(row['filtered_frontal']))
            data['kalman_estimate'].append(float(row['kalman_estimate']))
            data['raw_left'].append(float(row['raw_left']))
            data['raw_right'].append(float(row['raw_right']))
            data['kalman_gain'].append(float(row['kalman_gain']))
            data['delta_s'].append(float(row['delta_s']))
            data['action'].append(row['action'])
    return data


def plot_comparison(data, output_dir):
    """Genera gráficos comparativos de las señales."""
    t = np.array(data['time'])
    raw = np.array(data['raw_frontal'])
    filt = np.array(data['filtered_frontal'])
    kalman = np.array(data['kalman_estimate'])
    gain = np.array(data['kalman_gain'])
    delta_s = np.array(data['delta_s'])
    left = np.array(data['raw_left'])
    right = np.array(data['raw_right'])

    # ------------------------------------------------------------------
    # Figura 1: Comparación de señales frontales
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Laboratorio 2 - Comparación de Señales de Distancia Frontal',
                 fontsize=14, fontweight='bold')

    # Subplot 1: Señal cruda
    axes[0].plot(t, raw, 'b-', alpha=0.6, linewidth=0.8, label='Señal cruda (sensores frontales)')
    axes[0].set_ylabel('Distancia (m)')
    axes[0].set_title('a) Lectura cruda de sensores frontales')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim(0, 0.35)

    # Subplot 2: Señal filtrada (exponencial)
    axes[1].plot(t, filt, 'g-', alpha=0.8, linewidth=0.8, label='Señal filtrada (exponencial)')
    axes[1].set_ylabel('Distancia (m)')
    axes[1].set_title('b) Señal con filtro exponencial (α = 0.3)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim(0, 0.35)

    # Subplot 3: Estimación Kalman
    axes[2].plot(t, kalman, 'r-', alpha=0.8, linewidth=0.8, label='Estimación Kalman')
    axes[2].set_xlabel('Tiempo (s)')
    axes[2].set_ylabel('Distancia (m)')
    axes[2].set_title('c) Distancia estimada con Filtro de Kalman')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 0.35)

    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'comparacion_senales.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: comparacion_senales.png")

    # ------------------------------------------------------------------
    # Figura 2: Superposición de las tres señales
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, raw, 'b-', alpha=0.4, linewidth=0.6, label='Cruda')
    ax.plot(t, filt, 'g-', alpha=0.7, linewidth=1.0, label='Filtro exponencial')
    ax.plot(t, kalman, 'r-', alpha=0.9, linewidth=1.2, label='Kalman')
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Distancia (m)')
    ax.set_title('Superposición: Señal cruda vs Filtrada vs Kalman', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 0.35)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'superposicion_senales.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: superposicion_senales.png")

    # ------------------------------------------------------------------
    # Figura 3: Ganancia de Kalman en el tiempo
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, gain, 'm-', linewidth=0.8)
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Ganancia K')
    ax.set_title('Evolución de la Ganancia de Kalman', fontweight='bold')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'ganancia_kalman.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: ganancia_kalman.png")

    # ------------------------------------------------------------------
    # Figura 4: Sensores laterales + desplazamiento
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle('Sensores Laterales y Desplazamiento del Robot', fontweight='bold')

    axes[0].plot(t, left, 'c-', alpha=0.7, linewidth=0.8, label='Sensor izquierdo')
    axes[0].plot(t, right, 'y-', alpha=0.7, linewidth=0.8, label='Sensor derecho')
    axes[0].set_ylabel('Distancia (m)')
    axes[0].set_title('Lecturas de sensores laterales')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, delta_s * 1000, 'b-', linewidth=0.8)  # en mm
    axes[1].set_xlabel('Tiempo (s)')
    axes[1].set_ylabel('Δs (mm)')
    axes[1].set_title('Desplazamiento lineal por paso (desde encoders)')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, 'laterales_y_desplazamiento.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: laterales_y_desplazamiento.png")


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        csv_path = os.path.join(os.path.dirname(__file__), 'lab2_data.csv')

    if not os.path.exists(csv_path):
        print(f"ERROR: No se encuentra el archivo {csv_path}")
        print("Ejecuta primero la simulación en Webots para generar los datos.")
        sys.exit(1)

    output_dir = os.path.dirname(csv_path)
    print(f"Cargando datos desde: {csv_path}")
    data = load_data(csv_path)
    print(f"Muestras cargadas: {len(data['time'])}")

    plot_comparison(data, output_dir)
    print("\n¡Análisis completado!")


if __name__ == '__main__':
    main()
