"""
Script de análisis y visualización para el Laboratorio 2.
Lee el archivo CSV generado por lab2_controller.py y genera gráficos
comparativos de las señales cruda, filtrada y estimada por Kalman.

Uso:
    python3 plot_lab2.py simple          # Analiza escenario simple
    python3 plot_lab2.py complex         # Analiza escenario complejo
    python3 plot_lab2.py                 # Analiza ambos si existen
    python3 plot_lab2.py ruta/al/archivo.csv  # Archivo específico
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


def plot_comparison(data, output_dir, prefix="", scenario_label=""):
    """Genera gráficos comparativos de las señales."""
    t = np.array(data['time'])
    raw = np.array(data['raw_frontal'])
    filt = np.array(data['filtered_frontal'])
    kalman = np.array(data['kalman_estimate'])
    gain = np.array(data['kalman_gain'])
    delta_s = np.array(data['delta_s'])
    left = np.array(data['raw_left'])
    right = np.array(data['raw_right'])

    title_suffix = f" - Escenario {scenario_label}" if scenario_label else ""

    # ------------------------------------------------------------------
    # Figura 1: Comparación de señales frontales
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f'Laboratorio 2 - Comparación de Señales de Distancia Frontal{title_suffix}',
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
    fig.savefig(os.path.join(output_dir, f'{prefix}comparacion_senales.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: {prefix}comparacion_senales.png")

    # ------------------------------------------------------------------
    # Figura 2: Superposición de las tres señales
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, raw, 'b-', alpha=0.4, linewidth=0.6, label='Cruda')
    ax.plot(t, filt, 'g-', alpha=0.7, linewidth=1.0, label='Filtro exponencial')
    ax.plot(t, kalman, 'r-', alpha=0.9, linewidth=1.2, label='Kalman')
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Distancia (m)')
    ax.set_title(f'Superposición: Señal cruda vs Filtrada vs Kalman{title_suffix}', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 0.35)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, f'{prefix}superposicion_senales.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: {prefix}superposicion_senales.png")

    # ------------------------------------------------------------------
    # Figura 3: Ganancia de Kalman en el tiempo
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, gain, 'm-', linewidth=0.8)
    ax.set_xlabel('Tiempo (s)')
    ax.set_ylabel('Ganancia K')
    ax.set_title(f'Evolución de la Ganancia de Kalman{title_suffix}', fontweight='bold')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, f'{prefix}ganancia_kalman.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: {prefix}ganancia_kalman.png")

    # ------------------------------------------------------------------
    # Figura 4: Sensores laterales + desplazamiento
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle(f'Sensores Laterales y Desplazamiento del Robot{title_suffix}', fontweight='bold')

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
    fig.savefig(os.path.join(output_dir, f'{prefix}laterales_y_desplazamiento.png'), dpi=150)
    plt.close(fig)
    print(f"Gráfico guardado: {prefix}laterales_y_desplazamiento.png")


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Determinar qué escenarios procesar
    scenarios_to_process = []

    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg in ('simple', 'complex'):
            # Escenario específico
            csv_path = os.path.join(base_dir, f'lab2_data_{arg}.csv')
            if os.path.exists(csv_path):
                scenarios_to_process.append((arg, csv_path))
            else:
                print(f"ERROR: No se encuentra {csv_path}")
                print("Ejecuta primero la simulación en Webots con ese escenario.")
                sys.exit(1)
        else:
            # Ruta explícita a un CSV
            csv_path = arg
            if os.path.exists(csv_path):
                # Extraer nombre del escenario de la ruta
                fname = os.path.basename(csv_path)
                label = fname.replace('lab2_data_', '').replace('.csv', '')
                scenarios_to_process.append((label, csv_path))
            else:
                print(f"ERROR: No se encuentra {csv_path}")
                sys.exit(1)
    else:
        # Sin argumentos: buscar ambos escenarios
        for sc in ['simple', 'complex']:
            csv_path = os.path.join(base_dir, f'lab2_data_{sc}.csv')
            if os.path.exists(csv_path):
                scenarios_to_process.append((sc, csv_path))

        if not scenarios_to_process:
            # Fallback: buscar CSV genérico
            csv_path = os.path.join(base_dir, 'lab2_data.csv')
            if os.path.exists(csv_path):
                scenarios_to_process.append(('default', csv_path))

    if not scenarios_to_process:
        print("ERROR: No se encontraron archivos CSV.")
        print("Ejecuta primero la simulación en Webots para generar los datos.")
        print("\nUso: python3 plot_lab2.py [simple|complex|csv_path]")
        sys.exit(1)

    for scenario_name, csv_path in scenarios_to_process:
        print(f"\n{'='*55}")
        print(f"Procesando escenario: {scenario_name.upper()}")
        print(f"Archivo: {csv_path}")
        print(f"{'='*55}")

        data = load_data(csv_path)
        print(f"Muestras cargadas: {len(data['time'])}")

        # Guardar gráficos con prefijo del escenario
        prefix = f"{scenario_name}_"
        plot_comparison(data, base_dir, prefix=prefix, scenario_label=scenario_name.upper())

    print("\n¡Análisis completado!")


if __name__ == '__main__':
    main()
