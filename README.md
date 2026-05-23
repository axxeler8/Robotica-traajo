# Laboratorio 1: Simulación de un Robot Móvil Diferencial en Webots

## Descripción

Simulación de un robot móvil diferencial (**e-puck**) en **Webots** para comprender su cinemática. El controlador ejecuta secuencialmente una serie de experimentos que demuestran cómo las velocidades de las ruedas determinan la trayectoria del robot.

### Modelo Cinemático

```
v = (vr + vl) / 2        # velocidad lineal
ω = (vr - vl) / L        # velocidad angular
```

Donde `vr` y `vl` son las velocidades de las ruedas derecha e izquierda, y `L` es la distancia entre ruedas.

## Cómo Ejecutar la Simulación

1. **Instalar Webots** desde [cyberbotics.com](https://cyberbotics.com/)
2. **Abrir Webots**
3. Ir a `File → Open World...`
4. Seleccionar el archivo `worlds/laboratorio1.wbt`
5. La simulación iniciará automáticamente, ejecutando todos los experimentos en secuencia
6. Observar la consola de Webots para ver las descripciones de cada fase

## Experimentos y Resultados

### Experimento 1: Movimiento Recto (`vr = vl = 3.0`)

El robot avanza en **línea recta**. Ambas ruedas giran a la misma velocidad, por lo que `ω = 0` y no hay giro.

### Experimento 2: Trayectoria Curva (`vl = 2.0, vr = 4.0`)

El robot sigue una **trayectoria curva** hacia la izquierda. La rueda derecha gira más rápido que la izquierda, causando un giro con `ω > 0`.

### Experimento 3: Rotación en el Lugar (`vl = -3.0, vr = 3.0`)

El robot **gira sobre sí mismo** sin desplazarse. Las ruedas giran en sentidos opuestos, `v = 0` y `ω` es máximo.

### Extensión: Perturbaciones en los Actuadores

Se modifica de forma aleatoria la velocidad en cada iteración añadiendo ruido a un movimiento recto base (`v = 3.0`). Al comparar con la trayectoria ideal, se observa una _trayectoria con variaciones_, donde el robot experimenta constantes desviaciones.

### Desafío 1: Círculo

Manteniendo una diferencia constante entre las velocidades de las ruedas durante un tiempo prolongado, el robot traza un **círculo completo**.

### Desafío 2 (Opcional): Figura en 8

Dos círculos completos concatenados en **direcciones opuestas** con una velocidad tangencial forman el 8 (uno girando hacia la izquierda y otro hacia la derecha).

## Video Demostrativo

En el siguiente video se muestra la ejecución completa de la simulación, incluyendo todos los experimentos y desafíos:

[![Video de la simulación](https://img.shields.io/badge/Ver%20Video-Simulación%20Webots-blue?style=for-the-badge)](video/lab1_demo.mp4)

> El video se encuentra en la carpeta `video/` del repositorio.

## Preguntas de Análisis

**1. ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?**
El robot se mueve en **línea recta**, ya que la velocidad angular es cero (ω = 0).

**2. ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?**
El robot describe una **trayectoria curva**. Gira hacia el lado de la rueda más lenta. A mayor diferencia de velocidades, menor es el radio de curvatura.

**3. ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?**
El robot **rota sobre su propio eje** sin desplazarse linealmente (v = 0), ya que las velocidades se cancelan.

**4. ¿Qué tipo de movimiento permite dibujar un círculo?**
Un movimiento con **velocidades diferentes pero constantes** en ambas ruedas. Esto produce una velocidad angular constante que traza una circunferencia.

## Estructura del Proyecto

```
Robotica/
├── README.md
├── .gitignore
├── worlds/
│   └── laboratorio1.wbt
├── controllers/
│   └── lab1_controller/
│       └── lab1_controller.py
└── video/                ← video demostrativo de la simulación
```

## Herramientas

- **Webots** — Simulador de robots
- **Python** — Lenguaje del controlador

---

# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores

## Descripción

Sistema de navegación reactiva para el robot **e-puck** en Webots, utilizando
sensores de distancia y encoders de rueda. Se implementa un **filtro de Kalman**
escalar para estimar la distancia frontal al obstáculo más cercano, combinando
la predicción por movimiento (encoders) con la corrección por medición
(sensores frontales).

## Sensores utilizados

| Sensor               | Dispositivo Webots       | Función                                  |
|----------------------|--------------------------|------------------------------------------|
| Frontal izquierdo    | `ps7`                    | Medir obstáculos al frente               |
| Frontal derecho      | `ps0`                    | Medir obstáculos al frente               |
| Lateral izquierdo    | `ps5`                    | Decidir dirección de giro                |
| Lateral derecho      | `ps2`                    | Decidir dirección de giro                |
| Encoder izquierdo    | `left wheel sensor`      | Estimar desplazamiento lineal            |
| Encoder derecho      | `right wheel sensor`     | Estimar desplazamiento lineal            |

## Esquema de estimación (Filtro de Kalman escalar)

### Variable a estimar
`d_k`: distancia frontal al obstáculo más cercano en el instante k.

### Etapa de predicción
```
d̂_k⁻ = d̂_{k-1} - Δs_k
P_k⁻ = P_{k-1} + Q
```
Donde `Δs_k` es el avance lineal estimado desde los encoders:
`Δs = (Δθ_L · r + Δθ_R · r) / 2`

### Etapa de corrección
```
K_k = P_k⁻ / (P_k⁻ + R)
d̂_k = d̂_k⁻ + K_k · (z_k - d̂_k⁻)
P_k = (1 - K_k) · P_k⁻
```
Donde `z_k` es la medición del sensor frontal (`min(ps0, ps7)`).

### Parámetros
- `R = 0.001` — Varianza de la medición (ruido del sensor)
- `Q = 0.0001` — Varianza del proceso (incertidumbre del modelo)

## Lógica de navegación reactiva

```
if d̂_k > SAFE_DISTANCE (0.15 m):
    AVANZAR recto
else:
    if sensor_izquierdo < sensor_derecho:
        GIRAR a la DERECHA
    else:
        GIRAR a la IZQUIERDA
```

## Frecuencia de muestreo

- `Ts = 32 ms` (TIME_STEP)
- `fs = 31.25 Hz`
- Duración de simulación: 120 s (~3750 muestras)

## Cómo ejecutar

1. Abrir Webots
2. `File → Open World...` → `worlds/laboratorio2.wbt`
3. La simulación ejecuta la navegación reactiva automáticamente
4. Los datos se guardan en `controllers/lab2_controller/lab2_data.csv`

## Análisis de resultados

Después de la simulación, generar los gráficos comparativos:

```bash
cd controllers/lab2_controller
python3 plot_lab2.py
```

Esto genera:
- `comparacion_senales.png` — Señal cruda, filtrada y Kalman
- `superposicion_senales.png` — Las tres señales superpuestas
- `ganancia_kalman.png` — Evolución de la ganancia K
- `laterales_y_desplazamiento.png` — Sensores laterales y Δs

## Estructura del Proyecto (completa)

```
Robotica/
├── README.md
├── worlds/
│   ├── laboratorio1.wbt
│   └── laboratorio2.wbt
├── controllers/
│   ├── lab1_controller/
│   │   └── lab1_controller.py
│   └── lab2_controller/
│       ├── lab2_controller.py
│       └── plot_lab2.py
└── video/
```
