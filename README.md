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
# Robotica-traajo
