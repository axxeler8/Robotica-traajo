# Laboratorio 1: Control Cinemático de un Robot Diferencial

## Integrantes

| Nombre          |
| --------------- |
| Martín Cevallos |
| Carlos Abarza   |
| Matías Vergara  |

## Objetivo

Implementar y analizar el control cinemático de un robot diferencial **e-puck** en Webots,
ejecutando movimientos rectos, trayectorias curvas, rotación en el lugar y figuras
geométricas (círculo, figura en 8), incluyendo una extensión con perturbaciones
aleatorias para estudiar la robustez del sistema.

## Robot

**e-puck diferencial** — 2 ruedas con motores independientes.

| Parámetro             | Valor    |
| --------------------- | -------- |
| Radio de rueda        | 0.0205 m |
| Distancia entre ruedas | 0.052 m  |
| Velocidad máxima      | 6.28 rad/s |
| Time step             | 16 ms    |

## Experimentos

| # | Experimento | vl (rad/s) | vr (rad/s) | Duración | Descripción |
|---|------------|------------|------------|----------|-------------|
| 1 | Movimiento recto | 3.0 | 3.0 | 6 s | Avance en línea recta |
| 2 | Trayectoria curva | 2.0 | 4.0 | 4 s | Curva suave hacia la izquierda |
| 3 | Rotación en el lugar | -3.0 | 3.0 | 3 s | Giro sobre su propio eje |
| 4 | Recto con perturbaciones | 3.0 | 3.0 | 6 s | Ruido aleatorio acumulativo (±0.8) |
| 5 | Círculo | 2.0 | 4.0 | calc. | Una vuelta completa |
| 6 | Figura en 8 | variable | variable | ~10.5 s | Trayectoria sinusoidal |

## Modelo cinemático

Para un robot diferencial con velocidad de rueda izquierda `vl` y derecha `vr`:

```
v = (vr + vl) / 2          velocidad lineal
ω = (vr - vl) / L          velocidad angular (L = 0.052 m)
```

## Extensión con perturbaciones

Se agrega ruido aleatorio acumulativo a las velocidades de las ruedas para simular
imperfecciones del terreno:

```
noise(t) = noise(t-1) + U(-0.03, 0.03)
noise ∈ [-0.8, 0.8]
v_rueda = v_nominal + noise
```

## Cómo ejecutar

1. Abrir Webots
2. `File → Open World...` → `worlds/laboratorio1.wbt`
3. La simulación ejecuta los 6 experimentos secuencialmente con pausas entre cada uno
4. Observar la consola de Webots para ver el progreso

## Estructura

```
├── README.md
├── controllers/lab1_controller/
│   └── lab1_controller.py
├── worlds/
│   └── laboratorio1.wbt
└── video/
    └── lab1_demo.mp4
```
