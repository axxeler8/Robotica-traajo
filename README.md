# Proyecto Final: Navegación Autónoma con Planificación de Rutas (A*) en Webots

**Asignatura:** ICI 4150 — Robótica y Sistemas Autónomos 2026-01  
**Línea seleccionada:** Línea A — Planificación de Rutas  

## Integrantes

| Nombre           |
| ---------------- |
| Martín Cevallos  |
| Carlos Abarza    |
| Matías Vergara   |

## Objetivo

Navegación autónoma de un robot e-puck en Webots desde un punto inicial hasta una meta en entornos con obstáculos, integrando control cinemático diferencial, sensores de proximidad, encoders, planificación global con A* y evitación reactiva.

---

## Robot y Sensores

**e-puck diferencial** — 2 ruedas (radio 0.0205 m, distancia entre ruedas 0.052 m, radio del robot ~0.037 m).

| Sensores                | Función                              |
| ----------------------- | ------------------------------------ |
| ps0–ps7 (proximidad IR) | Detección de obstáculos 360°         |
| Encoders de rueda       | Estimación de desplazamiento (odometría) |

---

## Escenarios de Prueba

### Simple
- Arena 2×2 m, 5 bloques dispersos
- Inicio: (-0.75, -0.75), Meta: (0.75, 0.75)

### Complejo
- Arena 2×2 m, 12 elementos (4 muros zigzag + 8 bloques)
- Inicio: (-0.75, -0.75), Meta: (0.75, 0.75)

---

## Algoritmo: A* sobre Grilla de Ocupación

Grilla de **40×40 celdas** (5 cm/celda) con **inflación de 1 celda** (~5 cm) para compensar el radio del robot.

- **Movimiento:** 8 direcciones (4 cardinales + 4 diagonales)
- **Heurística:** Distancia euclídea
- **Anti-corner-cutting:** Diagonal bloqueada si celda adyacente ocupada
- **Suavizado:** Reducción de waypoints con línea de visión (Bresenham)

### Control de Movimiento

Control proporcional de heading para seguir los waypoints:

```
ω = Kp · error_heading        (Kp = 3.0)
v_izq = v_base − ω·L/(2·R)    (v_base = 3.14 rad/s)
v_der = v_base + ω·L/(2·R)
```

### Evitación Reactiva

Los 8 sensores (filtrados con EMA, α=0.7) determinan 3 niveles de amenaza. La evasión tiene prioridad sobre el seguimiento de ruta:

| Nivel      | Sensor   | Acción                          |
| ---------- | -------- | ------------------------------- |
| Detección  | > 100    | Arco suave avanzando            |
| Crítico    | > 250    | Giro en el lugar                |
| Peligro    | > 500    | Retrocede girando (1.6 s)       |

Dirección: se compara suma de sensores izquierdos (ps4–ps7) vs derechos (ps0–ps3).

---

## Resultados

### Escenario Simple

| Métrica                        | Valor   |
| ------------------------------ | ------- |
| Meta alcanzada                 | Sí      |
| Celdas exploradas por A*       | 40      |
| Waypoints tras suavizado       | 3       |
| Longitud ruta planificada      | 2.195 m |
| Tiempo total                   | 79.3 s  |
| Longitud trayectoria ejecutada | 2.311 m |
| Diferencia plan/real           | 0.116 m |
| Error final de posición        | 0.039 m |
| Casi-colisiones                | 26      |

### Escenario Complejo

| Métrica                        | Valor   |
| ------------------------------ | ------- |
| Meta alcanzada                 | Sí      |
| Celdas exploradas por A*       | 45      |
| Waypoints tras suavizado       | 7       |
| Longitud ruta planificada      | 2.426 m |
| Tiempo total                   | 106.6 s |
| Longitud trayectoria ejecutada | 2.618 m |
| Diferencia plan/real           | 0.192 m |
| Error final de posición        | 0.039 m |
| Casi-colisiones                | 28      |

---

## Instrucciones para Ejecutar

1. Abrir Webots → `File → Open World...`
2. Seleccionar `worlds/proyecto_final_simple.wbt` o `worlds/proyecto_final_complejo.wbt`
3. La simulación inicia automáticamente. Ver resultados en la consola de Webots.

---

## Conclusiones

- A* proporciona rutas óptimas evitando que el robot quede atrapado en mínimos locales.
- La arquitectura de dos capas (planificación global + evasión reactiva) permite manejar la estrategia general y reaccionar a tiempo real.
- El control cinemático diferencial del Lab 1 y los sensores/encoders del Lab 2 se integran completamente.

### Limitaciones

- Error odométrico acumulativo en trayectorias largas.
- Grilla estática: obstáculos no previstos dependen solo de la evasión reactiva.
- Sensores IR de corto alcance (~5 cm) limitan el tiempo de reacción.
