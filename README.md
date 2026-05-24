# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión de Sensores

## Integrantes

| Nombre          |
| --------------- |
| Martín Cevallos |
| Carlos Abarza   |
| Matías Vergara  |

## Objetivo

Implementar un sistema de navegación reactiva para el robot **e-puck** en Webots,
utilizando sensores de distancia y encoders de rueda, aplicando filtrado sobre
las mediciones y empleando un **filtro de Kalman** escalar para estimar la
distancia frontal a obstáculos y mejorar la toma de decisiones.

## Sensores utilizados

| Sensor            | Dispositivo Webots   | Función                       |
| ----------------- | -------------------- | ----------------------------- |
| Frontal izquierdo | `ps7`                | Medir obstáculos al frente    |
| Frontal derecho   | `ps0`                | Medir obstáculos al frente    |
| Lateral izquierdo | `ps5`                | Decidir dirección de giro     |
| Lateral derecho   | `ps2`                | Decidir dirección de giro     |
| Encoder izquierdo | `left wheel sensor`  | Estimar desplazamiento lineal |
| Encoder derecho   | `right wheel sensor` | Estimar desplazamiento lineal |

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

- `R = 0.0005` — Varianza de la medición (ruido del sensor)
- `Q = 0.0001` — Varianza del proceso (incertidumbre del modelo)

## Lógica de navegación reactiva

```
if z_frontal < CRITICAL_DISTANCE (0.008 m):   # Emergencia: sensor crudo
    GIRAR (usando sensores laterales)
elif d̂_k > SAFE_DISTANCE (0.04 m)
     AND filtered > SAFE_DISTANCE:             # Kalman + filtro: vía libre
    AVANZAR recto
else:                                         # Obstáculo detectado
    GIRAR (usando sensores laterales)
```

> **Nota:** Los sensores IR del e-puck retornan valores de proximidad (mayor = más
> cerca), no distancia en metros. Se convierte internamente usando la lookup table
> del sensor: `distancia = 0.05 × (1 - valor/1024)`. El robot avanza **solo** si
> tanto la estimación Kalman como la señal filtrada indican vía libre, evitando
> colisiones por retardo del filtro. Además, se usa _innovation gating_: si la
> diferencia entre medición y predicción supera 10 cm, el Kalman se resetea
> confiando directamente en la medición.

## Frecuencia de muestreo

- `Ts = 32 ms` (TIME_STEP)
- `fs = 31.25 Hz`
- Duración de simulación: 120 s (~3750 muestras)

## Escenarios de prueba

### Escenario Simple (`worlds/laboratorio2.wbt`)

Arena de **0.65×0.65 m** con **4 obstáculos dispersos**. El robot parte desde el centro de la arena mirando hacia +X.

| Obstáculo | Tipo | Color | Posición (x, y) | Tamaño |
|-----------|------|-------|------------------|--------|
| obstacle_1 | Caja | Rojo | (0.15, -0.05) | 4×4 cm |
| obstacle_2 | Caja | Verde | (-0.12, 0.10) | 4×4 cm |
| obstacle_3 | Caja | Azul | (0.10, 0.15) | 5×3 cm |
| obstacle_4 | Cilindro | Amarillo | (-0.08, -0.12) | r=2 cm |

Este diseño evalúa la evitación de obstáculos puntuales en espacio abierto, ideal para comparar señal cruda vs Kalman con aproximaciones individuales a cada obstáculo.

### Escenario Complejo (`worlds/laboratorio2_complex.wbt`)

Arena de **0.8×0.8 m** con un **pasillo central con chicane** y **7 obstáculos dispersos**. El robot parte desde (0, -0.30) mirando hacia +Y.

**Pasillo central:**
- Dos paredes paralelas en x=±0.06 (ancho del corredor: **12 cm**), de 20 cm de largo
- Chicane 1 (naranja): pared lateral en (-0.06, 0.15), fuerza giro a la derecha
- Chicane 2 (azul): pared lateral en (0.06, 0.25), fuerza giro a la izquierda

**Obstáculos en zona abierta:**

| Obstáculo | Tipo | Color | Posición (x, y) | Tamaño |
|-----------|------|-------|------------------|--------|
| obstacle_big_1 | Caja | Rojo | (-0.25, 0.25) | 6×6 cm |
| obstacle_big_2 | Caja | Verde | (0.25, 0.20) | 5×8 cm |
| obstacle_cyl_1 | Cilindro | Amarillo | (0.15, -0.10) | r=3 cm |
| obstacle_wall_1 | Caja | Violeta | (-0.25, -0.20) | 4×10 cm |
| obstacle_wall_2 | Caja | Gris | (0.20, 0.05) | 12×1 cm |
| obstacle_cyl_2 | Cilindro | Cian | (-0.18, -0.05) | r=2 cm |
| obstacle_exit | Caja | Rosa | (0.0, 0.35) | 8×3 cm |

Este diseño combina navegación en espacio confinado (pasillo con chicane) y evitación en zona abierta, permitiendo analizar la adaptación del Kalman a distintos niveles de exigencia.


## Cómo ejecutar

1. Abrir Webots
2. **Escenario simple**: `File → Open World...` → `worlds/laboratorio2.wbt`
3. **Escenario complejo**: `File → Open World...` → `worlds/laboratorio2_complex.wbt`
4. La simulación ejecuta la navegación reactiva automáticamente (120 s)
5. Los datos se guardan como `lab2_data_simple.csv` o `lab2_data_complex.csv`
6. Para generar los gráficos de análisis:

```bash
cd controllers/lab2_controller
python3 plot_lab2.py            # Ambos escenarios
python3 plot_lab2.py simple     # Solo escenario simple
python3 plot_lab2.py complex    # Solo escenario complejo
```

## Análisis de las señales registradas

Se registraron 3750 muestras por escenario (120 s de simulación a 31.25 Hz). A continuación se describe el comportamiento observado en cada tipo de señal.

### Señal cruda de los sensores frontales

La señal cruda corresponde al mínimo entre los sensores `ps0` (frontal derecho) y `ps7` (frontal izquierdo), convertida a metros mediante la lookup table del sensor IR del e-puck.

**Comportamiento observado:**

- La señal presenta una **alta variabilidad** con transiciones abruptas entre el valor máximo (0.30 m, sin obstáculo) y valores cercanos a cero cuando el robot detecta un obstáculo.
- En el escenario simple, la desviación estándar es de **0.100 m** con un coeficiente de variación del **18.4%** (excluyendo muestras saturadas), lo que refleja un ruido significativo inherente a los sensores IR de proximidad.
- En el escenario complejo, la variabilidad aumenta (CV = **23.3%**), consistente con la mayor cantidad de obstáculos y aproximaciones frecuentes.
- Se registraron **638 cambios bruscos** (>0.02 m entre muestras consecutivas) en el escenario simple y **614** en el complejo, indicando que la señal cruda oscila rápidamente al pasar cerca de obstáculos.
- El valor mínimo registrado fue **0.0197 m** (simple) y **0.0138 m** (complejo), confirmando que el robot detectó obstáculos a distancias muy cortas.
- Solo el **1.5%** (simple) y **2.9%** (complejo) de las muestras cayeron por debajo del umbral de peligro de 0.04 m.

### Señal con filtro exponencial (α = 0.3)

Se aplicó un filtro exponencial de primer orden con constante de suavizado α = 0.3:

```
filtered_k = α · z_k + (1 - α) · filtered_{k-1}
```

**Comparación con la señal cruda:**

- El filtro **reduce la variabilidad de alta frecuencia**, suavizando los picos instantáneos de la señal cruda.
- Sin embargo, introduce un **retardo notable**: la señal filtrada reacciona con varios pasos de retraso ante cambios bruscos de distancia (por ejemplo, al entrar al rango de un obstáculo).
- En el escenario simple, el valor mínimo filtrado fue **0.0313 m** versus **0.0197 m** crudo, mostrando que el filtro no alcanza a reflejar la proximidad real cuando el robot se acerca rápidamente.
- En el escenario complejo, el mínimo filtrado fue **0.0150 m** vs **0.0138 m** crudo, una diferencia menor que indica que en pasillos estrechos el robot permanece más tiempo cerca de los obstáculos, permitiendo que el filtro converja.
- Este retardo es potencialmente **peligroso para la navegación**: si el robot se basara únicamente en la señal filtrada, podría chocar antes de que el filtro refleje la cercanía real del obstáculo. Por esta razón, la lógica de navegación también verifica la señal cruda como condición de emergencia.

### Estimación con filtro de Kalman

El filtro de Kalman escalar fusiona dos fuentes de información:

- **Predicción**: distancia anterior menos el avance estimado con encoders (`d̂_k⁻ = d̂_{k-1} - Δs_k`)
- **Corrección**: ajuste con la medición real de los sensores frontales (`d̂_k = d̂_k⁻ + K_k · (z_k - d̂_k⁻)`)

**Análisis de la ganancia de Kalman:**

- La ganancia K oscila entre **0.358** y **0.953**, con una media de **~0.40** en ambos escenarios.
- En régimen estacionario (robot en movimiento constante), K converge a valores cercanos a **0.36**, lo que indica que el filtro pondera un **~36% la medición** y un **~64% la predicción**, confiando mayoritariamente en el modelo de movimiento.
- Cuando ocurre un _innovation gating_ (diferencia entre medición y predicción > 10 cm), K se dispara a **~0.95** y el filtro se resetea confiando casi exclusivamente en la medición. Esto ocurre al aparecer o desaparecer un obstáculo súbitamente del campo de detección.
- La estimación Kalman presenta una desviación estándar muy similar a la señal cruda (**0.0107 m** vs **0.0108 m** en simple), pero con transiciones **más suaves** gracias a la integración de la predicción por encoders.

### Desplazamiento estimado desde encoders

El avance lineal del robot se estima a partir de la diferencia angular de los encoders de cada rueda:

```
Δs = (Δθ_L · r + Δθ_R · r) / 2
```

donde `r = 0.0205 m` es el radio de la rueda del e-puck.

**Resultados:**

- El desplazamiento promedio por paso fue de **~0.74 mm** (consistente con la velocidad de avance de 1.2 rad/s × 0.0205 m × 0.032 s ≈ 0.79 mm).
- El desplazamiento total acumulado fue de **2.77 m** (simple) y **2.76 m** (complejo) en 120 segundos.
- Los encoders registraron una rotación total de **112.4 rad** (izquierdo) y **158.4 rad** (derecho) en el escenario simple, con una diferencia de **45.9 rad** que refleja los giros realizados durante la navegación.
- En el escenario complejo, la diferencia entre encoders fue menor (**38.7 rad**), indicando giros de menor amplitud pero más frecuentes por los pasillos estrechos.

**Fuentes de error:**

- Deslizamiento de las ruedas durante los giros bruscos.
- Resolución finita de los encoders y discretización temporal del paso de simulación.
- La estimación asume movimiento puramente lineal, sin compensar el giro diferencial.

## Gráficos

### Escenario Simple

| Gráfico                                                                                | Descripción                       |
| -------------------------------------------------------------------------------------- | --------------------------------- |
| ![Comparación Simple](controllers/lab2_controller/simple_comparacion_senales.png)      | Señal cruda, filtrada y Kalman    |
| ![Superposición Simple](controllers/lab2_controller/simple_superposicion_senales.png)  | Superposición de las tres señales |
| ![Ganancia Kalman Simple](controllers/lab2_controller/simple_ganancia_kalman.png)      | Evolución de la ganancia K        |
| ![Laterales Simple](controllers/lab2_controller/simple_laterales_y_desplazamiento.png) | Sensores laterales y Δs           |
| ![Encoders Simple](controllers/lab2_controller/simple_encoders_y_sensores_crudos.png)  | Encoders y sensores crudos        |

### Escenario Complejo

| Gráfico                                                                                   | Descripción                       |
| ----------------------------------------------------------------------------------------- | --------------------------------- |
| ![Comparación Complejo](controllers/lab2_controller/complex_comparacion_senales.png)      | Señal cruda, filtrada y Kalman    |
| ![Superposición Complejo](controllers/lab2_controller/complex_superposicion_senales.png)  | Superposición de las tres señales |
| ![Ganancia Kalman Complejo](controllers/lab2_controller/complex_ganancia_kalman.png)      | Evolución de la ganancia K        |
| ![Laterales Complejo](controllers/lab2_controller/complex_laterales_y_desplazamiento.png) | Sensores laterales y Δs           |
| ![Encoders Complejo](controllers/lab2_controller/complex_encoders_y_sensores_crudos.png)  | Encoders y sensores crudos        |

## Resultados en los escenarios de prueba

### Escenario Simple

| Métrica                    | Observación                                                                                                                                                                                                                                  |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Estabilidad del movimiento | Alta estabilidad: el robot avanzó el **94.4%** del tiempo en línea recta, con correcciones laterales suaves. La trayectoria fue fluida con transiciones graduales entre avance y giro.                                                       |
| Giros innecesarios         | Se registraron **55 cambios de acción** en total (transiciones entre FORWARD/TURN). Los giros se concentran en las proximidades de los 4 obstáculos, sin oscilaciones innecesarias en zonas libres.                                          |
| Evitación de colisiones    | El robot **no colisionó** con ningún obstáculo. La distancia mínima registrada fue de **0.0197 m**, superior a la distancia de contacto del e-puck. El sistema de emergencia (distancia crítica) activó giros preventivos de forma efectiva. |
| Acciones registradas       | FORWARD: 3539 (94.4%), TURN_LEFT: 188 (5.0%), TURN_RIGHT: 23 (0.6%)                                                                                                                                                                          |

### Escenario Complejo

| Métrica                    | Observación                                                                                                                                                                                                                                                                   |
| -------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Estabilidad del movimiento | Buena estabilidad considerando la complejidad del entorno: **94.0%** del tiempo en avance. En los pasillos estrechos (chicane) se observaron correcciones laterales más frecuentes pero aún controladas.                                                                      |
| Giros innecesarios         | **40 cambios de acción**, paradójicamente menos que en el escenario simple. Esto se debe a que en el pasillo estrecho los giros son más prolongados (secuencias de evitación más largas) y el robot no oscila entre avanzar y girar.                                          |
| Evitación de colisiones    | El robot **navegó exitosamente** el pasillo con chicane sin colisionar. La distancia mínima frontal fue **0.0138 m**, menor que en el escenario simple, reflejando la mayor cercanía a las paredes del corredor. El 2.9% de las muestras estuvo en zona de peligro (<0.04 m). |
| Acciones registradas       | FORWARD: 3525 (94.0%), TURN_LEFT: 215 (5.7%), TURN_RIGHT: 10 (0.3%)                                                                                                                                                                                                           |

### Comparación entre escenarios

| Aspecto                               | Escenario Simple | Escenario Complejo   |
| ------------------------------------- | ---------------- | -------------------- |
| Muestras en zona de peligro (<0.04 m) | 1.5%             | 2.9% (casi el doble) |
| Cambios de acción                     | 55               | 40                   |
| Distancia mínima frontal              | 0.0197 m         | 0.0138 m             |
| Giros a la izquierda                  | 188 (5.0%)       | 215 (5.7%)           |
| CV señal cruda (sin saturación)       | 18.4%            | 23.3%                |
| Desplazamiento total                  | 2.77 m           | 2.76 m               |

**Análisis:**

- El escenario complejo generó **más situaciones de peligro** (2.9% vs 1.5%), con distancias mínimas más reducidas por los pasillos estrechos y las chicanes.
- La ganancia de Kalman se comportó de forma similar en ambos escenarios (media ~0.40), indicando que los parámetros `R` y `Q` están bien calibrados para ambos tipos de entorno.
- La **fusión sensorial fue más crítica en el escenario complejo**: la mayor variabilidad de la señal cruda (CV 23.3% vs 18.4%) hace que las decisiones basadas solo en la señal cruda sean menos confiables. El Kalman estabilizó estas oscilaciones, permitiendo al robot mantener una trayectoria fluida en pasillos confinados.
- En el escenario simple, los obstáculos son puntuales y dispersos, por lo que la señal cruda alcanza para navegar. En el complejo, la continuidad de las paredes del pasillo requiere una estimación más estable que solo el Kalman proporciona.

## Conclusiones

1. **Ventajas del filtro de Kalman frente a la señal cruda y el filtro exponencial:**
   El filtro de Kalman combina la información de movimiento (encoders) con la percepción del entorno (sensores IR), produciendo una estimación que es a la vez reactiva y estable. A diferencia de la señal cruda, que presenta transiciones abruptas y ruido de alta frecuencia, el Kalman suaviza la estimación sin perder reactividad ante obstáculos reales gracias al mecanismo de _innovation gating_. Comparado con el filtro exponencial, el Kalman no introduce retardo significativo porque su predicción por encoders anticipa los cambios de distancia antes de que el sensor los confirme.

2. **Efecto del entorno en la navegación reactiva:**
   El entorno complejo demostró ser significativamente más exigente: duplicó las muestras en zona de peligro y redujo la distancia mínima frontal en un 30%. Sin embargo, el sistema logró navegar ambos escenarios sin colisiones, validando la robustez de la fusión sensorial. En espacios confinados, la corrección lateral proporcional fue clave para mantener la trayectoria centrada.

3. **Limitaciones observadas:**
   - Los sensores IR del e-puck tienen un rango máximo de ~5 cm, lo que limita la anticipación ante obstáculos lejanos.
   - La estimación del avance por encoders asume movimiento lineal y no compensa el giro diferencial, introduciendo error acumulativo.
   - El _innovation gating_ con umbral fijo (10 cm) puede ser demasiado agresivo en escenarios con cambios graduales de distancia.
   - La conversión sensor→distancia depende de la lookup table del fabricante, que puede no ser perfectamente lineal.

4. **Mejoras propuestas:**
   - Incorporar más sensores (ps1, ps6) para eliminar puntos ciegos angulares entre los sensores frontales y laterales.
   - Implementar un filtro de Kalman extendido (EKF) que considere el giro diferencial en la predicción.
   - Adaptar los parámetros Q y R dinámicamente según la velocidad del robot y la variabilidad reciente de las mediciones.
   - Añadir un mapa local con memoria de obstáculos recientes para evitar oscilaciones en esquinas cerradas.

## Estructura del Proyecto (completa)

```
Robotica/
├── README.md
├── worlds/
│   ├── laboratorio1.wbt
│   ├── laboratorio2.wbt              ← Escenario SIMPLE
│   └── laboratorio2_complex.wbt      ← Escenario COMPLEJO
├── controllers/
│   ├── lab1_controller/
│   │   └── lab1_controller.py
│   └── lab2_controller/
│       ├── lab2_controller.py
│       └── plot_lab2.py
└── video/
```
