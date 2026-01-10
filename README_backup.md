# Proyecto de Planificación Global - Robot Cuadrúpedo Unitree Go2

## Descripción del Proyecto

Este proyecto implementa un sistema de planificación de trayectorias global para un robot cuadrúpedo Unitree Go2 en entorno simulado (Gazebo). El sistema utiliza el algoritmo de Dijkstra implementado desde cero (sin Nav2) para calcular el camino más corto entre la posición actual del robot y un objetivo definido por el usuario en RViz.

El proyecto está dividido en dos fases principales:
1. **Mapeo del entorno**: Generación del mapa 2D del mundo `small_house` usando SLAM Toolbox
2. **Planificación global**: Cálculo de trayectorias óptimas sobre el mapa generado usando Dijkstra

### Características principales:
- ✅ Mapeo SLAM del entorno `small_house` en Gazebo
- ✅ Implementación pura del algoritmo de Dijkstra (sin librerías de planificación)
- ✅ Planificación sobre mapas de ocupación (OccupancyGrid)
- ✅ Visualización en tiempo real en RViz
- ✅ Recálculo automático de trayectorias al cambiar el objetivo

---

## Mapa Generado con SLAM

El siguiente mapa fue generado utilizando SLAM Toolbox en el mundo `small_house` de AWS RoboMaker:

![Mapa generado con SLAM](src/go2-luisjara/robots/configs/go2_config/maps/map.png)

> **Nota Importante**: El mapa generado automáticamente por SLAM presentó problemas de calidad y completitud. Para permitir pruebas de planificación, el mapa fue completado y corregido manualmente. Se está trabajando en mejorar el proceso de mapeo para obtener resultados más precisos de forma automática.

**Características del mapa:**
- **Dimensiones**: 343 × 221 celdas
- **Resolución**: 0.05 metros por celda (~17.15m × 11.05m)
- **Formato**: PGM (Portable Gray Map)
- **Origen**: Definido en `map.yaml`

---

## Resultados de Planificación

A continuación se muestra un ejemplo de la trayectoria planificada por el algoritmo de Dijkstra:

<!-- ![Trayectoria planificada en RViz](ruta/a/trajectory_image.png) -->

> **⚠️ Estado Actual**: No se ha logrado generar correctamente la planificación de trayectorias. El sistema está en desarrollo y se están depurando los problemas relacionados con:
> - Conversión de coordenadas entre el mapa y el sistema de referencia del mundo
> - Sincronización entre la odometría del robot y el mapa SLAM
> - Validación de celdas navegables en el mapa de ocupación
>
> Este proyecto continúa en desarrollo activo para lograr su funcionamiento completo.

---

## Algoritmo Utilizado

### **Dijkstra para Planificación Global**

#### Descripción del Algoritmo

El algoritmo de Dijkstra es un método de búsqueda en grafos que encuentra el camino más corto entre un nodo inicial y un nodo objetivo. En este proyecto, se aplica sobre una grilla 2D que representa el mapa del entorno.

**Principio de funcionamiento:**
1. Se mantiene una cola de prioridad ordenada por costo acumulado
2. Se expande el nodo con menor costo
3. Se actualizan los costos de los vecinos
4. Se repite hasta alcanzar el objetivo
5. Se reconstruye el camino desde el objetivo hasta el inicio

#### Variables Principales

**En la clase `GlobalPlannerDijkstra`:**

- `self.map_data` (numpy.ndarray): Matriz 2D que representa el mapa de ocupación
  - Valores < 50: Celdas libres (navegables)
  - Valores ≥ 50: Obstáculos
  - Valores = -1: Celdas desconocidas (tratadas como obstáculos)

- `self.map_resolution` (float): Resolución del mapa en metros por celda (0.05m)

- `self.map_origin` (tuple): Coordenadas (x, y) del origen del mapa en el mundo

- `self.map_width`, `self.map_height` (int): Dimensiones del mapa en celdas

- `self.current_pose` (tuple): Posición actual del robot (x, y) en coordenadas del mundo

- `self.goal_pose` (tuple): Posición objetivo (x, y) en coordenadas del mundo

**En el método `dijkstra()`:**

- `open_set` (heap): Cola de prioridad que almacena tuplas `(costo, (i, j))`
  - Se usa `heapq` para mantener el orden automáticamente
  - Extrae siempre el nodo con menor costo acumulado

- `g_score` (dict): Diccionario que mapea cada celda `(i, j)` a su costo mínimo conocido desde el inicio
  - Clave: `(i, j)` - Posición en la grilla
  - Valor: Costo acumulado (número de pasos)

- `came_from` (dict): Diccionario que almacena el "padre" de cada celda visitada
  - Permite reconstruir el camino completo al final
  - Clave: `(i, j)` - Celda actual
  - Valor: `(i_padre, j_padre)` - Celda desde la que se llegó

#### Modificaciones y Decisiones de Diseño

1. **Conectividad 4-vecinos**: Se eligió conectividad de 4 direcciones (arriba, abajo, izquierda, derecha) en lugar de 8-vecinos para simplificar el cálculo y evitar movimientos diagonal que podrían ser problemáticos para un robot cuadrúpedo.

2. **Costo uniforme**: Cada movimiento tiene costo = 1, independientemente de la dirección. Esto es adecuado para robots holonómicos o entornos sin preferencias direccionales.

3. **Umbral de ocupación**: Se usa 50% como umbral para determinar si una celda es navegable:
   ```python
   return cell_value >= 0 and cell_value < 50
   ```
   Valores en el rango [0, 49] se consideran libres, [50, 100] ocupados.

4. **QoS Transient Local**: El suscriptor del mapa usa `QoSDurabilityPolicy.TRANSIENT_LOCAL` para recibir el último mapa publicado aunque se suscriba después de que el mapa fue publicado:
   ```python
   map_qos = QoSProfile(
       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
       reliability=QoSReliabilityPolicy.RELIABLE,
       history=QoSHistoryPolicy.KEEP_LAST,
       depth=1
   )
   ```

5. **Conversión de coordenadas**: Se implementaron funciones bidireccionales para convertir entre:
   - Coordenadas del mundo (metros): `(x, y)`
   - Índices de grilla (celdas): `(i, j)`
   
   ```python
   i = int((y - origin_y) / resolution)
   j = int((x - origin_x) / resolution)
   ```

6. **Recálculo dinámico**: El planificador recalcula automáticamente la trayectoria cada vez que se define un nuevo objetivo mediante el callback `goal_callback()`.

#### Complejidad del Algoritmo

- **Temporal**: O((V + E) log V) donde V es el número de celdas y E el número de conexiones
  - Para un mapa de 343×221 celdas: ~75,803 nodos
  - Con 4-conectividad: E ≈ 4V
  
- **Espacial**: O(V) para almacenar `g_score`, `came_from` y `open_set`

---

### Limitaciones

1. **Solo planificación**: No incluye control del robot
2. **Costo uniforme**: No considera costos diferenciados por terreno
3. **Sin suavizado**: La trayectoria sigue la grilla estrictamente
4. **4-conectividad**: No permite movimientos diagonales
