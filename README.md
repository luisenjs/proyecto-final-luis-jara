# Planificación Global con Dijkstra - Robot Cuadrúpedo Unitree Go2

## Descripción del Proyecto

Este proyecto implementa un sistema completo de navegación autónoma para el robot cuadrúpedo Unitree Go2 en entorno simulado (Gazebo). El sistema integra dos componentes principales: generación de mapas mediante SLAM y planificación de trayectorias globales usando el algoritmo de Dijkstra implementado desde cero.

---

## Componentes del Proyecto

### 1. Mapeo con SLAM
Generación automática del mapa 2D del entorno usando SLAM Toolbox con el robot Unitree Go2 equipado con un LIDAR Velodyne.

### 2. Planificación Global
Cálculo de trayectorias óptimas usando el algoritmo de Dijkstra sobre el mapa de ocupación generado, creando waypoints navegables para el robot.

---

## Mapeo del Entorno

### Mapa Generado por SLAM

El siguiente mapa fue generado usando **SLAM Toolbox** en el mundo `small_house` de AWS RoboMaker:

![Mapa generado con SLAM](images/mapping/map_slam.png)

> **⚠️ Problema Identificado**: Durante el proceso de mapeo se detectó un problema crítico con los marcos de referencia en RViz. Al mover el robot con teleoperación, el mapa se desplaza según se mueve el robot en lugar de mantenerse fijo, generando un mapeo incorrecto y sobrepuesto.

### Video del Proceso de Mapeo

📹 **[Ver video del mapeo en YouTube](TU_LINK_AQUI)**

El video muestra el comportamiento errático del mapeo donde:
- El robot rota en su posición para escanear el entorno inicialmente
- El mapa se genera pero con desplazamiento continuo

### Mapa Corregido Manualmente

Debido a los problemas de mapeo, se realizó una corrección manual del mapa basándose en las lecturas parciales del LIDAR:

![Mapa corregido manualmente](images/mapping/map.png)

**Proceso de corrección:**
1. Se tomó como base el mapa generado por SLAM con rotaciones estacionarias
2. Se completaron las zonas faltantes usando la información del entorno en Gazebo

### Proceso de Mapeo

**Comandos utilizados:**

```bash
# Terminal 1 - Lanzar Gazebo con robot y LIDAR
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house

# Terminal 2 - Lanzar SLAM Toolbox
ros2 launch go2_config slam.launch.py use_sim_time:=true

# Terminal 3 - Teleoperación del robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Guardar el mapa generado
cd images/mapping
ros2 run nav2_map_server map_saver_cli -f map_slam
```

### Créditos del Mapeo

- **Robot**: Unitree Go2 - Modelo URDF y configuración obtenida del repositorio [unitree-go2-ros2](https://github.com/widegonz/unitree-go2-ros2)

---

## Planificación Global de Trayectorias

### Visualización de la Planificación

La siguiente imagen muestra la trayectoria planificada usando el algoritmo de Dijkstra sobre el mapa corregido manualmente:

![Planificación con Dijkstra](images/planning/planning.png)

*La trayectoria en rojo representa el camino óptimo calculado desde la posición actual del robot hasta el objetivo definido en RViz usando la herramienta "2D Goal Pose".*

### Implementación del Algoritmo de Dijkstra

El algoritmo de Dijkstra se implementó desde cero en Python para calcular el camino más corto entre dos puntos en el mapa de ocupación 2D.

**Principio de funcionamiento:**

```python
1. Inicializar cola de prioridad con posición inicial (costo = 0)
2. Mientras la cola no esté vacía:
   a. Extraer celda con menor costo acumulado
   b. Si es el objetivo, reconstruir y retornar camino
   c. Para cada vecino navegable (4-conectividad):
      - Calcular nuevo costo
      - Si es mejor que el conocido, actualizar y agregar a cola
3. Si no se encuentra camino, retornar None
```

**Archivo principal:** `src/global_planner/global_planner/global_planner_dijkstra.py`

**Características de la implementación:**

- **Conectividad**: 4-vecinos (arriba, abajo, izquierda, derecha)
- **Costo**: Uniforme (costo = 1 por movimiento)
- **Estructura de datos**: `heapq` para cola de prioridad
- **Conversión de coordenadas**: Métodos para transformar entre coordenadas del mundo (metros) y celdas del mapa

**Complejidad computacional:**
- **Tiempo**: O((V + E) log V) donde V = número de celdas (~75,803 para mapa 343×221)
- **Espacio**: O(V) para almacenar costos, padres y cola de prioridad

### Generación de Waypoints

El planificador genera waypoints (puntos de ruta) espaciados uniformemente a lo largo de la trayectoria calculada.

**Parámetros de waypoints:**

```python
WAYPOINT_SPACING = 0.5  # metros entre waypoints
```

**¿Por qué 0.5 metros?**

1. **Resolución del mapa**: Con resolución de 0.05m/celda, 0.5m = 10 celdas, lo cual es suficiente para capturar cambios de dirección sin sobresaturar la lista de waypoints.

2. **Control del robot**: Un espaciado de 0.5m permite que el controlador del robot tenga tiempo suficiente para ajustar su trayectoria entre waypoints consecutivos.

3. **Eficiencia computacional**: Reducir el número de waypoints (vs. publicar cada celda del camino) minimiza la carga de procesamiento y comunicación en ROS 2.

4. **Suavizado implícito**: Al espaciar los waypoints, se reduce el efecto "escalonado" de seguir la grilla estrictamente, aunque el camino sigue siendo subóptimo en cuanto a suavidad.

**Proceso de generación:**

```python
def generate_waypoints(path, spacing=0.5):
    """
    Genera waypoints espaciados a lo largo del camino
    
    Args:
        path: Lista de celdas (i, j) del camino
        spacing: Distancia en metros entre waypoints
    
    Returns:
        Lista de coordenadas (x, y) en metros
    """
    waypoints = []
    accumulated_distance = 0.0
    
    for i in range(len(path) - 1):
        current = path[i]
        next_point = path[i + 1]
        
        # Calcular distancia euclidiana
        distance = calculate_distance(current, next_point)
        accumulated_distance += distance
        
        # Agregar waypoint cada 'spacing' metros
        if accumulated_distance >= spacing:
            waypoints.append(convert_to_world_coords(next_point))
            accumulated_distance = 0.0
    
    # Asegurar que el objetivo final esté incluido
    waypoints.append(convert_to_world_coords(path[-1]))
    
    return waypoints
```

### Uso del Planificador

**Lanzamiento automático (recomendado):**

El sistema cuenta con un launch file que inicia todos los componentes necesarios automáticamente:

```bash
# Terminal 1 - Lanzar Gazebo con el robot
ros2 launch go2_config gazebo_velodyne.launch.py world:=small_house

# Terminal 2 - Lanzar planificador (incluye map_server, planner y RViz)
ros2 launch global_planner planner_with_map.launch.py
```

El segundo comando inicia automáticamente:
- **map_server**: Carga el mapa corregido desde `go2_config/maps/map.yaml`
- **global_planner_dijkstra**: Nodo de planificación que espera objetivos
- **RViz2**: Interfaz gráfica preconfigurada con todas las visualizaciones

**Uso en RViz:**
1. Espera a que RViz abra automáticamente (ya configurado con Fixed Frame: `map`)
2. Verifica que se vean:
   - El mapa del entorno (gris = libre, negro = obstáculos)
   - El robot en su posición actual
3. Haz clic en la herramienta **"2D Goal Pose"** en la barra superior
4. Haz clic en el mapa donde quieras que vaya el robot
5. La trayectoria roja aparecerá automáticamente

**Lanzamiento con mapa personalizado:**

Si generaste un mapa nuevo con SLAM, puedes usarlo sin recompilar:

```bash
ros2 launch global_planner planner_with_map.launch.py map:=/ruta/completa/a/tu_mapa.yaml
```

**Topics publicados:**
- `/global_path` (nav_msgs/Path): Trayectoria planificada con waypoints espaciados 0.5m
- `/map` (nav_msgs/OccupancyGrid): Mapa de ocupación cargado por map_server

**Topics suscritos:**
- `/odom` (nav_msgs/Odometry): Posición actual del robot (desde Gazebo)
- `/goal_pose` (geometry_msgs/PoseStamped): Objetivo definido en RViz

---

## Notas Adicionales

### Estado Actual del Proyecto

**⚠️ Proyecto en Desarrollo (Work in Progress)**

Este proyecto se encuentra actualmente en fase de desarrollo y presenta los siguientes problemas conocidos:

#### Problemas Críticos

1. **Mapeo con marcos de referencia incorrectos**:
   - El frame `map` generado por SLAM no se mantiene fijo en el mundo
   - El robot parece moverse en su lugar mientras el mapa se desplaza
   - **Impacto**: Imposibilidad de generar mapas completos automáticamente

### Dependencias del Proyecto

**Sistema:**
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo 11

**Paquetes ROS 2:**
```bash
ros-humble-desktop
ros-humble-gazebo-ros-pkgs
ros-humble-navigation2
ros-humble-slam-toolbox
ros-humble-robot-localization
```

**Paquetes Python:**
- rclpy
- numpy
- heapq (biblioteca estándar)

### Compilación del Proyecto

```bash
cd ~/proyecto-final-luis-jara
colcon build
source install/setup.bash
```