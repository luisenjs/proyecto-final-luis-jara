# Global Planner - Dijkstra

Nodo de planificación global implementado desde cero usando el algoritmo de Dijkstra.

## Características

- ✅ Implementación pura de Dijkstra (sin Nav2 ni librerías externas)
- ✅ Planificación sobre mapas de ocupación (OccupancyGrid)
- ✅ Conectividad 4-vecinos
- ✅ Visualización en RViz
- ✅ Recálculo automático al definir nuevo objetivo

## Requisitos

- ROS 2 Humble
- Mapa generado previamente con SLAM Toolbox
- Gazebo + RViz

## Instalación

```bash
cd ~/Desktop/Example/luisjara_ws
colcon build --packages-select global_planner
source install/setup.bash
```

## Uso

### 1. Lanzar la simulación con el mapa guardado

**Terminal 1 - Gazebo:**
```bash
cd ~/Desktop/Example/luisjara_ws
source install/setup.bash
ros2 launch go2_config gazebo.launch.py
```

**Terminal 2 - Cargar mapa y Nav2 (sin SLAM):**
```bash
source install/setup.bash
ros2 launch go2_config navigate.launch.py map:=/ruta/al/mapa.yaml
```

O si solo quieres visualización sin Nav2:

```bash
# Publicar el mapa
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/ruta/al/mapa.yaml

# Lanzar RViz
rviz2
```

### 2. Lanzar el planificador global

**Terminal 3 - Planificador Dijkstra:**
```bash
source install/setup.bash
ros2 run global_planner global_planner_dijkstra
```

### 3. Configurar RViz

1. Abre RViz (si no está abierto): `rviz2`
2. Configura el Fixed Frame a `map`
3. Agrega visualizaciones:
   - **Add** → **Map** → Topic: `/map`
   - **Add** → **RobotModel**
   - **Add** → **Path** → Topic: `/global_path` → Color: Verde/Azul
   - **Add** → **TF**

4. Habilita la herramienta **2D Goal Pose** en la barra superior

### 4. Planificar trayectoria

1. En RViz, haz clic en el botón **"2D Goal Pose"** (flecha verde en la barra superior)
2. Haz clic en el mapa donde quieres que vaya el robot
3. Arrastra para definir la orientación (opcional)
4. El planificador calculará automáticamente la trayectoria
5. La trayectoria aparecerá como una línea verde/azul en RViz

## Topics

### Suscripciones
- `/map` (`nav_msgs/OccupancyGrid`) - Mapa del entorno
- `/odom` (`nav_msgs/Odometry`) - Odometría del robot
- `/goal_pose` (`geometry_msgs/PoseStamped`) - Objetivo definido en RViz

### Publicaciones
- `/global_path` (`nav_msgs/Path`) - Trayectoria planificada

## Algoritmo

### Dijkstra Clásico

1. **Entrada**: Mapa de ocupación, posición inicial, posición objetivo
2. **Conversión**: Coordenadas mundo → índices de grilla
3. **Planificación**: 
   - Cola de prioridad por costo acumulado
   - Exploración 4-vecinos (arriba, abajo, izquierda, derecha)
   - Costo uniforme = 1 por movimiento
4. **Salida**: Camino más corto en coordenadas de grilla
5. **Conversión**: Índices de grilla → coordenadas mundo
6. **Publicación**: Path en frame `map`

### Manejo de obstáculos

- Celdas con valor `< 50` → Libres (navegables)
- Celdas con valor `>= 50` → Ocupadas (obstáculos)
- Celdas con valor `-1` → Desconocidas (tratadas como obstáculos)

## Logs esperados

```
[INFO] [global_planner_dijkstra]: Nodo de planificación global Dijkstra iniciado
[INFO] [global_planner_dijkstra]: Esperando mapa en /map...
[INFO] [global_planner_dijkstra]: Esperando odometría en /odom...
[INFO] [global_planner_dijkstra]: Define un objetivo con "2D Goal Pose" en RViz
[INFO] [global_planner_dijkstra]: Mapa recibido: 334x220, resolución=0.05m
[INFO] [global_planner_dijkstra]: Nuevo objetivo recibido: (5.23, -2.45)
[INFO] [global_planner_dijkstra]: Planificando desde (120, 85) hasta (156, 95)...
[INFO] [global_planner_dijkstra]: Camino encontrado con 87 puntos
[INFO] [global_planner_dijkstra]: Trayectoria publicada en /global_path
```

## Solución de problemas

### El planificador no encuentra camino
- ✅ Verifica que el objetivo no esté en un obstáculo (celda negra en RViz)
- ✅ Asegúrate de que existe un camino navegable entre inicio y objetivo
- ✅ Revisa que el mapa esté publicándose: `ros2 topic echo /map --once`

### No se visualiza la trayectoria en RViz
- ✅ Verifica que agregaste el display **Path** con topic `/global_path`
- ✅ Confirma que el Fixed Frame está en `map`
- ✅ Verifica que la trayectoria se está publicando: `ros2 topic echo /global_path --once`

### El robot no se mueve
- ⚠️ **Normal** - Este nodo solo planifica, NO controla el robot
- ℹ️ La trayectoria es solo para visualización
- ℹ️ Para que el robot siga la trayectoria necesitas un controlador local

## Estructura del código

```
global_planner/
├── global_planner/
│   ├── __init__.py
│   └── global_planner_dijkstra.py  # Nodo principal
├── package.xml
├── setup.py
├── README.md
└── resource/
```

## Autor

Sistema ROS 2 Humble - Enero 2026

## Licencia

Proyecto académico - Unitree Go2
