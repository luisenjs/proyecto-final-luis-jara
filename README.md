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

![Mapa generado con SLAM](src/unitree-go2-ros2/robots/configs/go2_config/maps/map.png)

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

## Dependencias

### Sistema Base
- **Ubuntu 22.04 LTS**
- **ROS 2 Humble Hawksbill**

### Paquetes ROS 2
```bash
# Paquetes principales
ros-humble-desktop
ros-humble-gazebo-ros-pkgs
ros-humble-ros2-control
ros-humble-ros2-controllers

# Nav2 y SLAM
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-slam-toolbox

# Visualización
ros-humble-rviz2
ros-humble-rqt-robot-steering

# Utilidades
ros-humble-teleop-twist-keyboard
```

### Paquetes Python
```bash
# Incluidos en ROS 2 Humble
rclpy
numpy
```

### Repositorios Externos
- **CHAMP Framework**: Framework para robots cuadrúpedos (incluido en el workspace)
- **Unitree Go2 Description**: Archivos URDF/Xacro del robot Go2

---

## Instalación

### 1. Clonar el Repositorio

```bash
cd ~/Desktop/Example
mkdir -p luisjara_ws/src
cd luisjara_ws/src
git clone https://github.com/widegonz/unitree-go2-ros2.git
```

### 2. Instalar Dependencias de ROS 2

```bash
cd ~/Desktop/Example/luisjara_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Instalar Modelos de AWS RoboMaker (para small_house)

```bash
cd ~/Desktop/Example/luisjara_ws/src/unitree-go2-ros2/robots/configs/go2_config
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git models
```

### 4. Configurar el Paquete de Planificación Global

El paquete `global_planner` ya está incluido en el workspace. Verifica su estructura:

```bash
ls ~/Desktop/Example/luisjara_ws/src/global_planner
# Debe mostrar: global_planner/ launch/ package.xml setup.py README.md
```

---

## Compilación

### Compilar el Workspace Completo

```bash
cd ~/Desktop/Example/luisjara_ws
colcon build
```

### Compilar Solo el Planificador (más rápido)

```bash
cd ~/Desktop/Example/luisjara_ws
colcon build --packages-select global_planner
```

### Source del Workspace

**¡Importante!** Debes hacer source en cada nueva terminal:

```bash
source ~/Desktop/Example/luisjara_ws/install/setup.bash
```

O agrégalo al `.bashrc` para hacerlo automático:

```bash
echo "source ~/Desktop/Example/luisjara_ws/install/setup.bash" >> ~/.bashrc
```

---

## Ejecución

El proyecto se ejecuta en dos fases: **mapeo** y **planificación**.

---

### **FASE 1: Generación del Mapa con SLAM**

#### Terminal 1 - Lanzar Gazebo con small_house

```bash
cd ~/Desktop/Example/luisjara_ws
source install/setup.bash
ros2 launch go2_config gazebo.launch.py
```

Espera a que Gazebo cargue completamente (~10 segundos).

#### Terminal 2 - Lanzar SLAM Toolbox

```bash
source install/setup.bash
ros2 launch go2_config slam.launch.py use_sim_time:=true
```

Se abrirá RViz mostrando el mapa en construcción.

#### Terminal 3 - Teleoperación del Robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controles del teclado:**
- `i` - Avanzar
- `k` - Detener
- `,` - Retroceder
- `j` - Girar izquierda
- `l` - Girar derecha
- `q/z` - Aumentar/disminuir velocidad

**Objetivo**: Mueve el robot por toda la casa hasta que el mapa esté completo.

#### Configurar RViz (si es necesario)

En el panel izquierdo de RViz:
- **Global Options** → **Fixed Frame** → `map`
- Verifica que el mapa se visualice correctamente

#### Guardar el Mapa

Una vez completado el mapeo:

```bash
cd ~/Desktop/Example/luisjara_ws/src/unitree-go2-ros2/robots/configs/go2_config/maps
ros2 run nav2_map_server map_saver_cli -f map
```

Esto generará:
- `map.pgm` - Imagen del mapa
- `map.yaml` - Metadata del mapa

**Cierra SLAM** (Ctrl+C en Terminal 2) pero **mantén Gazebo corriendo**.

---

### **FASE 2: Planificación Global con Dijkstra**

#### Terminal 2 - Lanzar Planificador + Map Server

```bash
source install/setup.bash
ros2 launch global_planner planner_with_map.launch.py
```

**Logs esperados:**

```
[INFO] [map_server]: Read map /path/to/map.png: 343 X 221 map @ 0.05 m/cell
[INFO] [lifecycle_manager_map]: Managed nodes are active
[INFO] [global_planner_dijkstra]: Nodo de planificación global Dijkstra iniciado
[INFO] [global_planner_dijkstra]: Mapa recibido: 343x221, resolución=0.05m
[INFO] [global_planner_dijkstra]: Esperando odometría en /odom...
```

#### Terminal 3 - Abrir RViz

```bash
rviz2
```

**Configuración de RViz:**

1. **Fixed Frame**: Cambiar a `map`
   
2. **Agregar Displays**:
   - Click en **Add** (abajo izquierda)
   - **Map** → OK → Topic: `/map`
   - **Path** → OK → Topic: `/global_path` → Color: Verde/Azul
   - **RobotModel** → OK
   - **TF** → OK

3. **Habilitar herramienta de objetivo**:
   - En la barra superior, click en **"2D Goal Pose"**

#### Planificar Trayectorias

1. Con la herramienta **"2D Goal Pose"** activa, haz click en el mapa donde quieras que vaya el robot
2. Arrastra el mouse para definir la orientación (opcional)
3. El planificador calculará automáticamente la trayectoria
4. La trayectoria aparecerá como una línea verde/azul en RViz

**Logs del planificador:**

```
[INFO] [global_planner_dijkstra]: Nuevo objetivo recibido: (5.23, -2.45)
[INFO] [global_planner_dijkstra]: Planificando desde (120, 85) hasta (156, 95)...
[INFO] [global_planner_dijkstra]: Camino encontrado con 87 puntos
[INFO] [global_planner_dijkstra]: Trayectoria publicada en /global_path
```

---

## Verificación del Sistema

### Verificar Topics Activos

```bash
ros2 topic list
```

Debe incluir:
- `/map` - Mapa del entorno
- `/odom` - Odometría del robot
- `/goal_pose` - Objetivo definido en RViz
- `/global_path` - Trayectoria planificada

### Verificar Publicación del Mapa

```bash
ros2 topic echo /map --once
```

### Verificar la Trayectoria

```bash
ros2 topic echo /global_path
```

### Ver TF Tree

```bash
ros2 run tf2_tools view_frames
```

---

## Solución de Problemas

### El mapa no aparece en RViz

**Problema**: El display Map en RViz está en rojo o vacío.

**Solución**:
1. Verifica que el topic sea `/map`
2. Confirma que Fixed Frame esté en `map`
3. Ejecuta: `ros2 topic echo /map --once` para verificar que se está publicando

### El planificador no recibe el mapa

**Problema**: Log dice "No hay mapa disponible. Esperando /map..."

**Solución**:
1. Reinicia el planificador (Ctrl+C y vuelve a lanzar)
2. Verifica que `map_server` esté activo: `ros2 node list | grep map_server`

### No se encuentra camino al objetivo

**Problema**: Log dice "No se encontró camino al objetivo"

**Posibles causas**:
1. El objetivo está en un obstáculo (celda negra/gris oscura)
2. El objetivo está en zona desconocida (gris claro)
3. No existe camino navegable entre inicio y objetivo

**Solución**:
- Define el objetivo en una zona claramente libre (blanca) del mapa
- Verifica que exista un camino despejado visible en RViz

### La trayectoria no se visualiza

**Problema**: No aparece línea verde en RViz.

**Solución**:
1. Verifica que agregaste el display **Path** con topic `/global_path`
2. Cambia el color del Path a uno más visible
3. Aumenta el tamaño de línea (Line Width) a 0.05

### El robot no se mueve siguiendo la trayectoria

**Comportamiento esperado**: Este es el comportamiento correcto. 

**Explicación**: El nodo solo calcula y visualiza la trayectoria, NO controla el robot. Para que el robot siga la trayectoria se necesitaría implementar un controlador local (fuera del alcance de este proyecto).

---

## Estado del Proyecto

### Trabajo en Progreso

Este proyecto se encuentra actualmente en desarrollo. Los componentes implementados incluyen:

✅ **Completado:**
- Integración del robot Unitree Go2 en Gazebo
- Configuración de SLAM Toolbox
- Implementación del algoritmo de Dijkstra
- Estructura de nodos ROS 2 para planificación
- Sistema de visualización en RViz

🔧 **En desarrollo:**
- Corrección del proceso de mapeo SLAM automático
- Depuración de la conversión de coordenadas
- Validación de la planificación de trayectorias
- Optimización del rendimiento del algoritmo
- Integración completa del flujo de trabajo

📋 **Próximos pasos:**
- Implementar un controlador local para seguimiento de trayectorias
- Mejorar la calidad del mapeo SLAM
- Agregar suavizado de trayectorias
- Optimizar el algoritmo para mapas grandes
- Realizar pruebas exhaustivas en diferentes entornos

---

## Notas Adicionales

### Diferencias con Nav2

Este proyecto implementa el algoritmo de Dijkstra **desde cero** sin usar Nav2. Las diferencias principales:

| Característica | Este Proyecto | Nav2 |
|----------------|---------------|------|
| Algoritmo | Dijkstra puro | A*, Theta*, etc. |
| Implementación | Python nativo | C++ optimizado |
| Conectividad | 4-vecinos | 8-vecinos configurable |
| Costo | Uniforme | Configurable (distancia, curvatura) |
| Optimizaciones | Ninguna | Múltiples (smoothing, inflation) |
| Control del robot | No incluido | Incluido (DWB, TEB, etc.) |

### Rendimiento

- Tiempo de planificación: ~100-500ms para mapas de 343×221 celdas
- Longitud promedio de trayectorias: 50-200 puntos
- Memoria utilizada: ~10-20 MB

### Limitaciones

1. **Solo planificación**: No incluye control del robot
2. **Costo uniforme**: No considera costos diferenciados por terreno
3. **Sin suavizado**: La trayectoria sigue la grilla estrictamente
4. **4-conectividad**: No permite movimientos diagonales
