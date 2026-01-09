#!/usr/bin/env python3
"""
Nodo de planificación global usando el algoritmo de Dijkstra.
Implementación desde cero sin usar Nav2 ni librerías de planificación.

Autor: Sistema ROS 2 Humble
Fecha: Enero 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import heapq
import numpy as np


class GlobalPlannerDijkstra(Node):
    """
    Nodo que implementa planificación global con Dijkstra.
    """

    def __init__(self):
        super().__init__('global_planner_dijkstra')
        
        # Variables internas
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.current_pose = None
        self.goal_pose = None
        
        # QoS profile para el mapa (transient_local para recibir el último mapa publicado)
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Suscripciones
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publicador
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        
        self.get_logger().info('Nodo de planificación global Dijkstra iniciado')
        self.get_logger().info('Esperando mapa en /map...')
        self.get_logger().info('Esperando odometría en /odom...')
        self.get_logger().info('Define un objetivo con "2D Goal Pose" en RViz')

    def map_callback(self, msg):
        """
        Callback del mapa. Almacena la información del OccupancyGrid.
        """
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Convertir el mapa 1D a matriz 2D
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        if not hasattr(self, '_map_received'):
            self.get_logger().info(f'Mapa recibido: {self.map_width}x{self.map_height}, resolución={self.map_resolution}m')
            self._map_received = True

    def odom_callback(self, msg):
        """
        Callback de odometría. Almacena la posición actual del robot.
        """
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg):
        """
        Callback del objetivo. Se ejecuta cada vez que se define un nuevo goal en RViz.
        Dispara la planificación de trayectoria.
        """
        self.goal_pose = (
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        self.get_logger().info(f'Nuevo objetivo recibido: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})')
        
        # Planificar trayectoria
        self.plan_path()

    def world_to_grid(self, x, y):
        """
        Convierte coordenadas del mundo (x, y) a índices de grilla (i, j).
        
        Args:
            x (float): Coordenada X en el mundo
            y (float): Coordenada Y en el mundo
            
        Returns:
            tuple: (i, j) índices de la grilla, o None si está fuera de límites
        """
        if self.map_origin is None or self.map_resolution is None:
            return None
        
        # Calcular índices
        i = int((y - self.map_origin[1]) / self.map_resolution)
        j = int((x - self.map_origin[0]) / self.map_resolution)
        
        # Verificar límites
        if 0 <= i < self.map_height and 0 <= j < self.map_width:
            return (i, j)
        return None

    def grid_to_world(self, i, j):
        """
        Convierte índices de grilla (i, j) a coordenadas del mundo (x, y).
        
        Args:
            i (int): Índice fila
            j (int): Índice columna
            
        Returns:
            tuple: (x, y) coordenadas en el mundo
        """
        x = self.map_origin[0] + (j + 0.5) * self.map_resolution
        y = self.map_origin[1] + (i + 0.5) * self.map_resolution
        return (x, y)

    def is_free(self, i, j):
        """
        Verifica si una celda está libre (navegable).
        
        Args:
            i (int): Índice fila
            j (int): Índice columna
            
        Returns:
            bool: True si la celda está libre, False si está ocupada
        """
        if self.map_data is None:
            return False
        
        if 0 <= i < self.map_height and 0 <= j < self.map_width:
            cell_value = self.map_data[i, j]
            # Celda libre: valor < 50, ocupada: >= 50, desconocida: -1
            return cell_value >= 0 and cell_value < 50
        return False

    def get_neighbors(self, i, j):
        """
        Obtiene los vecinos válidos (4-conectividad) de una celda.
        
        Args:
            i (int): Índice fila
            j (int): Índice columna
            
        Returns:
            list: Lista de tuplas (i, j) de vecinos navegables
        """
        neighbors = []
        # Arriba, abajo, izquierda, derecha
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        
        for di, dj in directions:
            ni, nj = i + di, j + dj
            if self.is_free(ni, nj):
                neighbors.append((ni, nj))
        
        return neighbors

    def dijkstra(self, start_grid, goal_grid):
        """
        Implementación del algoritmo de Dijkstra para encontrar el camino más corto.
        
        Args:
            start_grid (tuple): Posición inicial (i, j) en la grilla
            goal_grid (tuple): Posición objetivo (i, j) en la grilla
            
        Returns:
            list: Lista de tuplas (i, j) representando el camino, o None si no existe
        """
        # Cola de prioridad: (costo, (i, j))
        open_set = [(0, start_grid)]
        
        # Diccionarios para almacenar costos y padres
        g_score = {start_grid: 0}
        came_from = {}
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            # Si llegamos al objetivo, reconstruir camino
            if current == goal_grid:
                return self.reconstruct_path(came_from, current)
            
            # Si encontramos un camino mejor, ignorar
            if current_cost > g_score.get(current, float('inf')):
                continue
            
            # Explorar vecinos
            for neighbor in self.get_neighbors(current[0], current[1]):
                # Costo uniforme = 1 por cada movimiento
                tentative_g_score = g_score[current] + 1
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    # Encontramos un mejor camino a este vecino
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_set, (tentative_g_score, neighbor))
        
        # No se encontró camino
        return None

    def reconstruct_path(self, came_from, current):
        """
        Reconstruye el camino desde el objetivo hasta el inicio.
        
        Args:
            came_from (dict): Diccionario de padres
            current (tuple): Nodo actual (objetivo)
            
        Returns:
            list: Lista de tuplas (i, j) del camino completo
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()  # Del inicio al objetivo
        return path

    def plan_path(self):
        """
        Planifica la trayectoria usando Dijkstra y publica el resultado.
        """
        # Verificar que tenemos toda la información necesaria
        if self.map_data is None:
            self.get_logger().warn('No hay mapa disponible. Esperando /map...')
            return
        
        if self.current_pose is None:
            self.get_logger().warn('No hay odometría disponible. Esperando /odom...')
            return
        
        if self.goal_pose is None:
            self.get_logger().warn('No hay objetivo definido.')
            return
        
        # Convertir posiciones a grilla
        start_grid = self.world_to_grid(self.current_pose[0], self.current_pose[1])
        goal_grid = self.world_to_grid(self.goal_pose[0], self.goal_pose[1])
        
        if start_grid is None:
            self.get_logger().error('Posición inicial fuera del mapa')
            return
        
        if goal_grid is None:
            self.get_logger().error('Posición objetivo fuera del mapa')
            return
        
        if not self.is_free(start_grid[0], start_grid[1]):
            self.get_logger().warn('Posición inicial está en un obstáculo')
            return
        
        if not self.is_free(goal_grid[0], goal_grid[1]):
            self.get_logger().warn('Posición objetivo está en un obstáculo')
            return
        
        self.get_logger().info(f'Planificando desde {start_grid} hasta {goal_grid}...')
        
        # Ejecutar Dijkstra
        path_grid = self.dijkstra(start_grid, goal_grid)
        
        if path_grid is None:
            self.get_logger().warn('No se encontró camino al objetivo')
            return
        
        self.get_logger().info(f'Camino encontrado con {len(path_grid)} puntos')
        
        # Convertir el camino de grilla a coordenadas del mundo
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, j in path_grid:
            x, y = self.grid_to_world(i, j)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Sin rotación
            
            path_msg.poses.append(pose)
        
        # Publicar el camino
        self.path_pub.publish(path_msg)
        self.get_logger().info('Trayectoria publicada en /global_path')


def main(args=None):
    """
    Función principal del nodo.
    """
    rclpy.init(args=args)
    node = GlobalPlannerDijkstra()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
