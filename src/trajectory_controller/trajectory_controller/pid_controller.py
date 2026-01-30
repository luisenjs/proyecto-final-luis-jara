#!/usr/bin/env python3
"""
Controlador PID para seguimiento de trayectorias.
Sigue waypoints del /global_path usando control PID simple.

Autor: Luis Jara
Fecha: Enero 2026
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math
import time


class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # ========== Parámetros PID ==========
        # PID Lineal (velocidad hacia adelante)
        self.kp_v = 0.5      # Proporcional velocidad
        self.ki_v = 0.0      # Integral velocidad (normalmente 0)
        self.kd_v = 0.1      # Derivativo velocidad
        
        # PID Angular (giro)
        self.kp_w = 2.0      # Proporcional angular
        self.ki_w = 0.0      # Integral angular
        self.kd_w = 0.3      # Derivativo angular
        
        # Límites de velocidad
        self.max_v = 0.5     # m/s velocidad lineal máxima
        self.max_w = 1.0     # rad/s velocidad angular máxima
        
        # Tolerancias
        self.waypoint_tolerance = 0.2  # metros para considerar "llegó al waypoint"
        self.goal_tolerance = 0.25     # metros para considerar "llegó a la meta final"
        
        # ========== Estado del robot ==========
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # yaw en radianes
        self.have_odom = False
        
        # Odometría relativa (para evitar drift)
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        
        # ========== Waypoints ==========
        self.waypoints = []      # Lista de (x, y)
        self.waypoints_relative = []  # Waypoints en coordenadas relativas
        self.current_wp_idx = 0  # Índice del waypoint actual
        self.have_path = False
        
        # ========== Errores PID ==========
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0
        self.integral_v = 0.0
        self.integral_w = 0.0
        
        # ========== Métricas ==========
        self.start_time = None
        self.distance_traveled = 0.0
        self.prev_x = None
        self.prev_y = None
        self.trajectory_active = False
        
        # ========== ROS Subscriptions ==========
        self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # ========== ROS Publisher ==========
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ========== Timers ==========
        self.create_timer(0.1, self.control_loop)      # 10 Hz control
        self.create_timer(1.0, self.print_metrics)     # 1 Hz métricas
        
        self.get_logger().info('🚀 Controlador PID iniciado')
        self.get_logger().info('📡 Esperando /global_path y /odom...')
    
    # ==================== CALLBACKS ====================
    
    def path_callback(self, msg):
        """Recibe el path del planificador global"""
        if len(msg.poses) < 1:
            self.get_logger().warn('⚠️  Path vacío recibido')
            return
        
        if not self.have_odom:
            self.get_logger().warn('⚠️  Esperando odometría del robot...')
            return
        
        # Guardar waypoints como lista de tuplas (x, y)
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # Guardar posición inicial del robot como referencia
        self.initial_x = self.current_x
        self.initial_y = self.current_y
        self.initial_theta = self.current_theta
        
        # Convertir waypoints a coordenadas RELATIVAS al robot
        self.waypoints_relative = []
        for wx, wy in self.waypoints:
            rel_x = wx - self.initial_x
            rel_y = wy - self.initial_y
            self.waypoints_relative.append((rel_x, rel_y))
        
        self.current_wp_idx = 0
        self.have_path = True
        
        # Resetear métricas
        self.start_time = time.time()
        self.distance_traveled = 0.0
        self.prev_x = self.current_x
        self.prev_y = self.current_y
        self.trajectory_active = True
        
        # Resetear PID
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0
        self.integral_v = 0.0
        self.integral_w = 0.0
        
        # Calcular distancia total teórica
        total_dist = 0.0
        for i in range(len(self.waypoints_relative) - 1):
            dx = self.waypoints_relative[i+1][0] - self.waypoints_relative[i][0]
            dy = self.waypoints_relative[i+1][1] - self.waypoints_relative[i][1]
            total_dist += math.hypot(dx, dy)
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info(f'📍 Posición inicial: ({self.initial_x:.2f}, {self.initial_y:.2f})')
        self.get_logger().info(f'📍 Trayectoria recibida: {len(self.waypoints_relative)} waypoints')
        self.get_logger().info(f'📏 Distancia teórica: {total_dist:.2f} m')
        self.get_logger().info('═══════════════════════════════════════')
    
    def odom_callback(self, msg):
        """Recibe la odometría del robot"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convertir quaternion a yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.have_odom = True
        
        # Calcular distancia recorrida
        if self.trajectory_active and self.prev_x is not None:
            dx = self.current_x - self.prev_x
            dy = self.current_y - self.prev_y
            self.distance_traveled += math.hypot(dx, dy)
        
        self.prev_x = self.current_x
        self.prev_y = self.current_y
    
    # ==================== CONTROL PID ====================
    
    def control_loop(self):
        """Loop principal de control PID (10 Hz)"""
        if not self.have_odom or not self.have_path or not self.trajectory_active:
            return
        
        if self.current_wp_idx >= len(self.waypoints_relative):
            self.stop_robot()
            return
        
        # Calcular posición RELATIVA del robot
        robot_rel_x = self.current_x - self.initial_x
        robot_rel_y = self.current_y - self.initial_y
        
        # Obtener waypoint actual (en coordenadas relativas)
        goal_x, goal_y = self.waypoints_relative[self.current_wp_idx]
        
        # Calcular errores
        dx = goal_x - robot_rel_x
        dy = goal_y - robot_rel_y
        distance_to_goal = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        
        # Error angular (normalizado a [-pi, pi])
        error_theta = self.normalize_angle(angle_to_goal - self.current_theta)
        
        # ¿Llegamos al waypoint actual?
        if distance_to_goal < self.waypoint_tolerance:
            self.current_wp_idx += 1
            
            # ¿Era el último waypoint?
            if self.current_wp_idx >= len(self.waypoints_relative):
                self.stop_robot()
                self.print_final_summary()
                self.trajectory_active = False
                return
            
            # Resetear integrales al cambiar de waypoint
            self.integral_v = 0.0
            self.integral_w = 0.0
            return
        
        # ========== PID Lineal (velocidad hacia adelante) ==========
        error_v = distance_to_goal
        self.integral_v += error_v * 0.1  # dt = 0.1s
        derivative_v = (error_v - self.prev_error_v) / 0.1
        
        v_cmd = self.kp_v * error_v + self.ki_v * self.integral_v + self.kd_v * derivative_v
        self.prev_error_v = error_v
        
        # Si el error angular es grande, reducir velocidad lineal
        if abs(error_theta) > 0.5:  # ~30 grados
            v_cmd *= 0.3  # Reducir a 30%
        
        # Limitar velocidad lineal
        v_cmd = max(0.0, min(self.max_v, v_cmd))
        
        # ========== PID Angular (giro) ==========
        self.integral_w += error_theta * 0.1
        derivative_w = (error_theta - self.prev_error_w) / 0.1
        
        w_cmd = self.kp_w * error_theta + self.ki_w * self.integral_w + self.kd_w * derivative_w
        self.prev_error_w = error_theta
        
        # Limitar velocidad angular
        w_cmd = max(-self.max_w, min(self.max_w, w_cmd))
        
        # ========== Publicar comandos ==========
        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd)
    
    # ==================== UTILIDADES ====================
    
    def normalize_angle(self, angle):
        """Normaliza ángulo a [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def stop_robot(self):
        """Detiene el robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
    
    def print_metrics(self):
        """Imprime métricas en tiempo real (1 Hz)"""
        if not self.trajectory_active or self.start_time is None:
            return
        
        elapsed = time.time() - self.start_time
        
        # Limpiar consola y mostrar métricas
        print('\033[2J\033[H')  # Clear screen
        print('═══════════════════════════════════════')
        print('     SEGUIMIENTO DE TRAYECTORIA PID')
        print('═══════════════════════════════════════')
        print(f'⏱️  Tiempo transcurrido: {elapsed:.2f} s')
        print(f'📏 Distancia recorrida: {self.distance_traveled:.2f} m')
        print(f'📍 Waypoint actual: {self.current_wp_idx + 1}/{len(self.waypoints_relative)}')
        print('═══════════════════════════════════════')
    
    def print_final_summary(self):
        """Imprime resumen final al terminar"""
        if self.start_time is None:
            return
        
        elapsed = time.time() - self.start_time
        avg_speed = self.distance_traveled / elapsed if elapsed > 0 else 0.0
        
        print('\n')
        print('═══════════════════════════════════════')
        print('     🏁 TRAYECTORIA COMPLETADA 🏁')
        print('═══════════════════════════════════════')
        print(f'⏱️  Tiempo total: {elapsed:.2f} s')
        print(f'📏 Distancia total: {self.distance_traveled:.2f} m')
        print(f'⚡ Velocidad promedio: {avg_speed:.2f} m/s')
        print('═══════════════════════════════════════')
        
        self.get_logger().info('✅ Trayectoria completada con éxito')


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
