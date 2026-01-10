import os
import cv2
import csv
import yaml
import numpy as np
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_motion_planning.utils import Grid, SearchFactory
from pathlib import Path


def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)  # asegurar Path
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)


    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    # Binarizar: 1 = ocupado, 0 = libre
    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    # Engrosar obstáculos según el factor
    if downsample_factor > 12:
        map_bin = cv2.dilate(map_bin, np.ones((5, 5), np.uint8), iterations=2)
    elif downsample_factor >= 4:
        map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)
    # para 1-3 no se dilata

    # Downsampling con interpolación adecuada
    map_bin = map_bin.astype(np.float32)
    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    map_bin = cv2.resize(map_bin, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Re-binarizar según nivel
    if downsample_factor > 12:
        map_bin = (map_bin > 0.10).astype(np.uint8)
    elif downsample_factor >= 4:
        map_bin = (map_bin > 0.25).astype(np.uint8)
    else:
        map_bin = (map_bin >= 0.5).astype(np.uint8)

    # Ajustar resolución
    resolution *= downsample_factor

    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)
    obstacles = {(x, h - 1 - y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
    env.update(obstacles)
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)


def map_to_world(x_map, y_map, resolution, origin, image_height):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)


def resample_waypoints(path, resolution, origin, image_height, target_distance):
    """
    Resample waypoints to have a specific distance between them.
    
    Args:
        path: List of (x_map, y_map) coordinates
        resolution: Map resolution in meters/pixel
        origin: Map origin
        image_height: Height of the map
        target_distance: Desired distance between waypoints in meters
    
    Returns:
        List of (x_world, y_world) waypoints with specified separation
    """
    if not path:
        return []
    
    # Convert path to world coordinates
    world_path = []
    for x_map, y_map in reversed(path):  # invertir para ir de start a goal
        x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
        world_path.append((x, y))
    
    if len(world_path) < 2:
        return world_path
    
    # Resample to target distance
    resampled = [world_path[0]]  # Start with first point
    accumulated_dist = 0.0
    
    for i in range(1, len(world_path)):
        x1, y1 = world_path[i-1]
        x2, y2 = world_path[i]
        
        segment_length = np.hypot(x2 - x1, y2 - y1)
        accumulated_dist += segment_length
        
        # Add intermediate points if segment is long
        while accumulated_dist >= target_distance:
            # Calculate how far back we need to go
            overshoot = accumulated_dist - target_distance
            ratio = (segment_length - overshoot) / segment_length if segment_length > 0 else 0
            
            # Interpolate new point
            new_x = x1 + ratio * (x2 - x1)
            new_y = y1 + ratio * (y2 - y1)
            resampled.append((new_x, new_y))
            
            # Update for next iteration
            x1, y1 = new_x, new_y
            accumulated_dist -= target_distance
            segment_length = np.hypot(x2 - x1, y2 - y1)
    
    # Always include the last point
    if resampled[-1] != world_path[-1]:
        resampled.append(world_path[-1])
    
    return resampled


def save_path_as_csv(path, filename, resolution, origin, image_height):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x_map, y_map in reversed(path):  # invertir para ir de start a goal
            x, y = map_to_world(x_map, y_map, resolution, origin, image_height)
            writer.writerow([x, y])


def save_waypoints_csv(waypoints, filename):
    """
    Save waypoints (already in world coordinates) to CSV.
    
    Args:
        waypoints: List of (x, y) tuples in world coordinates
        filename: Output CSV filename
    """
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x, y in waypoints:
            writer.writerow([x, y])


if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "small_house.yaml"
    downsample_factor = 3  # Ajusta este valor según lo que necesites

    x_start, y_start = 1.0, 2.0
    x_goal, y_goal = 10.0, 5.0

    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    env = grid_from_map(map_bin)

    start = world_to_map(x_start, y_start, resolution, origin)
    goal = world_to_map(x_goal, y_goal, resolution, origin)

    print(f"Start (map): {start}, Goal (map): {goal}")
    planner = SearchFactory()("dijkstra", start=start, goal=goal, env=env)
    
    print("Ejecutando Dijkstra...")
    planner.run()  # Ejecuta y muestra visualización
    
    cost, path, expand = planner.plan()
    
    if path:
        # Generar waypoints con 0.5 metros de separación
        waypoints_05m = resample_waypoints(path, resolution, origin, map_bin.shape[0], 0.5)
        save_waypoints_csv(waypoints_05m, "dijkstra_waypoints_05m.csv")
        print(f"✓ Waypoints 0.5m guardados: dijkstra_waypoints_05m.csv ({len(waypoints_05m)} puntos)")
        
        # Generar waypoints con 1 metro de separación
        waypoints_1m = resample_waypoints(path, resolution, origin, map_bin.shape[0], 1.0)
        save_waypoints_csv(waypoints_1m, "dijkstra_waypoints_1m.csv")
        print(f"✓ Waypoints 1.0m guardados: dijkstra_waypoints_1m.csv ({len(waypoints_1m)} puntos)")
        
        print(f"\nCosto total de la ruta: {cost:.2f}")
        print(f"Nodos explorados: {len(expand)}")
        
        # Visualizar waypoints con separación de 0.5m
        import matplotlib.pyplot as plt
        plt.figure(figsize=(12, 5))
        
        plt.subplot(1, 2, 1)
        plt.imshow(map_bin, cmap='gray', origin='upper')
        plt.title('Dijkstra - Waypoints 0.5m')
        # Convertir waypoints de mundo a mapa para visualizar
        wp_05_map = [(int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)) 
                     for x, y in waypoints_05m]
        if wp_05_map:
            xs, ys = zip(*wp_05_map)
            # Invertir ys para que coincida con origin='upper'
            h = map_bin.shape[0]
            ys = [h - 1 - y for y in ys]
            plt.plot(xs, ys, 'b-', linewidth=2, label='Ruta')
            plt.plot(xs, ys, 'ro', markersize=4, label=f'Waypoints ({len(waypoints_05m)})')
            plt.plot(xs[0], ys[0], 'go', markersize=10, label='Inicio')
            plt.plot(xs[-1], ys[-1], 'mo', markersize=10, label='Meta')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.subplot(1, 2, 2)
        plt.imshow(map_bin, cmap='gray', origin='upper')
        plt.title('Dijkstra - Waypoints 1.0m')
        # Convertir waypoints de mundo a mapa para visualizar
        wp_1m_map = [(int((x - origin[0]) / resolution), int((y - origin[1]) / resolution)) 
                     for x, y in waypoints_1m]
        if wp_1m_map:
            xs, ys = zip(*wp_1m_map)
            # Invertir ys para que coincida con origin='upper'
            h = map_bin.shape[0]
            ys = [h - 1 - y for y in ys]
            plt.plot(xs, ys, 'b-', linewidth=2, label='Ruta')
            plt.plot(xs, ys, 'ro', markersize=4, label=f'Waypoints ({len(waypoints_1m)})')
            plt.plot(xs[0], ys[0], 'go', markersize=10, label='Inicio')
            plt.plot(xs[-1], ys[-1], 'mo', markersize=10, label='Meta')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('dijkstra_waypoints_comparison.png', dpi=150, bbox_inches='tight')
        print("\n✓ Comparación guardada: dijkstra_waypoints_comparison.png")
        plt.show()
    else:
        print("✗ No se encontró ruta")
