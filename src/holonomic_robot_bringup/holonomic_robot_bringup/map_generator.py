#!/usr/bin/env python3
"""
Generate a static occupancy grid map from a polygon.
Interior = FREE, Exterior = OCCUPIED (walls surround the polygon).
"""

import math
import os

import numpy as np
from PIL import Image, ImageDraw
import yaml


def world_to_pixel(point, min_x, min_y, height, resolution):
    """Convert world coordinates (meters) to pixel coordinates for the map."""
    x, y = point
    px = int(round((x - min_x) / resolution))
    py = height - int(round((y - min_y) / resolution))  # Flip Y axis to image coords
    return px, py


def rectangle_corners(center, size, yaw_deg):
    """Return rectangle corners in world frame given center, size, and yaw (deg)."""
    cx, cy = center
    sx, sy = size
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = sx / 2.0, sy / 2.0
    local = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
    world = []
    for lx, ly in local:
        wx = cx + lx * c - ly * s
        wy = cy + lx * s + ly * c
        world.append((wx, wy))
    return world


def generate_polygon_map(polygon_points, rectangles=None, resolution=0.05, output_dir='.'):
    """
    Generate an occupancy grid map from polygon points.
    Everything INSIDE polygon = FREE (254)
    Everything OUTSIDE polygon = OCCUPIED (0) - acts as walls
    
    Args:
        polygon_points: List of (x, y) tuples defining the FREE space boundary
        resolution: Map resolution in meters/pixel
        output_dir: Directory to save map files
    """
    # Convert points to numpy array
    points = np.array(polygon_points)
    
    # Find bounds
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    
    # Add padding (this will be the wall/occupied area)
    padding = 0.5  # meters of walls around the polygon
    min_x -= padding
    min_y -= padding
    max_x += padding
    max_y += padding
    
    # Calculate map dimensions
    width = int((max_x - min_x) / resolution)
    height = int((max_y - min_y) / resolution)
    
    print(f"Generating map...")
    print(f"  Interior bounds: x=[{points.min(axis=0)[0]:.2f}, {points.max(axis=0)[0]:.2f}], "
          f"y=[{points.min(axis=0)[1]:.2f}, {points.max(axis=0)[1]:.2f}]")
    print(f"  Map size: {width}x{height} pixels")
    print(f"  Resolution: {resolution} m/pixel")
    
    # Create map filled with OCCUPIED (0) - everything is a wall by default
    map_data = np.full((height, width), 0, dtype=np.uint8)
    
    # Create PIL image for drawing
    img = Image.fromarray(map_data)
    draw = ImageDraw.Draw(img)
    
    # Convert polygon points to pixel coordinates
    pixel_points = []
    for x, y in polygon_points:
        pixel_points.append(world_to_pixel((x, y), min_x, min_y, height, resolution))
    
    print(f"  Polygon vertices: {len(pixel_points)}")
    
    # Fill INSIDE polygon with FREE space (254)
    draw.polygon(pixel_points, fill=254, outline=254)

    # Draw rectangles as OCCUPIED obstacles if provided
    if rectangles:
        print(f"  Rectangles: {len(rectangles)}")
        for idx, rect in enumerate(rectangles, start=1):
            corners_world = rectangle_corners(rect['center'], rect['size'], rect.get('yaw_deg', 0.0))
            corners_px = [world_to_pixel(pt, min_x, min_y, height, resolution) for pt in corners_world]
            draw.polygon(corners_px, fill=0, outline=0)
            print(f"    [{idx}] center={rect['center']} size={rect['size']} yaw={rect.get('yaw_deg', 0.0)}°")
    
    # Convert back to numpy array
    map_data = np.array(img)
    
    # Save map image (PGM format)
    map_img = Image.fromarray(map_data, mode='L')
    map_file = os.path.join(output_dir, 'polygon_map.pgm')
    map_img.save(map_file)
    
    # Create YAML metadata
    yaml_data = {
        'image': 'polygon_map.pgm',
        'resolution': float(resolution),
        'origin': [float(min_x), float(min_y), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    yaml_file = os.path.join(output_dir, 'polygon_map.yaml')
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    # Calculate interior dimensions
    interior_width = points.max(axis=0)[0] - points.min(axis=0)[0]
    interior_height = points.max(axis=0)[1] - points.min(axis=0)[1]
    
    print(f"\n✓ Map generated successfully:")
    print(f"  Files:")
    print(f"    - {map_file}")
    print(f"    - {yaml_file}")
    print(f"  Interior FREE space: {interior_width:.2f}m × {interior_height:.2f}m")
    print(f"  Exterior walls (padding): {padding}m on all sides")
    print(f"  Origin: ({min_x:.2f}, {min_y:.2f}, 0.0)")
    print(f"\n  Color coding:")
    print(f"    - WHITE (254) = FREE SPACE (inside polygon)")
    print(f"    - BLACK (0)   = OCCUPIED/WALLS (outside polygon)")


if __name__ == '__main__':
    # Simple 3m x 2m room with an alcove
    polygon = [
        (0.0, 0.0),
        (0.6, 0.0),
        (0.6, 0.5),
        (2.4, 0.5),
        (2.4, 0.0),
        (3.0, 0.0),
        (3.0, 2.0),
        (0.0, 2.0),
    ]

    # Inline rectangle obstacles (center (m), size (m), yaw (deg))
    rectangles = [
        {'center': (0.175, 0.8), 'size': (0.2, 0.15), 'yaw_deg': 90.0},
        {'center': (0.175, 1.6), 'size': (0.2, 0.15), 'yaw_deg': 90.0},
        {'center': (2.825,0.8), 'size': (0.2, 0.15), 'yaw_deg': 90.0},
        {'center': (2.825, 1.6), 'size': (0.2, 0.15), 'yaw_deg': 90.0},
        {'center': (1.150, 1.2), 'size': (0.2, 0.15), 'yaw_deg': 0.0},
        {'center': (1.850, 1.2), 'size': (0.2, 0.15), 'yaw_deg': 0.0},
        {'center': (1.150, 1.825), 'size': (0.2, 0.15), 'yaw_deg': 0.0},
        {'center': (1.900, 1.825), 'size': (0.2, 0.15), 'yaw_deg': 0.0},
    ]

    # Get output directorycd 
    import sys
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = os.path.dirname(__file__)
        output_dir = os.path.join(os.path.dirname(output_dir), 'maps')

    os.makedirs(output_dir, exist_ok=True)

    print(f"Output directory: {output_dir}\n")

    # Generate map - 1cm resolution
    generate_polygon_map(polygon, rectangles=rectangles, resolution=0.01, output_dir=output_dir)

    print("\nTo use this map:")
    print("  ros2 launch holonomic_robot_bringup full_navigation.launch.py use_amcl:=true")
