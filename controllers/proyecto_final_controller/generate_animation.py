import csv
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import os

def read_csv(path):
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        return list(reader)

def generate_gif(scenario):
    base_dir = os.path.dirname(__file__)
    data_path = os.path.join(base_dir, f'data_{scenario}.csv')
    grid_path = os.path.join(base_dir, f'grid_{scenario}.csv')
    path_path = os.path.join(base_dir, f'path_{scenario}.csv')
    
    if not os.path.exists(data_path):
        print(f"No data for {scenario}")
        return

    data = read_csv(data_path)
    grid = read_csv(grid_path)
    path_data = read_csv(path_path)
    
    # Filter A* waypoints
    wp_list = [row for row in path_data if row['type'] == 'wp_world']
    wp_x = [float(row['x']) for row in wp_list]
    wp_y = [float(row['y']) for row in wp_list]
    
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-1.1, 1.1)
    ax.set_ylim(-1.1, 1.1)
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    
    # Draw arena walls
    ax.add_patch(patches.Rectangle((-1.0, -1.0), 2.0, 2.0, fill=False, edgecolor='black', lw=2))
    
    # Draw grid obstacles
    for row in grid:
        x = -1.0 + int(row['col']) * 0.05
        y = -1.0 + int(row['row']) * 0.05
        ax.add_patch(patches.Rectangle((x, y), 0.05, 0.05, color='gray', alpha=0.5))
        
    # Draw A* path
    ax.plot(wp_x, wp_y, 'ro--', alpha=0.7, label='A* Path')
    
    # Start and Goal
    ax.plot(-0.75, -0.75, 'bo', markersize=10, label='Start')
    ax.plot(0.75, 0.75, 'go', markersize=10, label='Goal')
    
    line, = ax.plot([], [], 'b-', lw=2, label='Odometry')
    robot_circle = patches.Circle((0, 0), radius=0.037, color='blue', alpha=0.5)
    ax.add_patch(robot_circle)
    
    heading_line, = ax.plot([], [], 'r-', lw=2)
    
    ax.legend(loc='lower left')
    
    time_text = ax.text(-1.0, 1.05, '', fontsize=10)
    
    skip = 10
    frames = range(0, len(data), skip)
    
    xs, ys = [], []
    for d in data:
        xs.append(float(d['x']))
        ys.append(float(d['y']))
    
    def init():
        line.set_data([], [])
        robot_circle.center = (-0.75, -0.75)
        heading_line.set_data([], [])
        time_text.set_text('')
        return line, robot_circle, heading_line, time_text

    def animate(i):
        line.set_data(xs[:i+1], ys[:i+1])
        
        curr = data[i]
        cx, cy, ctheta = float(curr['x']), float(curr['y']), float(curr['theta'])
        
        robot_circle.center = (cx, cy)
        
        hx = cx + 0.05 * math.cos(ctheta)
        hy = cy + 0.05 * math.sin(ctheta)
        heading_line.set_data([cx, hx], [cy, hy])
        
        time_text.set_text(f"T={float(curr['time']):.1f}s | Act={curr['action']}")
        
        return line, robot_circle, heading_line, time_text

    anim = FuncAnimation(fig, animate, init_func=init, frames=frames, interval=50, blit=True)
    
    gif_path = f'/home/axeler8/.gemini/antigravity-ide/brain/dd182ba0-9851-4f22-a1a7-7fb51e2f2646/artifacts/animation_{scenario}.gif'
    print(f"Saving {gif_path}...")
    anim.save(gif_path, writer='pillow', fps=20)
    print("Saved!")
    plt.close(fig)

if __name__ == '__main__':
    generate_gif('simple')
