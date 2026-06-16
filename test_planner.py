import sys
sys.path.append('controllers/proyecto_final_controller')
from proyecto_final_controller import OccupancyGrid, AStarPlanner, SCENARIOS, INFLATION

def test():
    sc = SCENARIOS['simple']
    grid = OccupancyGrid()
    for obs in sc['obstacles']:
        grid.add_obstacle(*obs, inflate=INFLATION)
        
    start = grid.world_to_grid(sc['start_x'], sc['start_y'])
    goal = grid.world_to_grid(sc['goal_x'], sc['goal_y'])
    
    path = AStarPlanner.find_path(grid, start, goal)
    smoothed = AStarPlanner.smooth_path(path, grid)
    
    print("Smoothed path:")
    for wp in smoothed:
        print(grid.grid_to_world(*wp))

test()
