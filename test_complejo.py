import sys
sys.path.append('controllers/proyecto_final_controller')
from proyecto_final_controller import OccupancyGrid, AStarPlanner, SCENARIOS, INFLATION

def test():
    sc = SCENARIOS['complejo']
    grid = OccupancyGrid()
    for obs in sc['obstacles']:
        grid.add_obstacle(*obs, inflate=INFLATION)
        
    start = grid.world_to_grid(sc['start_x'], sc['start_y'])
    goal = grid.world_to_grid(sc['goal_x'], sc['goal_y'])
    
    print(f"Start cell: {start}, is_free: {grid.is_free(*start)}")
    print(f"Goal cell: {goal}, is_free: {grid.is_free(*goal)}")
    
    path = AStarPlanner.find_path(grid, start, goal)
    print(f"Path length: {len(path)}")

test()
