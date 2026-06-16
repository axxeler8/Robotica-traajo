import sys
sys.path.append('controllers/proyecto_final_controller')
from proyecto_final_controller import OccupancyGrid, AStarPlanner, SCENARIOS, INFLATION

def test():
    sc = SCENARIOS['simple']
    grid = OccupancyGrid()
    for obs in sc['obstacles']:
        grid.add_obstacle(*obs, inflate=INFLATION)
        
    start = grid.world_to_grid(-0.725, -0.725)
    goal = grid.world_to_grid(0.675, -0.625)
    
    print(f"Start: {start}, Goal: {goal}")
    print(f"LOS clear? {AStarPlanner._los(grid, start, goal)}")

    # Print the line
    r0, c0 = start
    r1, c1 = goal
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        free = grid.is_free(r, c)
        print(f"Cell {r}, {c} -> free? {free}")
        if r == r1 and c == c1: break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc

test()
