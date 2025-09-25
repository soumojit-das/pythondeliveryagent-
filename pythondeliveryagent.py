# Save this file as delivery_agent.py

import argparse
import time
import heapq

ALL_MAPS = {
    "small": """
S1111
11X11
111X1
1X111
1111G
""",
    "medium": """
S11X111111
122X111111
1222222111
1111X11X11
1111X11X11
1111111XG1
1111111X11
1111111111
""",
    "large": """
S11111111111111111111
1222222222222X111111
1222222222222X111111
111111111X111X111111
1X1111111X111X111111
1X1111111X111X111111
1X1111111X111X111111
1X111111111111111111
111111111111111111G1
""",
    "dynamic": """
S1111111
111X1111
111X1111
11111111
111X1111
111X1111
1111111G
"""
}

class CityGrid:
    def __init__(self, layout_str):
        self.grid, self.start, self.goal = self.parse_layout(layout_str)
        self.rows = len(self.grid)
        self.cols = len(self.grid[0])
        self.dynamic_blocks = set()

    def parse_layout(self, layout_str):
        matrix, start_node, goal_node = [], None, None
        clean_lines = [line.strip() for line in layout_str.strip().split('\n')]
        for r, line in enumerate(clean_lines):
            row_data = []
            for c, char in enumerate(line):
                if char == 'S':
                    start_node = (r, c)
                    row_data.append(1)
                elif char == 'G':
                    goal_node = (r, c)
                    row_data.append(1)
                elif char == 'X':
                    row_data.append(float('inf'))
                else:
                    row_data.append(int(char))
            matrix.append(row_data)
        if not start_node or not goal_node:
            raise ValueError("Map needs a Start (S) and Goal (G).")
        return matrix, start_node, goal_node

    def get_cell_cost(self, pos):
        r, c = pos
        if pos in self.dynamic_blocks or self.grid[r][c] == float('inf'):
            return float('inf')
        return self.grid[r][c]

    def is_in_bounds(self, pos):
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols

    def get_adjacent_nodes(self, pos):
        r, c = pos
        adj = []
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if self.is_in_bounds((nr, nc)) and self.grid[nr][nc] != float('inf'):
                adj.append((nr, nc))
        return adj

    def set_dynamic_blocks(self, block_list):
        self.dynamic_blocks = set()
        for pos in block_list:
            if self.is_in_bounds(pos):
                self.dynamic_blocks.add(pos)
        print(f"Dynamic blocks updated at: {block_list}")

    def render(self, path=[], agent_loc=None):
        path_nodes = set(path)
        for r in range(self.rows):
            line = ""
            for c in range(self.cols):
                pos = (r, c)
                if pos == agent_loc:
                    line += " A "
                elif pos == self.start:
                    line += " S "
                elif pos == self.goal:
                    line += " G "
                elif pos in self.dynamic_blocks:
                    line += " D "
                elif self.grid[r][c] == float('inf'):
                    line += "███"
                elif pos in path_nodes:
                    line += " . "
                else:
                    line += f" {self.grid[r][c]} "
            print(line)

def ucs_pathfinder(grid_map, start_node, end_node):
    q = [(0, [start_node], start_node)]
    seen = {start_node}
    nodes_checked = 0

    while q:
        cost, path, current = heapq.heappop(q)
        nodes_checked += 1

        if current == end_node:
            return path, cost, nodes_checked

        for neighbor in grid_map.get_adjacent_nodes(current):
            if neighbor not in seen:
                seen.add(neighbor)
                new_cost = cost + grid_map.get_cell_cost(neighbor)
                heapq.heappush(q, (new_cost, path + [neighbor], neighbor))
    
    return None, float('inf'), nodes_checked

def a_star_pathfinder(grid_map, start_node, end_node):
    def heuristic(p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    g_scores = {start_node: 0}
    f_score = heuristic(start_node, end_node)
    
    q = [(f_score, [start_node], start_node)]
    seen = {start_node}
    nodes_checked = 0

    while q:
        _, path, current = heapq.heappop(q)
        nodes_checked += 1

        if current == end_node:
            return path, g_scores[end_node], nodes_checked

        for neighbor in grid_map.get_adjacent_nodes(current):
            tentative_g = g_scores[current] + grid_map.get_cell_cost(neighbor)
            
            if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                g_scores[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, end_node)
                heapq.heappush(q, (f_score, path + [neighbor], neighbor))
                seen.add(neighbor)
    
    return None, float('inf'), nodes_checked

class Agent:
    def __init__(self, world):
        self.world = world
        self.current_path = []
        self.location = world.start

    def find_route(self, algo):
        t_start = time.time()
        
        if algo == 'ucs':
            result = ucs_pathfinder(self.world, self.world.start, self.world.goal)
        elif algo == 'astar':
            result = a_star_pathfinder(self.world, self.world.start, self.world.goal)
        else:
            raise ValueError("Algorithm not supported.")
            
        t_end = time.time()
        
        path, cost, nodes = result
        self.current_path = path
        
        return path, cost, nodes, t_end - t_start

    def replan_simulation(self):
        print("--- Replanning Simulation ---")
        
        print("\n1. Calculating initial route...")
        initial_route, _, _, _ = a_star_pathfinder(self.world, self.location, self.world.goal)
        if not initial_route:
            print("Failed to find initial route.")
            return
            
        self.current_path = initial_route
        print(f"Route found: length {len(initial_route)}")
        self.world.render(path=self.current_path, agent_loc=self.location)

        move_count = len(self.current_path) // 2
        print(f"\n2. Agent moving {move_count} steps...")
        self.location = self.current_path[move_count]
        
        block_pos = self.current_path[move_count + 2]
        self.world.set_dynamic_blocks([block_pos])
        
        print(f"\nAgent at {self.location}, path blocked!")
        self.world.render(path=self.current_path, agent_loc=self.location)

        print("\n3. Recalculating route...")
        new_route, _, _, _ = a_star_pathfinder(self.world, self.location, self.world.goal)
        
        if not new_route:
            print("Failed to find new route.")
            return
            
        print(f"New route found from {self.location}")
        
        final_route = self.current_path[:move_count+1] + new_route[1:]
        self.world.render(path=final_route, agent_loc=self.location)
        print("\n--- Simulation Complete ---")


def main():
    parser = argparse.ArgumentParser(description="Delivery Agent Sim")
    parser.add_argument("map", type=str, choices=ALL_MAPS.keys(), help="Map name")
    parser.add_argument("algo", type=str, choices=['ucs', 'astar', 'replan_demo'], help="Algorithm")
    
    args = parser.parse_args()
    
    layout = ALL_MAPS.get(args.map)
    
    try:
        world = CityGrid(layout)
        vehicle = Agent(world)
    except ValueError as e:
        print(f"Map Error: {e}")
        return

    print(f"Running on '{args.map}' with '{args.algo}'...")
    print("Initial State:")
    world.render()
    print("-" * 20)

    if args.algo == 'replan_demo':
        vehicle.replan_simulation()
    else:
        path, cost, nodes_expanded, exec_time = vehicle.find_route(args.algo)

        if path:
            print("Path Found!")
            print(f"  Cost: {cost}")
            print(f"  Nodes Expanded: {nodes_expanded}")
            print(f"  Time: {exec_time:.6f}s")
            print("\nFinal Path:")
            world.render(path=path)
        else:
            print("No path found.")
            print(f"  Nodes Expanded: {nodes_expanded}")
            print(f"  Time: {exec_time:.6f}s")

if __name__ == "__main__":
    main()