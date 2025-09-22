import heapq
import math

# A class to represent a single node in the search grid
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        # Heuristic cost (estimated distance to the goal)
        self.h = 0
        # This is f-cost, which is just the heuristic for Greedy Best-First Search
        self.f = 0

    # Override the equality operator to compare positions
    def __eq__(self, other):
        return self.position == other.position

    # Override the less-than operator for use with heapq (priority queue)
    def __lt__(self, other):
        return self.f < other.f

# Get all valid neighboring nodes from the current node
def get_neighbors(grid, node, allow_diagonals=False):
    neighbors = []
    # Cardinal directions (up, down, left, right)
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    if allow_diagonals:
        # Add diagonal directions
        directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])

    for move in directions:
        node_pos = (node.position[0] + move[0], node.position[1] + move[1])

        # Check if the neighbor is within the grid boundaries
        if not (0 <= node_pos[0] < len(grid) and 0 <= node_pos[1] < len(grid[0])):
            continue

        # Check for obstacles ('1' represents a wall)
        if grid[node_pos[0]][node_pos[1]] == '1':
            continue

        neighbors.append(Node(node, node_pos))
    return neighbors

# Reconstruct the path from the end node back to the start
def get_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1] # Reverse the path to show it from start to end

# The main Greedy Best-First Search algorithm
def greedy_best_first_search(grid, start, end, heuristic_name='manhattan', allow_diagonals=False):
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    closed_set = set()
    nodes_explored = 0

    # Push the start node onto the priority queue
    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        _, current_node = heapq.heappop(open_list)
        nodes_explored += 1

        if current_node == end_node:
            path = get_path(current_node)
            return path, len(path) - 1, nodes_explored

        closed_set.add(current_node.position)

        for neighbor in get_neighbors(grid, current_node, allow_diagonals):
            if neighbor.position in closed_set:
                continue

            # Calculate the heuristic cost
            dx = abs(neighbor.position[0] - end_node.position[0])
            dy = abs(neighbor.position[1] - end_node.position[1])

            if heuristic_name == 'euclidean':
                neighbor.h = math.sqrt(dx**2 + dy**2)
            elif heuristic_name == 'diagonal':
                neighbor.h = dx + dy + (math.sqrt(2) - 2) * min(dx, dy)
            else:  # Default to Manhattan distance
                neighbor.h = dx + dy
            
            # The f-cost is the heuristic in Greedy Best-First Search
            neighbor.f = neighbor.h

            # Add the neighbor to the open list if it's not already there
            if not any(open_node for _, open_node in open_list if open_node == neighbor):
                heapq.heappush(open_list, (neighbor.f, neighbor))

    return None, 0, nodes_explored

if __name__ == "__main__":
    # S: Start, G: Goal, 1: Wall, 0: Open, Z: Ghost Zone
    haunted_house_grid = [
        ['S', '0', '0', '0', '0', '1', '0'],
        ['0', '1', '1', '1', '0', '1', '0'],
        ['0', '0', '0', '0', '0', '0', '0'],
        ['0', '1', '0', '1', '1', '1', '1'],
        ['0', '1', '0', '0', '0', '0', '0'],
        ['0', '1', '1', '0', '1', '1', '0'],
        ['0', '0', '0', '0', '0', '0', 'G']
    ]
    start_pos = (0, 0)
    end_pos = (6, 6)
    
    print("--- Pathfinding with Greedy Best-First Search ---")
    
    path_g, len_g, exp_g = greedy_best_first_search(haunted_house_grid, start_pos, end_pos)

    print(f"\nGreedy Search Results:")
    if path_g:
        print(f"Path Found: {path_g}")
        print(f"Path Length: {len_g}")
        print(f"Nodes Explored: {exp_g}")
    else:
        print("No path found.")
        print(f"Nodes Explored: {exp_g}")