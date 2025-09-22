from flask import Flask, render_template_string, request
import heapq
import math
import time
from collections import deque
app = Flask(__name__)
# Coordinates for each location
decimal_coordinates = {
    "Main Gate": (12.9716, 77.5946),
    "Admin Block": (12.9720, 77.5950),
    "Central Junction": (12.9730, 77.5960),
    "Academic Block 1": (12.9740, 77.5970),
    "Academic Block 2": (12.9750, 77.5980),
    "Academic Block 3": (12.9760, 77.5990),
    "Food Court": (12.9770, 77.6000),
    "Hostel": (12.9780, 77.6010),
    "Mini Mart": (12.9790, 77.6020),
    "Sports Complex": (12.9800, 77.6030),
    "Laundry": (12.9810, 77.6040)
}
# Haversine distance function
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000
    return c * r
# Create graph from coordinates
def create_campus_graph():
    graph = {location: [] for location in decimal_coordinates.keys()}
    connections = [
        ("Main Gate", "Admin Block"),
        ("Admin Block", "Central Junction"),
        ("Central Junction", "Academic Block 1"),
        ("Central Junction", "Academic Block 2"),
        ("Central Junction", "Academic Block 3"),
        ("Academic Block 2", "Food Court"),
        ("Food Court", "Laundry"),
        ("Food Court", "Hostel"),
        ("Food Court", "Mini Mart"),
        ("Hostel", "Mini Mart"),
        ("Food Court", "Sports Complex"),
        ("Laundry", "Hostel")
    ]
    for loc1, loc2 in connections:
        lat1, lon1 = decimal_coordinates[loc1]
        lat2, lon2 = decimal_coordinates[loc2]
        distance = haversine_distance(lat1, lon1, lat2, lon2)
        graph[loc1].append({'node': loc2, 'distance': round(distance, 2)})
        graph[loc2].append({'node': loc1, 'distance': round(distance, 2)})
    return graph
# BFS algorithm
def breadth_first_search(graph, start, goal):
    start_time = time.time()
    queue = deque([(start, [start], 0)])
    visited = set()
    nodes_explored = 0
    while queue:
        current_node, path, total_distance = queue.popleft()
        nodes_explored += 1
        if current_node == goal:
            return path, nodes_explored, total_distance, time.time() - start_time
        if current_node not in visited:
            visited.add(current_node)
            for neighbor_info in graph[current_node]:
                neighbor = neighbor_info['node']
                distance = neighbor_info['distance']
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor], total_distance + distance))
    return None, nodes_explored, 0, time.time() - start_time
# DFS algorithm
def depth_first_search(graph, start, goal):
    start_time = time.time()
    stack = [(start, [start], 0)]
    visited = set()
    nodes_explored = 0
    while stack:
        current_node, path, total_distance = stack.pop()
        nodes_explored += 1
        if current_node == goal:
            return path, nodes_explored, total_distance, time.time() - start_time
        if current_node not in visited:
            visited.add(current_node)
            neighbors = []
            for neighbor_info in graph[current_node]:
                neighbor = neighbor_info['node']
                distance = neighbor_info['distance']
                if neighbor not in visited:
                    neighbors.append((neighbor, path + [neighbor], total_distance + distance))
            for neighbor_data in reversed(neighbors):
                stack.append(neighbor_data)
    return None, nodes_explored, 0, time.time() - start_time
# UCS algorithm
def uniform_cost_search(graph, start, goal):
    start_time = time.time()
    queue = [(0, start, [start])]
    visited = set()
    nodes_explored = 0
    while queue:
        cost, current_node, path = heapq.heappop(queue)
        nodes_explored += 1
        if current_node == goal:
            return path, nodes_explored, cost, time.time() - start_time
        if current_node not in visited:
            visited.add(current_node)
            for neighbor_info in graph[current_node]:
                neighbor = neighbor_info['node']
                distance = neighbor_info['distance']
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + distance, neighbor, path + [neighbor]))
    return None, nodes_explored, 0, time.time() - start_time
# Euclidean search using haversine heuristic
def euclidean_search(graph, start, goal):
    start_time = time.time()
    queue = [(haversine_distance(*decimal_coordinates[start], *decimal_coordinates[goal]), 0, start, [start])]
    visited = set()
    nodes_explored = 0
    while queue:
        _, cost, current_node, path = heapq.heappop(queue)
        nodes_explored += 1
        if current_node == goal:
            return path, nodes_explored, cost, time.time() - start_time
        if current_node not in visited:
            visited.add(current_node)
            for neighbor_info in graph[current_node]:
                neighbor = neighbor_info['node']
                distance = neighbor_info['distance']
                if neighbor not in visited:
                    heuristic = haversine_distance(*decimal_coordinates[neighbor], *decimal_coordinates[goal])
                    heapq.heappush(queue, (cost + distance + heuristic, cost + distance, neighbor, path + [neighbor]))
    return None, nodes_explored, 0, time.time() - start_time
# HTML template
HTML_TEMPLATE = """
    
    
Campus Navigation
    <form method="get"></form method="get">
        <label for="start">Start Location:</label for="start">
        <select name="start"></select name="start">
            {% for loc in locations %}
            <option value="{{ loc }}" {% if loc == start %}selected{% endif %}>{{ loc }}</option value="{{ loc }}" {% if loc == start %}selected{% endif %}>
            {% endfor %}
        
        <label for="goal">Destination:</label for="goal">
        <select name="goal"></select name="goal">
            {% for loc in locations %}
            <option value="{{ loc }}" {% if loc == goal %}selected{% endif %}>{{ loc }}</option value="{{ loc }}" {% if loc == goal %}selected{% endif %}>
            {% endfor %}
        
        <input type="submit" value="navigate"></input type="submit" value="navigate">
    
    {% if best_result %}
    
Best Path Result
    
Algorithm: {{ best_result.algorithm }}
    
Path: {{ best_result.path | join(' → ') }}
    
Nodes Explored: {{ best_result.nodes }}
    
Distance: {{ best_result.distance | round(2) }} meters
    
Execution Time: {{ best_result.time | round(4) }} seconds
    {% endif %}
"""
@app.route('/', methods=['GET'])
def index():
    graph = create_campus_graph()
    start = request.args.get('start', 'Main Gate')
    goal = request.args.get('goal', 'Laundry')
    algorithms = {
        'BFS': breadth_first_search,
        'DFS': depth_first_search,
        'UCS': uniform_cost_search,
        'Euclidean': euclidean_search
    }
    results = []
    for name, func in algorithms.items():
        path, nodes, distance, exec_time = func(graph, start, goal)
        if path:
            results.append({
                'algorithm': name,
                'path': path,
                'nodes': nodes,
                'distance': distance,
                'time': exec_time
            })
    best_result = min(results, key=lambda x: x['distance']) if results else None
    return render_template_string(HTML_TEMPLATE,
                                  locations=list(decimal_coordinates.keys()),
                                  start=start,
                                  goal=goal,
                                  best_result=best_result)
if __name__ == '__main__':
   app.run(debug=True)



#'''MAPS API KEY: AIzaSyB632TYxfLOjjdLx5mBoOKLbAIAzmms6w0'''




