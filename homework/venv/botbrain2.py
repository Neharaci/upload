from flask import Flask, render_template_string, request
import heapq
import math
import time
from collections import deque

app = Flask(__name__)

# Coordinates for each campus location
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

# Create graph with distances
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000
    return c * r

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

campus_graph = create_campus_graph()

# Search algorithms
# BFS and DFS are not included as they are not optimal for weighted graphs

def uniform_cost_search(graph, start, goal):
    start_time = time.time()
    queue = [(0, start, [start])]
    visited = set()
    explored = []
    while queue:
        cost, current_node, path = heapq.heappop(queue)
        explored.append(current_node)
        if current_node == goal:
            return {
                'algorithm': 'UCS',
                'path': path,
                'nodes_explored': len(explored),
                'distance': cost,
                'execution_time': time.time() - start_time,
                'explored_nodes': explored
            }
        if current_node in visited:
            continue
        visited.add(current_node)
        for neighbor_info in graph[current_node]:
            neighbor = neighbor_info['node']
            distance = neighbor_info['distance']
            if neighbor not in visited:
                heapq.heappush(queue, (cost + distance, neighbor, path + [neighbor]))
    return None

def euclidean_search(graph, start, goal):
    start_time = time.time()
    queue = [(haversine_distance(*decimal_coordinates[start], *decimal_coordinates[goal]), start, [start], 0)]
    visited = set()
    explored = []
    while queue:
        _, current_node, path, total_distance = heapq.heappop(queue)
        explored.append(current_node)
        if current_node == goal:
            return {
                'algorithm': 'A* (Euclidean)',
                'path': path,
                'nodes_explored': len(explored),
                'distance': total_distance,
                'execution_time': time.time() - start_time,
                'explored_nodes': explored
            }
        if current_node in visited:
            continue
        visited.add(current_node)
        for neighbor_info in graph[current_node]:
            neighbor = neighbor_info['node']
            distance = neighbor_info['distance']
            if neighbor not in visited:
                g_cost = total_distance + distance
                h_cost = haversine_distance(*decimal_coordinates[neighbor], *decimal_coordinates[goal])
                f_cost = g_cost + h_cost
                heapq.heappush(queue, (f_cost, neighbor, path + [neighbor], g_cost))
    return None

# HTML template
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
 <title>Campus Navigator</title>
 <style>
  body { font-family: Arial; margin: 20px; }
  .container { display: flex; flex-direction: column; gap: 20px; }
  .top-pane { padding: 20px; border: 1px solid #ccc; }
  .bottom-pane { padding: 20px; border: 1px solid #ccc; }
  label, select, button { margin: 5px; }
 </style>
</head>
<body>
 <h2>Campus Navigation System</h2>
 <div class="container">
  <div class="top-pane">
   <form method="GET">
    <label for="start">Source:</label>
    <select name="start" required>
     {% for loc in locations %}
      <option value="{{ loc }}" {% if loc == start %}selected{% endif %}>{{ loc }}</option>
     {% endfor %}
    </select>
    <label for="goal">Destination:</label>
    <select name="goal" required>
     {% for loc in locations %}
      <option value="{{ loc }}" {% if loc == goal %}selected{% endif %}>{{ loc }}</option>
     {% endfor %}
    </select>
    <button type="submit">Navigate</button>
   </form>
   {% if result %}
    <h3>Best Algorithm: {{ result.algorithm }}</h3>
    <p><strong>Path:</strong> {{ result.path | join(' → ') }}</p>
    <p><strong>Nodes Explored:</strong> {{ result.nodes_explored }}</p>
    <p><strong>Distance:</strong> {{ result.distance | round(2) }} meters</p>
    <p><strong>Execution Time:</strong> {{ result.execution_time | round(4) }} seconds</p>
    <p><strong>Exploration Steps:</strong> {{ result.explored_nodes | join(', ') }}</p>
   {% endif %}
  </div>
  <div class="bottom-pane">
   {% if result %}
    <h3>Navigation Map</h3>
    <img src="https://maps.googleapis.com/maps/api/staticmap?size=600x400&maptype=roadmap{% for node in result.path %}&markers=color:red%7Clabel:{{ loop.index }}%7C{{ coordinates[node][0] }},{{ coordinates[node][1] }}{% endfor %}&path=color:blue|weight:5{% for node in result.path %}|{{ coordinates[node][0] }},{{ coordinates[node][1] }}{% endfor %}&key=AIzaSyB632TYxfLOjjdLx5mBoOKLbAIAzmms6w0" alt="Map">
   {% endif %}
  </div>
 </div>
</body>
</html>
"""

@app.route('/', methods=['GET'])
def index():
    start = request.args.get('start')
    goal = request.args.get('goal')
    result = None
    if start and goal and start in campus_graph and goal in campus_graph:
        # Run only the algorithms suitable for weighted graphs
        results = [
            uniform_cost_search(campus_graph, start, goal),
            euclidean_search(campus_graph, start, goal)
        ]
        # Filter out any None results before finding the minimum
        valid_results = [res for res in results if res is not None]
        
        if valid_results:
            result = min(valid_results, key=lambda x: x['distance'])
        else:
            result = None # Or handle the case where no path is found
    return render_template_string(HTML_TEMPLATE, locations=list(campus_graph.keys()), start=start, goal=goal, result=result, coordinates=decimal_coordinates)

if __name__ == '__main__':
    app.run(debug=True)