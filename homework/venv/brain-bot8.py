from flask import Flask, render_template_string, request
import heapq
import math
import time
from collections import deque

app = Flask(__name__)

# Function to convert DMS (degrees, minutes, seconds) to decimal degrees
def dms_to_decimal(degrees, minutes, seconds, direction):
    decimal = degrees + minutes / 60 + seconds / 3600
    if direction in ['S', 'W']:
        decimal *= -1
    return round(decimal, 6)

# GPS coordinates in DMS format
dms_coordinates = {
    "Main Gate": ("13°13'16.7\"N", "77°45'18.2\"E"),
    "Admin Block": ("13°13'19.9\"N", "77°45'18.9\"E"),
    "Academic Block 1": ("13°13'24.0\"N", "77°45'17.7\"E"),
    "Academic Block 2": ("13°13'24.2\"N", "77°45'21.4\"E"),
    "Academic Block 3": ("13°13'20.6\"N", "77°45'22.6\"E"),
    "Hostel": ("13°13'28.3\"N", "77°45'32.6\"E"),
    "Food Court": ("13°13'29.5\"N", "77°45'26.0\"E"),
    "Sports Complex": ("13°13'42.3\"N", "77°45'29.8\"E"),
    "Central Junction": ("13°13'22.0\"N", "77°45'20.2\"E"),
    "Laundry": ("13°13'28.3\"N", "77°45'25.4\"E"),
    "Mini Mart": ("13°13'28.4\"N", "77°45'32.9\"E")
}

# Convert all coordinates to decimal degrees
decimal_coordinates = {}
for location, (lat_dms, lon_dms) in dms_coordinates.items():
    lat_deg, lat_min, lat_sec, lat_dir = int(lat_dms.split("°")[0]), int(lat_dms.split("°")[1].split("'")[0]), float(lat_dms.split("'")[1].split('"')[0]), lat_dms[-1]
    lon_deg, lon_min, lon_sec, lon_dir = int(lon_dms.split("°")[0]), int(lon_dms.split("°")[1].split("'")[0]), float(lon_dms.split("'")[1].split('"')[0]), lon_dms[-1]
    lat_decimal = dms_to_decimal(lat_deg, lat_min, lat_sec, lat_dir)
    lon_decimal = dms_to_decimal(lon_deg, lon_min, lon_sec, lon_dir)
    decimal_coordinates[location] = (lat_decimal, lon_decimal)

# Create graph with distances
def haversine_distance(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    # This is the line to correct. Change *2 to **2
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
def breadth_first_search(graph, start, goal):
    start_time = time.time()
    queue = deque([(start, [start], 0)])
    visited = set()
    explored = []
    while queue:
        current, path, dist = queue.popleft()
        explored.append(current)
        if current == goal:
            return path, len(explored), dist, time.time() - start_time, explored
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                if neighbor['node'] not in visited:
                    queue.append((neighbor['node'], path + [neighbor['node']], dist + neighbor['distance']))
    return [], len(explored), 0, time.time() - start_time, explored

def depth_first_search(graph, start, goal):
    start_time = time.time()
    stack = [(start, [start], 0)]
    visited = set()
    explored = []
    while stack:
        current, path, dist = stack.pop()
        explored.append(current)
        if current == goal:
            return path, len(explored), dist, time.time() - start_time, explored
        if current not in visited:
            visited.add(current)
            for neighbor in reversed(graph[current]):
                if neighbor['node'] not in visited:
                    stack.append((neighbor['node'], path + [neighbor['node']], dist + neighbor['distance']))
    return [], len(explored), 0, time.time() - start_time, explored

def uniform_cost_search(graph, start, goal):
    start_time = time.time()
    queue = [(0, start, [start])]
    visited = set()
    explored = []
    while queue:
        cost, current, path = heapq.heappop(queue)
        explored.append(current)
        if current == goal:
            return path, len(explored), cost, time.time() - start_time, explored
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                if neighbor['node'] not in visited:
                    heapq.heappush(queue, (cost + neighbor['distance'], neighbor['node'], path + [neighbor['node']]))
    return [], len(explored), 0, time.time() - start_time, explored

def euclidean_search(graph, start, goal):
    start_time = time.time()
    queue = [(haversine_distance(*decimal_coordinates[start], *decimal_coordinates[goal]), start, [start], 0)]
    visited = set()
    explored = []
    while queue:
        _, current, path, dist = heapq.heappop(queue)
        explored.append(current)
        if current == goal:
            return path, len(explored), dist, time.time() - start_time, explored
        if current not in visited:
            visited.add(current)
            for neighbor in graph[current]:
                if neighbor['node'] not in visited:
                    heuristic = haversine_distance(*decimal_coordinates[neighbor['node']], *decimal_coordinates[goal])
                    heapq.heappush(queue, (heuristic, neighbor['node'], path + [neighbor['node']], dist + neighbor['distance']))
    return [], len(explored), 0, time.time() - start_time, explored

@app.route("/", methods=["GET"])
def index():
    start = request.args.get("start")
    goal = request.args.get("goal")
    results = {}
    best_algo = ""
    best_distance = float('inf')
    best_nodes = float('inf')
    best_path = []

    if start and goal and start in campus_graph and goal in campus_graph:
        algorithms = {
            "BFS": breadth_first_search,
            "DFS": depth_first_search,
            "UCS": uniform_cost_search,
            "Euclidean": euclidean_search
        }
        for name, func in algorithms.items():
            path, nodes, dist, exec_time, explored = func(campus_graph, start, goal)
            results[name] = {
                "path": path,
                "nodes": nodes,
                "distance": dist,
                "time": exec_time,
                "explored": explored
            }
            if path:
                if dist < best_distance or (dist == best_distance and nodes < best_nodes):
                    best_distance = dist
                    best_nodes = nodes
                    best_algo = name
                    best_path = path

    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Campus Navigator</title>
        <style>
            body { font-family: Arial; margin: 0; padding: 0; }
            .container { display: flex; flex-direction: column; height: 100vh; }
            .top-pane { display: flex; flex: 1; padding: 10px; }
            .left-pane, .right-pane { flex: 1; padding: 10px; }
            .bottom-pane { flex: 1; padding: 10px; }
            select, button { padding: 5px; margin: 5px; }
            .result-box { background: #f0f0f0; padding: 10px; margin-top: 10px; border-radius: 5px; }
        </style>
    </head>
    <body>
    <div class="container">
        <div class="top-pane">
            <div class="left-pane">
                <h2>Campus Navigation</h2>
                <form method="get">
                    <label>Source:</label>
                    <select name="start">
                        {% for loc in locations %}
                            <option value="{{loc}}" {% if loc == start %}selected{% endif %}>{{loc}}</option>
                        {% endfor %}
                    </select><br>
                    <label>Destination:</label>
                    <select name="goal">
                        {% for loc in locations %}
                            <option value="{{loc}}" {% if loc == goal %}selected{% endif %}>{{loc}}</option>
                        {% endfor %}
                    </select><br>
                    <button type="submit">Navigate</button>
                </form>
            </div>
            <div class="right-pane">
                {% if results %}
                    <h3>Algorithm Results</h3>
                    {% for algo, res in results.items() %}
                        <div class="result-box">
                            <strong>{{algo}}</strong><br>
                            Path: {{res.path}}<br>
                            Nodes Explored: {{res.nodes}}<br>
                            Distance: {{res.distance | round(2)}} meters<br>
                            Time: {{res.time | round(4)}} seconds<br>
                            Explored: {{res.explored}}
                        </div>
                    {% endfor %}
                    <h3>Best Algorithm</h3>
                    <div class="result-box">
                        <strong>{{best_algo}}</strong><br>
                        Reason: Shortest distance of {{best_distance | round(2)}} meters with minimum nodes explored ({{best_nodes}})
                    </div>
                {% endif %}
            </div>
        </div>
        <div class="bottom-pane">
            {% if best_path %}
                <h3>Google Maps Path</h3>
                <img src="https://maps.googleapis.com/maps/api/staticmap?size=600x400
                    {% for loc, coord in coords.items() %}
                        {% if loc == start %}
                            &markers=color:green|label:S|{{coord[0]}},{{coord[1]}}
                        {% elif loc == goal %}
                            &markers=color:red|label:E|{{coord[0]}},{{coord[1]}}
                        {% else %}
                            &markers=color:gray|label:{{loc}}|{{coord[0]}},{{coord[1]}}
                        {% endif %}
                    {% endfor %}
                    &path=color:0xff0000ff|weight:5
                    {% for node in best_path %}
                        |{{coords[node][0]}},{{coords[node][1]}}
                    {% endfor %}
                    &key=AIzaSyB632TYxfLOjAIzaSyAsqSWenLwW3zuBJxHxCdzSxTn4YV_L0IkjdLx5mBoOKLbAIAzmms6w0" alt="Map">
            {% endif %}
        </div>
    </div>
    </body>
    </html>
    """,
    locations=list(campus_graph.keys()),
    start=start,
    goal=goal,
    results=results,
    best_algo=best_algo,
    best_distance=best_distance,
    best_nodes=best_nodes,
    best_path=best_path,
    coords=decimal_coordinates)

if __name__ == "__main__":
    app.run(debug=True)