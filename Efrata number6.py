#EFRATA ARON 2200705249 22/X/5249/PS
# number 6
# order of expanded states, the path returned by graph search, and unexpanded states

from collections import deque
import heapq

# Define the weighted graph as a dictionary
weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'B': 1, 'A': 2, 'C': 3},
    'C': {'A': 2, 'B': 3, 'D': 4, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'C': 4, 'D': 1}
}

# Heuristics for each node
heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

# Breadth-First Search (BFS)
def breadth_first_search(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])
    expanded_states = []

    while queue:
        node, path = queue.popleft()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, []  # Return an empty list for unexpanded states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None, expanded_states, []  


# Depth-First Search (DFS)
def depth_first_search(graph, start, goal):
    visited = set()
    stack = [(start, [start])]
    expanded_states = []

    while stack:
        node, path = stack.pop()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

    return None, expanded_states, []


# Uniform Cost Search (UCS)
def Uniform_Cost_Search(graph, start, goal):
    visited = set()
    priority_queue = [(0, start, [start])]
    expanded_states = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    new_cost = cost + graph[node][neighbor]
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

    return None, expanded_states, []  


# A* Search
def A_Star_Search(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        if len(priority_queue) == 0:
            break  # No path found
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    cost = path_cost(path) + graph[node][neighbor]
                    heapq.heappush(priority_queue, (cost + heuristics[neighbor], neighbor, path + [neighbor]))

    return None, expanded_states

# Greedy Search
def Greedy_Search(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (heuristics[neighbor], neighbor, path + [neighbor]))

    unexpanded_states = [node for node in graph if node not in visited]
    return None, expanded_states, unexpanded_states

# Calculate the path cost
def path_cost(path):
    cost = 0
    for i in range(len(path) - 1):
        cost += weighted_graph[path[i]][path[i + 1]]
    return cost

# Perform searches
start_node = 'S'
goal_node = 'G'

bfs_result = breadth_first_search(weighted_graph, start_node, goal_node)
dfs_result = depth_first_search(weighted_graph, start_node, goal_node)
ucs_result = Uniform_Cost_Search(weighted_graph, start_node, goal_node)
astar_result = A_Star_Search(weighted_graph, start_node, goal_node, heuristics)
greedy_result = Greedy_Search(weighted_graph, start_node, goal_node, heuristics)


#Print Breadth-First Search
bfs_path, bfs_expanded, bfs_unexpanded = breadth_first_search(weighted_graph, start_node, goal_node)
if bfs_path is not None:
    print("-------------------------------------------------------------")
    print("Breadth-First-Search")
    print("Path:", bfs_path)
else:
   
    print("BFS did not find a path.")
print("Expanded States:", bfs_expanded)
print("Unexpanded States:", bfs_unexpanded)
print()

#Print Depth-First Search
dfs_path, dfs_expanded, dfs_unexpanded = depth_first_search(weighted_graph, start_node, goal_node)
if dfs_path is not None:
    print("-------------------------------------------------------------")
    print("Depth-First-Search")
    print("Path:", dfs_path)
else:
    print("DFS did not find a path.")
print("Expanded States:", dfs_expanded)
print("Unexpanded States:", dfs_unexpanded)
print()

#Print Uniform Cost Search
ucs_path, ucs_expanded, ucs_unexpanded = Uniform_Cost_Search(weighted_graph, start_node, goal_node)
if ucs_path is not None:
    print("-------------------------------------------------------------")
    print("Uniform-Cost-Search")
    print("Path:", ucs_path)
else:
    print("UCS did not find a path.")
print("Expanded States:", ucs_expanded)
print("Unexpanded States:", ucs_unexpanded)
print()

astar_path, astar_expanded = A_Star_Search(weighted_graph, start_node, goal_node, heuristics)

#Print A* Search 
if astar_path is not None:
    print("-------------------------------------------------------------")
    print("A* Search")
    print("Path:", astar_path)
    print("Expanded States:", astar_expanded)
    print()
else:
    print("A* Search did not find a path.")



greedy_path, greedy_expanded = Greedy_Search(weighted_graph, start_node, goal_node, heuristics)

# Print Greedy Search 
if greedy_path is not None:
    print("-------------------------------------------------------------")
    print("Greedy-Search")
    print("Path:", greedy_path)
    print("Expanded States:", greedy_expanded)
    print()
    print("-------------------------------------------------------------")
    
else:
    print("Greedy Search did not find a path.")




