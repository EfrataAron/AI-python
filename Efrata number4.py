#EFRATA ARON 2200705249 22/X/5249/PS
# number 4
# implementing searches for graph search
import heapq

# Define the weighted graph as a dictionary
weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}

#DEPTH FIRST SEARCH
def depth_first_search(graph, start, goal):
    stack = [(start, [start])]
    while stack:
        (node, path) = stack.pop()
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                stack.append((neighbor, path + [neighbor]))

# BREADTH FIRST SEARCH
def breadth_first_search(graph, start, goal):
    queue = [(start, [start])]
    while queue:
        (node, path) = queue.pop(0)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                queue.append((neighbor, path + [neighbor]))

# UNIFORM COST SEARCH
def uniform_cost_search(graph, start, goal):
    priority_queue = [(0, start, [start])]
    while priority_queue:
        (cost, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor], cost + graph[node][neighbor]
            else:
                new_cost = cost + graph[node][neighbor]
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

# GREEDY SEARCH
def G(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], start, [start])]
    while priority_queue:
        (_, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                heapq.heappush(priority_queue, (heuristic[neighbor], neighbor, path + [neighbor]))

# A* SEARCH
def A_star_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], 0, start, [start])]
    while priority_queue:
        (_, cost, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor], cost + graph[node][neighbor]
            else:
                new_cost = cost + graph[node][neighbor]
                heapq.heappush(priority_queue, (new_cost + heuristic[neighbor], new_cost, neighbor, path + [neighbor]))




# Define a heuristic function for Greedy and A* search
heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

start_node = 'S'
goal_node = 'G'


# Perform searches and print paths
dfs_path = list(depth_first_search(weighted_graph, start_node, goal_node))[0]
bfs_path = list(breadth_first_search(weighted_graph, start_node, goal_node))[0]
ucs_path, ucs_cost = next(uniform_cost_search(weighted_graph, start_node, goal_node))
Greedy_search_path = list(G(weighted_graph, start_node, goal_node, heuristics))[0]
A_star_search_path, A_star_cost = next(A_star_search(weighted_graph, start_node, goal_node, heuristics))



def format(path):
    return " -> ".join(path)

print("-----------------------------------------------------")
print("Depth First Search Path:", format(dfs_path))
print()
print("Breadth First Search Path:", format(bfs_path))
print()
print("Uniform Cost Search Path:", format(ucs_path) + " Cost:", ucs_cost)
print()
print("Greedy Search Path:", format(Greedy_search_path))
print()

print("A* Search Path:", format(A_star_search_path) + " Cost:", A_star_cost)

print("-----------------------------------------------------")


