
#EFRATA ARON 2200705249 22/X/5249/PS
# number 5
# order of expanded states, the path returned by tree search, and unexpanded states

import heapq


weighted_graph = {
    'S': {'neighbors': {'A': 3, 'B': 1}, 'heuristic': 7},
    'A': {'neighbors': {'B': 2, 'C': 2}, 'heuristic': 5},
    'B': {'neighbors': {'C': 3}, 'heuristic': 7},
    'C': {'neighbors': {'D': 4, 'G': 4}, 'heuristic': 4},
    'D': {'neighbors': {'G': 1}, 'heuristic': 1},
    'G': {'neighbors': {}, 'heuristic': 0}
}

# Depth-First Search 
def depth_first_search_tree(graph, start, goal):
    stack = [(start, [])]
    visited = set()
    expanded_states = []

    while stack:
        node, path = stack.pop()
        expanded_states.append(node)
        if node == goal:
            return (
                "Depth-First Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

# Breadth-First Search 
def breadth_first_search_tree(graph, start, goal):
    queue = [(start, [])]
    visited = set()
    expanded_states = []

    while queue:
        node, path = queue.pop(0)
        expanded_states.append(node)
        if node == goal:
            return (
                "Breadth-First Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            queue.extend((neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# Uniform Cost Search 
def uniform_cost_search_tree(graph, start, goal):
    priority_queue = [(0, start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "Uniform Cost Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [node]))

# Greedy Search 
def greedy_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "Greedy Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            priority_queue.extend((graph[neighbor]['heuristic'], neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# A* Search 
def a_star_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], 0, start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "A* Search (Tree Search):",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost_so_far + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost + graph[neighbor]['heuristic'], new_cost, neighbor, path + [node]))


#  search algorithms for Tree Search
goal_state = 'G'
dfs_result = depth_first_search_tree(weighted_graph, 'S', goal_state)
bfs_result = breadth_first_search_tree(weighted_graph, 'S', goal_state)
ucs_result = uniform_cost_search_tree(weighted_graph, 'S', goal_state)
greedy_result = greedy_search_tree(weighted_graph, 'S', goal_state)
a_star_result = a_star_search_tree(weighted_graph, 'S', goal_state)


# Format and print results
def search_results(result):
    search_name, expanded_states, path, not_expanded = result
    print("-------------------------------------------------------------")
    print(f"{search_name}\nExpanded States: {expanded_states}\nPath: {path}\nNot Expanded States: {not_expanded}\n\n")

# Print results
search_results(("Depth-First Search (Tree Search):", dfs_result[1], dfs_result[2], dfs_result[3]))
search_results(("Breadth-First Search (Tree Search):", bfs_result[1], bfs_result[2], bfs_result[3]))
search_results(("Uniform Cost Search (Tree Search):", ucs_result[1], ucs_result[2], ucs_result[3]))
search_results(("Greedy Search (Tree Search):", greedy_result[1], greedy_result[2], greedy_result[3]))
search_results(("A* Search (Tree Search):", a_star_result[1], a_star_result[2], a_star_result[3]))
