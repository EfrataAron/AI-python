#EFRATA ARON 2200705249 22/X/5249/PS
# number 2
#using sets and dictionaries to represent into a python graph
# search algorithms

# Define a set of nodes
nodes = {'S', 'A', 'B', 'C', 'D', 'G'}

# Define a weighted directed graph using a dictionary-based representation
weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}

# Define a function to extract graph information
def extract_graph_info(graph):
   
    nodes = set(graph.keys())
   
    edges = []

    # Iterate through nodes and their neighbors
    for node, neighbors in graph.items():
        
        for neighbor, weight in neighbors.items():
       
            edges.append((node, neighbor, weight))

 
    return nodes, edges

#  extract nodes and edges from the graph by calling the function
nodes, edges = extract_graph_info(weighted_graph)


print("---------------------------------------")
print("Nodes:", nodes)
print("---------------------------------------")
print("Edges and their weights:")

for edge in edges:
    print(edge)
print("---------------------------------------")