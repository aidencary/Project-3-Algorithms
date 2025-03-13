# Project 3 Algorithms
# Aiden Cary, Dalton Gorham, and Nathan Wetherington
# Coded with the help of ChatGPT and Github Copilot

import networkx as nx
import matplotlib.pyplot as plt

# Graph class using NetworkX
class GraphNX:
    def __init__(self, directed=False):
        self.G = nx.DiGraph() if directed else nx.Graph()

    def add_edge(self, u, v):
        self.G.add_edge(u, v)

    def find_connected_components(self):
        if self.G.is_directed():
            raise ValueError("Connected components are for undirected graphs only.")
        
        components = list(nx.connected_components(self.G))
        return [list(comp) for comp in components]

    # Find the shortest path between two nodes
    # This may not be used in the final implementation
    def find_path(self, start, end):
        try:
            path = nx.shortest_path(self.G, source=start, target=end, method="dfs")
            return path
        except nx.NetworkXNoPath:
            return None

    # Find the topological sort of the graph
    def topological_sort(self):
        if not self.G.is_directed():
            raise ValueError("Topological sorting is only valid for directed graphs.")
        
        if not nx.is_directed_acyclic_graph(self.G):
            raise ValueError("Graph is not a DAG; topological sorting is not possible.")

        return list(nx.topological_sort(self.G))

    # Breadth-first search from sample code in Bb
    def bfs(self, start):
        visited, queue = set(), [start]
        p = []
        while queue:
            vertex = queue.pop(0)
            if vertex not in visited:
                visited.add(vertex)
                p.append(vertex)
                queue.extend(set(self.G.neighbors(vertex)) - visited)
        return p
    
    # Depth-first search from sample code in Bb
    def dfs(self, start, visited=None):
        global t
        t += 1
        print('DFS called', t, 'times.')

        if visited is None:
            visited = set()

        visited.add(start)

        for neighbor in set(self.G.neighbors(start)) - visited:
            self.dfs(neighbor, visited)
        return visited

    # Draws the graph with a structured layout matching the given image in the project description
    def draw_graph(self, highlight_path=None):
        """Draw the graph with a structured layout matching the given image"""
        pos = {
            'A': (0, 3), 'B': (1, 3), 'C': (2, 3), 'D': (3, 3),
            'E': (0, 2), 'F': (1, 2), 'G': (2, 2), 'H': (3, 2),
            'I': (0, 1), 'J': (1, 1), 'K': (2, 1), 'L': (3, 1),
            'M': (0, 0), 'N': (1, 0), 'O': (2, 0), 'P': (3, 0)
        }

        plt.figure(figsize=(6, 6))
        nx.draw(self.G, pos, with_labels=True, node_color='lightblue', edge_color='gray', node_size=1500, font_size=12, font_weight="bold")

        if highlight_path:
            path_edges = list(zip(highlight_path, highlight_path[1:]))
            nx.draw_networkx_edges(self.G, pos, edgelist=path_edges, edge_color='red', width=2)

        plt.show()

# Example Usage
g = GraphNX(directed=False)

# Updated edges based on the requirements
edges = [
    ('A', 'B'), ('B', 'C'), ('C', 'D'),
    ('A', 'E'), ('A', 'F'), ('B', 'F'), ('C', 'G'), ('D', 'G'),
    ('E', 'F'), ('E', 'I'), ('F', 'I'), ('I', 'J'), ('I', 'M'), 
    ('K', 'L'), ('L', 'P'), ('M', 'N'), 
    ('O', 'K'), ('K', 'H'), ('L', 'H')
]

for u, v in edges:
    g.add_edge(u, v)

# Initialize global variable for DFS call count
t = 0

# Example BFS call
bfs_path = g.bfs('A')
print("BFS path:", bfs_path)
g.draw_graph(highlight_path=bfs_path)

# Example DFS call
visited_nodes = g.dfs('A')
print("DFS visited nodes:", visited_nodes)
g.draw_graph(highlight_path=list(visited_nodes))