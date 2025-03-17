# Project 3 Algorithms
# Aiden Cary, Dalton Gorham, and Nathan Wetherington
# Coded with the help of ChatGPT and Github Copilot

# Program description:
# This program creates a graph using NetworkX and performs a breadth-first search (BFS) and depth-first search (DFS) on the graph.
# The graph is then drawn with a structured layout matching the given image in the project description.
# The BFS and DFS functions are called on the graph, and the visited nodes are printed.
# The graph is then drawn with the visited nodes highlighted in red.

# Order of graphs printed:
# 1. Initial graph before a search
# 2. Graph after BFS search with visited nodes highlighted in red
# 3. Graph after DFS search with visited nodes highlighted in red

import networkx as nx
import matplotlib.pyplot as plt

# Graph class using NetworkX
class GraphNX:
    def __init__(self, directed=False):
        self.G = nx.DiGraph() if directed else nx.Graph()

    def add_edge(self, u, v):
        self.G.add_edge(u, v)

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
        
        # Ensure all vertices are visited
        for node in self.G.nodes:
            if node not in visited:
                queue.append(node)
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
        
        # Ensure all vertices are visited
        for node in self.G.nodes:
            if node not in visited:
                self.dfs(node, visited)
        
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
    

         # Draws the graph with a structured layout matching the given image in the project description
    def draw_graph2(self, highlight_path=None):
        """Draw the graph with a structured layout matching the given image"""
        pos = {
            1: (.5, 3), 2: (1, 2), 3: (2, 3), 4: (0, 2.5),
            5: (3, 2.5 ), 6: (4, 2), 7: (3.5, 1), 8: (2.5, 1.5),
            9: (1.5, 1.7), 10: (2, 1), 11: (1, 1), 12: (.5, 1.8)
        }

        plt.figure(figsize=(6, 6))
        nx.draw(self.G, pos, with_labels=True, node_color='lightblue', edge_color='gray', node_size=1500, font_size=12, font_weight="bold")

        if highlight_path:
            path_edges = list(zip(highlight_path, highlight_path[1:]))
            nx.draw_networkx_edges(self.G, pos, edgelist=path_edges, edge_color='red', width=2)

        plt.show()


 # Find and print strongly connected components
    def find_scc(self):
        scc = list(nx.strongly_connected_components(self.G))
        print("Strongly Connected Components:")
        for component in scc:
            print(component)
        return scc
    

       # Draw the meta graph of strongly connected components
    def draw_meta_graph(self):
        scc = self.find_scc()
        meta_graph = nx.DiGraph()

        # Create a mapping from node to its SCC index
        node_to_scc = {}
        for i, component in enumerate(scc):
            for node in component:
                node_to_scc[node] = i

        # Add nodes for each SCC
        for i in range(len(scc)):
            meta_graph.add_node(i)

        # Add edges between SCCs
        for u, v in self.G.edges():
            if node_to_scc[u] != node_to_scc[v]:
                meta_graph.add_edge(node_to_scc[u], node_to_scc[v])

        labels = {i: ','.join(map(str, component)) for i, component in enumerate(scc)}

        # Draw the meta graph
        pos = nx.spring_layout(meta_graph)
        plt.figure(figsize=(6, 6))
        nx.draw(meta_graph, pos, labels= labels, node_color='lightblue', edge_color='gray', node_size=2500, font_size=12, font_weight="bold")
        plt.show()
        
        return meta_graph
    
    def topological_sort_meta_graph(self):
        meta_graph = self.draw_meta_graph()
        topo_order = list(nx.topological_sort(meta_graph))
        print("Topological Order of the Meta Graph:", topo_order)
        return topo_order





def q1():

        # Example Usage
    g = GraphNX(directed=False)

    # Updated edges based on the requirements
    edges = [
        ('A', 'B'), ('B', 'C'), ('C', 'D'),
        ('A', 'E'), ('A', 'F'), ('B', 'F'), ('C', 'G'), ('D', 'G'), ('G', 'J'),
        ('E', 'F'), ('E', 'I'), ('F', 'I'), ('I', 'J'), ('I', 'M'), 
        ('K', 'L'), ('L', 'P'), ('M', 'N'), 
        ('O', 'K'), ('K', 'H'), ('L', 'H')
    ]

    for u, v in edges:
        g.add_edge(u, v)

    # Initialize global variable for BFS and DFS call count
    
    # Draw the initial graph before a search
    g.draw_graph()

    # Example BFS call
    bfs_path = g.bfs('A')
    print("BFS path:", bfs_path)
    g.draw_graph(highlight_path=bfs_path)

    # Example DFS call
    visited_nodes = g.dfs('A')
    print("DFS visited nodes:", visited_nodes)
    g.draw_graph(highlight_path=list(visited_nodes))





def q2():
        

    digraph = GraphNX(directed=True)

    edges = [
        (1, 3),(2, 1), (3,2), (3, 5), (4, 1), (4, 2), (4, 12), (5, 6), (5, 8),
        (6, 7), (6, 8), (6, 10), (7, 10), (8, 10), (8, 9), (9,5), (9, 11), (10, 9),
        (10, 11), (11, 12)
    ]

    for u, v in edges:
        digraph.add_edge(u, v)

    digraph.draw_graph2()
    # Find and print strongly connected components
    digraph.find_scc()
    # Draw the meta graph of strongly connected components
    digraph.draw_meta_graph()
    # Topological sort of the meta graph
    digraph.topological_sort_meta_graph()


t = 0
# Quesiton 1
q1()
# Question 2
#q2()

