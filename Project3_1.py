# Project 3 Algorithms
# Authors: Aiden Cary, Dalton Gorham, and Nathan Wetherington
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
import heapq

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
    
    # Dijkstra's algorithm made with ChatGPT
    def dijkstra(self, start):
        """Apply Dijkstra's algorithm to find the shortest paths from the start node."""
        distances = {node: float('inf') for node in self.G.nodes}
        distances[start] = 0
        pq = [(0, start)]
        
        while pq:
            current_distance, current_node = heapq.heappop(pq)
            
            for neighbor in self.G.neighbors(current_node):
                weight = self.G[current_node][neighbor]['weight']
                distance = current_distance + weight
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    heapq.heappush(pq, (distance, neighbor))
        
        return distances
    
    # Finding the minimum spanning tree using Prim's algorithm
    # Made with ChatGPT
    def minimum_spanning_tree(self):
            """Generate and display the minimum spanning tree using Prim's algorithm."""
            mst = nx.minimum_spanning_tree(self.G)
            pos = nx.spring_layout(mst)
            plt.figure(figsize=(8, 6))
            nx.draw(mst, pos, with_labels=True, node_color='lightblue', edge_color='gray',
                    node_size=2000, font_size=12, font_weight="bold")
            edge_labels = {(u, v): self.G[u][v]['weight'] for u, v in mst.edges}
            nx.draw_networkx_edge_labels(mst, pos, edge_labels=edge_labels, font_size=10)
            plt.show()
            return mst

    
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

    def draw_graph3(self):
        # Define graph
        G = nx.Graph()

        # Add edges with weights
        edges = [
            ('A', 'B', 22), ('A', 'C', 9), ('A', 'D', 12),
            ('B', 'C', 35), ('B', 'F', 36), ('B', 'H', 34),
            ('C', 'D', 4), ('C', 'E', 65), ('C', 'F', 42),
            ('D', 'E', 33), ('D', 'I', 30),
            ('E', 'F', 18), ('E', 'G', 23),
            ('F', 'G', 39), ('F', 'H', 24),
            ('G', 'H', 25), ('G', 'I', 21), ('I', 'H', 19)
        ]

        # Add edges to the graph
        for u, v, w in edges:
            G.add_edge(u, v, weight=w)

        # Define positions manually for better visualization
        pos = {
            'A': (0, 2), 'B': (2, 4), 'C': (2, 2), 'D': (0, 0),
            'E': (3, 1), 'F': (4, 3), 'G': (5, 2), 'H': (6, 4),
            'I': (6, 0)
        }

        # Draw the graph
        plt.figure(figsize=(8, 6))
        nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray',
                node_size=2000, font_size=12, font_weight="bold")

        # Draw edge labels (weights)
        edge_labels = {(u, v): w for u, v, w in edges}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=10)

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
    
    # Topological sort of the meta graph
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
    print("Initial Graph")
    g.draw_graph()

    # Example BFS call
    bfs_path = g.bfs('A')
    print("Graph after BFS")
    print("BFS path:", bfs_path)
    g.draw_graph(highlight_path=bfs_path)

    # Example DFS call
    visited_nodes = g.dfs('A')
    print("Graph after DFS")
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

    # Draw the initial graph before a search
    print("Initial Graph")
    digraph.draw_graph2()
    # Find and print strongly connected components
    print("Strongly Connected Components")
    digraph.find_scc()
    # Draw the meta graph of strongly connected components
    print("Meta Graph of Strongly Connected Components")
    digraph.draw_meta_graph()
    # Topological sort of the meta graph
    print("Topological Sort of the Meta Graph")
    digraph.topological_sort_meta_graph()


def q3():
    g = GraphNX(directed=False)

    # Add edges with weights
    edges = [
        ('A', 'B', 22), ('A', 'C', 9), ('A', 'D', 12),
        ('B', 'C', 35), ('B', 'F', 36), ('B', 'H', 34),
        ('C', 'D', 4), ('C', 'E', 65), ('C', 'F', 42),
        ('D', 'E', 33), ('D', 'I', 30),
        ('E', 'F', 18), ('E', 'G', 23),
        ('F', 'G', 39), ('F', 'H', 24),
        ('G', 'H', 25), ('G', 'I', 21), ('I', 'H', 19)
    ]

    for u, v, w in edges:
        g.G.add_edge(u, v, weight=w)

    # Draw the initial weighted undirected graph
    print("Initial Graph")
    g.draw_graph3()
    
    # Dijkstra's Shortest Path
    start = 'A'
    print("Dijkstra's Shortest Path from", start)
    shortest_paths = g.dijkstra(start)
    print(shortest_paths)
    
    # Minimum Spanning Tree
    print("Minimum Spanning Tree")
    g.minimum_spanning_tree()

t = 0

def main():

    # Main loop to run the program
    while True:
        choice = input("Enter 'q1' to run Question 1, 'q2' to run Question 2, or 'q3' to run Question 3 (or 'exit' to quit): ").strip().lower()
        choice = choice.lower()
        if choice == 'q1':
            q1()
        elif choice == 'q2':
            q2()
        elif choice == 'q3':
            q3()
        elif choice == 'exit':
            break
        else:
            print("Invalid choice. Please enter 'q1', 'q2', or 'exit'.")

if __name__ == "__main__":
    main()

