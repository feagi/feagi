"""
Source mostly from: https://gist.github.com/Vini2/f13a1fc6776319416982a7105844dc27#file-python-igraph-example-ipynb

todo: cortical areas without any initial connection to other areas are not displayed.
"""
from igraph import *

graph = Graph(directed=True)


class DirectGraph:
    def __init__(self, graph_, edges, weights, labels):
        # Count number of vertices
        unique_edges = set()
        for _ in edges:
            for __ in _:
                unique_edges.add(__)
        vertices_count = len(unique_edges)

        # Create graph
        self.g = graph_

        # Add 5 vertices
        self.g.add_vertices(vertices_count)

        # Add ids and labels to vertices
        for i in range(len(self.g.vs)):
            self.g.vs[i]["id"] = i
            self.g.vs[i]["label"] = str(labels[i])

        # Add edges
        self.g.add_edges(edges)

        # Add weights and edge labels
        self.g.es['weight'] = weights
        self.g.es['label'] = weights
        
        # Visual Configuration
        self.visual_style = {}

        self.out_name = "./vis/graph.png"

        # Set bbox and margin
        self.visual_style["bbox"] = (3000, 3000)
        self.visual_style["margin"] = 100

        # Set vertex size
        self.visual_style["vertex_size"] = 150

        # Set vertex label size
        self.visual_style["vertex_label_size"] = 22

        # Don't curve the edges
        self.visual_style["edge_curved"] = False

        # Set the layout
        # my_layout = self.g.layout_grid()
        my_layout = self.g.layout_kamada_kawai()
        self.visual_style["layout"] = my_layout

    def graph_in_bw(self):
        # Set vertex colours
        self.visual_style["vertex_color"] = 'white'

        # Plot the graph
        plot(self.g, self.out_name, **self.visual_style)

    def graph_in_color(self):
        # Set vertex colours
        self.g.vs["color"] = ["red", "green", "pink", "yellow", "orange"]

        # Plot the graph
        plot(self.g, self.out_name, **self.visual_style)


if __name__ == '__main__':
    graph_edges = [(0, 4), (0, 1), (0, 3), (1, 2), (1, 3), (2, 4), (3, 4), (3, 0)]
    graph_weights = [8, 6, 3, 5, 6, 4, 9, 50]
    graph_labels = ['v1', 'v2', 'v3', 'v4', 'v5']

    directed_graph = DirectGraph(graph, edges=graph_edges, weights=graph_weights, labels=graph_labels)
    directed_graph.graph_in_color()


# print("Number of vertices in the graph:", g.vcount())
# print("Number of edges in the graph", g.ecount())
# print("Is the graph directed:", g.is_directed())
# print("Maximum degree in the graph:", g.maxdegree())
# print("Adjacency matrix:\n", g.get_adjacency())
#
#
#
#
# print(g.neighbors.__doc__)
#
# print(g.neighbors(0, mode=ALL))
#
#
#
#
# print(g.bfs.__doc__)
# print(g.bfs(0))
#
#
#
#
#
# print(g.get_shortest_paths.__doc__)
#
# print("The shortest paths from vertex 0:", g.get_shortest_paths(0))
# print("The shortest paths from vertex 0 to vertex 4:", g.get_shortest_paths(0, to=4))
#
#
#
#
#
# print(g.laplacian.__doc__)
# print("Laplacian matrix of a graph:\n",g.laplacian())
#
#
#
# print(g.maxflow.__doc__)
# print(g.maxflow(0,4,weights).__doc__)
#
# maxflow = g.maxflow(0,4,weights)
#
# print("The maximum flow value:", maxflow.value)
# print("The flow values on each edge:", maxflow.flow)
# print("Tedge IDs in the minimal cut of the flow:", maxflow.cut)
# print("The vertex IDs in the parts created created by the cut:", maxflow.partition)
