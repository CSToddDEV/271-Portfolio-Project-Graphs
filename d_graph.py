# Course: CS261 - Data Structures
# Author: Calvin Todd
# Assignment: 6 - Portfolio Project
# Description: In this assignment we build a directed graph class!

class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        This method adds a new vertex to the graph.  It will return a single integer, the number of vertices in the
        graph after addition.
        """
        self.v_count += 1

        new_v = []
        for i in range(self.v_count):
            new_v.append(0)

        if self.v_count > 1:
            for v in self.adj_matrix:
                v.append(0)

        self.adj_matrix.append(new_v)

        return self.v_count

    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        This method adds a new edge to the graph.  It populates the list corresponding with the initial vertex,
        and within that the list index corresponding to the destination vertex.  The value is the weight of the edge.
        If either the vertices do not exist, they both reference the same vertex, or the weight is not positive, this
        method does nothing.
        """
        if src == dst:
            return

        elif src >= self.v_count or dst >= self.v_count:
            return

        elif weight <= 0:
            return

        else:
            self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        This method removes and edge between the two provided vertices.  If either vertex does not exist, or there is no
        edge between them, this method does nothing.
        """
        if src > self.v_count - 1 or dst > self.v_count - 1:
            return

        elif src < 0 or dst < 0:
            return

        elif self.adj_matrix[src][dst] == 0:
            return

        else:
            self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        THis method returns a list of vertices of the graph.
        """
        vertices = []

        for i in range(len(self.adj_matrix)):
            if len(self.adj_matrix[i]) > 0:
                vertices.append(i)

        return vertices

    def get_edges(self) -> []:
        """
        This method returns a list of edges from the graph in the form of a tuple.  The first element in the tuple will
        be the index of the origin vertex, the second will be the index of the destination vertex, and the third
        will be the weight of the side.
        """
        edges = []

        for source in range(len(self.adj_matrix)):
            for destination in range(len(self.adj_matrix[source])):
                if self.adj_matrix[source][destination] > 0:
                    edges.append((source, destination, self.adj_matrix[source][destination]))

        return edges


    def is_valid_path(self, path: []) -> bool:
        """
        This method takes a list of vertex indices and returns True or False depending whether they are a valid path
        connected by edges
        """
        if not path:
            return True

        truth = True

        for i in range(len(path) - 1):
            if not self.adj_matrix[path[i]][path[i + 1]] > 0:
                truth = False

        return truth

    def recursive_dfs_helper(self, visited, start, end):
        """
        Recursive he;per for the dfs method
        """
        if end is not None and end == start:
            visited.append(start)
            return visited

        if start not in visited:
            visited.append(start)
            for i in range(len(self.adj_matrix[start])):
                if self.adj_matrix[start][i] > 0:
                    visited = self.recursive_dfs_helper(visited, i, end)

        return visited



    def dfs(self, v_start, v_end=None) -> []:
        """
        This method performs a recursive Depth First Search.  If an ending vertex is provided
        then the search will only go up to that vertex.  If the ending vertex is not in the graph, then
        it will act as if there is no ending vertex.  If the start vertex is not in the graph,
        the method returns an em
        pty list.
        """
        visited = []

        if v_start >= self.v_count or self.adj_matrix[v_start] == []:
            return visited

        visited = self.recursive_dfs_helper(visited, v_start, v_end)

        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        This method performs a Breadth First Search of the graph based on teh provided start index
        """
        visited = []
        queue = []

        visited.append(v_start)
        if v_start == v_end:
            return visited
        queue.append(v_start)

        while queue:
            v = queue.pop(0)

            for i in range(len(self.adj_matrix[v])):
                if self.adj_matrix[v][i] > 0:
                    if i not in visited:
                        visited.append(i)
                        if i == v_end:
                            return visited
                        queue.append(i)

        return visited

    def recursive_has_cycle(self, v, visited, recursive_visited):
        """
        This method is a recursive helper for has_cycle.
        """
        visited.append(v)
        recursive_visited.append(v)

        for i in range(len(self.adj_matrix[v])):
            if self.adj_matrix[v][i] > 0:
                if i not in visited:
                    truth = self.recursive_has_cycle(i, visited, recursive_visited)
                    if truth is True:
                        return True
                elif i in recursive_visited:
                    return True

        recursive_visited.remove(v)
        return False

    def has_cycle(self):
        """
        This method returns True if there is at least one cycle in the graph.
        """
        truth = False
        visited = []
        recursive_visited = []

        for vertex in range(len(self.adj_matrix)):
            if vertex not in visited:
                truth = self.recursive_has_cycle(vertex, visited, recursive_visited)
                if truth is True:
                    return truth

        return truth

    def dijkstra(self, src: int) -> []:
        """
        This method returns a list of the shortest path using dijkstra's algorithm where each index coincides with the
        length of the shortest path form the src index.  If the other index can not be reached it is infinity.
        """
        # Create distance, vertex, and previous sets
        distance = []
        vertices = []
        for i in range(self.v_count):
            vertices.append(i)
            distance.append(float('inf'))

        #Set distance of src node
        distance[src] = 0

        while vertices:
            #Get vertex with shortest distance
            min_distance = None
            for vertex in vertices:
                if min_distance is None or distance[vertex] < distance[min_distance]:
                    min_distance = vertex
            vertices.remove(min_distance)

            #Check distance of neighbor nodes for shortest path
            for i in range(len(self.adj_matrix[min_distance])):
                if self.adj_matrix[min_distance][i] > 0:
                    distance_holder = distance[min_distance] + self.adj_matrix[min_distance][i]
                    if distance_holder < distance[i]:
                        distance[i] = distance_holder

        return distance

if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
