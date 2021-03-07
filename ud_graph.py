# Course: CS 261
# Author: Calvin Todd
# Assignment: 6 - Portfolio Project
# Description: Creating Undirected Graphs


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #

    def add_vertex(self, v: str) -> None:
        """
        This method adds a new vertex to the graph.  If the vertex already exists in the graph, nothing, happens.
        """
        if v in self.adj_list.keys():
            return
        else:
            self.adj_list[v] = []
        
    def add_edge(self, u: str, v: str) -> None:
        """
        This method adds a new edge to the graph, connecting two vertices with the provided names.  If the either of the
        vertices do not exist, then they will be created.  If the edge already exists, or if the provided vertices are
        the same, nothing will happen.
        """
        if v == u:
            return

        if u not in self.adj_list.keys():
            self.add_vertex(u)

        if v not in self.adj_list.keys():
            self.add_vertex(v)

        if v in self.adj_list[u]:
            return

        self.adj_list[v].append(u)
        self.adj_list[u].append(v)

    def remove_edge(self, v: str, u: str) -> None:
        """
        This method removes an edge between two vertices.  If either do not exist or there is no edge, nothing happens.
        """
        if v not in self.adj_list.keys() or u not in self.adj_list.keys():
            return

        if v not in self.adj_list[u] or u not in self.adj_list[v]:
            return

        self.adj_list[v].remove(u)
        self.adj_list[u].remove(v)

    def remove_vertex(self, v: str) -> None:
        """
        This method removes the requested vertex and all its incident edges.  If the vertex doesn't exist, nothing
        happens.
        """
        if v not in self.adj_list.keys():
            return

        while len(self.adj_list[v]) != 0:
            self.remove_edge(v, self.adj_list[v][0])

        del self.adj_list[v]

    def get_vertices(self) -> []:
        """
        Return list of vertices in the graph (any order)
        """
        return list(self.adj_list.keys())

    def get_edges(self) -> []:
        """
        Return list of edges in the graph (any order)
        """
        vertices = self.get_vertices()
        edges = []

        while len(vertices) != 0:
            vertex = vertices[0]
            for v in self.adj_list[vertex]:
                if v in vertices:
                    edges.append((vertex, v))
            vertices.remove(vertex)

        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Return true if provided path is valid, False otherwise
        """
        for vertex in path:
            if vertex not in self.adj_list.keys():
                return False

        valid_path = True

        if len(path) == 0:
            return valid_path

        i = 0

        while valid_path is True and i != (len(path) - 1):
            if path[i + 1] not in self.adj_list[path[i]]:
                valid_path = False
            i += 1

        return valid_path

    def dfs_recursive(self, visited, v_start, v_end=None, truth=False):
        """
        Recursive helper for the dfs function
        """
        if v_end is not None and v_end == v_start:
            visited.append(v_start)
            truth = True
            return visited, truth

        if v_start not in visited:
            visited.append(v_start)
            self.adj_list[v_start].sort()
            for vertex in self.adj_list[v_start]:
                if truth is not True:
                    visited, truth = self.dfs_recursive(visited, vertex, v_end)

        return visited, truth

    def dfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during DFS search
        Vertices are picked in alphabetical order
        """
        visited = []

        visited, truth = self.dfs_recursive(visited, v_start, v_end)

        return visited

    def bfs(self, v_start, v_end=None) -> []:
        """
        Return list of vertices visited during BFS search
        Vertices are picked in alphabetical order
        """
        visited = []
        queue = []

        visited.append(v_start)
        if v_start == v_end:
            return visited
        queue.append(v_start)

        while len(queue) != 0:
            v = queue.pop(0)

            self.adj_list[v].sort()
            for vertex in self.adj_list[v]:
                if vertex not in visited:
                    visited.append(vertex)
                    if vertex == v_end:
                        return visited
                    queue.append(vertex)

        return visited

    def count_connected_components(self):
        """
        Return number of connected components in the graph
        """
        vertices = self.get_vertices()

        connect = 0

        while len(vertices) != 0:
            connected = self.dfs(vertices[0])
            for vertex in connected:
                if vertex in vertices:
                    vertices.remove(vertex)
            connect += 1

        return connect

    def has_cycle_recursive(self, v, visited, parent):
        """
        Recursive helper for has_cycle
        """
        visited[v] = True

        for vertex in self.adj_list[v]:
            if not visited[vertex]:
                if self.has_cycle_recursive(vertex, visited, v):
                    return True
            elif vertex != parent:
                return True

        return False

    def has_cycle(self):
        """
        Return True if graph contains a cycle, False otherwise
        """
        vertices = self.get_vertices()

        for vertex in vertices:
            visited = {}
            for v in vertices:
                visited[v] = False
            truth = self.has_cycle_recursive(vertex, visited, None)
            if truth is True:
                return truth

        return truth


if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
