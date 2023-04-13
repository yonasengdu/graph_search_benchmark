from pprint import pprint
from queue import Queue, PriorityQueue


class Graph:
    def __init__(self):
        self.adjacencyList: map = {}

    def show(self):
        pprint(self.adjacencyList)

    def nodeExists(self, node: str):
        """returns true if node exists in this graph"""
        return node in self.adjacencyList

    def edgeExists(self, node: str, to: str):
        """checks if 'node' exists and then checks if an edge from 'node' to 'to' 
        exists. returns true if both exist"""
        if not self.nodeExists(node):
            raise Exception(f"node {node} does not exist in the graph.")
        if not self.nodeExists(to):
            raise Exception(f"node {to} does not exist in the graph.")
        for entry in self.adjacencyList[node]:
            if entry[0] == to:
                return True
        return False

    def undirectedEdgeExists(self, node: str, other: str):
        """checks if both 'node' and 'other' exist; and then checks if edges exist in
          both directions. returns True if all these exist"""
        if not self.nodeExists(node):
            raise Exception(f"node {node} does not exist in the graph.")
        if not self.nodeExists(other):
            raise Exception(f"node {other} does not exist in the graph.")
        forwardEdge = False
        backwardEdge = False
        for entry in self.adjacencyList[node]:
            if entry[0] == other:
                forwardEdge = True
        for entry in self.adjacencyList[other]:
            if entry[0] == node:
                backwardEdge = True
        return forwardEdge and backwardEdge

    def addNode(self, node: str):
        # raise error if the node already exists in the graph.
        if self.nodeExists(node):
            raise Exception(f"node {node} already exists in the graph.")
        # add the node to the graph
        self.adjacencyList[node] = []

    def deleteNode(self, node: str):
        try:
            # delete all the edges out of this node, and the node itself.
            self.adjacencyList.pop(node)
        except KeyError:
            # raise error if node does not exist
            raise Exception(
                "the node {node} does not exist in the graph; thus, it can't be deleted.")
        # finally, remove all the edges into the deleted node
        for key in self.adjacencyList:
            self.adjacencyList[key] = [entry for entry in self.adjacencyList[key] if entry[0] != node]

    def addEdge(self, node: str, to: str, cost: float = 0):
        # check if the edge doesn't already exist
        if self.edgeExists(node, to):
            raise Exception(f"the edge from {node} to {to} already exists.")
        # add the edge
        self.adjacencyList[node].append((to, cost))

    def addUndirectedEdge(self, node: str, other: str, forwardCost: float = 0, backwardCost: float = 0):
        # making sure the undirected edge doesn't already exist by doing the following two ...
        # making sure a forward directed edge doesn't exist
        if self.edgeExists(node, other):
            raise Exception(
                f"There already exists a directed graph from {node} to {other}.")
        # making sure a backward directed edge doesn't exist
        if self.edgeExists(other, node):
            raise Exception(
                f"There already exists a directed graph from {other} to {node}.")
        # add the undirected edge
        self.adjacencyList[node].append((other, forwardCost))
        self.adjacencyList[other].append((node, backwardCost))

    def deleteEdge(self, node: str, to: str):
        # raise error if the edge does not exist.
        if not self.edgeExists(node, to):
            raise Exception(
                f"There is no edge from {node} to {to}; thus it can't be deleted.")
        # delete the edge
        for i in range(len(self.adjacencyList[node])):
            if self.adjacencyList[node][i][0] == to:
                self.adjacencyList[node].pop(i)
                break

    def deleteUndirectedEdge(self, node: str, other: str):
        # the two methods below will check for existence of a two-way end delete it.
        self.deleteEdge(node, other)
        self.deleteEdge(other, node)

    def dfs(self, start: str, target: str):
        """returns a dictionary with the shape:
        {
        path: [list of nodes from 'start' to 'target']
        cost: total cost of going through the path (should be float)
        }
        if no path found, return null
        """

        def backtrack(parent_map: dict, start: str, goal: str):
            """We use this function to backtrack through a parent_map that contains a path from start to goal. it returns the correct path list."""
            path = [goal]
            current = goal
            while (current != start):
                path.append(parent_map[current][0])
                current = parent_map[current][0]
            # finally, we need to reverse the path list.
            answer = [path[len(path)-i-1] for i in range(len(path))]
            return answer

        # let's make the stack and a visited set
        stack = []
        stack.append(start)
        visited = set()
        visited.add(start)
        # a map from child to (parent, child_cost)
        parent_map = {start: (None, 0)}
        # time for the loop
        while (len(stack) != 0):
            current = stack.pop()
            visited.add(current)
            if (current == target):
                # now we have the solution, let's backtrack through the parent_map
                path = backtrack(parent_map, start, current)
                return {
                    'path': path,
                    'cost': parent_map[target][1]
                }
            for entry in self.adjacencyList[current]:
                neighbor, cost = entry
                if (neighbor not in visited):
                    totalCost = cost + parent_map[current][1]
                    parent_map[neighbor] = (current, totalCost)
                    stack.append(neighbor)
        print("Solution not found.")
        return None

    def bfs(self, start: str, target: str):
        """returns
        {
        path: [list of nodes from 'start' to 'target'] (path should be the optimal solution)
        cost: total cost of going through the path (should be float)
        }"""
        def backtrack(parent_map: dict, start: str, goal: str):
            """We use this function to backtrack through a parent_map that contains a path from start to goal. it returns the correct path list."""
            path = [goal]
            current = goal
            while (current != start):
                path.append(parent_map[current][0])
                current = parent_map[current][0]
            # finally, we need to reverse the path list.
            answer = [path[len(path)-i-1] for i in range(len(path))]
            return answer

        # let's make the queue and a visited set
        queue = Queue()
        queue.put(start)
        visited = set()
        visited.add(start)
        # a map from child to (parent, child_cost)
        parent_map = {start: (None, 0)}
        # time for the loop
        # check this line if there's some mysterious bug.
        while (not queue.empty()):
            current = queue.get()
            visited.add(current)
            if (current == target):
                # now we have the solution, let's backtrack through the parent_map
                path = backtrack(parent_map, start, current)
                return {
                    'path': path,
                    'cost': parent_map[target][1]
                }
            for entry in self.adjacencyList[current]:
                neighbor, cost = entry
                if (neighbor not in visited):
                    totalCost = cost + parent_map[current][1]
                    if (neighbor not in parent_map or parent_map[neighbor][-1] > totalCost):
                        # the above if condition guarantees that the parent_map stores the best cost so far.
                        parent_map[neighbor] = (current, totalCost)
                    queue.put(neighbor)
                    visited.add(neighbor)
        print("Solution not found.")
        return None

    def ucs(self, start: str, target: str):
        """returns
        {
        path: [list of nodes from 'start' to 'target'] (path should be the optimal solution)
        cost: total cost of going through the path (should be float)
        }"""
        def backtrack(parent_map: dict, start: str, goal: str):
            """We use this function to backtrack through a parent_map that contains a path from start to goal. it returns the correct path list."""
            path = [goal]
            current = goal
            while (current != start):
                path.append(parent_map[current][0])
                current = parent_map[current][0]
            # finally, we need to reverse the path list.
            answer = [path[len(path)-i-1] for i in range(len(path))]
            return answer

        # let's make the queue and a visited set
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set()
        # a map from child to (parent, child_cost)
        parent_map = {start: (None, 0)}
        # time for the loop
        # again, this line is not reliable so keep an eye on it.
        while (not queue.empty()):
            cost, current = queue.get()
            if (current in visited):
                continue
            visited.add(current)
            if (current == target):
                # now we have the solution, let's backtrack through the parent_map
                path = backtrack(parent_map, start, current)
                return {
                    'path': path,
                    'cost': parent_map[target][-1]
                }
            for entry in self.adjacencyList[current]:
                neighbor, cost = entry
                if (neighbor not in visited):
                    totalCost = cost + parent_map[current][-1]
                    if (neighbor not in parent_map or parent_map[neighbor][-1] > totalCost):
                        parent_map[neighbor] = (current, totalCost)

                    # print(f"enqueueing: {state} with cost: {totalCost}")
                    queue.put((totalCost, neighbor))
        print("Solution not found.")
        return None

    # not sure about this signature. check it out
    def iterativeDeepeningSearch(self, start: str, target: str):
        """returns a dictionary with the shape:
        {
        path: [list of nodes from 'start' to 'target']
        cost: total cost of going through the path (should be float)
        }
        if no path found, return None
        """
        def deepSearchWithLimit(currentStart,target,visitedOnCurrentPath,currentDepthLimit,costOfCurrentPath):
            if currentStart == target:
                self.totalPathCost += costOfCurrentPath
                return [currentStart]
            if currentDepthLimit == 0:
                return None
            else:
                if currentStart not in visitedOnCurrentPath:
                    visitedOnCurrentPath.add(currentStart)
                    for neighborData in self.adjacencyList[currentStart]:
                        neighbor,neighbor_cost = neighborData
                        currentDepthLimit -= 1
                        if neighbor not in visitedOnCurrentPath:
                            path = deepSearchWithLimit(neighbor,target,visitedOnCurrentPath,currentDepthLimit,neighbor_cost)
                            if path:
                                self.totalPathCost += costOfCurrentPath
                                return [currentStart] + path
                return None
            

        searchDepthLimit = 0
        self.totalPathCost = 0
        Cost = 0
        while True:
            visited = set()
            path = deepSearchWithLimit(start,target,visited,searchDepthLimit,Cost)
            if path:
                return {
                  "path" : path,
                  "cost" :  self.totalPathCost
                }
            searchDepthLimit += 1

    # not sure about this signature. check it out
    def bidirectionalSearch(self, start: str, target: str):
        """returns a dictionary with the following shape:
        {
        path: [list of nodes from 'start' to 'target']
        cost: total cost of going through the path (should be float)
        }
        if no path found, return None
        """
        if start == target:
            return {'path': [start], 'cost': 0}
        # initialize the start and end queues, explored sets, and common node variable
        start_queue = Queue()
        end_queue = Queue()
        start_queue.put(start)
        end_queue.put(target)
        start_explored = set([start])
        end_explored = set([target])
        common_node = None

        # initialize the dictionaries to keep track of the path and cost from the start and end nodes
        start_path = {start: [start]}
        end_path = {target: [target]}
        start_cost = {start: 0}
        end_cost = {target: 0}

        # perform the bidirectional BFS search
        while not start_queue.empty() and not end_queue.empty():
            # explore from the starting node
            current = start_queue.get()
            for neighbor, weight in self.adjacencyList[current]:
                if neighbor not in start_explored:
                    start_explored.add(neighbor)
                    start_path[neighbor] = start_path[current] + [neighbor]
                    start_cost[neighbor] = start_cost[current] + weight
                    start_queue.put(neighbor)

                # check for intersection with the end search
                if neighbor in end_explored:
                    common_node = neighbor
                    break

            if common_node:
                break

            # explore from the ending node
            current = end_queue.get()
            for neighbor, weight in self.adjacencyList[current]:
                if neighbor not in end_explored:
                    end_explored.add(neighbor)
                    end_path[neighbor] = [neighbor] + end_path[current]
                    end_cost[neighbor] = end_cost[current] + weight
                    end_queue.put(neighbor)

                # check for intersection with the start search
                if neighbor in start_explored:
                    common_node = neighbor
                    break

            if common_node:
                break

        # construct the final path and cost dictionary
        if common_node:
            path = start_path[common_node] + end_path[common_node][1:]
            cost = start_cost[common_node] + end_cost[common_node]
            return {'path': path, 'cost': cost}
        else:
            return None

    def greedySearch(self, start: str, target: str, heuristic: any):
        """returns a dictionary with the following shape:
        {
        path: [list of nodes from 'start' to 'target']
        cost: total cost of going through the path (should be float)
        }
        if no path found, return None
        """
        def backtrack(parent_map: dict, start: str, goal: str):
            """We use this function to backtrack through a parent_map that contains a path from start to goal. it returns the correct path list."""
            path = [goal]
            current = goal
            while (current != start):
                path.append(parent_map[current][0])
                current = parent_map[current][0]
            # finally, we need to reverse the path list.
            answer = [path[len(path)-i-1] for i in range(len(path))]
            return answer

        # let's make the queue and a visited set
        queue = PriorityQueue()
        queue.put((heuristic(start, target), start))
        visited = set()
        # a map from child to (parent, child_cost)
        parent_map = {start: (None, 0)}
        # time for the loop
        # again, this line is not reliable so keep an eye on it.
        while (not queue.empty()):
            _, current = queue.get()
            if (current in visited):
                continue
            visited.add(current)
            if (current == target):
                # now we have the solution, let's backtrack through the parent_map
                path = backtrack(parent_map, start, current)
                return {
                    'path': path,
                    'cost': parent_map[target][-1]
                }
            for entry in self.adjacencyList[current]:
                neighbor, cost = entry
                if (neighbor not in visited):
                    totalCost = cost + parent_map[current][-1]
                    parent_map[neighbor] = (current, totalCost)
                    queue.put((heuristic(neighbor, target), neighbor))
        print("Solution not found.")
        return None

    def aStarSearch(self, start: str, target: str, heuristic: any):
        """returns a dictionary with the following shape:
        {
        path: [list of nodes from 'start' to 'target'] (path should be the optimal solution)
        cost: total cost of going through the path (should be float)
        }"""
        def return_path(current_node):
            path = []

            while current_node:
                path.append(current_node[1])
                if current_node[1] == start:
                    return {"path": path[::-1], "cost": node_data[path[0]]["g_cost"]}
                current_node = node_data[current_node[1]]["prev"]

        node_data = {}
        for node in self.adjacencyList.keys():
            node_data[node] = {

                "h_cost": heuristic(node, target),
                "g_cost": float("inf"),
                "prev": None,
            }

        yet_to_be_visited = PriorityQueue()
        yet_to_be_visited.put((0, start))

        while yet_to_be_visited:
            node_weight, current_node = yet_to_be_visited.get()

            if current_node == target:
                return return_path((node_weight, current_node))

            for neighbor in self.adjacencyList[current_node]:
                new_cost = node_weight + neighbor[1]

                if new_cost < node_data[neighbor[0]]["g_cost"]:
                    node_data[neighbor[0]]["g_cost"] = new_cost
                    yet_to_be_visited.put((new_cost, neighbor[0]))
                    node_data[neighbor[0]]["prev"] = (
                        node_weight, current_node)

        return None

    def degree(self, node: str) -> float:
        """returns a float"""
        return len(self.adjacencyList[node])

    def closenessCentrality(self, node: str) -> float:
        # not sure about what this method should return. you guys do it.
        totalCost = 0
        for key in self.adjacencyList:
            if key == node: continue
            searchResult = self.ucs(node, key)
            if(searchResult != None):
                totalCost += searchResult['cost']
        if totalCost == 0: return 0
        return (len(self.adjacencyList) -1)/totalCost


    def eigenvalueCentrality(self, node: str, max_iterations=1000, tolerance=1e-6):
        # Check if node exists in graph
        if not self.nodeExists(node):
            raise Exception(f"node {node} does not exist in the graph.")

        # Create adjacency matrix from graph's adjacency list
        nodes = list(self.adjacencyList.keys())
        num_nodes = len(nodes)
        adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]
        for i in range(num_nodes):
            for j in range(num_nodes):
                if self.edgeExists(nodes[i], nodes[j]):
                    adjacency_matrix[i][j] = 1

        # Initialize eigenvector with all elements equal to 1
        eigenvector = [1] * num_nodes

        # Power iteration method to calculate largest eigenvalue and eigenvector
        for _ in range(max_iterations):
            new_eigenvector = [0] * num_nodes
            for i in range(num_nodes):
                for j in range(num_nodes):
                    new_eigenvector[i] += adjacency_matrix[i][j] * eigenvector[j]
            norm = sum(new_eigenvector)
            new_eigenvector = [x_i / norm for x_i in new_eigenvector]
            if max(abs(x_i - new_x_i) for x_i, new_x_i in zip(eigenvector, new_eigenvector)) < tolerance:
                break
            eigenvector = new_eigenvector

        # Return eigenvalue centrality of specific node
        return eigenvector[nodes.index(node)]

    def katzCentrality(self, node: str):
        # Check if node exists in graph
        if not self.nodeExists(node):
            raise Exception(f"node {node} does not exist in the graph.")

        # Create adjacency matrix from graph's adjacency list
        nodes = list(self.adjacencyList.keys())
        num_nodes = len(nodes)
        adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]
        for i in range(num_nodes):
            for j in range(num_nodes):
                if self.edgeExists(nodes[i], nodes[j]):
                    adjacency_matrix[i][j] = 1

        # Initialize centrality vector with all elements equal to 1.0
        centrality = [1.0] * num_nodes

        # Calculate Katz centrality using iterative method
        for _ in range(num_nodes):
            new_centrality = [1.0] * num_nodes
            for i in range(num_nodes):
                for j in range(num_nodes):
                    # Update new centrality value for node i
                    new_centrality[i] += 0.1 * adjacency_matrix[i][j] * centrality[j]
            # Update centrality vector with new values
            centrality = new_centrality

        # Return Katz centrality of specific node
        return centrality[nodes.index(node)]

    def pagerankCentrality(self, node: str):
        # Check if node exists in graph
        if not self.nodeExists(node):
            raise Exception(f"node {node} does not exist in the graph.")

        # Create adjacency matrix from graph's adjacency list
        nodes = list(self.adjacencyList.keys())
        num_nodes = len(nodes)
        adjacency_matrix = [[0] * num_nodes for _ in range(num_nodes)]
        for i in range(num_nodes):
            for j in range(num_nodes):
                if self.edgeExists(nodes[i], nodes[j]):
                    adjacency_matrix[i][j] = 1

        # Calculate out-degree of each node
        out_degree = [sum(row) for row in adjacency_matrix]

        # Initialize PageRank vector with all elements equal to 1 / num_nodes
        pagerank = [1 / num_nodes] * num_nodes

        # Calculate PageRank using iterative method with fixed damping factor and number of iterations
        for _ in range(10):
            new_pagerank = [0] * num_nodes
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if out_degree[j] > 0:
                        new_pagerank[i] += 0.85 * adjacency_matrix[j][i] * pagerank[j] / out_degree[j]
                new_pagerank[i] += 0.15 / num_nodes
            pagerank = new_pagerank

        # Return PageRank centrality of specific node
        return pagerank[nodes.index(node)]
