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
            for i in range(len(self.adjacencyList[key])):
                if self.adjacencyList[key][i][0] == node:
                    self.adjacencyList[key].pop(i)
                    break

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
        pass

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

    def closeness(self, node: str) -> float:
        # not sure about what this method should return. you guys do it.
        pass

    def eigenVector(self, node: str) -> float:
        # not sure about what this method should return. you buddies figure it out.
        pass

    def katz(self, node: str) -> float:
        # not sure about what this method should return. you buddies figure it out.
        pass

    def pageRAnk(self, node: str) -> float:
        # not sure about what this method should return. you buddies figure it out.
        pass
