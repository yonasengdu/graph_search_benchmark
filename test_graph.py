from graph import Graph
from heuristic import Heuristic

graph = Graph()

print("\nAdding nodes a, b, c, d, e, f")
for item in ['a', 'b', 'c', 'd', 'e', 'f']:
    graph.addNode(item)

graph.show()

graph.addEdge('a', 'b')
graph.addEdge('b', 'a')
graph.addEdge('b', 'c')
graph.addEdge('b', 'd')
graph.addEdge('a', 'd')
graph.addEdge('d', 'e')
graph.addEdge('b', 'e')
graph.addEdge('c', 'e')

print(f"\ndoes node a exist? : {graph.nodeExists('a')}")
print(f"does node z exist? : {graph.nodeExists('z')}")

print("""\n
after:
graph.addEdge('a', 'b')
graph.addEdge('b', 'a')
graph.addEdge('b', 'c')
graph.addEdge('b', 'd')
graph.addEdge('a', 'd')
graph.addEdge('d', 'e')
graph.addEdge('b', 'e')
graph.addEdge('c', 'e')
\n""")
      
graph.show()

graph.addUndirectedEdge('e', 'f')

print("""\n
after:
graph.addUndirectedEdge('e', 'f')
\n""")
      
graph.show()

try:
    print("\nTrying to add an existing edge ... ")
    graph.addEdge('a', 'b') # this should throw an exception
except Exception as e:
    print(e)

graph.deleteNode('f')

print("""\n
after:
graph.deleteNode('f')
\n""")

graph.show()

graph.deleteEdge('d', 'e')

print("""\n
after:
graph.deleteEdge('d', 'e')
\n""")

graph.show()

try:
    print("\nTrying to delete an edge from d to e (again, now it doesn't exist) ... ")
    graph.deleteEdge('d', 'e')
except Exception as e:
    print(e)

try:
    print("\nTrying to delete an edge from z to e ... ")
    graph.deleteEdge('z', 'e')
except Exception as e:
    print(e)

try:
    print("\nTrying to delete an edge from a to z ... ")
    graph.deleteEdge('a', 'z')
except Exception as e:
    print(e)

graph.deleteUndirectedEdge('a', 'b')

print("""\n
after:
graph.deleteUndirectedEdge('a', 'b')
\n""")

graph.show()


print("=================== Building a new graph ====================")


romania = Graph()

romania.addNode("Arad")
romania.addNode("Zerind")
romania.addNode("Sibiu")
romania.addNode("Timisoara")
romania.addNode("Oradea")
romania.addNode("Fagaras")
romania.addNode("Rimnicu Vilcea")
romania.addNode("Pitesti")
romania.addNode("Bucharest")

print("""\n
after:
romania.addNode("Arad")
romania.addNode("Zerind")
romania.addNode("Sibiu")
romania.addNode("Timisoara")
romania.addNode("Oradea")
romania.addNode("Fagaras")
romania.addNode("Rimnicu Vilcea")
romania.addNode("Pitesti")
romania.addNode("Bucharest")
\n""")
      
romania.show()

romania.addUndirectedEdge("Arad", "Zerind", 75, 75)
romania.addUndirectedEdge("Arad", "Timisoara", 118, 118)
romania.addUndirectedEdge("Zerind", "Oradea", 71, 71)
romania.addUndirectedEdge("Arad", "Sibiu", 140, 140)
romania.addUndirectedEdge("Oradea", "Sibiu", 151, 151)
romania.addUndirectedEdge("Sibiu", "Fagaras", 99, 99)
romania.addUndirectedEdge("Sibiu", "Rimnicu Vilcea", 80, 80)
romania.addUndirectedEdge("Rimnicu Vilcea", "Pitesti", 97, 97)
romania.addUndirectedEdge("Pitesti", "Bucharest", 101, 101)
romania.addUndirectedEdge("Fagaras", "Bucharest", 211, 211)


print("""\n
after:
romania.addUndirectedEdge("Arad", "Zerind", 75, 75)
romania.addUndirectedEdge("Arad", "Timisoara", 118, 118)
romania.addUndirectedEdge("Zerind", "Oradea", 71, 71)
romania.addUndirectedEdge("Arad", "Sibiu", 140, 140)
romania.addUndirectedEdge("Oradea", "Sibiu", 151, 151)
romania.addUndirectedEdge("Sibiu", "Fagaras", 99, 99)
romania.addUndirectedEdge("Sibiu", "Rimnicu Vilcea", 80, 80)
romania.addUndirectedEdge("Rimnicu Vilcea", "Pitesti", 97, 97)
romania.addUndirectedEdge("Pitesti", "Bucharest", 101, 101)
romania.addUndirectedEdge("Fagaras", "Bucharest", 211, 211)
\n""")


romania.show()


print("\nresult of doing depth-first search from Arad to Bucharest:")
result = romania.dfs("Arad", "Bucharest")
print(result)

print()
print("\nresult of doing breadth-first search from Arad to Bucharest:")
result = romania.bfs("Arad", "Bucharest")
print(result)

print()
print("\nresult of doing uniform-cost search from Arad to Bucharest:")
result = romania.ucs("Arad", "Bucharest")
print(result)

print()
print("\nresult of doing greedy search from Arad to Bucharest:")
result = romania.greedySearch("Arad", "Bucharest", Heuristic().distance)
print(result)

print()
print("\nresult of doing A* search from Arad to Bucharest:")
result = romania.aStarSearch("Arad", "Bucharest", Heuristic().distance)
print(result)


print("\n\n====================== working with a new graph =======================")

graph = Graph()

graph.addNode('a')
graph.addNode('b')
graph.addNode('c')
graph.addNode('d')
graph.addNode('e')
graph.addNode('s')
graph.addNode('g')

graph.addEdge('s', 'a', cost=1)
graph.addEdge('a', 'b', cost=1)
graph.addEdge('b', 'c', cost=1)
graph.addEdge('a', 'd', cost=3)
graph.addEdge('a', 'e', cost=8)
graph.addEdge('e', 'd', cost=1)
graph.addEdge('d', 'g', cost=2)


heuristicMap = {
    'a': 5,
    'b': 6,
    'c': 7,
    'd': 2,
    'e': 1,
    's': 6,
    'g': 0,
}

def heuristic(node: str, target: str):
    return heuristicMap[node]

print("""\n
Graph looks like:
\n""")

graph.show()

print("\nTesting degree lookups ... ")
print(f"degree of 'a': {graph.degree('a')}")
print(f"degree of 'd': {graph.degree('d')}")

print("\nhere's the heuristic. 'g' is considered to be the goal:")

print(heuristicMap)

print("\n doing greedy search from 's' to 'g' on the above graph ... ")
result = graph.greedySearch('s', 'g', heuristic=heuristic)
print(result)

print("\n doing ucs search from 's' to 'g' on the above graph ... ")
result = graph.ucs('s', 'g')
print(result)

print("\n doing A* search from 's' to 'g' on the above graph ... ")
result = graph.aStarSearch('s', 'g', heuristic=heuristic)
print(result)

print("\n doing dfs search from 's' to 'g' on the above graph ... ")
result = graph.dfs('s', 'g')
print(result)