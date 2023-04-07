from graph import Graph

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
      
graph.show()

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
graph.addUndirectedEdge("Arad", "Zerind", 75, 75)
graph.addUndirectedEdge("Arad", "Timisoara", 118, 118)
graph.addUndirectedEdge("Zerind", "Oradea", 71, 71)
graph.addUndirectedEdge("Arad", "Sibiu", 140, 140)
graph.addUndirectedEdge("Oradea", "Sibiu", 151, 151)
graph.addUndirectedEdge("Sibiu", "Fagaras", 99, 99)
graph.addUndirectedEdge("Sibiu", "Rimnicu Vilcea", 80, 80)
graph.addUndirectedEdge("Rimnicu Vilcea", "Pitesti", 97, 97)
graph.addUndirectedEdge("Pitesti", "Bucharest", 101, 101)
graph.addUndirectedEdge("Fagaras", "Bucharest", 211, 211)
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
