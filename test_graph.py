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