import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#allocate a Printer02 (defined in t001.cpp)
printer = tutorial.Printer02()
reader = tutorial.Reader01()

graph = []
graph += [
          reader["output"] >> printer["input"]
         ]

#Create a Plasm, the graph structure of ecto
plasm = ecto.Plasm()

#insert our instance into the graph so that it may be executed
plasm.connect(graph)

#execute in a tight loop 10 times
plasm.execute(3)
