import ecto #this must be imported before other ecto based python modules
import tutorial #this is our ecto module

#allocate cells
printer = tutorial.Printer02()
reader = tutorial.Reader01()

#A python list may be used to represent the graph
graph = [
          reader["output"] >> printer["input"]
        ]

plasm = ecto.Plasm()
plasm.connect(graph)
#executes forever until a module returns a non zero value
while 0 == plasm.execute():
    pass
