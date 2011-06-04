import sys;
if len(sys.argv) > 1:
    ecto_path = sys.argv[1].split(':')
    print ecto_path
    sys.path.insert(0,ecto_path[1])
    sys.path.insert(0,ecto_path[0])

