This is the implementation of the RRT path planner written in Python.
The script reads a list of obstacles from the CSV file. Planner samples from a uniform random distribution over the square [-0.5, 0.5] x [-0.5, 0.5] with a step = 0.1 and finds the closest node. Then saves CSV files with the shortest found path, nodes, and edges. Max tree size is 150.
Visualized in CoppeliaSim.