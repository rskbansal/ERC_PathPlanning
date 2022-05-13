# ERC-Path-Planning

Description
You will be given a set of obstacles and one source point and a destination point. All
coordinates will be given. You need to find a path from the source to the destination
using RRT. The path should not intersect any obstacles and should be a perfectly valid
path. You will also be given a bounding box in which you are allowed to sample the
points for making the tree.

The bounding box is the square with corners at (0, 0), (0, 100), (100, 100), (100, 0).
The obstacle list, start point and goal point are as follows.

```obstacle_list = [
  [(40, 0), (40, 40), (50, 50), (60, 40), (50, 40)],
  [(10, 10), (20, 20), (10, 30), (0, 20)],
  [(50, 60), (70, 80), (60, 100), (40, 80), (45, 100)],
  [(70, 20), (90, 20), (80, 40)]
]
start = (1, 1)
goal = (100, 1)

path = RRT(start,goal, obstacle_list)

visualize(path, obstacle_list)```
