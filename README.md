# Astar

<p align="center">
  <img src="/media/astarpath.jpg" width="500" height="300" />
</p>

# Overview
The project focus was to develop an online implementation of A* that does not have knowledge of the obstacles *a priori* and must re-plan from the current node when an obstacle is encountered. Additionally the robot drives the path while planning from its current location using an inverse kinematic controller. If the robot ends up in a cell other than the target cell, A* must select a new target cell. The robot's kinematics and noise added in the controller influence the path planning component by simulating stochasticity in the environment. All visualizations use the following color scheme: the start/end configurations are blue, the planned path using A* is red, the position of the robot is purple, the heading of the robot is depicted by a yellow arrow, all obstacles are black, and all unoccupied cells are white.

See this [document](https://github.com/bostoncleek/Astar/blob/master/doc/astar_planning.pdf). for more details and theory behind the implementation.

The grid discretizations bellow are 1m (left) and 0.1m (right).

<p align="center">
  <img src="/media/astarcoarse.gif" width="300" height="350" />
  <img src="/media/astarfine.gif" width="300" height="350" />
</p>

# How to run
Simply run:
```
python3 run.py
```

The file run.py calls function in partA.py for
question 3,5, and 7 then calls partB.py for 9, 10, and 11. When plots appear you must close the current plot for the
next one to show. For plots that involve driving, when the terminal reads "Goal reached by driving!" close the plot and the robot will drive the next path. File path to landmarks data file: "ds1_Landmark_Groundtruth.dat"
Line 10 of grid.py if need modify

# Dependencies:
- numpy
- matplotlib

# Implementation
### Heuristic
The implementation of A* use the admissible Octile distance heuristic. On a square grid the heuristic considers eight directions of movement.

### Algorithm
The online algorithm determines the costs of all the neighboring nodes, up to eight neighbors. If a neighboring node is already on the closed list, the neighboring node is ignored. The neighboring node with the smallest total cost is selected and placed on the open list. The open list will only have one node at a time in contrast to the naive implementation that will likely have an open list with more than one node.

The function bellow contains the implementation of the online implementation of A*. It relies on a priority queue (not required for the online algorithm but improves efficiency for the naive algorithm) to determine which is used to determine the node with the minimum cost.

```python
def online(self):
    """ A* online creates path from start to goal and
    only places lowest cost neighbor on openlist
    """

    while self.open_list:

        min_node = heapq.heappop(self.open_list)
        self.curr_node = min_node

        if self.curr_node.position == self.goal:
            print("Goal Reached by A* Planner")
            break

        self.closed_list.append(min_node)

        # determine neighboring cells and
        # compute cost of neighboring cells
        self.neighbor_cells_online()
```


First, create a class to store relevant data for each node.

```python
class Node(object):
    """ creates a node for Astar """

    def __init__(self, parent, position):
        # costs for node
        self.g = 0      # true cost
        self.h = 0      # heuristic cost
        self.f = 0      # total cost

        # parent nodes and position of node
        self.parent = parent
        self.position = position

    def __lt__(self, other):
      # method fo comparisson for heapq
      return self.f < other.f
```

Next, find the neighboring node with the smallest cost and place on the open list. The actions must not direct the path out of bounds of the grid. If the new node is already on the closed list at the same position the new node is not added.


```python
def neighbor_cells_online(self):
    """ determines neighboring cells for online algorithm """

    # clear all neighbors each time
    self.neighbors.clear()

    # current indices in grid world
    curr_x = self.curr_node.position[0]
    curr_y = self.curr_node.position[1]

    # evaluate neighbors
    for act in self.actions:
        # neighbors position
        temp_pos = [curr_x + act[0], curr_y + act[1]]

        # within bounds of grid world and not on lists
        if self.neighbor_bounds(temp_pos):
            continue


        # cost of potential new node
        new_node = self.neighbor_cost(temp_pos)

        # if on closed list ignore new node
        if new_node.position in [node.position for node in self.closed_list]:
            continue

        # append all neighbors for evaluation
        self.neighbors.append(new_node)


    # select neighbor with lowest cost
    best_neighbor = self.neighbors[0]
    for index, neighbor in enumerate(self.neighbors):
        if neighbor.f < best_neighbor.f:
            best_neighbor = neighbor

    # place lowest cost neighbor on openlist
    heapq.heappush(self.open_list, best_neighbor)
```


This function creates a new node and the determines the costs of that node.

```python
def neighbor_cost(self, cell):
    """ compute costs of a neighboring cell """

    # create new node
    parent = self.curr_node
    position = cell
    node = Node(parent, cell)

    # compute cost
    # check if grid contains obstacle were the neighbor is
    true_cost = self.curr_node.g + self.grid.world[cell[0]][cell[1]]

    heuristic_cost = self.heuristic(cell)

    node.g = true_cost
    node.h = heuristic_cost
    node.f = true_cost + heuristic_cost

    return node
```

### Inverse Kinematic Controller
The inverse kinematic controller models the robot as a kinematic unicycle, where the controls (u = [u1 u2]) are the linear and angular velocities. In the frame of the robot, the positive x-direction is forward and the positive y-direction is left. Positive angular velocity and angular position is considered counter clockwise. The kinematics are propagated forward using a *fourth order Runge-Kutta* integrator with a time step of 0.1s.

Given the pose of the robot (x, y, theta) and a target position (xT, yT) the nominal controls u1 and u2 are determined using the distance to the target and the error in the robot's heading towards the target. Both controls are issued at the same time allowing the robot to turn and move towards the target simultaneously.

The robot's pose is probabilistic because random perturbations are added to the nominal control inputs. The random perturbations are drawn from normal Gaussian distributions.

The values for the feedback gains were determined experimentally. If the gains are too high, the robot will over shoot the desired target and if they are too low, it will take a longer time to reach the target cell. If the robot's linear velocity is too high, the robot will have difficulty turning towards the target. Initially, the gains were set low and then incrementally increased until many oscillations in the robot's pose were observed. The gains were then slowly decreased until the oscillations decreased and the robot could achieve the target cell.
