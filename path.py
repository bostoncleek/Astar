"""
The file creates a path using A* both offline and online
"""


import heapq
import math
import numpy as np


class Node(object):
    """ creates a node for Astar """

    def __init__(self, parent, position):
        # costs for cell
        self.g = 0
        self.h = 0
        self.f = 0

        # parent nodes and position of node
        self.parent = parent
        self.position = position

    def __lt__(self, other):
        # method fo comparisson for heapq
        return self.f < other.f


class Astar(object):
    """ A* with objects known a priori """

    def __init__(self, start_world, goal_world, grid):

        self.start_world = start_world
        self.goal_world = goal_world

        # start and end in grid indixes
        self.grid = grid
        self.start = self.grid.cell_index(start_world)
        self.goal = self.grid.cell_index(goal_world)


        # start and end node
        self.start_node = Node(None, self.start)
        self.start_node.g = 0
        self.start_node.h = 0
        self.start_node.f = 0

        # opend and closed lists
        self.open_list = []
        self.closed_list = []

        # add start node
        self.open_list.append(self.start_node)

        # current node and index in open list
        self.curr_node = self.start_node

        # actions
        self.actions = [[0, -1], [0, 1],
                        [-1, 0], [1, 0],
                        [-1, -1], [-1, 1],
                        [1, -1], [1, 1]]


        self.path = []
        self.path_world = None

        #self.test_list = []
        #self.test_list.append(self.start_node)
        heapq.heapify(self.open_list)

        # list of neighbors for online algo
        self.neighbors = []

        # used for returning closed list
        self.closed = None

        # waypoint robot is driving to
        self.waypoint = None

        # waiting for robot to reach waypoint
        self.waypoint_reached = False



    def naive(self):
        """ A* naive creates path from start to goal
        considers placing all neighbors on openlist
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
            self.neighbor_cells_naive()


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



    def drive_online(self):
        """ drive while planning online this function determines next waypoint
        from the robots position

        Returns:
            The world position of the next waypoint for the robot to drive to
        """

        if self.open_list:

            min_node = heapq.heappop(self.open_list)
            self.curr_node = min_node

            if self.curr_node.position == self.goal:
                print("Goal Reached by A* Planner")

            self.closed_list.append(min_node)

            # determine neighboring cells and
            # compute cost of neighboring cells
            self.neighbor_cells_online()

        # return waypoint in world coordinates
        return self.grid.world_coord(min_node.position, 1)



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


    def neighbor_cells_naive(self):
        """ determines neighboring cells for naive algorithm """

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

            new_node = self.neighbor_cost(temp_pos)

            # check costs of new node
            if self.check_neighbor(new_node):
                continue

            heapq.heappush(self.open_list, new_node)


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



    def check_neighbor(self, new_node):
        """ This is only required for the naive A*. Checks if neighbors are
        on open and closed lists then compares the true cost if on open list

        Return True if we should ignore this neighbo
        """

        for node in self.open_list:
            if new_node.position == node.position:

                # ignore new_node because it has a high cost
                if node.g < new_node.g:
                    return True

                # update node because new_node is better
                elif node.g > new_node.g:
                    node.g = new_node.g
                    node.h = new_node.h
                    node.f = new_node.f
                    node.parent = new_node.parent

                    return True

        for node in self.closed_list:
            if new_node.position == node.position:
                return True

        return False


    def heuristic(self, cell):
        """ calculates the heurtist cost given a cell """
        dx = abs(cell[0] - self.goal[0])
        dy = abs(cell[1] - self.goal[1])

        #if planner == "naive":
        #return math.sqrt(dx**2 + dy**2)

        #elif planner == "online":
        D1 = 1
        D2 = math.sqrt(2)
        return D1 * (dx+dy) + (D2 - 2*D1)*min(dx,dy)


    def neighbor_bounds(self, pos):
        """ checks if a neightbor cell is within grid world

        Return True if we should ignore this neighbor
        """

        if pos[0] >= len(self.grid.cell_x) or pos[1] >= len(self.grid.cell_y):
            return True

        elif pos[0] < 0 or pos[1] < 0:
            return True

        return False


    def min_node(self):
        """ determines the node/index of lowest cost node """

        curr_n = self.open_list[0]
        curr_index = 0

        # node on open list with min cost
        for index, node in enumerate(self.open_list):
            if node.f < curr_n.f:
                curr_n = node
                curr_index = index

        return curr_n, curr_index


    def goal_reached(self):
        """ determines if goal has been reached """
        if self.curr_node.position == self.goal:
            print("Goal Reached!")
            return True

        return False


    def distance_to_goal_(self):
        """ computes distance from current node to goal """
        d = math.sqrt(math.pow(self.curr_node.position[0]-self.goal[0],2) + math.pow(self.curr_node.position[1]-self.goal[1],2))
        print("Distance to goal: ", d)


    def get_path(self):
        """ returns the path from start to goal """

        node =  self.curr_node
        while node is not None:

            self.path.append(node.position)
            node = node.parent

        self.path = self.path[::-1]
        #print(self.path)
        self.path_world = self.grid.world_coord(self.path, len(self.path))
        #print(self.path_world)







#
