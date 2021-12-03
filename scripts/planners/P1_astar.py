import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from .utils import plot_line_segments

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy: 'DetOccupancyGrid2D', resolution=1,
                        time_out_steps = 5000):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.directions = [(-1, -1), (-1, 0), (-1, 1),
                           (0, -1), (0, 1),
                           (1, -1), (1, 0), (1, 1)]

        self.path = None        # the final path as a list of states
        self.time_out_steps = time_out_steps

        # Initialization
        # the set containing the states that have been visited
        self.closed_set = set()
        # the set containing the states that are candidate for future expansion
        self.open_set = {self.x_init}


        # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.est_cost_through = {self.x_init: self.distance(self.x_init,self.x_goal)}
        # dictionary of the cost-to-arrive at state from start (often called g score)
        self.cost_to_arrive = {self.x_init: 0}
        # dictionary keeping track of each state's parent to reconstruct the path
        self.come_from = {}

    def is_free(self, x):
        """
        Checks if a give state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        return self.occupancy.is_free(x) and \
               self.statespace_lo[0] <= x[0] <= self.statespace_hi[0] and \
               self.statespace_lo[1] <= x[1] <= self.statespace_hi[1]
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        return np.linalg.norm(np.array(x1) - np.array(x2))
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return self.resolution * round(x[0] / self.resolution), self.resolution * round(x[1] / self.resolution)

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        x_x, x_y = x
        for dx, dy in self.directions:
            proposed_xy = (x_x + dx * self.resolution, x_y + dy * self.resolution)
            proposed_state = self.snap_to_grid(proposed_xy)
            if self.is_free(proposed_state):
                neighbors.append(proposed_state)
        ########## Code ends here ##########
        return neighbors

    def reconstruct_path(self, come_from):
        """
        Use the self.come_from map to reconstruct a path from the initial location to
        the goal location
        Output:
            A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.come_from[current])
            current = path[-1]
        return list(reversed(path))

    def plot_path(self, fig_num=0, show_init_label=True):
        """Plots the path found in self.path and the obstacles"""
        if self.path is None:
            return

        self.occupancy.plot(fig_num)

        solution_path = np.array(self.path) * self.resolution
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
        plt.scatter([self.x_init[0]*self.resolution, self.x_goal[0]*self.resolution], [self.x_init[1]*self.resolution, self.x_goal[1]*self.resolution], color="green", s=30, zorder=10)
        if show_init_label:
            plt.annotate(r"$x_{init}$", np.array(self.x_init)*self.resolution + np.array([.2, .2]), fontsize=16)
        plt.annotate(r"$x_{goal}$", np.array(self.x_goal)*self.resolution + np.array([.2, .2]), fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)

        plt.axis([0, self.occupancy.width, 0, self.occupancy.height])

    def plot_tree(self, point_size=15):
        plot_line_segments([(x, self.self.come_from[x]) for x in self.self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        plot_line_segments([(x, self.self.come_from[x]) for x in self.self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        px = [x[0] for x in self.self.open_set | self.self.closed_set if x != self.x_init and x != self.x_goal]
        py = [x[1] for x in self.self.open_set | self.self.closed_set if x != self.x_init and x != self.x_goal]
        plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)

    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
            None
        Output:
            Boolean, True if a solution from x_init to x_goal was found

        HINTS:  We're representing the open and closed sets using python's built-in
                set() class. This allows easily adding and removing items using
                .add(item) and .remove(item) respectively, as well as checking for
                set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        # Initialization
        print("ASTar: Started")
        if not self.is_free(self.x_goal):
            print(self.x_goal, self.statespace_lo, self.statespace_hi, self.occupancy.is_free(self.x_goal))
            return False, "AStar: Goal state is occupied"

        step_count = 0

        while self.open_set and step_count < self.time_out_steps:
            if step_count % 100 == 0:
                print("AStar: Step Count %d" % step_count)
            x_current = min(self.open_set, key=lambda x: self.est_cost_through[x])
            if x_current == self.x_goal:
                self.path = self.reconstruct_path(self.come_from)
                return_msg = "AStar: Solved after %d steps" % step_count
                return True, return_msg
            self.open_set.remove(x_current)
            self.closed_set.add(x_current)
            cost_to_arrive_x_current = self.cost_to_arrive[x_current]
            for x_neighbor in self.get_neighbors(x_current):
                if x_neighbor in self.closed_set:
                    continue
                c_neighbor = cost_to_arrive_x_current + self.distance(x_current, x_neighbor)
                if x_neighbor not in self.open_set:
                    self.open_set.add(x_neighbor)
                elif c_neighbor >= self.cost_to_arrive[x_neighbor]:
                    continue
                self.come_from[x_neighbor] = x_current
                self.cost_to_arrive[x_neighbor] = c_neighbor
                self.est_cost_through[x_neighbor] = c_neighbor + self.distance(x_neighbor, self.x_goal)
            step_count += 1
            
        if step_count == self.time_out_steps:
            return False, "AStar: Timeout after %d steps" % self.time_out_steps
        else:
            return False, "Astar: Stopped after exploring %d nodes" % step_count
        ########## Code ends here ##########

class DetOccupancyGrid2D(object):
    """
    A 2D state space grid with a set of rectangular obstacles. The grid is
    fully deterministic
    """
    def __init__(self, width, height, obstacles):
        self.width = width
        self.height = height
        self.obstacles = obstacles

    def is_free(self, x):
        """Verifies that point is not inside any obstacles"""
        for obs in self.obstacles:
            inside = True
            for dim in range(len(x)):
                if x[dim] < obs[0][dim] or x[dim] > obs[1][dim]:
                    inside = False
                    break
            if inside:
                return False
        return True

    def plot(self, fig_num=0):
        """Plots the space and its obstacles"""
        fig = plt.figure(fig_num)
        ax = fig.add_subplot(111, aspect='equal')
        for obs in self.obstacles:
            ax.add_patch(
            patches.Rectangle(
            obs[0],
            obs[1][0]-obs[0][0],
            obs[1][1]-obs[0][1], color='yellow'))
        ax.set(xlim=(0,self.width), ylim=(0,self.height))
