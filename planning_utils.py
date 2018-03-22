from enum import Enum
from queue import PriorityQueue

import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),
            ]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)

    UP_LEFT = (-1, -1, 1.42412)
    UP_RIGHT = (-1, 1, 1.42412)
    DOWN_LEFT = (1, -1, 1.42412)
    DOWN_RIGHT = (1, 1, 1.42412)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = list(Action)

    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid.remove(Action.UP_LEFT)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid.remove(Action.UP_RIGHT)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid.remove(Action.DOWN_LEFT)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid.remove(Action.DOWN_RIGHT)

    return valid


def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def heuristic(position, goal_position):
    h = abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])
    return h


def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)


def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon


# We're using collinearity here, but you could use Bresenham as well!
def prune_path(path):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])

        # If the 3 points are in a line remove
        # the 2nd point.
        # The 3rd point now becomes and 2nd point
        # and the check is redone with a new third point
        # on the next iteration.
        if collinearity_check(p1, p2, p3):
            # Something subtle here but we can mutate
            # `pruned_path` freely because the length
            # of the list is check on every iteration.
            pruned_path.remove(pruned_path[i + 1])
        else:
            i += 1
    return pruned_path


def bresenham(p1, p2):
    sign = 1

    if p1[0] > p2[0]:
        _x0 = int(p1[0])
        _x1 = int(p2[0])

        _y0 = int(p1[1])
        _y1 = int(p2[1])

        p1 = (_x1, _y1)
        p2 = (_x0, _y0)

    if p1[1] > p2[1]:
        sign = -1

    x1, y1 = p1
    x2, y2 = p2

    cells = []

    m = abs(y2 - y1) / abs(x2 - x1)

    if sign == 1:
        line_val = y1
        i = x1
        j = y1
        r = i
        k = x2
        while i < x2:
            cells.append([i, j])
            if line_val + m > j + 1:
                j += 1
            else:
                line_val += m
                i += sign
    else:
        line_val = y2
        i = x2
        j = y2

        while x1 < i:
            cells.append([i, j])
            if line_val + m > j + 1:
                j += 1
            else:
                line_val += m
                i += sign

    cells.append([i, j])
    return np.array(cells)


def bres_prune_path(grid, pruned_path):
    pp = list(pruned_path)

    p1 = (int(pp[0][0]), int(pp[0][1]))
    path = [p1]

    i = 1

    while i < len(pp):
        p2 = (int(pp[i][0]), int(pp[i][1]))

        b_pp = bresenham(p1, p2)

        for p in b_pp:
            x = int(p[0])
            y = int(p[1])

            if grid[x][y] == 1:
                p1 = pp[i - 1]
                path.append(pp[i - 1])
                break

        i += 1
    path.append(pp[-1])
    return path
