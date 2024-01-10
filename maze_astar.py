import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse


class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """

    def __init__(self, state, cost_from_start, parent=None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():

    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = []  # state
        self.m, self.n = map.shape
        self.map_index = map_index

    def draw(self, node):
        path = []
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)

        draw(self.map, path[::-1], self.map_index)

    def goal_test(self, current_state):
        return current_state == goal_state

    def get_cost(self, current_state, next_state):
        return 1

    def get_successors(self, state):
        successors = []
        row,column = state  # reutnr red location

        if self.map[row + 1][column] == 1.0:  # go right
            successors.append((row + 1, column))  # return a tuple of red location
        if self.map[row - 1][column] == 1.0:  # go left
            successors.append((row - 1, column))

        if self.map[row][column + 1] == 1.0:  # go up
            successors.append((row, column + 1))
            # return a tuple of red location
        if self.map[row][column - 1] == 1.0:  # go down
            successors.append((row, column - 1))
        return successors

    # heuristics function
    def heuristics(self, state):
        row, column = state  # row and colum of  red location
        r, c = self.goal_state  # postion of red in goal state
        m_distance = 0
        if (row,column) != (r,c):
            m_distance = abs(r - row) + abs(c - column)  # distance of current tile from goal tile

        return m_distance

    # priority of node
    def priority(self, node):
        return self.heuristics(node.state) + node.cost_from_start

    # solve it
    def solve(self):
        state = self.start_state  # start state is a tuple
        if self.goal_test(self.start_state):  # WE'VE ALREADY REACHED GOAL
            return
        node = Node(state, 0, None)  # NODE CONTAINS INFO ABOUT THE STATE , we use node to transverse
        self.visited.append(state)  # appended a tuple
        Queue = [node]

        while len(Queue) != 0:
            distance = np.inf
            least = None
            for item in Queue:  # so we can pop based on priority
                if self.priority(item) < distance:
                    distance = self.priority(item)
                    least = item  # save state with the least distance
            node = Queue.pop(Queue.index(least))  # the next node we expand and search in

            successors = self.get_successors(node.state)

            for next_state in successors:
                matching = False
                for item in self.visited:
                    if np.array_equal(item, next_state):  # IF WE ALR HAVE NEXT_STATE IN VISITED
                        matching = True
                if not matching:  # if sucessor hasn't been visited and we've not created a node for it
                    next_cost = node.cost_from_start + self.get_cost(node.state, next_state)
                    next_node = Node(next_state, next_cost, node)
                    if self.goal_test(
                            next_state) is True:  # if the next sucessor is the goal call it to draw it
                        self.draw(next_node)
                        return
                    Queue.append(next_node)  # put the next node at the end of the queue
            self.visited.append(node.state)  # done adding all the successor of the node to heap so its visited


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required=True, type=int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1

    data = np.load('map_' + str(index) + '.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
