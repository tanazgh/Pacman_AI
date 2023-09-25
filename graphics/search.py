# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depth_first_search(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    frontier = util.Stack()
    # Just location, like [7, 7]
    start = problem.getStartState()
    # (location, path)
    start_node = (start, [])
    frontier.push(start_node)
    reached = list()

    while not frontier.isEmpty():
        # node[0] is location, while node[1] is path
        node = frontier.pop()
        reached.append(node[0])
        if problem.isGoalState(node[0]):
            return node[1]
        successors = problem.getSuccessors(node[0])
        for next in successors:
            if next[0] not in reached:
                frontier.push((next[0], node[1] + [next[1]]))

def breadth_first_search(problem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()
    # Just location, like [7, 7]
    start = problem.getStartState()
    # (location, path)
    start_node = (start, [])
    frontier.push(start_node)
    reached = list()
    reached.append(start)

    while not frontier.isEmpty():
        # node[0] is location, while node[1] is path
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        successors = problem.getSuccessors(node[0])
        for next in successors:
            if next[0] not in reached:
                reached.append(next[0])
                frontier.push((next[0], node[1] + [next[1]]))

def uniform_cost_search(problem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()
    # Just location, like [7, 7]
    start = problem.getStartState()
    # (location, path, cost)
    start_node = (start, [], 0)
    frontier.push(start_node, 0)
    reached = list()

    while not frontier.isEmpty():
        # node[0] is location, while node[1] is path, while node[2] is cumulative cost
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in reached:
            reached.append(node[0])
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in reached:
                    cost = node[2] + successor[2]
                    frontier.push((successor[0], node[1] + [successor[1]], cost), cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def astar_search(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    frontier = util.PriorityQueue()
    # Just location, like [7, 7]
    start = problem.getStartState()
    # (location, path, cost)
    start_node = (start, [], 0)
    frontier.push(start_node, 0)
    reached = list()

    while not frontier.isEmpty():
        # node[0] is location, while node[1] is path, while node[2] is cumulative cost
        node = frontier.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in reached:
            reached.append(node[0])
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in reached:
                    cost = node[2] + successor[2]
                    totalCost = cost + heuristic(successor[0], problem)
                    frontier.push((successor[0], node[1] + [successor[1]], cost), totalCost)


# Abbreviations
bfs = breadth_first_search
dfs = depth_first_search
astar = astar_search
ucs = uniform_cost_search
