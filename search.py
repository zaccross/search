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


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """

    # Get Start State
    start = problem.getStartState()

    # if this is already the goal, stop here
    if problem.isGoalState(start):
        return []

    # Inititilize stack for the fringe for FIFO
    fringe = util.Stack()

    # Visited is a list for consistency
    visited = [start]

    # Get first children from start and add to fringe.
    children = problem.getSuccessors(start)
    for child in children:
        if child[0] not in visited:
            new_node = [child[0], [child[1]]]
            fringe.push(new_node)

    # While the fringe is not empty, keep checking for goal and expanding.
    while not fringe.isEmpty():
        solution = _non_cost_search_help(fringe, visited, problem)
        if solution is not None:
            return solution

    # If no solution exists we get here
    print("something went wrong")
    return None


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    # Get Start State
    start = problem.getStartState()

    # Use a Queue for FIFO Fringe System
    fringe = util.Queue()

    # Using a list for visited... why?
    visited = [start]

    children = problem.getSuccessors(start)
    for child in children:
        if child[0] not in visited:
            new_node = [child[0], [child[1]]]
            fringe.push(new_node)

    while not fringe.isEmpty():
        solution = _non_cost_search_help(fringe, visited,problem)
        if solution is not None:
            return solution
    print("something went wrong")
    return None


def _searcher(problem, fringe, visited):
    """Helper function for repeated steps in the looping at end of each search function"""
    return None

def _non_cost_search_help(fringe, visited, problem):
    """Function to handle the node expansion, goal tests, and fringe updates for the bfs and
    dfs searches. Fringe is defined within functions so this should work as general helper
    Non-Cost because it does not append to priority queue..."""

    # Pop the next element off of the fringe
    node = fringe.pop()

    # If its the goal state, return the path
    if problem.isGoalState(node[0]):
        return node[1]

    # Otherwise append the children to the fringe if they haven't been expanded
    else:

        if node[0] not in visited:
            visited.append(node[0])
            children = problem.getSuccessors(node[0])

            for child in children:
                if child[0] not in visited:
                    new_node = [child[0], node[1] + [child[1]]]
                    fringe.push(new_node)

            return None


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    def bfs_help(fringe, visited):

        node = fringe.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        else:
            if node[0] not in visited:
                visited.add(node[0])
                children = problem.getSuccessors(node[0])
                check = False
                for child in children:
                    if child[0] not in visited:
                        check = True
                        path_cost = child[2]+node[2]
                        new_node = [child[0], node[1] + [child[1]], path_cost]

                        fringe.push(new_node, path_cost)

                return None

    start = problem.getStartState()
    fringe = util.PriorityQueue()
    visited = set(start)

    children = problem.getSuccessors(start)
    for child in children:
        print(child)
        if child[0] not in visited:
            cost = child[2]
            new_node = [child[0], [child[1]], cost]
            fringe.push(new_node, cost)
    while not fringe.isEmpty():
        solution = bfs_help(fringe, visited)
        if solution is not None:
            return solution
    print("something went wrong")
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #"*** YOUR CODE HERE ***"

    # right now trying to fix list / hashing problems
    def bfs_help(fringe, visited, heuristic, problem, listCheck):

        node = fringe.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        else:

            if node[0] not in visited:

                if listCheck:
                    visited.append(node[0])
                else:
                    visited.add(node[0])

                children = problem.getSuccessors(node[0])

                for child in children:
                    if child[0] not in visited:

                        path_cost = child[2]+node[2]
                        new_node = [child[0], node[1] + [child[1]], path_cost]

                        fringe.push(new_node, path_cost + heuristic(child[0], problem))

                return None


    #

    start = problem.getStartState()
    print(start)

    fringe = util.PriorityQueue()
    list_check = False

    # I'm sorry this is bad :(
    visited = set()
    if type(start) == list or type(start) == tuple:
        visited = [start]
        list_check = True
    else:
        print(type(start))
        visited = set(start)


    children = problem.getSuccessors(start)
    for child in children:

        cost = child[2]
        new_node = [child[0], [child[1]], cost]

        fringe.push(new_node, cost + heuristic(child[0], problem))
    while not fringe.isEmpty():
        solution = bfs_help(fringe, visited, heuristic, problem, list_check)
        if solution is not None:
            return solution
    print("something went wrong")
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


def search_helper(fringe, visited, problem, heuristic=None):
    node = fringe.pop()
    if problem.isGoalState(node[0]):
        return node

    else:

        if node[0] not in visited:
            visited.append(node[0])
            children = problem.getSuccessors(node[0])
            for child in list(children):
                if list(child[0]) not in list(visited):
                    path_cost = child[2] + node[2]
                    new_node = [child[0], node[1] + [child[1]], path_cost]

                    if heuristic:
                        fringe.push(new_node, path_cost + heuristic(child[0], problem))

                    else:
                        fringe.push(new_node, path_cost)

        return None