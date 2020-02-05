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

import time
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    startState = problem.getStartState()
    visited = [startState]
    path = []
    fringe = util.Stack()
    parentDict = {}

    neighbours = problem.getSuccessors(problem.getStartState())#.sorted(key = lambda element: element[1])
    for neighbour in neighbours:
        fringe.push(neighbour)
        parentDict[neighbour[0]] = (startState, neighbour[1])

    currentNodePosition = None
    finalNode = None

    while not fringe.isEmpty():
        currentNode = fringe.pop()
        currentNodePosition = currentNode[0]
        if currentNodePosition in visited:
            continue
        else:
            visited.append(currentNodePosition)
        if problem.isGoalState(currentNodePosition):
            finalNode = currentNode
            break
        else:

            for neighbour in problem.getSuccessors(currentNodePosition):
                if neighbour[0] in visited:
                    continue
                else:
                    fringe.push(neighbour)
                    parentDict[neighbour[0]] = (currentNodePosition, neighbour[1])

    if finalNode == None:
        return []
    else:
        tempNode = finalNode
        while True:
            if tempNode[0] == startState:
                break
            parentNode = parentDict[tempNode[0]]
            tempNode = parentNode
            path.append(parentNode[1])

        path.reverse()

    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()
    visited = [startState]
    path = []
    fringe = util.Queue()
    statesInFringe = []
    parentDict = {}

    neighbours = problem.getSuccessors(startState)
    for neighbour in neighbours:
        fringe.push(neighbour)
        statesInFringe.append(neighbour[0])
        parentDict[neighbour[0]] = (startState, neighbour[1])

    currentNodePosition = None
    finalNode = None

    while not fringe.isEmpty():
        currentNode = fringe.pop()
        currentNodePosition = currentNode[0]
        statesInFringe.remove(currentNodePosition)

        if currentNodePosition in visited:
            continue
        else:
            visited.append(currentNodePosition)

        if problem.isGoalState(currentNodePosition):
            finalNode = currentNode
            break
        else:

            for neighbour in problem.getSuccessors(currentNodePosition):
                if neighbour[0] in visited or neighbour[0] in statesInFringe:
                    continue
                else:
                    fringe.push(neighbour)
                    statesInFringe.append(neighbour[0])
                    parentDict[neighbour[0]] = (currentNodePosition, neighbour[1])


    if finalNode == None:
        return []
    else:
        tempNode = finalNode
        while True:
            if tempNode[0] == startState:
                break
            parentNode = parentDict[tempNode[0]]
            tempNode = parentNode
            path.append(parentNode[1])

        path.reverse()

    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()
    startCost = 0
    visited = [startState]
    path = []
    fringe = util.PriorityQueue()
    statesInFringe = []
    parentDict = {}

    neighbours = problem.getSuccessors(startState)
    for neighbour in neighbours:
        # print(neighbour)
        cost = startCost + neighbour[2]
        fringe.push(priority = cost, item = neighbour[0])
        statesInFringe.append(neighbour[0])
        parentDict[neighbour[0]] = (startState, neighbour[1])

    currentNodePosition = None
    finalNode = None

    while not fringe.isEmpty():
        currentNode = fringe.pop()
        currentNodePosition = currentNode[0]
        currentNodeCost = currentNode[1]
        statesInFringe.remove(currentNodePosition)

        # print(currentNode)
        # assert False

        if currentNodePosition in visited:
            continue
        else:
            visited.append(currentNodePosition)

        if problem.isGoalState(currentNodePosition):
            finalNode = currentNodePosition
            break
        else:

            for neighbour in problem.getSuccessors(currentNodePosition):
                # print(neighbour)
                neighbourCostAddition = neighbour[2]
                if neighbour[0] not in visited and neighbour[0] not in statesInFringe:
                    totalCost = currentNodeCost + neighbourCostAddition
                    fringe.push(priority = totalCost, item = neighbour[0])
                    statesInFringe.append(neighbour[0])
                    parentDict[neighbour[0]] = (currentNodePosition, neighbour[1])
                elif neighbour[0] in statesInFringe:
                    totalCost = currentNodeCost + neighbourCostAddition
                    flag = fringe.update(item = neighbour[0], priority = totalCost)
                    if flag == True:
                        parentDict[neighbour[0]] = (currentNodePosition, neighbour[1])

    # print(parentDict)
    if finalNode == None:
        return []
    else:
        # print("Final Node : ", finalNode)
        tempNode = parentDict[finalNode]
        # print("First Temp Node Print : ", tempNode)
        path.append(tempNode[1])
        while True:
            # print("Again Temp Node Print : ", tempNode)
            if tempNode[0] == startState:
                break
            parentNode = parentDict[tempNode[0]]
            tempNode = parentNode
            path.append(tempNode[1])

        path.reverse()

    return path

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
