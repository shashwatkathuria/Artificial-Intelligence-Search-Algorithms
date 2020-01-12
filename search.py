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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    "*** YOUR CODE HERE ***"

    # Initializing variables required
    startState = problem.getStartState()
    visited = [startState]
    path = []

    # For DFS, LIFO - Stack
    fringe = util.Stack()

    # To store parent info for backtracking
    parentDict = {}

    # Starting the problem with the neighbours of the start state
    neighbours = problem.getSuccessors(problem.getStartState())#.sorted(key = lambda element: element[1])

    # Pushing neighbours and appropriate info into dict
    for neighbour in neighbours:
        neighbourPosition = neighbour[0]
        edgeInfo = neighbour[1]
        fringe.push(neighbour)
        parentDict[neighbourPosition] = (startState, edgeInfo)

    # For storing current node position and final node
    currentNodePosition = None
    finalNode = None

    # Runs while the fringe is not empty
    while not fringe.isEmpty():
        # Popping topmost element of stack
        currentNode = fringe.pop()
        # Storing info of position of current node
        currentNodePosition = currentNode[0]

        # Skipping to next iteration if already visited
        if currentNodePosition in visited:
            continue
        # Otherwise appending to visited list
        else:
            visited.append(currentNodePosition)

        # Breaking out of while loop if goal state
        if problem.isGoalState(currentNodePosition):
            # Storing goal state for further processing
            finalNode = currentNode
            break
        # Otherwise processing neighbours into fringe
        else:

            # For all neighbours of the current node
            for neighbour in problem.getSuccessors(currentNodePosition):
                neighbourPosition = neighbour[0]
                edgeInfo = neighbour[1]
                # Skipping to next iteration if already visited
                if neighbourPosition in visited:
                    continue
                # Else pushing into fringe and parent dict with appropriate info
                else:
                    fringe.push(neighbour)
                    parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

    # Returning empty path if no final state is found
    if finalNode == None:
        return []
    # Else backtracking the path found till final node
    else:

        # Temporary node to store backtracking iteration data
        tempNode = finalNode
        while True:
            tempNodePosition = tempNode[0]
            # Breaking if start state reached
            if tempNodePosition == startState:
                break
            # Updating temporary node to be its parent
            parentNode = parentDict[tempNodePosition]
            backtrackEdgeInfo = parentNode[1]
            tempNode = parentNode
            # Storing backtrack edge info
            path.append(backtrackEdgeInfo)

        # Reversing to get the backtracked path from start to final node
        path.reverse()

    # Returning path
    return path

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # Initializing variables required
    startState = problem.getStartState()
    visited = [startState]
    path = []

    # For BFS, FIFO - Queue
    fringe = util.Queue()
    # To keep track of the states in the fringe
    statesInFringe = []
    # To store parent info for backtracking
    parentDict = {}

    # Starting the problem with the neighbours of the start state
    neighbours = problem.getSuccessors(startState)

    # Pushing neighbours and appropriate info into dict
    for neighbour in neighbours:
        neighbourPosition = neighbour[0]
        edgeInfo = neighbour[1]
        fringe.push(neighbour)
        statesInFringe.append(neighbourPosition)
        parentDict[neighbourPosition] = (startState, edgeInfo)

    # For storing current node position and final node
    currentNodePosition = None
    finalNode = None

    # Runs while the fringe is not empty
    while not fringe.isEmpty():
        # Popping frontmost element of queue
        currentNode = fringe.pop()
        # Storing info of position of current node
        currentNodePosition = currentNode[0]
        # Removing from track of states in fringe
        statesInFringe.remove(currentNodePosition)

        # Skipping to next iteration if already visited
        if currentNodePosition in visited:
            continue
        # Otherwise appending to visited list
        else:
            visited.append(currentNodePosition)

        # Breaking out of while loop if goal state
        if problem.isGoalState(currentNodePosition):
            # Storing goal state for further processing
            finalNode = currentNode
            break
        # Otherwise processing neighbours into fringe
        else:

            # For all neighbours of the current node
            for neighbour in problem.getSuccessors(currentNodePosition):
                neighbourPosition = neighbour[0]
                edgeInfo = neighbour[1]
                # Skipping to next iteration if already visited or in fringe
                if neighbourPosition in visited or neighbourPosition in statesInFringe:
                    continue
                # Else pushing into fringe, states in fringe and parent dict with appropriate info
                else:
                    fringe.push(neighbour)
                    statesInFringe.append(neighbourPosition)
                    parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

    # Returning empty path if no final state is found
    if finalNode == None:
        return []
    # Else backtracking the path found till final node
    else:

        # Temporary node to store backtracking iteration data
        tempNode = finalNode
        while True:
            tempNodePosition = tempNode[0]
            # Breaking if start state reached
            if tempNodePosition == startState:
                break
            # Updating temporary node to be its parent
            parentNode = parentDict[tempNodePosition]
            backtrackEdgeInfo = parentNode[1]
            tempNode = parentNode
            # Storing backtrack edge info
            path.append(backtrackEdgeInfo)

        # Reversing to get the backtracked path from start to final node
        path.reverse()

    # Returning path
    return path

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"

    # Initializing variables required
    startState = problem.getStartState()
    startCost = 0
    visited = [startState]
    path = []

    # For UCS, Priority Queue
    fringe = util.PriorityQueue()
    # To keep track of the states in the fringe
    statesInFringe = []
    # To store parent info for backtracking
    parentDict = {}
    # To keep track of cost of path till visited node
    costDict = {}

    # Starting the problem with the neighbours of the start state
    neighbours = problem.getSuccessors(startState)

    # Pushing neighbours and appropriate info into dict
    for neighbour in neighbours:
        neighbourPosition = neighbour[0]
        edgeInfo = neighbour[1]
        neighbourCostAddition = neighbour[2]

        # Saving cost of path till now in cost dict
        cost = startCost + neighbourCostAddition
        costDict[neighbourPosition] = cost

        # Pushing node in appropriate places
        fringe.push(item = neighbourPosition, priority = cost)
        statesInFringe.append(neighbourPosition)
        # Saving parent node for backtracking info
        parentDict[neighbourPosition] = (startState, edgeInfo)

    # For storing current node position and final node
    currentNodePosition = None
    finalNode = None

    # Runs while the fringe is not empty
    while not fringe.isEmpty():
        # Popping least value element of priority queue
        currentNodePosition = fringe.pop()
        # Storing info of position of current node
        currentNodeCost = costDict[currentNodePosition]
        # Removing from track of states in fringe
        statesInFringe.remove(currentNodePosition)

        # Skipping to next iteration if already visited
        if currentNodePosition in visited:
            continue
        # Otherwise appending to visited list
        else:
            visited.append(currentNodePosition)

        # Breaking out of while loop if goal state
        if problem.isGoalState(currentNodePosition):
            # Storing goal state for further processing
            finalNode = currentNodePosition
            break
        # Otherwise processing neighbours into fringe
        else:

            # For all neighbours of the current node
            for neighbour in problem.getSuccessors(currentNodePosition):
                neighbourPosition = neighbour[0]
                edgeInfo = neighbour[1]
                neighbourCostAddition = neighbour[2]

                # Adding neighbour node not if visited and not in fringe
                if neighbourPosition not in visited and neighbourPosition not in statesInFringe:
                    # Calculating total path cost till neighbour node
                    totalCost = currentNodeCost + neighbourCostAddition
                    # Storing total path cost in cost dict
                    costDict[neighbourPosition] = totalCost

                    # Pushing node in appropriate places
                    fringe.push(item = neighbourPosition, priority = totalCost)
                    statesInFringe.append(neighbourPosition)
                    # Saving parent node for backtracking info
                    parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

                # Otherwise if the neighbour node is already in the fringe
                elif neighbourPosition in statesInFringe:

                    # Calculating new cost
                    totalCost = currentNodeCost + neighbourCostAddition
                    # Passing new cost value to update function to appropriately
                    # update if required
                    fringe.update(item = neighbourPosition, priority = totalCost)

                    # Updating values if the new value is lesser than the old value
                    if totalCost < costDict[neighbourPosition]:
                        costDict[neighbourPosition] = totalCost
                        parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

    # Returning empty path if no final state is found
    if finalNode == None:
        return []
    # Else backtracking the path found till final node
    else:
        # Temporary node to store backtracking iteration data
        # Getting parent of final node
        tempNode = parentDict[finalNode]
        # Storing into backtrack edge into path
        path.append(tempNode[1])
        while True:
            tempNodePosition = tempNode[0]
            # Breaking if start state reached
            if tempNodePosition == startState:
                break
            # Updating temporary node to be its parent
            parentNode = parentDict[tempNodePosition]
            backtrackEdgeInfo = parentNode[1]
            tempNode = parentNode
            # Storing backtrack edge info
            path.append(backtrackEdgeInfo)

        # Reversing to get the backtracked path from start to final node
        path.reverse()

    # Returning path
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

    # Initializing variables required
    startState = problem.getStartState()
    startHeuristic = heuristic(startState, problem)
    startCost = 0
    visited = [startState]
    path = []

    # Dicts for storing values of functions g and h
    # g = total cost till from starting till current node
    # h = heuristic cost till from current node till final node
    g = {}
    h = {}

    # For UCS, Priority Queue With Function (f = g + h)
    fringe = util.PriorityQueueWithFunction(priorityFunction = lambda item : g[item] + h[item])
    # To keep track of the states in the fringe
    statesInFringe = []
    # To store parent info for backtracking
    parentDict = {}

    # Starting the problem with the neighbours of the start state
    neighbours = problem.getSuccessors(startState)

    # Pushing neighbours and appropriate info into dict
    for neighbour in neighbours:
        neighbourPosition = neighbour[0]
        edgeInfo = neighbour[1]
        neighbourCostAddition = neighbour[2]

        # Getting g and h values of neighbour node
        gValue = startCost + neighbourCostAddition
        hValue = heuristic(neighbourPosition, problem)

        # Storing g and h values of neighbour node into dicts
        h[neighbourPosition] = hValue
        g[neighbourPosition] = gValue

        # Pushing node in appropriate places
        fringe.push(item = neighbourPosition)
        statesInFringe.append(neighbourPosition)
        # Saving parent node for backtracking info
        parentDict[neighbourPosition] = (startState, edgeInfo)

    # For storing current node position and final node
    currentNodePosition = None
    finalNode = None

    # Runs while the fringe is not empty
    while not fringe.isEmpty():
        # Popping least value element of priority queue
        currentNodePosition = fringe.pop()

        # Getting actual path cost till current node
        currentNodeCost = g[currentNodePosition]

        # Removing from track of states in fringe
        statesInFringe.remove(currentNodePosition)

        # Skipping to next iteration if already visited
        if currentNodePosition in visited:
            continue
        # Otherwise appending to visited list
        else:
            visited.append(currentNodePosition)

        # Breaking out of while loop if goal state
        if problem.isGoalState(currentNodePosition):
            # Storing goal state for further processing
            finalNode = currentNodePosition
            break
        # Otherwise processing neighbours into fringe
        else:

            # For all neighbours of the current node
            for neighbour in problem.getSuccessors(currentNodePosition):
                neighbourPosition = neighbour[0]
                edgeInfo = neighbour[1]
                neighbourCostAddition = neighbour[2]

                # Calculating h value of neighbour node
                gValue = currentNodeCost + neighbourCostAddition
                hValue = heuristic(neighbourPosition, problem)

                # Storing h value of a state which never changes
                h[neighbourPosition] = hValue

                # Adding neighbour node not if visited and not in fringe
                if neighbourPosition not in visited and neighbourPosition not in statesInFringe:
                    # Storing g value of neighbour node
                    g[neighbourPosition] = gValue

                    # Pushing node in appropriate places
                    fringe.push(item = neighbourPosition)
                    statesInFringe.append(neighbourPosition)
                    # Saving parent node for backtracking info
                    parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

                # Otherwise if the neighbour node is already in the fringe
                elif neighbourPosition in statesInFringe:

                    # Updating the values if new g value is better (smaller) than old value
                    if gValue < g[neighbourPosition]:
                        # Storing new g value in dict
                        g[neighbourPosition] = gValue

                        # Storing node in fringe again because this time it gets
                        # stored with a new value instead of the old value in
                        # the current fringe so that this gets more priority
                        # and eliminates the chances of the older value to pop
                        # once this is popped, which happens earlier
                        fringe.push(item = neighbourPosition)
                        statesInFringe.append(neighbourPosition)
                        parentDict[neighbourPosition] = (currentNodePosition, edgeInfo)

    # Returning empty path if no final state is found
    if finalNode == None:
        return []
    # Else backtracking the path found till final node
    else:
        # Temporary node to store backtracking iteration data
        # Getting parent of final node
        tempNode = parentDict[finalNode]
        # Storing into backtrack edge into path
        path.append(tempNode[1])
        while True:
            tempNodePosition = tempNode[0]
            # Breaking if start state reached
            if tempNodePosition == startState:
                break
            # Updating temporary node to be its parent
            parentNode = parentDict[tempNodePosition]
            backtrackEdgeInfo = parentNode[1]
            tempNode = parentNode
            # Storing backtrack edge info
            path.append(backtrackEdgeInfo)

        # Reversing to get the backtracked path from start to final node
        path.reverse()

    # Returning path
    return path

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
