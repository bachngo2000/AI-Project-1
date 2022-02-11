# search.py
# ---------
# This codebase is adapted from UC Berkeley AI. Please see the following information about the license.

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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    print(type(problem))  # I added
    start_state = problem.getStartState()  # get the start state
    # make a list for expanded nodes
    expanded = []
    # make a dictionary to map a given state (key) in the path to the state before it in the path
    # The edgeTo map stores backpointers:
    # each vertex remembers what edge was used to arrive at it!
    edge_to = dict()
    # make a list for actions that reach the goal.
    actions = []
    # make a stack for the perimeter/frontier nodes and initialize it with start
    perimeter = util.Stack()
    perimeter.push(start_state)
    # while the stack not empty pop out the top node
    while not perimeter.isEmpty():
        cur_state = perimeter.pop()   # current state whose children we'll be looking at
        # check to see if the current state pass the goal test
        if not problem.isGoalState(cur_state):
            if cur_state not in expanded:  # if current state has not already been expanded
                expanded.append(cur_state)
                # get the list of all successor states from fromState in the order provided by getSuccessors function
                successors_list = problem.getSuccessors(cur_state)  #???? what order
                # loop through the successor states
                for triple in successors_list:
                    # add the current state successors to the perimeter
                    # the first in the successor list will go at the bottom of stack ?????
                    if triple[0] not in expanded:
                        perimeter.push(triple[0])
                        # update the dictionary for the new successors, key is the successor, value is the edge to it
                        edge_to[triple[0]] = (cur_state, triple[1])      #({triple[0]: (cur_state, triple[1])})
        else: # if the current state is the goal :)
            break

            # return a list of actions that reaches the goal.
            # starting from current state which is goal look up the dictionary for back pointers
            # this while loop is building the path
            # however G or the goal has not been expanded to dictionary doesn't have G
            # look up in the values of your dictionary for G and find the corresponding key
    #print('here is your dictionary')
    #print(edge_to)
    #print(cur_state)

    while True:
        edge = edge_to.get(cur_state)[1]
        print(edge)
        actions.insert(0, edge)  # insert at the front for the correct ordering
        cur_state = edge_to.get(cur_state)[0]   # update the loop variable
        if cur_state is start_state:
            break

    return actions


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    start_state = problem.getStartState()  # get the start state
    # make a list for expanded nodes
    expanded = []
    # make a dictionary to map a given state (key) in the path to the state before it in the path
    # The edgeTo map stores backpointers:
    # each vertex remembers what edge was used to arrive at it!
    edge_to = dict()
    # make a list for actions that reach the goal.
    actions = []
    # make a stack for the perimeter/frontier nodes and initialize it with start
    perimeter = util.Queue()
    perimeter.push(start_state)
    # while the stack not empty pop out the top node
    while not perimeter.isEmpty():
        cur_state = perimeter.pop()   # current state whose children we'll be looking at
        # check to see if the current state pass the goal test
        if not problem.isGoalState(cur_state):
            if cur_state not in expanded:  # if current state has not already been expanded
                expanded.append(cur_state)
                # get the list of all successor states from fromState in the order provided by getSuccessors function
                successors_list = problem.getSuccessors(cur_state)  #???? what order
                # loop through the successor states
                for triple in successors_list:
                    # add the current state successors to the perimeter
                    # the first in the successor list will go at the bottom of stack ?????
                    if triple[0] not in expanded:
                        perimeter.push(triple[0])
                        # do not overwrite if there is already an edge established between two nodes.
                        # edgeTo stores backpointers. each node rememebers what node was used to arrive at
                        if triple[0] not in edge_to.keys():
                            edge_to[triple[0]] = (cur_state, triple[1])      #({triple[0]: (cur_state, triple[1])})
        else: # if the current state is the goal :)
            break

            # return a list of actions that reaches the goal.
            # starting from current state which is goal look up the dictionary for back pointers
            # this while loop is building the path
            # however G or the goal has not been expanded to dictionary doesn't have G
            # look up in the values of your dictionary for G and find the corresponding key
    print('++++ here is your dictionary++++')
    print(edge_to)

    while True:
        edge = edge_to.get(cur_state)[1]
        print(edge)
        actions.insert(0, edge)  # insert at the front for the correct ordering
        cur_state = edge_to.get(cur_state)[0]   # update the loop variable
        if cur_state is start_state:
            break

    return actions


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()  # get the start state
    # make a set for expanded nodes
    expanded = []
    # make a dictionary to map a given state (key) in the path to the state before it in the path
    # The edgeTo map stores backpointers:
    # each vertex remembers what edge was used to arrive at it!
    edge_to = dict()
    edge_to[start_state] = ((None, None), 'N/A')

    dist_to = dict()
    dist_to[start_state] = 0

    # make a list for actions that reach the goal.
    actions = []
    # make a min priority queue for the perimeter/frontier nodes and initialize it with start
    perimeter = util.PriorityQueue()
    perimeter.push(start_state, 0)
    # while the min priority queue is not empty pop out the node with smallest so far distance
    while not perimeter.isEmpty():
        cur_state = perimeter.pop()   # current state whose children we'll be looking at, in UCS it is the closest node
        # check to see if the current state pass the goal test
        print(cur_state)
        if not problem.isGoalState(cur_state):
            if cur_state not in expanded:  # if current state has not already been expanded
                expanded.append(cur_state)
                # get the list of all successor states from fromState in the order provided by getSuccessors function
                successors_list = problem.getSuccessors(cur_state)  #???? what order
                # loop through the successor states, in what order they come out?
                for triple in successors_list:
                    # print(successors_list)
                    # add the current state successors to the perimeter
                    # check the best previous path to the successor state
                    if triple[0] in dist_to.keys():
                        old_dist = dist_to.get(triple[0])
                    else:
                        old_dist = int(999999999999999)

                    # claculate the new distance to the successor node from current state
                    new_dist = dist_to.get(cur_state) + triple[2]

                    if new_dist < old_dist:
                        dist_to[triple[0]] = new_dist
                        edge_to[triple[0]] = (cur_state, triple[1])
                        if triple[0] not in dist_to.keys():  # if perimeter doesn't have the successor simply enqueue it
                            print(triple[0])
                            perimeter.push(triple[0], new_dist)
                        # update the priority of the successor if exists in perimeter queue
                        else:
                            perimeter.update(triple[0], new_dist)

        else: # if the current state is the goal :)
            break   # break out of the while loop and stop the iteration over the nodes

    # return a list of actions that reaches the goal.
    # starting from current state which is goal look up the dictionary for back pointers
    # this while loop is building the path
    # however G or the goal has not been expanded to dictionary doesn't have G
    # look up in the values of your dictionary for G and find the corresponding key
    print('++++ here is your edge_to dictionary++++')
    print(edge_to)

    print('++++ here is your dist_to dictionary++++')
    print(dist_to)

    print('++++ here is your cur state ++++')
    print(cur_state)

    while True:
        edge = edge_to.get(cur_state)[1]   # this is the last edge to the goal
        actions.insert(0, edge)  # insert at the front of the action list for the correct ordering
        cur_state = edge_to.get(cur_state)[0]   # update the loop variable to its predecessor

        print(cur_state)

        if cur_state is start_state:
            break
    return actions



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    start_state = problem.getStartState()  # get the start state
    # make a set for expanded nodes
    expanded = []
    # make a dictionary to map a given state (key) in the path to the state before it in the path
    # The edgeTo map stores backpointers:
    # each vertex remembers what edge was used to arrive at it!
    edge_to = dict()
    edge_to[start_state] = (None, 'N/A')

    dist_to = dict()
    dist_to[start_state] = 0

    combined_cost = dict()
    start_cost = 0 + heuristic(start_state, problem)
    combined_cost[start_state] = start_cost

    # make a min priority queue for the perimeter/frontier nodes and initialize it with start
    perimeter = util.PriorityQueue()
    perimeter.push(start_state, start_cost)

    # make a list for actions that reach the goal.
    actions = []

    # while the min priority queue is not empty pop out the node with smallest so far distance
    while not perimeter.isEmpty():
        cur_state = perimeter.pop()  # current state whose children we'll be looking at, in UCS it is the closest node
        # check to see if the current state pass the goal test
        if not problem.isGoalState(cur_state):
            if cur_state not in expanded:  # if current state has not already been expanded
                expanded.append(cur_state)
                # get the list of all successor states from fromState in the order provided by getSuccessors function
                successors_list = problem.getSuccessors(cur_state)  # ???? what order
                # loop through the successor states, in what order they come out?
                for triple in successors_list:
                    # enqueue the successors of current state

                    # check the best previous so-far cost to the successor state and update distTo
                    if triple[0] in dist_to.keys():
                        old_dist = dist_to.get(triple[0])
                    else:
                        old_dist = int(999999999999999)
                    # claculate the new distance and the new cost to the successor node from current state
                    new_dist = dist_to.get(cur_state) + triple[2]
                    if new_dist < old_dist:
                        dist_to[triple[0]] = new_dist
                        edge_to[triple[0]] = (cur_state, triple[1])

                        # update the cobmined cost for the successor
                        combined_cost[triple[0]] = dist_to[triple[0]] + heuristic(triple[0], problem)

                    # if perimeter doesn't have the successor simply enqueue it
                    if triple[0] not in dist_to.keys():
                        perimeter.push(triple[0], combined_cost[triple[0]])
                    # update the priority of the successor if exists in perimeter queue
                    else:
                        perimeter.update(triple[0], new_dist)

        else:  # if the current state is the goal :)
            break  # break out of the while loop and stop the iteration over the nodes

    # return a list of actions that reaches the goal.
    # starting from current state which is goal look up the dictionary for back pointers
    # this while loop is building the path
    # however G or the goal has not been expanded to dictionary doesn't have G
    # look up in the values of your dictionary for G and find the corresponding key
    print('++++ here is your edge_to dictionary++++')
    print(edge_to)

    print('++++ here is your dist_to dictionary++++')
    print(dist_to)

    print('++++ here is your cur state ++++')
    print(cur_state)

    while True:
        edge = edge_to.get(cur_state)[1]  # this is the last edge to the goal
        actions.insert(0, edge)  # insert at the front of the action list for the correct ordering
        cur_state = edge_to.get(cur_state)[0]  # update the loop variable to its predecessor

        print(cur_state)

        if cur_state is start_state:
            break
    return actions

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
