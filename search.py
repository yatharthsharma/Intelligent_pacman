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

    # Hash of Visited node
    hash_visited = {}

    # Initializing stack
    st = util.Stack()
    # pushing the first node
    st.push((problem.getStartState(),[],0))

    # while stack is not empty
    while(not st.isEmpty()):
        #pop the node
        state = st.pop()
        # if the goal state is reached return with the actions
        if problem.isGoalState(state[0]):
            break
        #if not already visited
        elif state[0] not in hash_visited:
            # mark it visited
            hash_visited[state[0]] = 1
            for state_succ in problem.getSuccessors(state[0]):
                # add all the successors if not visited in the stack
                #append the action in the tuple with the next action
                # ((x,y), [previous actions],cost] --> ((x,y), [previous actions+ current action],cost]
                state_succ_tup = state[1] + [state_succ[1]]
                #since tuple is immutatble creating a new tuple
                state_succ_final = (state_succ[0],state_succ_tup,state_succ[2])
                st.push(state_succ_final)

    return state[1]


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the searchdepthFirstSearch tree first."""

    # Hash of Visited node
    hash_visited = {}

    # Initializing Queue
    st = util.Queue()
    # pushing the first node
    st.push((problem.getStartState(), [], 0))
    # while queue is not empty
    while (not st.isEmpty()):
        # pop the node
        state = st.pop()
        # if the goal state is reached return with the actions
        if problem.isGoalState(state[0]):

            break
        elif state[0] not in hash_visited:
            #if not already visited
            hash_visited[state[0]] = 1

            # mark it visited
            for state_succ in problem.getSuccessors(state[0]):
                # add all the successors if not visited in the queue
                # append the action in the tuple with the next action
                # ((x,y), [previous actions],cost] --> ((x,y), [previous actions+ current action],cost]
                state_succ_tup = state[1] + [state_succ[1]]
                # since tuple is immutatble creating a new tuple
                state_succ_final = (state_succ[0], state_succ_tup, state_succ[2])
                st.push(state_succ_final)

    return state[1]

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    hash_visited = {}

    # Initializing a priority queue
    st = util.PriorityQueue()
    # pushing the first node with an arbitarry cost of 0
    st.update((problem.getStartState(), [], 0),0)
    while (not st.isEmpty()):
        # pop the node
        state = st.pop()
        # if the goal state is reached return with the actions
        if problem.isGoalState(state[0]):
            break
        elif state[0] not in hash_visited:
            #if not already visited
            hash_visited[state[0]] = 1
            for state_succ in problem.getSuccessors(state[0]):
                # add all the successors if not visited in the stack
                # append the action in the tuple with the next action
                # ((x,y), [previous actions],cost] --> ((x,y), [previous actions+ current action],cost]
                state_succ_tup = state[1] + [state_succ[1]]
                # since tuple is immutatble creating a new tuple

                #for the cost, we add the cost to visit the successor node with the previous cost
                total_cost = state[2] + state_succ[2]
                state_succ_final = (state_succ[0], state_succ_tup, total_cost)
                st.update(state_succ_final,total_cost)
    return state[1]



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    hash_visited = {}

    # Initializing stack
    st = util.PriorityQueue()
    # pushing the first node with an arbitarry cost of 0
    st.update((problem.getStartState(), [], 0),0)
    while (not st.isEmpty()):
        # pop the node
        state = st.pop()
        # if the goal state is reached return with the actions
        if problem.isGoalState(state[0]):
            break
        elif state[0] not in hash_visited:
            hash_visited[state[0]] = 1
            for state_succ in problem.getSuccessors(state[0]):
                # add all the successors if not visited in the stack
                # append the action in the tuple with the next action
                # ((x,y), [previous actions],cost] --> ((x,y), [previous actions+ current action],cost]
                state_succ_tup = state[1] + [state_succ[1]]
                # since tuple is immutatble creating a new tuple

                #for the cost, we add the cost to visit the successor node with the previous cost 
                #and a heuristics to reach the node 
                total_cost = state[2] + state_succ[2]
                state_succ_final = (state_succ[0], state_succ_tup, total_cost)
                st.update(state_succ_final,total_cost+ heuristic(state_succ[0],problem))
    return state[1]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
