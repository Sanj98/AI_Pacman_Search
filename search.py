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
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    initial_node = problem.getStartState()
    opened = util.Stack()
    closed = set()
    opened.push((initial_node, []))
    while not opened.isEmpty():
        node = opened.pop()
        state, actions = node
        if state not in closed:
            closed.add(state)
            if problem.isGoalState(state):
                return actions
            for successor in problem.getSuccessors(state):
                succ_state, succ_action, succ_cost = successor
                new_node = succ_state, actions + [succ_action]
                opened.push(new_node)
    return 0
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

# In both pratical task and Assignment 1
def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 3 ***"
    initial_node = (problem.getStartState(),[],0)
    initial_state, initial_action, initial_cost = initial_node
    opened = util.PriorityQueue()
    closed = set()
    initial_heuristic = heuristic(initial_state, problem)
    opened.push(initial_node, initial_heuristic)
    while not opened.isEmpty():
      node = opened.pop()
      state, actions, cost = node
      if state not in closed:
         closed.add(state)
         if problem.isGoalState(state):
            return actions
         for successor in problem.getSuccessors(state):
            succ_state, succ_action, succ_cost = successor
            new_node = succ_state, actions + [succ_action], cost + succ_cost
            succ_heuristic = heuristic(succ_state, problem)
            f = (cost + succ_cost) + succ_heuristic
            opened.push(new_node, f)
    return 0
    util.raiseNotDefined()

# Extensions Assignment 1
def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""
    "*** YOUR CODE HERE FOR TASK 1 ***"
    path = []
    initial_node = (problem.getStartState(),'',0)
    for depth in range(0, 1000):
        path, goalFound, remaining = DLS(initial_node, path, problem, depth)
        if goalFound:
           return path
        elif not remaining:
           return None


def DLS(node, path, problem, depth):
  state, action, cost = node
  if depth == 0:
    if problem.isGoalState(state):
       return path, True, True
    else:
       return path, False, True
  elif depth > 0:
    any_remaining = False
    for successor in  problem.getSuccessors(state):
       succ_state, succ_action, succ_cost = successor
       path.append(succ_action)
       path, goalFound, remaining = DLS(successor, path, problem , depth-1)
       if goalFound:
          return path, True, True
       if remaining:
          any_remaining = True
       if len(path) > 0:
          path.pop()
    return path, False, any_remaining
    util.raiseNotDefined()

def enforcedHillClimbing(problem, heuristic=nullHeuristic):
    """
    Local search with heuristic function.
    You DO NOT need to implement any heuristic, but you DO have to call it.
    The heuristic function is "manhattanHeuristic" from searchAgent.py.
    It will be pass to this function as second arguement (heuristic).
    """
    "*** YOUR CODE HERE FOR TASK 2 ***"
    "Data structure to be used - Queue"
    initial_node = (problem.getStartState(), '', 0, [])
    state, action, cost, dist = initial_node
    while not problem.isGoalState(state):
       initial_node = improve(initial_node, problem, heuristic)
       state, action, cost, dist = initial_node
    dist = dist + [(state, action)]
    actions = [action[1] for action in dist]
    del actions[0]
    return actions


def improve(node, problem, heuristic):
    opened = util.Queue()
    closed = set()
    opened.push(node)
    best_h = heuristic(node[0], problem)
    while not opened.isEmpty():
       node = opened.pop()
       state, action, cost, dist = node
       if state not in closed:
          closed.add(state)
          success_h = heuristic(state, problem)
          if success_h < best_h:
              return node
          for successor in problem.getSuccessors(state):
              succ_state, succ_action, succ_cost = successor
              new_node = (succ_state, succ_action, cost + succ_cost, dist + [(state, action)])
              opened.push(new_node)
    return 0
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
ehc = enforcedHillClimbing
