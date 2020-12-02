# AI Pacman Search Algorithms

The aim of this project is to use AI search techniques and derive heuristics in Pacman, as well as to understand the Python-based Pacman infrastructure.

## Implementation

Two search algorithms have been implemented which are as follows - 

1. **Iterative Deepening Search** Algorithm, a blind search algorithm.

2. **Enforced Hill Climbing** Algorithm, a local search algorithm. The heuristic used in this algorithm is Manhattan Distance. 

## How to run the algorithms ?

1. To run Iterative Deeepening, type the following code - 

   `python pacman.py -l tinyMaze -p SearchAgent -a fn=ids`

2. To run Enforced Hill Climbing, type the following code - 

   `python pacman.py -l mediumMaze -p SearchAgent -a fn=ehc,heuristic=manhattanHeuristic`

*Note - If you are using python3 replace python with python3 in the commands above. Also, other layouts are available in the layouts directory, and you can easily create you own!*

## Acknowledgement

This is [Project 1 - Search](http://ai.berkeley.edu/search.html) from the set of [UC Pacman Projects](http://ai.berkeley.edu/project_overview.html).
