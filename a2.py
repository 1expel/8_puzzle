# Dijkstra's Algorithm vs. Hamming Distance vs. Manhatten Distance
# https://ece.uwaterloo.ca/~dwharder/aads/Algorithms/N_puzzles/
# Dijkstra's considers 139466 states
# Hamming Distance considers 127643 states
# Manhatten Distance considers 2751 states

# IDEAL DATA STRUCTURE
# https://ece.uwaterloo.ca/~dwharder/aads/Algorithms/N_puzzles/
# The data structure used to efficiently solve the A* algorithm is a modified heap 
# which is able to allow the user to update the priority in O(ln(n)) time: 
# a index to each entry is stored in a hash table and when the priority is updated, 
# the index allows the heap to, if necessary, percolate the object up.

# BEST FIRST SEARCH: "Artifical Intelligence, Structures and Strategies for Complex Problem Solving"
# George Luger & Addison Wesley, 6th edition

# case: child not open or closed
    # generate heuristic & add to open
# case: child already open
    # if child was reached by a shorter path
        # give the state on open a shorter path
        # *** SIMPLIFIED *** - add child with shorter path to open - child with longer path will not be executed
# case: child already closed
    # if child was reached by a shorter path
        # remove state from closed
        # add child to open

# *** THOUGHTS ***
# if child is already open, new shorter path, change the 1 child's parent???
# can't cuz my open is in a heap
# dup child in serted into the heap will have priority due to shorter path tho so its okay

# CODE: https://www.geeksforgeeks.org/8-puzzle-problem-using-branch-and-bound/

# code overview: this code solves the 8 puzzle program using a heuristic search
# implemented by a pirority queue using pythons min heap

# *** DEEPER THOUGHTS ***
# can a child thats open have children open? no because that child would be close.
# the only scenario is if that child was closed and then was re open
# so what if we wanna update that child with a shorter path?
# the children would not be updated with that shorter level
# but the new children of the shorter path parent would be priositzed, and both sets of children nodes would be duplicates again
# this would trigger the update child path function, the old children would be updated

# SCENARIO
# closed node re opened with a shorter path
# that closed node becomes open, its children might become duplicates in already open, but the new children have a shorter path, 
    # so we update the children's parent, and cost
# those children did not have any more children, so the updated is fully complete
# IF those children did have more open children, the newer children will have a shorter path, and the old children are simply updated

# MINIMIZING
# dont check closed or open - 104 iterations
# check closed - 46 iterations
# check closed and open - 

# --- LIBRARIES ---

# python heap ds: https://docs.python.org/3/library/heapq.html
from heapq import heappush, heappop

# https://docs.python.org/3/library/copy.html
from copy import deepcopy

# --- CONSTANTS ---

# empty value placeholder
EMPTY_VAL = 0

# 3x3 matrix, can be adjusted any nxn matrix
N = 3

# row adjusters & column adjusters

ROW = [-1, 0, 1, 0]
COL = [0, 1, 0, -1]

# --- DATA STRUCTURES ---

# priority queue implemented by heapq library
class PriorityQueue:

    def __init__(self):
        self.pq = []
        return

    def insert(self, node):
        self.pq.append(node)
        return

    def remove(self):
        min_node_index = 0
        for i in range(1, len(self.pq)):
            if(self.pq[i] < self.pq[min_node_index]):
                min_node_index = i
        return self.pq.pop(min_node_index)

    # SHOULD BE IN NODE DS?
    def _update_child_path(self, old_node, new_node):
        old_node.parent = new_node.parent
        old_node.level = new_node.level
        return
    
    def is_open(self, new_node):
        for i in range(len(self.pq)):
            node = self.pq[i]
            if(node.matrix == new_node.matrix):
                if(new_node < node):
                    self._update_child_path(node, new_node) # YEEEEEET TTTTTTTT
                return True
        return False
    
    def is_empty(self):
        return len(self.pq) == 0

class Closed:

    def __init__(self):
        self.closed = []
        return

    def insert(self, node):
        self.closed.append(node)
        return

    def is_closed(self, new_node):
        for i in range(len(self.closed)):
            node = self.closed[i]
            if(node.matrix == new_node.matrix):
                if(new_node < node):
                    self.closed.pop(i)
                    return False
                else:
                    return True
        return False    

# node class containing parent node, matrix state, empty index in matrix, tree level, and heuritc / cost to goal state
class Node:

    def __init__(self, parent, matrix, empty_index, level, heuristic):
        self.parent = parent
        self.matrix = matrix
        self.empty_index = empty_index
        self.level = level
        self.heuristic = heuristic
        return
    
    # note: on the hardest instances of the puzzle, you must use greedy algorithm instead of A* (optimal)
    # necessary so min heap knows how to evaluate nodes priority
    def __lt__(self, node):
        return self.heuristic < node.heuristic # greedy
        # return (self.level + self.heuristic) < (node.level + node.heuristic) # A*

    def generate_child_node(self, new_empty_index, goal):
        child_matrix = deepcopy(self.matrix)
        x1 = self.empty_index[0]
        y1 = self.empty_index[1]
        x2 = new_empty_index[0]
        y2 = new_empty_index[1]
        temp = child_matrix[x1][y1]
        child_matrix[x1][y1] = child_matrix[x2][y2]
        child_matrix[x2][y2] = temp
        child = Node(
                    self, 
                    child_matrix, 
                    new_empty_index, 
                    self.level + 1, 
                    calc_manhattan_dist(child_matrix, goal)
                )
        return child

    def _print_matrix(self):
        for row in self.matrix:
            for val in row:
                print(val, end=" ")
            print()
        return

    def print_path(self):
        if(self.parent): self.parent.print_path()
        self._print_matrix()
        print()
        return

# --- AUXILIARY FUNCTIONS ---

def is_valid_index(index):
    return -1 < index[0] < N and -1 < index[1] < N

def generate_moves(parent):
    moves = []
    for i in range(4): 
        new_empty_index = [
            parent.empty_index[0] + ROW[i],
            parent.empty_index[1] + COL[i]
        ]
        if(is_valid_index(new_empty_index)): moves.append(new_empty_index)
    return moves

def _flatten_array(array):
    flat_array = []
    for i in range(N):
        for j in range(N):
            flat_array.append(array[i][j])
    return flat_array

def calc_manhattan_dist(array1, array2): 
    flat_array = _flatten_array(array2)
    dist = 0
    for i in range(N):
        for j in range(N):
            index = flat_array.index(array1[i][j])
            dist += abs(index // N - i)
            dist += abs(index % N - j)
    return dist

# https://www.geeksforgeeks.org/check-instance-8-puzzle-solvable/
# modified from this source
def is_solvable(start, goal):
    flat_start = _flatten_array(start)
    flat_goal = _flatten_array(goal)
    inv_count, inv_count2 = 0, 0
    for i in range(0, 9):
        for j in range(i + 1, 9):
            if(flat_start[j] != EMPTY_VAL and flat_start[i] != EMPTY_VAL and flat_start[i] > flat_start[j]):
                inv_count += 1
            if(flat_goal[j] != EMPTY_VAL and flat_goal[i] != EMPTY_VAL and flat_goal[i] > flat_goal[j]):
                inv_count2 += 1
    return inv_count, inv_count2

# --- IMPLEMENTATION ---

# best first search - modified textbook approach
def solve(start, empty_index, goal):
    inv_count, inv_count2 = is_solvable(start, goal)
    if(inv_count % 2 != inv_count2 % 2):
        print('not solvable')
        return
    pq = PriorityQueue()
    closed = Closed()
    root = Node(
        None,
        start,
        empty_index,
        0,
        calc_manhattan_dist(start, goal)
    )
    pq.insert(root)
    while(not pq.is_empty()):
        node = pq.remove()
        closed.insert(node)
        if(node.matrix == goal): return node.print_path()
        moves = generate_moves(node)
        for move in moves:
            child_node = node.generate_child_node(move, goal)
            if(not closed.is_closed(child_node) and not pq.is_open(child_node)): 
                pq.insert(child_node)
    return
    
# --- DRIVER CODE ---

'''
# TEXTBOOK: SOLVABLE
start = [[2, 8, 3],
         [1, 6, 4],
         [7, 0, 5]]

empty_index = [2, 1]

goal = [[1, 2, 3],
        [8, 0, 4],
        [7, 6, 5]]

solve(start, empty_index, goal)
'''

'''
# ASSIGNMENT: SOLVABLE
start = [[3, 8, 2],
         [1, 0, 7],
         [4, 5, 6]]

empty_index = [1, 1]

goal = [[3, 2, 1],
        [4, 0, 8],
        [5, 6, 7]]

solve(start, empty_index, goal)
'''

'''
# ASSIGNMENT: UNSOLVABLE
start = [[8, 3, 2],
         [1, 0, 7],
         [4, 5, 6]]

empty_index = [1, 1]

goal = [[3, 2, 1],
        [4, 0, 8],
        [5, 6, 7]]

solve(start, empty_index, goal)
'''


# HARDEST SOLVABLE: http://w01fe.com/blog/2009/01/the-hardest-eight-puzzle-instances-take-31-moves-to-solve/
start = [[8, 6, 7],
         [2, 5, 4],
         [3, 0, 1]]

empty_index = [2, 1]

goal = [[1, 2, 3],
        [4, 5, 6],
        [7, 8, 0]]

solve(start, empty_index, goal)


# THOUGHTS

# WHAT IS THE POINT OF IMPLEMENTS WITH PQ IF WE NEED TO CHECK OPEN AND CLOSED FOR EVERY CHILD?
# WE CAN EASILY FIND MIN WHEN SINCE WE HAVE TO CHECK O(N) EVERY ITERATION ANYWAYS

# if i wasnt using heap, would updating path be easy? all i would have to do is...
# take old node, look at its max 4 children, update their parents, done

# does simplified verson work?

# ADDD DETECT UNSOLVALBE CODE - https://www.geeksforgeeks.org/check-instance-8-puzzle-solvable/

# implement an open array and do my fixed verion

# TO DO: CHECK IF IS_SOLVABLE, CHECK CODE, TEST CODE, TEST EVERYTHING