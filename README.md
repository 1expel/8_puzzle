# 8_puzzle

## Problem

Write a program to solve the 8 tile puzzle

## Solution

- written in python
- implments open list using a modified priority queue that allows searching and a closed list
- visited nodes / states go into the closed list and are only re-opened if a shorter path is found
- visiting a new node already open, if reached by a shorter path, update path, else do not open that new node
- checks if a puzzle is solvable before solving is attempted
- can solve the hardest instance of a 8 puzzle (depth 31 best case) using a greedy algorithm
- greedy algorithm is based on selecting the next best choice using Manhattan distance as the heuristic
- implements PQ, Visited, and Node data structures
