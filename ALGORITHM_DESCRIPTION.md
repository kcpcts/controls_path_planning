# Path Planning Algorithm 

## Overview

This submission implements a risk-aware pathfinding algorithm using the A* search algorithm.

## My Thought Process

### Initial Problem Analysis
When I first looked at this challenge, I thought of these aspects:

1. Multi-objective optimization: The algorithm needs to balance path length and risk - sometimes a longer, safer path is better than a shorter, risky one
2. Hard constraints: Some requirements are absolute (avoid keep-out zones, stay within range) while others are preferences (minimize risk)
3. Classic pathfinding problem: The A* algorithm's design fits this problem of finding optimal paths in a grid with obstacles and costs

### Why I Chose A* Algorithm

Initially, I considered using Dijkstra's algorithm with weights since I learned it in one of my CS courses, but did more research and found that A* is superior for this problem because:

- Goal-directed search: The heuristic guides the search toward the destination, making it much more efficient
- Optimality guarantee: With an admissible heuristic, A* guarantees the shortest path
- Handles complex costs: Can easily incorporate multiple cost factors (distance, risk, smoothness)


### Algorithm Design Decisions
Cost Function Design:
```
Total Cost = Distance + Risk Penalty + Turn Penalty
```

I designed the cost function to handle three objectives:
1. Distance (primary): Actual movement cost, must stay within 50-unit limit
2. Risk penalty (secondary): Small penalty (0.1) for high-risk areas to prefer less risky routes
3. Turn penalty (tertiary): Tiny penalty (0.01) for direction changes, makes smoother paths

Heuristic Choice:
I used Chebyshev distance (max of horizontal/vertical distances) because:
- It's admissible for 8-connected grids (never overestimates)
- Accounts for diagonal movement capability
- Simple and fast to calculate

Parameter Tuning:
- Risk penalty = 0.1: Small enough to not block necessary paths, large enough to prefer safer routes
- Turn penalty = 0.01: Very small, so it only affects tie-breaking between equally good paths

## How the Algorithm Works

1. Start with the starting position, calculate initial heuristic to goal
2. For each position, look at all 8 possible moves (N, S, E, W, NE, NW, SE, SW)
3. Skip moves that go out of bounds or into keep-out zones
4. Compute distance, risk penalty, and turn penalty for each valid move
5. Always process the position with lowest f_cost = g_cost + heuristic
6. Stop when we reach the destination and reconstruct the path


### Examples of Tradeoffs Made
- **Knot**: Accepts a path through many high-risk areas because it's the only way to reach within 50 units
- Takes slightly longer but completely safe paths when distance budget allows