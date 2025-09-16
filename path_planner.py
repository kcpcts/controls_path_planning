"""
    Risk-Aware Path Planning Module
    
    This module implements pathfinding using A* algorithm.
    - Uses heuristic guidance for efficient pathfinding
    - Prefers straight paths over zigzag routes
"""
import typing
from queue import PriorityQueue
import heapq
import math

import numpy as np
from typing import Dict, List, Tuple, Optional

from map_info import Coordinate, Destination, MapInfo


class PathPlanner:
    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        """
            map_info: Contains the risk map, start position, and constraints
            destinations: List of target locations to reach
        """
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations
        
        # These control how the algorithm makes trade-offs
        self.high_risk_penalty = 0.1  # Small penalty for each high-risk cell (value 1)
        self.distance_weight = 1      # Base weight for distance traveled
        self.turn_penalty = 0.01      # Tiny penalty for changing direction (encourages straight paths)

    def plan_paths(self):
        """
        Plan optimal paths to all destinations using A* algorithm with risk and turn awareness.
        
        The algorithm optimizes for (in order of priority):
        1. Path distance (primary constraint - must be ≤ maximum_range)
        2. Risk accumulation (minimize high-risk cells)
        3. Turn minimization (just for fun)
        
        This method processes each destination independently and stores the result.
        """
        # Loop through and find the best path to each destination
        for site in self.destinations:
            # Try to find a valid path from start to destination
            path = self._find_path(self.map_info.start_coord, site.coord)
            
            if path:
                # store the path in the destination object
                site.set_path(path)
            else:
                # store empty path (invalid)
                site.set_path([])
    
    def _find_path(self, start: Coordinate, goal: Coordinate) -> Optional[List[Coordinate]]:
        """
        Find optimal path using A* algorithm with multi-objective optimization.
        
        A* uses a heuristic to guide the search toward the goal, making it more efficient
        than Dijkstra's while still guaranteeing the optimal path.
        
        Cost function: distance + risk_penalty + turn_penalty
        - distance: Actual movement distance (counts toward range limit)
        - risk_penalty: Small penalty for high-risk cells (0.1 per cell)
        - turn_penalty: Very small penalty for direction changes (0.01 per turn, just to prefer straight paths)
        
        Args:
            start: Starting coordinate (where the vehicle begins)
            goal: Goal coordinate (destination to reach)
            
        Returns:
            List of coordinates representing the optimal path, or None if unreachable
        """
        # Priority queue stores: (f_cost, g_cost, coordinate, previous_direction)
        # A* processes nodes by lowest f_cost = g_cost + heuristic
        # Direction is stored as (de, dn) tuple (change in east/north from previous step)
        open_set = []
        start_h = self._heuristic(start, goal)
        heapq.heappush(open_set, (start_h, 0, start, None))  # f_cost=heuristic, g_cost=0
        
        # Data structures to track our search progress
        came_from: Dict[Coordinate, Coordinate] = {}    # Which coordinate we came from to reach each point
        g_cost: Dict[Coordinate, float] = {start: 0}    # Total cost (distance + penalties) for priority
        g_distance: Dict[Coordinate, float] = {start: 0} # Pure distance only (for range checking)
        visited = set()  # Coordinates we've already fully processed
        
        # Main A* pathfinding loop
        while open_set:
            # Get the coordinate with the lowest f_cost (g_cost + heuristic) to process next
            current_f_cost, current_g_cost, current, prev_direction = heapq.heappop(open_set)
            
            # Skip if we've already fully processed this coordinate
            # (This can happen if we added the same coordinate multiple times with different costs)
            if current in visited:
                continue
                
            # Mark this coordinate as fully processed
            visited.add(current)
            
            # Check if we reached the goal
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            # Stop exploring from this coordinate if we're already too far
            # (No point continuing if we've exceeded the maximum allowed distance)
            if g_distance.get(current, 0) > self.map_info.maximum_range:
                continue
                
            # Look at all possible moves from current position (8-connected grid)
            # This includes horizontal, vertical, and diagonal moves
            for neighbor in self._get_neighbors(current):
                # Skip if we've already processed this neighbor OR if it's invalid
                # Invalid means: out of bounds, in a keep-out zone, etc.
                if neighbor in visited or not self._is_valid_coordinate(neighbor):
                    continue
                    
                # Calculate how far this move would take us
                distance = self._calculate_distance(current, neighbor)
                tentative_distance = g_distance[current] + distance
                
                # Skip this move if it would take us beyond our maximum allowed range
                if tentative_distance > self.map_info.maximum_range:
                    continue
                
                # Calculate extra costs for this move (risk and turns)
                risk_penalty, turn_penalty = self._calculate_move_penalties(current, neighbor, prev_direction)
                direction = (neighbor.e - current.e, neighbor.n - current.n)  # Direction of this move
                
                # Total cost = actual distance + small penalties for risk and turns
                # Penalties don't count toward range limit (separate from distance)
                tentative_cost = g_cost[current] + distance + risk_penalty + turn_penalty
                
                # Is this a better way to reach this neighbor than any path we've found before?
                if neighbor not in g_cost or tentative_cost < g_cost[neighbor]:
                    # Yes! Remember how we got here and add it to our exploration queue
                    came_from[neighbor] = current              # Remember the path
                    g_cost[neighbor] = tentative_cost          # Remember the total cost
                    g_distance[neighbor] = tentative_distance  # Remember the actual distance
                    
                    # Calculate A* f_cost = g_cost + heuristic
                    h_cost = self._heuristic(neighbor, goal)
                    f_cost = tentative_cost + h_cost
                    heapq.heappush(open_set, (f_cost, tentative_cost, neighbor, direction))
        
        # All possible paths have been explored but none reach the goal
        return None
    
    def _get_neighbors(self, coord: Coordinate) -> List[Coordinate]:
        """
        Get all 8-connected neighbors of a coordinate.
        """
        neighbors = []
        
        # Check all 8 possible directions around the current position
        for de in [-1, 0, 1]:      # Change in East coordinate: West, Same, East
            for dn in [-1, 0, 1]:  # Change in North coordinate: South, Same, North
                # Skip the center position (no movement)
                if de == 0 and dn == 0:
                    continue
                
                # Create a new coordinate by adding the offset
                neighbor = Coordinate(coord.e + de, coord.n + dn)
                neighbors.append(neighbor)
                
        return neighbors
    
    def _is_valid_coordinate(self, coord: Coordinate) -> bool:
        """
        Check if a coordinate is valid for pathfinding.
        
        A coordinate is invalid if:
        1. It's outside the map boundaries
        2. It's in a "keep-out" zone (risk value = 2)
        
        Args:
            coord: The coordinate to check
            
        Returns:
            True if the coordinate is valid to move to, False otherwise
        """
        # Check if coordinate is within map boundaries
        # Map is 60x40, so valid East coordinates are 0-59, North coordinates are 0-39
        if (coord.e < 0 or coord.e >= self.map_info.risk_zones.shape[0] or
            coord.n < 0 or coord.n >= self.map_info.risk_zones.shape[1]):
            return False  # Outside map boundaries
        
        # Check if this location is a "keep-out" zone (forbidden area)
        risk_value = self.map_info.risk_zones[coord.e, coord.n]
        if risk_value == MapInfo.KEEP_OUT_VALUE:  # (2)
            return False  # Cannot enter keep-out zones
            
        # If we get here, the coordinate is valid (either low-risk=0 or high-risk=1)
        return True
    
    def _calculate_move_penalties(self, current: Coordinate, neighbor: Coordinate, 
                                prev_direction: Optional[Tuple[int, int]]) -> Tuple[float, float]:
        """
        Calculate additional penalties for a move beyond just distance.
        
        This function determines extra costs for:
        1. Moving into high-risk areas 
        2. Changing direction (turning)
        
        Args:
            current: The coordinate we're moving from
            neighbor: The coordinate we're moving to
            prev_direction: The direction we came from (None if first move)
        
        Returns:
            Tuple of (risk_penalty, turn_penalty)
        """
        # Calculate risk penalty for the destination cell
        risk_penalty = 0
        if self.map_info.risk_zones[neighbor.e, neighbor.n] == MapInfo.HIGH_RISK_VALUE:  # HIGH_RISK_VALUE = 1
            risk_penalty = self.high_risk_penalty  # Add small penalty for high-risk cells
        
        # Calculate turn penalty if we're changing direction
        turn_penalty = 0
        if prev_direction is not None:  # Only check for turns if we have a previous direction
            # Calculate the direction of this move
            direction = (neighbor.e - current.e, neighbor.n - current.n)
            
            # If this direction is different from previous, we're turning
            if direction != prev_direction:
                turn_penalty = self.turn_penalty  # Add tiny penalty for turns
        
        return risk_penalty, turn_penalty
    
    def _calculate_distance(self, from_coord: Coordinate, to_coord: Coordinate) -> float:
        """
        Calculate the actual distance between two coordinates.
        
        In an 8-connected grid:
        - Horizontal/vertical moves have distance 1.0
        - Diagonal moves have distance √2 ≈ 1.414
        
        This ensures that diagonal shortcuts are properly accounted for in path length.
        
        Args:
            from_coord: Starting coordinate
            to_coord: Ending coordinate
            
        Returns:
            The Euclidean distance between the two coordinates
        """
        # Calculate the change in East and North coordinates
        de = abs(to_coord.e - from_coord.e)  # Horizontal distance
        dn = abs(to_coord.n - from_coord.n)  # Vertical distance
        
        # Determine the type of move and calculate appropriate distance
        if de == 1 and dn == 1:
            # Diagonal move: distance is the hypotenuse, √2
            return math.sqrt(2) 
        else:
            # Horizontal or vertical move: distance is simply 1
            return 1.0
    
    def _heuristic(self, coord: Coordinate, goal: Coordinate) -> float:
        """
        Heuristic function for A* pathfinding.
        
        Uses Chebyshev distance which is admissible for 8-connected grids.
        This estimates the minimum remaining distance to the goal.
        
        Args:
            coord: Current coordinate
            goal: Goal coordinate
            
        Returns:
            Estimated remaining distance (always ≤ actual distance)
        """
        # Calculate horizontal and vertical distances to goal
        de = abs(goal.e - coord.e)  # East-West distance
        dn = abs(goal.n - coord.n)  # North-South distance
        
        # Chebyshev distance = max(horizontal, vertical) distance
        # Perfect for 8-connected grids where you can move diagonally
        return max(de, dn) * self.distance_weight
    
    def _reconstruct_path(self, came_from: Dict[Coordinate, Coordinate], 
                         current: Coordinate) -> List[Coordinate]:
        """
        Reconstruct the complete path from start to goal using the came_from map.
        
        During pathfinding, we remember which coordinate we came from to reach each point.
        This function traces back through those breadcrumbs to build the complete path.
        
        Args:
            came_from: Dictionary mapping each coordinate to the coordinate we came from
            current: The goal coordinate (where we ended up)
            
        Returns:
            Complete path from start to goal as a list of coordinates
        """
        # Start with the goal and work backwards
        path = [current]
        
        # Follow the breadcrumbs back to the start
        while current in came_from:
            current = came_from[current]  # Move to previous coordinate in the path
            path.append(current)          
        
        # We built the path backwards (goal to start), so reverse it
        path.reverse()
        return path  # Now it goes from start to goal