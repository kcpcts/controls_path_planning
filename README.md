# Risk-Aware Path Planning

An implementation of the A* pathfinding algorithm that balances safety, efficiency, and path smoothness with multi-objective optimization.

## Overview

The algorithm finds optimal paths that:

- **Avoid obstacles** (keep-out zones)
- **Minimize risk** (prefer low-risk over high-risk areas)
- **Stay within range limits** (maximum distance constraints)
- **Prefer smooth paths** (minimize unnecessary turns)

## Algorithm

The pathfinding uses the **A* algorithm** with a custom cost function:

```
Total Cost = Distance + Risk Penalty + Turn Penalty
```

Where:
- **Distance**: Actual movement distance (counts toward range limit)
- **Risk Penalty**: Small penalty for high-risk cells (0.1 per cell)
- **Turn Penalty**: Tiny penalty for direction changes (0.01 per turn, just to prefer more consistent, straighter paths)

## Quick Start

### Prerequisites
- Python 3.X
- Required packages: `numpy`, `matplotlib`, `PyYAML`

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd controls_path_planning

# Install dependencies
pip install -r requirements.txt
```

### Running the Algorithm
```bash
# Run the pathfinder and see results
python3 test_planner.py
```