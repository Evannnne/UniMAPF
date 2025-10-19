# UniMAPF
A hybrid prioritized-planning and CBS-based multi-agent pathfinding solver implemented in Unity, designed for dynamic insertion of new agents at any point during runtime with variable speeds, radii, and priorities.

![multi-agent-gif](https://github.com/user-attachments/assets/8c5d871d-16cc-4d48-a711-f89f2bcf923c)

## Technical Overview
This project integrates state-of-the-art multi-agent pathfinding (MAPF) techniques (Conflict-Based Search (CBS), Independence Detection, and Prioritized Planning) into a unified, asynchronous scheduling framework. Agents issue path requests to a scheduler, which get computed asynchronously on a secondary thread and passed to the agent upon completion.

When path requests are made, they are placed into a priority queue. At each `tickInterval`, the scheduler processes pending requests, selecting all agents of the highest current priority. These requests are solved together using a CBS instance, where each agent’s sub-planner (A*) is constrained to avoid paths belonging to higher-priority agents whose routes have already been finalized.

After new paths are found, they are validated against all existing paths of equal or lower priority:
- Collision with a lower-priority path: the lower-priority agent is invalidated.
- Collision with an equal-priority path: that path and all newly computed paths are invalidated.
Invalidating all new paths enables Independence Detection, preventing thrashing cycles (e.g., agents A and B find valid paths that later conflict with C, then C conflicts with A, creating infinite replan loops). Instead, all interdependent agents are merged into a single CBS batch on the next tick.

Example:
Suppose agents A, B, and C share the same priority. A already has a path, and B and C request new paths simultaneously. A CBS instance computes disjoint paths for B and C. Their paths are compared to A’s; A and B collide, so A, B, and C are all re-queued. On the next tick, all three are solved jointly in a CBS instance, yielding a conflict-free, optimal solution.

For constraint checking, this system employs a custom Segment-Based Collision algorithm that provides an exact geometric test for collisions between agents of arbitrary radii, speeds, and progress along their paths. Determining whether movement along an edge collides with another agent’s trajectory is O(log N), where N is the length of the other agent’s path; detecting collisions between two full paths is O(N), where N is the length of the longer path. The implementation has been validated extensively and reports a collision iff the Euclidean distance between two agents is strictly less than the sum of their radii at some time t.

## Architecture
The system consists of:

**IHighLevelAgent:** Interface defining agent-level behavior. Contains callbacks for OnPathInvalidated() and OnPathCompleted(), a Path property exposing the current path, and an AgentDefinition describing radius, speed, and priority.

**PathRequest**: Data structure representing a pathfinding request. Includes a timestamp, the associated IHighLevelAgent, optional mixins for custom per-node cost evaluation and goal-criteria checks, and a startPosition from which the search begins.

**ICostEvaluator / IHeuristicEvaluator**: Interfaces for mixins that define custom cost and heuristic functions, allowing flexible optimization or behavioral tuning.

**INodeConstraint**: Interface for defining pathfinding constraints (e.g., PathIntersectionConstraint), used to encode spatial-temporal exclusion rules.

**NavigationDispatcher**: Central scheduler responsible for processing pending path requests, detecting collisions, and re-queuing invalidated agents.

**BestFirstSolver**: Core pathfinding engine implementing a modular best-first search adaptable to classic A*, Prioritized Planning, or CBS depending on node type. It integrates the evaluator and constraint interfaces for runtime flexibility.

**SegmentedPath**: Data structure representing a sequence of path segments. Provides continuous-time interpolation (position-at-time queries), with edge-path and path-path collision detection functions implemented in UniMAPFPathfindingUtility.

## Algorithm Summary
```
OnTick():
    if not pathfindingInProgress:
        requests ← all pending requests with highest precedence
        remove requests from queue
        pathfindingInProgress ← true
        async SolveGroup(requests)

SolveGroup(requests):
    constraints ← paths of higher-precedence agents
    solvers ← create A* subplanners for each request given constraints
    newPaths ← CBS(solvers)

    if CBS success:
        equalPrecedenceConflict ← false
        for newPath in newPaths:
            for agent in activeAgents:
                if Collision(newPath, agent.path)
                    if agent has lower or equal precedence:
                      invalidate(agent)
                    if agent has equal precedence:
                      equalPrecedenceConflict ← true # Independence Detection
        if equalPrecedenceConflict = false:
            assign newPaths to agents
        else:
            invalidate all requesting agents
    else:
        invalidate all requesting agents

  pathfindingInProgress ← false

# Agents automatically re-request paths on invalidation; these requests get picked up on next tick.
```
## Findings
The system successfully demonstrated real-time integration of Conflict-Based Search (CBS), Prioritized Planning, and Independence Detection for multi-agent pathfinding with continuous space and time. The framework correctly handled dynamic agent requests, arbitrary radii and speeds, and geometric collision checks via the custom Segment-Based Collision (SBC) algorithm.

However, experiments revealed that while small and moderately constrained instances perform well, computation time increases exponentially with problem density due to combinatorial explosion in the state space (multiple entries for the same node at different times). Key observations include:
- Precomputed heuristics cause pathological slowdown in constrained settings, as they bias the search toward spatially optimal but temporally infeasible regions.
- Weighted heuristics (e.g., w=2) yield major speedups (≈2×–5×) with no cost penalty.
- Penalizing re-expansion of repeated states reduces runtime by ~50% on average with negligible cost increases.
- Even with these optimizations, highly constrained cases remain intractable beyond 5–7 agents under 100 ms per request.

Overall, the algorithm achieved roughly 50% of the desired performance target and remained correct and stable in all test cases. The dominant limiting factor is excessive low-level node expansion caused by redundant wait-state exploration.

## Limitations / Future Work
In dense MAPF, compute time explodes. The low-level best-first search expands many wait states at the same (position, increasing time) when constraints block progress; standard goal-distance heuristics bias toward “stay near goal,” which starves exploratory detours. CBS amplifies this by re-invoking low-level search under growing constraint sets, multiplying redundant wait expansions.

Future work:
- Safe-Interval Path Planning (SIPP): Replace per-tick with safe intervals to compress time; this was attempted, but the geometric formulation made it difficult to approach.
- - CBS Heuristics: Using heuristics such as a Weighted Dependency Graph could greatly reduce the multiplicative performacne drops from repeated CBS expansions.
- Sub-Path/Windowed Planning: Do multi-agent planning over small windows only, and extend that window only if conflicts persist
- Minimal Decision Diagrams: Build MDDs for agents for the low-level search under CBS
- Symmetry Breaking: Better penalizing back-and-forth moves and clycic waits
- Parallelization: Solve low-level searches concurrently

## License
Copyright 2025 Evan Sarkozi

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
