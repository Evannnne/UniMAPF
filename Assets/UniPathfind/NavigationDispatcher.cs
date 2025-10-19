#define LOG_VERBOSE
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

namespace UniMAPF.Pathfinding
{
    /// <summary>
    /// A wrapper for NavNode that keeps track of time
    /// </summary>
    public class TimeBasedNavNode : IGraphNode, INodePositionProperty, INodeTimeProperty, INodeAgentProperty, INodeWaitCountProperty
    {
        public int WaitCount { get; }
        public NavNode root;
        public Vector3 Position { get; private set; }
        public float Time { get; private set; }
        public AgentDefinition Agent { get; private set; }

        public TimeBasedNavNode(AgentDefinition srcAgent, NavNode baseNode, float time, int waitCount = 0)
        {
            root = baseNode;
            Time = time;
            Position = baseNode.position;
            Agent = srcAgent;
            WaitCount = waitCount;
        }

        public override bool Equals(object obj)
        {
            var tb = obj as TimeBasedNavNode;
            if (tb != null)
            {
                return tb.root == root;
            }
            else return false;
        }
        public override int GetHashCode()
        {
            return root.GetHashCode();
        }

        public IEnumerable<IGraphNode> GetAdjacents(float costBound)
        {
            List<IGraphNode> adjacents = new List<IGraphNode>();
            foreach(var adj in root.GetAdjacents(costBound))
            {
                var castedAdj = (NavNode)adj;
                adjacents.Add(new TimeBasedNavNode(Agent, castedAdj, Time + Vector3.Distance(Position, castedAdj.Position) / Agent.speed, waitCount: WaitCount));
            }
            adjacents.Add(new TimeBasedNavNode(Agent, root, Time + 1.0f, waitCount: WaitCount + 1));
            return adjacents;
        }

        public override string ToString()
        {
            return $"(pos={Position}, time={Time})";
        }
    }

    /// <summary>
    /// A definition for an agent, defining its radius, speed, and priority
    /// </summary>
    [Serializable]
    public class AgentDefinition
    {
        public float radius;
        public float speed;
        public int priority;
    } 
    
    public interface IHighLevelAgent
    {
        /// <summary>
        /// The position of the Agent
        /// </summary>
        public Vector3 Position { get; }
        
        /// <summary>
        /// The current Path of the Agent
        /// </summary>
        public SegmentedPath Path { get; set; }

        /// <summary>
        /// The Definition of the agent (i.e. Radius, Speed, Priority)
        /// </summary>
        public AgentDefinition Definition { get; }

        /// <summary>
        /// Called when an Agent's path is no longer valid. Agents are expected to re-request their desired path upon invalidation.
        /// </summary>
        public void OnPathInvalidated();
        
        /// <summary>
        /// Called when a new path is available
        /// </summary>
        public void OnNewPathComputed();
    }

    public class PathRequest
    {
        public float requestTime;
        public IHighLevelAgent agent;
        public List<ICostEvaluator<TimeBasedNavNode>> additionalCosts;
        public INodeGoalChecker<TimeBasedNavNode> goalCriteria;
        public Vector3 startPosition;
    }

    /// <summary>
    /// An adapter that allows Time-based nodes to use Floyd warshall. Multiplier should be the inverse of the agent's speed.
    /// </summary>
    public class APSPTimeBasedAdapter : IHeuristicEvaluator<TimeBasedNavNode>
    {
        private FloydWarshallLookupTableHeuristic<NavNode> m_baseLookup;
        private float m_agentSpeed;

        public APSPTimeBasedAdapter(FloydWarshallLookupTableHeuristic<NavNode> baseLookup, float agentSpeed, NavNode goalNode)
        {
            m_baseLookup = baseLookup;
            m_agentSpeed = agentSpeed;
            m_baseLookup.GoalNode = goalNode;
        }
        public float CalculateCost(TimeBasedNavNode node)
        {
            return m_baseLookup.CalculateCost(node.root) / m_agentSpeed;
        }
    }

    // High level summary:
    //
    // The NavigationDispatcher keeps track of Paths for HighLevelAgents, ensuring all agents are collision-free. To use it:
    // 1. Call RequestPath(), giving an Agent, what Costs it wants to include in its pathfinding, and what it considers a Goal.
    // 2. The Dispatcher will then Automatically apply additional Heuristics to the created Solver depending on Goal Type, and other passed in properties.
    // 3. The Dispatcher will then put this information into a PathRequest that gets placed into a queue, to be pulled when UpdateRequests() is invoked.
    // 
    // When UpdateRequests is called, it will:
    // 1. Collect all requests of the current highest priority present
    // 2. Create a CBS meta-solver for the requests, and execute it on another thread
    // 3. Now, whether a path is found:
    //      a) True - Invalidate all existing paths of equal or higher priority that conflict with the new paths. If any paths of the same priority conflicted:
    //          i) True - Don't assign the newly found path, instead Invalidate the agent which should re-request a path
    //          ii) False - Assign the newly found path to the agent who requested it
    //      b) False - invalidate all requested agents, prompting a re-request
    // 
    // Ideally, when multiple agents of the same priority request a path, it is as follows:
    // 1. Agent A requests path
    // 2. Agent A gets Path A1
    // 3. Agent B requests path
    // 4. Agent B gets path B1
    // 5. B1 conflicts with A1, so put A,B back into queue
    // 6. Agents A,B request path
    // 7. Agents A,B get paths A2,B2
    // 8. Agent C requests path
    // 9. Agent C gets path C1
    // 10. C1 conflicts with path A1, so put A,B,C back into queue
    // 11. Agents A,B,C request path
    // 12. Agents A,B,C get paths A3, B3, C2
    public class NavigationDispatcher : MonoSingleton<NavigationDispatcher>
    {
        public float maxNavigationTimeLowLevelMs = 100;
        public float maxNavigationTimeMs = 500;

        public float hyperParam_heuristicWeight = 1f;
        public float hyperParam_conflictReanalysisRange = 0f;

        public bool debug_showExplorations;
        public int debug_showExplorationIndex;
        public int debug_showExplorationUntil;

        public float tickInterval = -1;

        public bool JobInProgress => m_pathfindingInProgress;

        [NonSerialized] public NavNode[] navNodes;
        [NonSerialized] public HashSet<IHighLevelAgent> registeredAgents = new HashSet<IHighLevelAgent>();
        private List<PathRequest> pathRequests = new List<PathRequest>();

        private bool m_pathfindingInProgress = false;
        private Queue<Action> m_queuedOnMainThread = new Queue<Action>();

        private void Start()
        {
            navNodes = FindObjectsOfType<NavNode>();
            Debug.Log($"Found {navNodes.Length} NavNodes");
        }

        private float m_elapsed;

        private void Update()
        {
            if (tickInterval > 0)
            {
                m_elapsed += Time.deltaTime;
                if(m_elapsed > tickInterval)
                {
                    UpdateRequests();
                    m_elapsed = 0;
                }
            }

            while (m_queuedOnMainThread.Count > 0)
                m_queuedOnMainThread.Dequeue().Invoke();
        }

        public void FlushRequests()
        {
            pathRequests.Clear();
        }

        public PositionGoalChecker<TimeBasedNavNode> GoalCheckerForPosition(Vector3 position) => new PositionGoalChecker<TimeBasedNavNode>(FindClosest(position).position);
        private NavNode FindClosest(Vector3 position)
        {
            if (navNodes.Length > 0)
            {
                NavNode best = navNodes[0];
                float bestDistance = Vector3.Distance(best.position, position);
                for (int i = 0; i < navNodes.Length; i++)
                {
                    float distance = Vector3.Distance(navNodes[i].position, position);
                    if (distance < bestDistance)
                    {
                        best = navNodes[i];
                        bestDistance = distance;
                    }
                }
                return best;
            }
            else
            {
                Debug.LogError("Error: No NavNodes detected.");
                return null;
            }
        }

        Color[] colors = new Color[] { Color.green, Color.red, Color.blue, Color.yellow, Color.magenta };
        List<TimeBasedNavNode>[] debug_explorationsToShow;
        public void Debug_ShowExplorations(List<TimeBasedNavNode>[] explorations)
        {
            debug_explorationsToShow = explorations;
        }
        public void OnDrawGizmos()
        {
            if (debug_showExplorations && debug_explorationsToShow != null)
            {
                int i = debug_showExplorationIndex;
                if (i < 0 || i >= debug_explorationsToShow.Length) return;
                Dictionary<Vector3, int> m_positionCounters = new Dictionary<Vector3, int>();
                Gizmos.color = colors[i % colors.Length];
                for (int j = 0; j < Mathf.Min(debug_showExplorationUntil, debug_explorationsToShow[i].Count); j++)
                {
                    var e = debug_explorationsToShow[i][j];
                    if (m_positionCounters.ContainsKey(e.Position))
                    {
                        m_positionCounters[e.Position] += 1;
                    }
                    else
                    {
                        m_positionCounters[e.Position] = 1;
                    }
                }
                foreach (var kvp in m_positionCounters)
                    Gizmos.DrawCube(kvp.Key + Vector3.up * kvp.Value / 2f, new Vector3(0.5f, kvp.Value, 0.5f));
            }
        }

        [Sirenix.OdinInspector.Button]
        public void PrintExplorationLog(int index)
        {
            if(debug_explorationsToShow != null && index >= 0 && index <= debug_explorationsToShow.Length)
            {
                for (int i = 0; i < debug_explorationsToShow[index].Count; i++)
                    Debug.Log(debug_explorationsToShow[index][i]);
            }
        }

        /// <summary>
        /// Request a path from the Dispatcher.
        /// Specify the Agent, the additional Costs it incurs, and the Goal criteria.
        /// </summary>
        public void RequestPath(IHighLevelAgent agent, List<ICostEvaluator<TimeBasedNavNode>> costEvaluators, INodeGoalChecker<TimeBasedNavNode> goalCriteria)
        {
            agent.Path = null;
            registeredAgents.Add(agent);
            pathRequests.Add(new PathRequest { requestTime = Time.time, agent = agent, additionalCosts = costEvaluators, goalCriteria = goalCriteria, startPosition = agent.Position });

#if LOG_VERBOSE
            // Debug.Log("Requested path for Agent " + agent.GetHashCode());
#endif
        }

        /// <summary>
        /// Update all pending requests
        /// </summary>
        [Sirenix.OdinInspector.Button]
        public void UpdateRequests()
        {
            if (!m_pathfindingInProgress)
            {
                // Order by priority
                pathRequests = pathRequests.OrderBy(r => r.agent.Definition.priority).ToList();
                if (pathRequests.Count > 0)
                {
                    // Get all agents with the same priority
                    float priority = pathRequests[0].agent.Definition.priority;
                    HashSet<PathRequest> equalPriorityRequests = new HashSet<PathRequest>();
                    foreach (var pathRequest in pathRequests)
                        if (pathRequest.agent.Definition.priority == priority)
                            equalPriorityRequests.Add(pathRequest);

                    // Remove all requests being handled
                    foreach (var req in equalPriorityRequests)
                        pathRequests.Remove(req);

                    // Begin a CBS job (with prioritized planning constraints) for these agents
                    m_pathfindingInProgress = true;
                    System.Threading.Tasks.Task.Run(() => PathfindFor(equalPriorityRequests));

#if LOG_VERBOSE
                    // Debug.Log($"Began pathfinding request for {equalPriorityRequests.Count} entities of priority {priority}");
#endif
                }
            }
        }

        // High Level Summary:
        // 1. Performs CBS on all provided requests of the same Priority Level
        // 2. For all existing paths, if they collide with the new paths, Invalidate them.
        //    This will achieve one of two effects:
        //        2a) If this agent is of equal priority to the requests, it will be bundled into the next CBS pass
        //        2b) If this agent is of lower priority to the requests, it will be computed after all pending requests of this priority are addressed
        private void PathfindFor(HashSet<PathRequest> requestSet)
        {
            try
            {
                List<PathRequest> requests = new List<PathRequest>(requestSet);
                float prio0 = requests[0].agent.Definition.priority;

                // Get constraints for all relevant paths of lower priority that must be avoided
                List<INodeConstraint<TimeBasedNavNode>> constraintSet = new List<INodeConstraint<TimeBasedNavNode>>();
                foreach (var agent in registeredAgents)
                    if (agent.Definition.priority < prio0 && agent.Path != null)
                        constraintSet.Add(new PathIntersectionConstraint<TimeBasedNavNode>(agent.Path));

                // Create Low-Level solvers
                BestFirstSolver<TimeBasedNavNode>[] solvers = new BestFirstSolver<TimeBasedNavNode>[requests.Count];
                AgentDefinition[] agentDefs = new AgentDefinition[requests.Count];
                for (int i = 0; i < solvers.Length; i++)
                {
                    // Create Cost and Heuristic Evaluators, constraint set, and pruning policy
                    List<ICostEvaluator<TimeBasedNavNode>> costs = SolverConstructors.BaseCosts().ExtendCosts(requests[i].additionalCosts);
                    List<IHeuristicEvaluator<TimeBasedNavNode>> heuristics = SolverConstructors.EmptyHeuristics();
                    if (requests[i].goalCriteria is PositionGoalChecker<TimeBasedNavNode>)
                    {
                        var targetPos = (requests[i].goalCriteria as PositionGoalChecker<TimeBasedNavNode>).Position;
                        heuristics.Add(new TimeHeuristicEvaluator<TimeBasedNavNode>(requests[i].agent.Definition.speed, targetPos));
                    }
                    List<INodeConstraint<TimeBasedNavNode>> constraints = constraintSet;
                    PermittedRevisitSetPruningPolicy<TimeBasedNavNode> pruningPolicy = new PermittedRevisitSetPruningPolicy<TimeBasedNavNode>() { analysisRange = hyperParam_conflictReanalysisRange };

                    // Construct agent and assign to higher-level solver
                    var agentSolver = new BestFirstSolver<TimeBasedNavNode>(
                        costs,
                        heuristics,
                        constraints,
                        new(requests[i].agent.Definition, FindClosest(requests[i].startPosition), requests[i].requestTime),
                        new() { requests[i].goalCriteria },
                        pruningPolicy
                    );
                    agentSolver.heuristicModifier = hyperParam_heuristicWeight;
                    solvers[i] = agentSolver;
                    agentDefs[i] = requests[i].agent.Definition;

                    // Add existing paths as permitted revisits
                    foreach (var agent in registeredAgents)
                    {
                        if (agent.Definition.priority < prio0 && agent.Path != null)
                        {
                            foreach (var pos in agent.Path.Points)
                            {
                                pruningPolicy.permittedRevisits.Add(new Vector3(pos.x, 0, pos.y));
                            }
                        }
                    }
                }

                // Create High-Level solver
                BestFirstSolver<ConflictBasedSearchNode> solver = new BestFirstSolver<ConflictBasedSearchNode>(
                    costEvaluators: new() { new CBSSumOfCostsEvaluator() },
                    heuristicEvaluators: new() { },
                    constraints: new() { new CBSSkipIfInvalidConstraint() },
                    start: new(solvers, agentDefs, maxNavigationTimeLowLevelMs),
                    goalChecker: new() { new LambdaGoalChecker<ConflictBasedSearchNode>((n) => n.IsValid && n.Conflicts.Count == 0) }
                );

                // Execute solver
                solver.Begin();
                solver.Execute(maxNavigationTimeMs);

                // On success, update all agents relevant to the request
                if (solver.Status == PathProviderStatus.Success)
                {
                    var resultNode = solver.ResultPath[solver.ResultPath.Count - 1];

                    Debug.Log($"[Success] " +
                        $"Total Duration: {solver.Elapsed} " +
                        $"Total HL Expansions: {solver.exploredNodes.Count} " +
                        $"Final LL Durations: {UniMAPFPathfindingUtility.ListToString(resultNode.LowLevelSolvers.Select(x => x.Elapsed).ToList())} " +
                        $"Final LL Expansions: {UniMAPFPathfindingUtility.ListToString(resultNode.LowLevelSolvers.Select(x => (x as BestFirstSolver<TimeBasedNavNode>).exploredNodes.Count).ToList())} " +
                        $"# Constraints: {resultNode.CurrentConstraintsPerAgent.Sum(x => x.Count)} " +
                        $"Cost: {resultNode.SumOfCosts}");

                    m_queuedOnMainThread.Enqueue(() =>
                    {
                        bool wasValid = true;
                        for (int i = 0; i < resultNode.SegmentedPaths.Length; i++)
                        {
                            // Invalidate all affected agents of equal or lower priority
                            foreach (var agent in registeredAgents)
                            {
                                if (agent.Path != null)
                                {
                                    if (agent.Definition.priority >= requests[i].agent.Definition.priority)
                                    {
                                        if (UniMAPFPathfindingUtility.ComputeSoonestCollision(agent.Path, requests[i].agent.Path) != float.NaN)
                                        {
                                            // Invalidate the other agent's path
                                            agent.Path = null;
                                            agent.OnPathInvalidated();

                                            // If we conflicted with an agent of equal priority, we must replan with all pending under consideration
                                            if (agent.Definition.priority == requests[i].agent.Definition.priority) wasValid = false;
                                        }
                                    }
                                }
                            }
                        }
                        for (int i = 0; i < resultNode.SegmentedPaths.Length; i++)
                        {
                            // If none of the new paths had any collisions with pre-existing equal priority paths, assign them
                            if (wasValid)
                            {
                                requests[i].agent.Path = resultNode.SegmentedPaths[i];
                                requests[i].agent.OnNewPathComputed();
                            }
                            // Otherwise, invalidate them all forcing a re-request
                            else
                            {
                                requests[i].agent.Path = null;
                                requests[i].agent.OnPathInvalidated();
                            }
                        }
                    });

                    m_queuedOnMainThread.Enqueue(() => Debug_ShowExplorations(resultNode.LowLevelSolvers.Select(x => x.exploredNodes).ToArray()));
                }
                // On failure, invalidate all agents forcing a re-request
                else m_queuedOnMainThread.Enqueue(() =>
                {
                    Debug.Log("Failed to find CBS solution - status: " + solver.Status + ", reason: " + solver.FailureReason);

                    foreach (var request in requests)
                    {
                        request.agent.OnPathInvalidated();
                    }
                });
            }
            catch(Exception e)
            {
                Debug.LogError(e);
            }

            // Untoggle pathfindingInProgress flag
            m_pathfindingInProgress = false;
        }
    }
}