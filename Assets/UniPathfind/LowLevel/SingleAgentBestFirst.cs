// #define SUPER_VERBOSE_SOLVING
using UniMAPF.Pathfinding;
using Priority_Queue;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.VisualScripting;
using UnityEngine;

public class EqualityPruningPolicy<NodeType> : IPruningPolicy<NodeType> where NodeType : IGraphNode
{
    protected Dictionary<NodeType, float> costSet = new Dictionary<NodeType, float>();
    public virtual void Reset()
    {
        costSet.Clear();
    }
    public virtual bool ShouldPrune(NodeType node, float cost)
    {
        if (!costSet.ContainsKey(node) || costSet[node] > cost)
        {
            costSet[node] = cost;
            return false;
        }
        return true;
    }
}
public class PermittedRevisitSetPruningPolicy<NodeType> : EqualityPruningPolicy<NodeType> where NodeType : IGraphNode, INodePositionProperty
{
    public HashSet<Vector3> permittedRevisits = new HashSet<Vector3>();
    public float analysisRange = 0;

    public override bool ShouldPrune(NodeType node, float cost)
    {
        if (base.ShouldPrune(node, cost))
        {
            if (analysisRange == 0) return !permittedRevisits.Contains(node.Position);
            else
            {
                foreach (var point in permittedRevisits)
                    if (Vector3.Distance(point, node.Position) < analysisRange)
                        return false;
                return true;
            }
        }
        else return false;
    }
}

/// <summary>
/// A Best First Solver that can have an arbitrary choice of next best node
/// </summary>
/// <typeparam name="GraphType"></typeparam>
/// <typeparam name="NodeType"></typeparam>
/// <typeparam name="EdgeType"></typeparam>
public class BestFirstSolver<NodeType> : Solver<NodeType>
        where NodeType : IGraphNode
{
    protected class QueueNode : FastPriorityQueueNode
    {
        public QueueNode previous;
        public NodeType content;

        public float Fitness => cost + heuristic;
        public float cost;
        public float heuristic;

        public int retraversals = 0;

        public QueueNode(QueueNode _previous, float _currentCost, float _currentHeuristic, NodeType _content, int _retraversals)
        {
            previous = _previous;
            content = _content;
            cost = _currentCost;
            heuristic = _currentHeuristic;
            retraversals = _retraversals;
        }
    }

    public float heuristicModifier = 1f;

    public List<NodeType> exploredNodes;
    protected FastPriorityQueue<QueueNode> frontier;

    public List<ICostEvaluator<NodeType>> costEvaluators;
    public List<IHeuristicEvaluator<NodeType>> heuristicEvaluators;
    public IPruningPolicy<NodeType> pruningPolicy; 

    protected Stopwatch totalDurationWatch;

    public override PathProviderStatus Execute(float permittedMilliseconds)
    {
        float threshold = permittedMilliseconds * TimeSpan.TicksPerMillisecond;
        System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
        watch.Start();

        if (Status != PathProviderStatus.InProgress) return Status;
        while (true)
        {
            if (watch.ElapsedTicks >= threshold) 
                return PathProviderStatus.InProgress;

            // Pop the next best node
            var current = GetNextBest();
            if (current != null)
            {
                // Add to the list of epxlored nodes
                exploredNodes.Add(current.content);

                /*
                var pp = current.content as INodePositionProperty;
                var tp = current.content as INodeTimeProperty;
                if(pp != null && tp != null) Debug.Log($"Expanded t={tp.Time}:{pp.Position}, cost={current.cost}, heuristic={current.heuristic}");
                */

                // If this is the goal, make a path
                bool isGoal = true;
                foreach (var criteria in m_goalCriteria)
                {
                    if (!criteria.IsSuitableGoal(current.content))
                    {
                        isGoal = false;
                        break;
                    }
                }
                if (isGoal)
                {
                    Elapsed = totalDurationWatch.ElapsedTicks / (float)TimeSpan.TicksPerMillisecond;
                    ResultPath = ConstructPath(current);
                    Status = PathProviderStatus.Success;
                    return Status;
                }

                // Expand each neighbor
                foreach(var successor in current.content.GetAdjacents(9999f))
                {
                    // Calculate cost
                    NodeType casted = (NodeType)successor;
                    QueueNode successorNode = new QueueNode(current, GetCostForNode(current.cost, current.content, casted), GetHeuristicForNode(current.content, casted) * heuristicModifier, casted, current.retraversals);

                    // Check constraints
                    int constraintViolationIndex = -1;
                    for(int i = 0; i < m_constraints.Count; i++)
                    {
                        var constraint = m_constraints[i];
                        if (constraint.IsConstrained(current.content, casted))
                        {
                            constraintViolationIndex = i;
                            break;
                        }
                    }
                    if (constraintViolationIndex != -1)
                    {
                        continue;
                    }

                    // If not yet explored, or a better path to a previously explored node
                    if(!pruningPolicy.ShouldPrune(casted, successorNode.cost)) 
                    {
                        frontier.Enqueue(successorNode, successorNode.Fitness);
                    }
                }
            }
            else break;
        }
        Status = PathProviderStatus.Failure;
        FailureReason = "Failed to find path from " + m_start;
        return Status;
    }

    public BestFirstSolver(
        List<ICostEvaluator<NodeType>> costEvaluators,
        List<IHeuristicEvaluator<NodeType>> heuristicEvaluators,
        List<INodeConstraint<NodeType>> constraints,
        NodeType start,
        List<INodeGoalChecker<NodeType>> goalChecker,
        IPruningPolicy<NodeType> pruningPolicy = default,
        int maxNodes = 100000
    ) : base(constraints, start, goalChecker)
    {
        exploredNodes = new List<NodeType>();
        frontier = new FastPriorityQueue<QueueNode>(maxNodes);
        if (pruningPolicy != null) this.pruningPolicy = pruningPolicy;
        else this.pruningPolicy = new EqualityPruningPolicy<NodeType>();

        this.costEvaluators = new(costEvaluators);
        this.heuristicEvaluators = new(heuristicEvaluators);
    }

    public override void Begin()
    {
        totalDurationWatch = new Stopwatch();
        totalDurationWatch.Start();
        exploredNodes.Clear();
        frontier.Clear();
        pruningPolicy.Reset();
        frontier.Enqueue(new QueueNode(null, 0, 0, m_start, 0), 0);
        Status = PathProviderStatus.InProgress;
    }

    /// <summary>
    /// Gets the next best node.
    /// May perform any additional computation necessary, including:
    ///  - secondary heuristic precomputation
    ///  - partial lookahead
    ///  - Etc.
    /// </summary>
    protected virtual QueueNode GetNextBest()
    {
        if (frontier.Count > 0)
            return frontier.Dequeue();
        else return null;
    }

    /// <summary>
    /// Get the cost for a node.
    /// </summary>
    /// <param name="previous"></param>
    /// <param name="current"></param>
    /// <returns></returns>
    protected virtual float GetCostForNode(float oldCost, NodeType previous, NodeType current)
    {
        float cost = 0;
        foreach (var evaluator in costEvaluators) 
            cost += evaluator.CalculateCost(oldCost, previous, current);
        return cost;
    }

    /// <summary>
    /// Get the heuristic for a node
    /// </summary>
    /// <param name="previous"></param>
    /// <param name="current"></param>
    /// <returns></returns>
    protected virtual float GetHeuristicForNode(NodeType previous, NodeType current)
    {
        float cost = 0;
        foreach (var evaluator in heuristicEvaluators)
            cost = Mathf.Max(cost, evaluator.CalculateCost(current));
        return cost;
    }

    /// <summary>
    /// Construct a path from a final node
    /// </summary>
    /// <returns>The resulting path</returns>
    private List<NodeType> ConstructPath(QueueNode finalNode)
    {
        List<NodeType> path = new List<NodeType>();
        void _AddToPath(QueueNode node)
        {
            if (node.previous != null) _AddToPath(node.previous);
            path.Add(node.content);
        }
        _AddToPath(finalNode);
        ResultCost = finalNode.cost;
        return path;
    }
}

/// <summary>
/// A euclidian distance cost evaluator for heuristics
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class EuclidianDistanceCostEvaluator<NodeType> : ICostEvaluator<NodeType> where NodeType : IGraphNode, INodePositionProperty
{
    public float CalculateCost(float oldCost, NodeType previous, NodeType current)
    {
        return oldCost + Vector3.Distance(previous.Position, current.Position);
    }
}

/// <summary>
/// A euclidian distance heuristic evaluator for heuristics
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class EuclidianDistanceHeuristicEvaluator<NodeType> : IHeuristicEvaluator<NodeType> where NodeType : IGraphNode, INodePositionProperty
{
    private NodeType m_goal;

    public EuclidianDistanceHeuristicEvaluator(NodeType goal)
    {
        m_goal = goal;
    }

    public float CalculateCost(NodeType current)
    {
        return Vector3.Distance(current.Position, m_goal.Position);
    }
}

/// <summary>
/// A time cost evaluator
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class TimeCostEvaluator<NodeType> : ICostEvaluator<NodeType> where NodeType : IGraphNode, INodeTimeProperty
{
    public float CalculateCost(float oldCost, NodeType previous, NodeType current)
    {
        /*
        if(previous is INodePositionProperty)
        {
            var prevPos = (previous as INodePositionProperty).Position;
            var nextPos = (current as INodePositionProperty).Position;
            Debug.Log("OldPos: " + prevPos + ", newPos" + nextPos + ", oldTime: " + previous.Time + ", newTime: " + current.Time);
        }
        */
        return oldCost + current.Time - previous.Time;
    }
}

/// <summary>
/// A time cost heuristic
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class TimeHeuristicEvaluator<NodeType> : IHeuristicEvaluator<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty
{
    public float AgentSpeed { get; private set; }
    public Vector3 Goal { get; private set; }
    public TimeHeuristicEvaluator(float agentSpeed, Vector3 goal)
    {
        AgentSpeed = agentSpeed;
        Goal = goal;
    }
    public float CalculateCost(NodeType current)
    {
        return Vector3.Distance(current.Position, Goal) / AgentSpeed;
    }
}

/// <summary>
/// A wait count heuristic
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class WaitCountHeuristicEvaluator<NodeType> : IHeuristicEvaluator<NodeType> where NodeType : IGraphNode, INodeWaitCountProperty
{
    public float baseFactor = 1f;
    public float power = 1f;
    public float CalculateCost(NodeType current)
    {
        return baseFactor * Mathf.Pow(current.WaitCount, power);
    }
}

public class HeuristicMultiplier<NodeType> : IHeuristicEvaluator<NodeType> where NodeType : IGraphNode
{
    IHeuristicEvaluator<NodeType> m_baseEvaluator;
    float m_multiplier;

    public HeuristicMultiplier(IHeuristicEvaluator<NodeType> baseEvaluator, float multiplier)
    {
        m_baseEvaluator = baseEvaluator;
        m_multiplier = multiplier;
    }
    public float CalculateCost(NodeType current)
    {
        return m_baseEvaluator.CalculateCost(current) * m_multiplier;
    }
}

/// <summary>
/// A constraint checker that checks collisions with an existing path
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class PathIntersectionConstraint<NodeType> : INodeConstraint<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty, INodeAgentProperty
{
    public SegmentedPath PathToCheck { get; private set; }
    public PathIntersectionConstraint(SegmentedPath pathToCheck)
    {
        PathToCheck = pathToCheck;
    }

    private Vector2 _Swiz(Vector3 input) => new Vector2(input.x, input.z);

    public bool IsConstrained(NodeType previous, NodeType current)
    {
        Vector3 p0 = _Swiz(previous.Position);
        Vector3 p1 = _Swiz(current.Position);
        float speed = previous.Agent.speed;
        float radius = previous.Agent.radius;
        float collisionTime = UniMAPFPathfindingUtility.ComputeSoonestCollision(PathToCheck, p0, p1, radius, speed, previous.Time);

        if (!float.IsNaN(collisionTime))
        {
#if SUPER_VERBOSE_SOLVING
            Debug.Log($"Cannot move from {p0} to {p1} starting at {previous.Time}! (collision at {collisionTime})");
#endif
            return true;
        }
        else return false;
    }
}

public class PositionGoalChecker<NodeType> : INodeGoalChecker<NodeType> where NodeType : IGraphNode, INodePositionProperty
{
    public Vector3 Position { get; private set; }
    public float ToleratedDistance { get; private set; }
    public PositionGoalChecker(Vector3 position, float toleratedDistance = 0)
    {
        Position = position;
        ToleratedDistance = toleratedDistance;
    }
    public bool IsSuitableGoal(NodeType goal)
    {
        return Vector3.Distance(Position, goal.Position) <= ToleratedDistance;
    }
}
public class LambdaGoalChecker<NodeType> : INodeGoalChecker<NodeType> where NodeType : IGraphNode
{
    private Func<NodeType, bool> m_lambda;
    public LambdaGoalChecker(Func<NodeType, bool> lambda) { m_lambda = lambda; }
    public bool IsSuitableGoal(NodeType goal) => m_lambda(goal);
}
public class TimeGoalChecker<NodeType> : INodeGoalChecker<NodeType> where NodeType : IGraphNode, INodeTimeProperty
{
    public float soonestEndTime;
    public bool IsSuitableGoal(NodeType node)
    {
        return node.Time >= soonestEndTime;
    }
}

/*
/// <summary>
/// A constraint checker that doesn't let a path finish early if constraints have not been evaluated
/// </summary>
/// <typeparam name="NodeType"></typeparam>
public class DoNotFinishEarlyConstraint<NodeType> : INodeConstraint<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty, INodeAgentProperty
{
    public float LastTime { get; private set; }
    public DoNotFinishEarlyConstraint(float lastTime)
    {
        LastTime = lastTime;
    }
    public bool IsConstrained(NodeType previous, NodeType current)
    {
        bool isConstrained = current.Time < LastTime;
        if (isConstrained)
        {
            Debug.Log("Cannot finish early! Tried to end at " + current.Time + ", which is before " + LastTime);
            return true;
        }
        else return false;
    }
}
*/