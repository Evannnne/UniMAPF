// #define SUPER_VERBOSE_LOGGING
using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class EdgeConflict
{
    public int agentA;
    public int agentB;
    
    public Vector3 agentAStart;
    public Vector3 agentAEnd;
    public float agentAStartTime;
    public float agentAEndTime;

    public Vector3 agentBStart;
    public Vector3 agentBEnd;
    public float agentBStartTime;
    public float agentBEndTime;

    public float collisionTime;
}

public class ConflictBasedSearchNode : IGraphNode
{
    public bool IsValid { get; private set; } = true;
    public BestFirstSolver<TimeBasedNavNode>[] LowLevelSolvers { get; private set; }
    public AgentDefinition[] AgentDefinitions { get; private set; }
    public CBSConstraintSet[] InjectedConstraints { get; private set; }
    public TimeGoalChecker<TimeBasedNavNode>[] InjectedGoalCriteria { get; private set; }

    public List<TimeBasedNavNode>[] Paths;
    public SegmentedPath[] SegmentedPaths;
    public List<EdgeConflict> Conflicts;
    public float SumOfCosts;
    public List<ITimedNodeConstraint<TimeBasedNavNode>>[] CurrentConstraintsPerAgent;
    public float[] ComputationTimes;

    public float maxSolveTime;

#if SUPER_VERBOSE_LOGGING
    private static int s_currentNodeID = 0;
    private int m_nodeID;
#endif

    public ConflictBasedSearchNode(BestFirstSolver<TimeBasedNavNode>[] lowLevelSolvers, AgentDefinition[] agentDefinitions, float maxLowLevelSolveTime = float.MaxValue)
    {
#if SUPER_VERBOSE_LOGGING
        s_currentNodeID = 0;
        m_nodeID = s_currentNodeID;
        s_currentNodeID++;
        Debug.Log($"Creating root node (id={m_nodeID})");
#endif

        // Initialize for start node
        LowLevelSolvers = lowLevelSolvers;
        AgentDefinitions = agentDefinitions;
        Paths = new List<TimeBasedNavNode>[AgentDefinitions.Length];
        SegmentedPaths = new SegmentedPath[AgentDefinitions.Length];
        InjectedConstraints = new CBSConstraintSet[AgentDefinitions.Length];
        InjectedGoalCriteria = new TimeGoalChecker<TimeBasedNavNode>[AgentDefinitions.Length];
        CurrentConstraintsPerAgent = new List<ITimedNodeConstraint<TimeBasedNavNode>>[AgentDefinitions.Length];
        ComputationTimes = new float[AgentDefinitions.Length];

        maxSolveTime = maxLowLevelSolveTime;

        // Compute all paths
        for (int i = 0; i < LowLevelSolvers.Length; i++)
        {
            // Find all single agent paths
            LowLevelSolvers[i].Begin();
            LowLevelSolvers[i].Execute(maxSolveTime);
            if (LowLevelSolvers[i].Status == PathProviderStatus.Failure || LowLevelSolvers[i].Status == PathProviderStatus.InProgress) 
            {
                IsValid = false; 
                return;
            }
            UpdatePath(i);

            // Inject a set of CBS constraints into the solver
            CurrentConstraintsPerAgent[i] = new List<ITimedNodeConstraint<TimeBasedNavNode>>();
            InjectedConstraints[i] = new CBSConstraintSet();
            InjectedGoalCriteria[i] = new TimeGoalChecker<TimeBasedNavNode>();
            LowLevelSolvers[i].AddConstraint(InjectedConstraints[i]);
            LowLevelSolvers[i].AddGoalCondition(InjectedGoalCriteria[i]);

        }

        // Get conflicts and sum of costs
        ComputeConflicts();
        ComputeCosts();
        LogDetails();
    }
    public ConflictBasedSearchNode(ConflictBasedSearchNode prior, EdgeConflict newConflict, bool splitLeft)
    {
#if SUPER_VERBOSE_LOGGING
        m_nodeID = s_currentNodeID;
        s_currentNodeID++;
        Debug.Log($"Creating new node (id={m_nodeID}, previous={prior.m_nodeID})");
#endif

        // If initialized from a prior node,
        // Copy over everything from it
        LowLevelSolvers = prior.LowLevelSolvers;
        AgentDefinitions = prior.AgentDefinitions;
        InjectedConstraints = prior.InjectedConstraints;
        InjectedGoalCriteria = prior.InjectedGoalCriteria;
        ComputationTimes = new float[LowLevelSolvers.Length];
        
        maxSolveTime = prior.maxSolveTime;

        Paths = new List<TimeBasedNavNode>[prior.AgentDefinitions.Length];
        SegmentedPaths = new SegmentedPath[prior.AgentDefinitions.Length];
        CurrentConstraintsPerAgent = new List<ITimedNodeConstraint<TimeBasedNavNode>>[prior.AgentDefinitions.Length];

        for (int i = 0; i < prior.SegmentedPaths.Length; i++)
        {
            Paths[i] = prior.Paths[i];
            SegmentedPaths[i] = prior.SegmentedPaths[i];
            CurrentConstraintsPerAgent[i] = new List<ITimedNodeConstraint<TimeBasedNavNode>>(prior.CurrentConstraintsPerAgent[i]);
            ComputationTimes[i] = prior.ComputationTimes[i];
        }

        // Now, either add a new conflict to the left or right agent in the EdgeConflict depending on 'splitLeft'
        int agentIndex = -1;
        if (splitLeft)
        {
            agentIndex = newConflict.agentA;

            if (newConflict.agentAEndTime == float.MaxValue)
                CurrentConstraintsPerAgent[agentIndex].Add(new CannotBeAtPositionDuringTimeConstraint<TimeBasedNavNode>(newConflict.agentAStart, newConflict.agentAStartTime, 0.01f));
            else CurrentConstraintsPerAgent[agentIndex].Add(new CannotBeAtPositionDuringTimeConstraint<TimeBasedNavNode>(newConflict.agentAEnd, newConflict.agentAEndTime, 0.01f));
        }
        else
        {
            agentIndex = newConflict.agentB;

            if (newConflict.agentBEndTime == float.MaxValue)
                CurrentConstraintsPerAgent[agentIndex].Add(new CannotBeAtPositionDuringTimeConstraint<TimeBasedNavNode>(newConflict.agentBStart, newConflict.agentBStartTime, 0.01f));
            else CurrentConstraintsPerAgent[agentIndex].Add(new CannotBeAtPositionDuringTimeConstraint<TimeBasedNavNode>(newConflict.agentBEnd, newConflict.agentBEndTime, 0.01f));
        }

        if(LowLevelSolvers[newConflict.agentA].pruningPolicy is PermittedRevisitSetPruningPolicy<TimeBasedNavNode>)
        {
            var pp = LowLevelSolvers[newConflict.agentA].pruningPolicy as PermittedRevisitSetPruningPolicy<TimeBasedNavNode>;
            pp.permittedRevisits.Add(newConflict.agentAStart);
            pp.permittedRevisits.Add(newConflict.agentAEnd);
            pp.permittedRevisits.Add(newConflict.agentBStart);
            pp.permittedRevisits.Add(newConflict.agentBEnd);
        }
        if (LowLevelSolvers[newConflict.agentB].pruningPolicy is PermittedRevisitSetPruningPolicy<TimeBasedNavNode>)
        {
            var pp = LowLevelSolvers[newConflict.agentB].pruningPolicy as PermittedRevisitSetPruningPolicy<TimeBasedNavNode>;
            pp.permittedRevisits.Add(newConflict.agentAStart);
            pp.permittedRevisits.Add(newConflict.agentAEnd);
            pp.permittedRevisits.Add(newConflict.agentBStart);
            pp.permittedRevisits.Add(newConflict.agentBEnd);
        }

        InjectedConstraints[agentIndex].currentConstraints = CurrentConstraintsPerAgent[agentIndex];

        float maxConstraintTime = -1;
        for (int i = 0; i < CurrentConstraintsPerAgent[agentIndex].Count; i++)
            maxConstraintTime = Mathf.Max(maxConstraintTime, CurrentConstraintsPerAgent[agentIndex][i].ConstraintActiveUntil);
        InjectedGoalCriteria[agentIndex].soonestEndTime = maxConstraintTime;

        if (!SolveForAgent(agentIndex)) return;
        UpdatePath(agentIndex);

        // Get conflicts and sum of costs
        ComputeConflicts();
        ComputeCosts();
        LogDetails();
    }

    private bool SolveForAgent(int agentIndex)
    {
        LowLevelSolvers[agentIndex].Begin();
        LowLevelSolvers[agentIndex].Execute(maxSolveTime);
        if (LowLevelSolvers[agentIndex].Status == PathProviderStatus.Failure || LowLevelSolvers[agentIndex].Status == PathProviderStatus.InProgress)
        {
            IsValid = false;
            return false;
        }
        else return true;
    }

    private void LogDetails()
    {
#if SUPER_VERBOSE_LOGGING
        Debug.Log("     Current node has constraints: ");
        for (int i = 0; i < CurrentConstraintsPerAgent.Length; i++)
            for (int j = 0; j < CurrentConstraintsPerAgent[i].Count; j++)
                Debug.Log($"        [Agent {i}]: {CurrentConstraintsPerAgent[i][j]}");

        Debug.Log("     Current node has paths:");
        foreach (var agentPath in Paths)
        {
            Debug.Log($"        {UniMAPFPathfindingUtility.ListToString(agentPath)}");
        }

        Debug.Log("     Current node has conflicts:");
        foreach (var conflict in Conflicts)
        {
            Debug.Log($"        " +
                $"[Agent {conflict.agentA}]: action_start={conflict.agentAStartTime}, action_end={conflict.agentAEndTime} {conflict.agentAStart}->{conflict.agentAEnd}; " +
                $"[Agent {conflict.agentB}]: action_start={conflict.agentBStartTime}, action_end={conflict.agentBEndTime} {conflict.agentBStart}->{conflict.agentBEnd}; " +
                $"time={conflict.collisionTime}");
        }
        Debug.Log($"     Current node has cost: {SumOfCosts}");
#endif
    }

    private void UpdatePath(int index)
    {
        // Get paths
        Paths[index] = LowLevelSolvers[index].ResultPath;
        SegmentedPaths[index] = new SegmentedPath(AgentDefinitions[index].radius, Paths[index], true);
        ComputationTimes[index] = LowLevelSolvers[index].Elapsed;
    }
    private void ComputeConflicts()
    {
        // Compute conflicts
        // Exploit symmetry to prevent duplicate checks (e.g. p[2] vs p[5] == p[5] vs p[2])
        Conflicts = new List<EdgeConflict>();
        for (int i = 0; i < Paths.Length; i++)
        {
            for (int j = 0; j < i; j++)
            {
                var soonest = UniMAPFPathfindingUtility.ComputeSoonestCollision(SegmentedPaths[i], SegmentedPaths[j]);
                if (!float.IsNaN(soonest))
                {
                    int offendingSegmentA = SegmentedPaths[i].GetSegmentIndexForTime(soonest);
                    int offendingSegmentB = SegmentedPaths[j].GetSegmentIndexForTime(soonest);

                    Conflicts.Add(new EdgeConflict
                    {
                        agentA = i,
                        agentAStart = _DeSwizzle(SegmentedPaths[i].Points[offendingSegmentA]),
                        agentAEnd = _DeSwizzle(SegmentedPaths[i].Points[offendingSegmentA + 1]),
                        agentAStartTime = SegmentedPaths[i].Times[offendingSegmentA],
                        agentAEndTime = SegmentedPaths[i].Times[offendingSegmentA + 1],

                        agentB = j,
                        agentBStart = _DeSwizzle(SegmentedPaths[j].Points[offendingSegmentB]),
                        agentBEnd = _DeSwizzle(SegmentedPaths[j].Points[offendingSegmentB + 1]),
                        agentBStartTime = SegmentedPaths[j].Times[offendingSegmentB],
                        agentBEndTime = SegmentedPaths[j].Times[offendingSegmentB + 1],

                        collisionTime = soonest
                    });
                }
            }
        }
    }
    /// <summary>
    /// De-swizzles a path. Important, because collisions are examined in 2D.
    /// </summary>
    private Vector3 _DeSwizzle(Vector2 input) => new Vector3(input.x, 0, input.y);

    private void ComputeCosts()
    {
        // Compute sum of costs
        SumOfCosts = 0;
        foreach (var path in SegmentedPaths)
            SumOfCosts += path.Times[path.Times.Count - 2] - path.Times[0];
    }

    public IEnumerable<IGraphNode> GetAdjacents(float costBound)
    {
        if (IsValid)
        {
            List<IGraphNode> result = new List<IGraphNode>();
            foreach (var conflict in Conflicts)
            {
                result.Add(new ConflictBasedSearchNode(this, conflict, true));
                result.Add(new ConflictBasedSearchNode(this, conflict, false));
            }
            return result;
        }
        else return new List<IGraphNode>();
    }
}
public class CannotTraverseEdgeAtTimeConstraint<NodeType> : ITimedNodeConstraint<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty
{
    public float ConstraintActiveUntil => time;

    public Vector3 p1;
    public Vector3 p2;
    public float time;

    public CannotTraverseEdgeAtTimeConstraint(Vector3 p1, Vector3 p2, float time)
    {
        this.p1 = p1;
        this.p2 = p2;
        this.time = time;
    }

    public bool IsConstrained(NodeType previous, NodeType next)
    {
        return previous.Position == p1 && next.Position == p2 && previous.Time == time;
    }

    public override string ToString()
    {
        return $"{time}: {p1}-/->{p2}";
    }
}
public class CannotBeAtPositionDuringIntervalConstraint<NodeType> : ITimedNodeConstraint<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty
{
    public float ConstraintActiveUntil => timeEnd;

    public Vector3 p1;
    public float timeStart;
    public float timeEnd;

    public CannotBeAtPositionDuringIntervalConstraint(Vector3 p1, float timeStart, float timeEnd)
    {
        this.p1 = p1;
        this.timeStart = timeStart;
        this.timeEnd = timeEnd;
    }

    public bool IsConstrained(NodeType previous, NodeType next)
    {
        return next.Position == p1 && timeStart <= next.Time && next.Time <= timeEnd;
    }

    public override string ToString()
    {
        return $"{timeStart}-{timeEnd}: !{p1}";
    }
}
public class CannotBeAtPositionDuringTimeConstraint<NodeType> : ITimedNodeConstraint<NodeType> where NodeType : IGraphNode, INodePositionProperty, INodeTimeProperty
{
    public float ConstraintActiveUntil => time;

    public Vector3 p1;
    public float time;
    public float window;

    public CannotBeAtPositionDuringTimeConstraint(Vector3 p1, float time, float window)
    {
        this.p1 = p1;
        this.time = time;
        this.window = window;
    }

    public bool IsConstrained(NodeType previous, NodeType next)
    {
        return next.Position.x == p1.x && next.Position.z == p1.z && Mathf.Abs(next.Time - time) <= window;
    }

    public override string ToString()
    {
        return $"{time}: !{p1}";
    }
}

public class CBSSkipIfInvalidConstraint : INodeConstraint<ConflictBasedSearchNode>
{
    public bool IsConstrained(ConflictBasedSearchNode previous, ConflictBasedSearchNode current)
    {
        return !current.IsValid;
    }
}
public class CBSSumOfCostsEvaluator : ICostEvaluator<ConflictBasedSearchNode>
{
    public float CalculateCost(float oldCost, ConflictBasedSearchNode previous, ConflictBasedSearchNode current)
    {
        return current.SumOfCosts;
    }
}
public class CBSConstraintSet : INodeConstraint<TimeBasedNavNode>
{
    public List<ITimedNodeConstraint<TimeBasedNavNode>> currentConstraints = new List<ITimedNodeConstraint<TimeBasedNavNode>>();
    public bool IsConstrained(TimeBasedNavNode previous, TimeBasedNavNode current)
    {
        foreach (var constraint in currentConstraints)
        {
            if (constraint.IsConstrained(previous, current)) return true;
        }
        return false;
    }

    public override string ToString()
    {
        string result = "[";
        for (int i = 0; i < currentConstraints.Count; i++)
            result += currentConstraints[i].ToString() + (i < currentConstraints.Count - 1 ? "," : "");
        result += "]";
        return result;
    }
}