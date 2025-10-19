using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Tester_PrioritizedPlanning : MonoBehaviour
{
    public NavNode agentAStart;
    public NavNode agentAEnd;
    public AgentDefinition agentA;

    public NavNode agentBStart;
    public NavNode agentBEnd;
    public AgentDefinition agentB;

    [Sirenix.OdinInspector.Button]
    public void Test()
    {
        // First, get a path for A
        TimeBasedNavNode startA = new TimeBasedNavNode(agentA, agentAStart, 0);
        TimeBasedNavNode endA = new TimeBasedNavNode(agentA, agentAEnd, -1);
        BestFirstSolver<TimeBasedNavNode> solverA = new BestFirstSolver<TimeBasedNavNode>(
            new List<ICostEvaluator<TimeBasedNavNode>> { new TimeCostEvaluator<TimeBasedNavNode>() },
            new List<IHeuristicEvaluator<TimeBasedNavNode>> { new TimeHeuristicEvaluator<TimeBasedNavNode>(agentA.speed, endA.Position) },
            new List<INodeConstraint<TimeBasedNavNode>> { },
            startA,
            new List<INodeGoalChecker<TimeBasedNavNode>> { new LambdaGoalChecker<TimeBasedNavNode>((node) => node.root == agentAEnd) }
        );
        solverA.Begin();
        solverA.Execute(1000f);

        if (solverA.ResultPath == null)
            Debug.LogError("Failed to find path for A");

        string resultA = "Path A: ";
        foreach (var p in solverA.ResultPath)
            resultA += p + ",";
        Debug.Log(resultA);

        // Next, get a path for B
        SegmentedPath pathA = new SegmentedPath(agentA.radius, solverA.ResultPath, extendToInfinity: true);
        TimeBasedNavNode startB = new TimeBasedNavNode(agentB, agentBStart, 0);
        TimeBasedNavNode endB = new TimeBasedNavNode(agentB, agentBEnd, 0);
        BestFirstSolver<TimeBasedNavNode> solverB = new BestFirstSolver<TimeBasedNavNode>(
            new List<ICostEvaluator<TimeBasedNavNode>> { new TimeCostEvaluator<TimeBasedNavNode>() },
            new List<IHeuristicEvaluator<TimeBasedNavNode>> { new TimeHeuristicEvaluator<TimeBasedNavNode>(agentB.speed, endB.Position) },
            new List<INodeConstraint<TimeBasedNavNode>> { new PathIntersectionConstraint<TimeBasedNavNode>(pathA) },
            startB,
            new List<INodeGoalChecker<TimeBasedNavNode>> { new LambdaGoalChecker<TimeBasedNavNode>((node) => node.root == agentBEnd && node.Time >= pathA.Times[pathA.Times.Count - 2]) }
        );
        solverB.Begin();
        solverB.Execute(1000f);
        if (solverB.ResultPath == null)
            Debug.LogError("Failed to find path for B");

        string resultB = "Path B: ";
        foreach (var p in solverB.ResultPath)
            resultB += p + ",";
        Debug.Log(resultB);
    }
}
