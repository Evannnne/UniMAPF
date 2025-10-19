using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Tester_ConflictBasedSearch : MonoBehaviour
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
        // Construct the solver for B
        TimeBasedNavNode startA = new TimeBasedNavNode(agentA, agentAStart, 0);
        TimeBasedNavNode endA = new TimeBasedNavNode(agentA, agentAEnd, -1);
        BestFirstSolver<TimeBasedNavNode> solverA = new BestFirstSolver<TimeBasedNavNode>(
            new List<ICostEvaluator<TimeBasedNavNode>> { new TimeCostEvaluator<TimeBasedNavNode>() },
            new List<IHeuristicEvaluator<TimeBasedNavNode>> { new TimeHeuristicEvaluator<TimeBasedNavNode>(agentA.speed, endA.Position) },
            new List<INodeConstraint<TimeBasedNavNode>> { },
            startA,
            new List<INodeGoalChecker<TimeBasedNavNode>>() { new LambdaGoalChecker<TimeBasedNavNode>((node) => node.root == endA.root) }
        );

        // Construct the solver for B
        TimeBasedNavNode startB = new TimeBasedNavNode(agentB, agentBStart, 0);
        TimeBasedNavNode endB = new TimeBasedNavNode(agentB, agentBEnd, -1);
        BestFirstSolver<TimeBasedNavNode> solverB = new BestFirstSolver<TimeBasedNavNode>(
            new List<ICostEvaluator<TimeBasedNavNode>> { new TimeCostEvaluator<TimeBasedNavNode>() },
            new List<IHeuristicEvaluator<TimeBasedNavNode>> { new TimeHeuristicEvaluator<TimeBasedNavNode>(agentB.speed, endB.Position) },
            new List<INodeConstraint<TimeBasedNavNode>> { },
            startB,
            new List<INodeGoalChecker<TimeBasedNavNode>>() { new LambdaGoalChecker<TimeBasedNavNode>((node) => node.root == endB.root) }
        );

        // Construct the meta-solver
        BestFirstSolver<ConflictBasedSearchNode> cbsSolver = new BestFirstSolver<ConflictBasedSearchNode>(
            new List<ICostEvaluator<ConflictBasedSearchNode>> { new CBSSumOfCostsEvaluator() },
            new List<IHeuristicEvaluator<ConflictBasedSearchNode>> { },
            new List<INodeConstraint<ConflictBasedSearchNode>> { new CBSSkipIfInvalidConstraint() },
            new ConflictBasedSearchNode(new BestFirstSolver<TimeBasedNavNode>[] { solverA, solverB }, new AgentDefinition[] { agentA, agentB }),
            new List<INodeGoalChecker<ConflictBasedSearchNode>>() { new LambdaGoalChecker<ConflictBasedSearchNode>((n) => n.IsValid && n.Conflicts.Count == 0) }
        );

        // Run the meta solver
        cbsSolver.Begin();
        cbsSolver.Execute(float.MaxValue);
        if (cbsSolver.ResultPath != null)
        {
            ConflictBasedSearchNode last = cbsSolver.ResultPath.Last();
            Debug.Log("A path: " +  UniMAPFPathfindingUtility.ListToString(last.Paths[0]));
            Debug.Log("B path: " + UniMAPFPathfindingUtility.ListToString(last.Paths[1]));
        }
        else Debug.LogError("Failed to find CBS solution.");
    }
}