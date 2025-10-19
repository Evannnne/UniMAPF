using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UniMAPF.Pathfinding {
    public static class SolverConstructors
    {
        public static BestFirstSolver<TimeBasedNavNode> CreatePrioritizedPlanner(AgentDefinition agent, NavNode start, ICostEvaluator<TimeBasedNavNode> costEvaluators, INodeGoalChecker<TimeBasedNavNode> goalCriteria)
        {
            return null; // @TEMP
            /*
            List<INodeConstraint<TimeBasedNavNode>> constraints = new List<INodeConstraint<TimeBasedNavNode>>();
            BestFirstSolver<TimeBasedNavNode> solverB = new BestFirstSolver<TimeBasedNavNode>(
                new List<ICostEvaluator<TimeBasedNavNode>> { new TimeCostEvaluator<TimeBasedNavNode>() },
                new List<ICostEvaluator<TimeBasedNavNode>> { new TimeHeuristicEvaluator<TimeBasedNavNode>(agentB.speed, endB) },
                new List<INodeConstraint<TimeBasedNavNode>> { new PathIntersectionConstraint<TimeBasedNavNode>(pathA) },
                new TimeBasedNavNode(agent, start, Time.time),
                new List<INodeGoalChecker<TimeBasedNavNode>> { goalCriteria }
            );
            */
        }

        public static List<ICostEvaluator<TimeBasedNavNode>> BaseCosts()
        {
            return new List<ICostEvaluator<TimeBasedNavNode>>() { new TimeCostEvaluator<TimeBasedNavNode>() };
        }
        public static List<ICostEvaluator<TimeBasedNavNode>> ExtendCosts(this List<ICostEvaluator<TimeBasedNavNode>> list, List<ICostEvaluator<TimeBasedNavNode>> extra)
        {
            list.AddRange(extra);
            return list;
        }
        public static List<IHeuristicEvaluator<TimeBasedNavNode>> BaseHeuristics(float speed, Vector3 goal)
        {
            return new List<IHeuristicEvaluator<TimeBasedNavNode>>() { new TimeHeuristicEvaluator<TimeBasedNavNode>(speed, goal) };
        }
        public static List<IHeuristicEvaluator<TimeBasedNavNode>> EmptyHeuristics() => new();
    }
}