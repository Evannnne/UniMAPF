using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UniMAPF.Pathfinding
{
    [ExecuteInEditMode]
    public class NavNodeBenchmarker : MonoBehaviour
    {
        public NavNode startNode;
        public NavNode endNode;
        public bool useFeature_precomputedTable;

        private NavNode[] nodes;
        private void Awake()
        {
            nodes = GetComponentsInChildren<NavNode>();
        }

        [Sirenix.OdinInspector.Button]
        public float Pathfind(bool showAnnotations = true)
        {
            NavNode node1 = startNode;
            NavNode node2 = endNode;

            if (node1 == null || node2 == null) Debug.LogError("Cannot pathfind with null navnodes!");

            List<ICostEvaluator<NavNode>> costs = new List<ICostEvaluator<NavNode>>();
            costs.Add(new EuclidianDistanceCostEvaluator<NavNode>());

            List<IHeuristicEvaluator<NavNode>> heuristics = new List<IHeuristicEvaluator<NavNode>>();
            heuristics.Add(new EuclidianDistanceHeuristicEvaluator<NavNode>(node2));
            if (useFeature_precomputedTable)
            {
                var floydWarshallLookup = UniMAPF.ComputeAPSP(nodes, costs.ToArray());
                floydWarshallLookup.GoalNode = node2;
                heuristics.Add(floydWarshallLookup);
            }
            BestFirstSolver<NavNode> solver = new BestFirstSolver<NavNode>(
                costs,
                heuristics,
                new List<INodeConstraint<NavNode>> { },
                node1, 
                new List<INodeGoalChecker<NavNode>>() { new LambdaGoalChecker<NavNode>(x => x == node2) }
            );
            solver.Begin();

            System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
            watch.Start();
            if (solver.Execute(100) == PathProviderStatus.Success)
            {
                float elapsed = ((float)watch.ElapsedTicks / System.TimeSpan.TicksPerMillisecond);
                if (showAnnotations)
                {
                    foreach(var searched in solver.exploredNodes)
                    {
                        Debug.DrawLine(searched.position, searched.position + Vector3.up, Color.yellow, 5f);
                    }

                    Debug.Log("Found path of length: " + solver.ResultPath.Count + " in time: " + elapsed.ToString("0.00") + "ms, expanding " + solver.exploredNodes.Count + " nodes.");
                    for (int i = 0; i < solver.ResultPath.Count - 1; i++)
                        Debug.DrawLine(solver.ResultPath[i].transform.position, solver.ResultPath[i + 1].transform.position, Color.green, 5f);
                }
                return elapsed;
            }
            else
            {
                if (showAnnotations) Debug.Log("Failed to find path.");
                return 0;
            }
        }
    }
}