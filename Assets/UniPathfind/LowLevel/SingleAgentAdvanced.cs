using UnityEngine;
using System.Collections.Generic;
using System;

namespace UniMAPF.Pathfinding
{
    public class FloydWarshallLookupTableHeuristic<NodeType> : IHeuristicEvaluator<NodeType> where NodeType : IGraphNode
    {
        private float[,] m_costs;
        private Dictionary<NodeType, int> m_indexer;

        private int m_goalNodeIndex = -1;
        private NodeType m_goalNode;
        public NodeType GoalNode {

            get => m_goalNode;
            set
            {
                m_goalNodeIndex = m_indexer[value];
                m_goalNode = value;
            } 
        }

        public float CalculateCost(NodeType next)
        {
            if (m_goalNodeIndex != -1)
            {
                int current = m_indexer[next];
                return m_costs[current, m_goalNodeIndex];
            }
            else throw new Exception("Goal node not set!");
        }

        public FloydWarshallLookupTableHeuristic(float[,] costs, Dictionary<NodeType, int> indexer)
        {
            m_costs = costs;
            m_indexer = indexer;
        }
    }
}