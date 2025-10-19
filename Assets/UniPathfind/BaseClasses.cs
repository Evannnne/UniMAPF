using System;
using System.Collections.Generic;
using UnityEngine;

namespace UniMAPF.Pathfinding
{
    /// <summary>
    /// An interface for a Graph Node.
    /// Needs to be able to provide its adjacents
    /// </summary>
    public interface IGraphNode
    {
        public IEnumerable<IGraphNode> GetAdjacents(float costBound);
    }

    /// <summary>
    /// A function that provides a cost for a Node
    /// </summary>
    public interface ICostEvaluator<NodeType>
        where NodeType : IGraphNode 
        
    {
        public float CalculateCost(float previousCost, NodeType previous, NodeType current);
    }

    public interface IHeuristicEvaluator<NodeType>
        where NodeType : IGraphNode
    {
        public float CalculateCost(NodeType node);
    }

    /// <summary>
    /// A function that determines whether or not a node is constrained
    /// </summary>
    public interface INodeConstraint<NodeType> where NodeType : IGraphNode
    {
        public bool IsConstrained(NodeType previous, NodeType current);
    }

    /// <summary>
    /// A timed constraint
    /// </summary>
    /// <typeparam name="NodeType"></typeparam>
    public interface ITimedNodeConstraint<NodeType> : INodeConstraint<NodeType> where NodeType : IGraphNode
    {
        public float ConstraintActiveUntil { get; }
    }

    /// <summary>
    /// A property for nodes having positions
    /// </summary>
    public interface INodePositionProperty
    {
        public Vector3 Position { get; }
    }

    /// <summary>
    /// A property for nodes having times
    /// </summary>
    public interface INodeTimeProperty
    {
        public float Time { get; }
    }

    /// <summary>
    /// A property for nodes having a number of waits
    /// </summary>
    public interface INodeWaitCountProperty
    {
        public int WaitCount { get; }
    }

    /// <summary>
    /// A property for nodes having a reference to agent definitions
    /// </summary>
    public interface INodeAgentProperty
    {
        public AgentDefinition Agent { get; }
    }

    /// <summary>
    /// A function that determines if something should be pruned
    /// </summary>
    /// <typeparam name="NodeType"></typeparam>
    public interface IPruningPolicy<NodeType> where NodeType : IGraphNode
    {
        public bool ShouldPrune(NodeType node, float cost);
        public void Reset();
    }

    /// <summary>
    /// A property for checking if the node is a goal
    /// </summary>
    public interface INodeGoalChecker<NodeType> where NodeType : IGraphNode
    {
        public bool IsSuitableGoal(NodeType previous);
    } 

    /// <summary>
    /// The status of a Path Provider
    /// </summary>
    public enum PathProviderStatus
    {
        NotStarted = 0,
        InProgress = 1,
        Success = 2,
        SubOptimalSuccess = 3,
        Failure = 4
    }

    /// <summary>
    /// A Solver that can compute a path for an agent
    /// </summary>
    /// <typeparam name="NodeType"></typeparam>
    public abstract class Solver<NodeType>
        where NodeType : IGraphNode
    {
        protected NodeType m_start;
        protected NodeType m_goal;

        protected List<INodeGoalChecker<NodeType>> m_goalCriteria;
        protected List<INodeConstraint<NodeType>> m_constraints;

        public string FailureReason { get; protected set; }

        public float SpeedBias { get; set; }
        public PathProviderStatus Status { get; protected set; }
        public List<NodeType> ResultPath { get; protected set; }
        public float ResultCost { get; protected set; }
        public float Elapsed { get; protected set; }

        public abstract PathProviderStatus Execute(float permittedMilliseconds);

        public virtual void Begin()
        {

        }
        public Solver(List<INodeConstraint<NodeType>> constraints, NodeType start, List<INodeGoalChecker<NodeType>> goalCriteria) 
        {
            m_start = start;
            m_constraints = new(constraints);
            m_goalCriteria = new(goalCriteria);
        }

        public void AddConstraint(INodeConstraint<NodeType> constraint)
        {
            m_constraints.Add(constraint);
        }
        public void AddGoalCondition(INodeGoalChecker<NodeType> goalChecker)
        {
            m_goalCriteria.Add(goalChecker);
        }
    }
}