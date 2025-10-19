using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UniMAPF.Pathfinding;

public class SimpleTestAgent : MonoBehaviour, IHighLevelAgent
{
    public Color agentColor = Color.green;

    public Vector3 Position => transform.position;
    public SegmentedPath Path { get; set; }
    public AgentDefinition Definition => agentDefinition;

    public AgentDefinition agentDefinition;

    public GameObject targetPositionObject;

    [Sirenix.OdinInspector.ReadOnly] public float m_animationTime = -1;

    public void OnPathInvalidated() 
    {
        // Debug.Log("Path invalidated!");
    }
    public void OnNewPathComputed() 
    { 
        m_animationTime = 0;
    }

    public void Update()
    {
        if(m_animationTime != -1) m_animationTime += Time.deltaTime;
    }

    public void OnDrawGizmos()
    {
        Gizmos.color = agentColor;
        Gizmos.DrawSphere(transform.position, Definition.radius / 2f);
        Gizmos.color = Color.Lerp(agentColor, Color.white, 0.5f);
        if(targetPositionObject) Gizmos.DrawSphere(targetPositionObject.transform.position, Definition.radius / 2f);

        if(Path != null)
        {
            Gizmos.color = Color.Lerp(agentColor, Color.clear, 0.5f);
            for(int i = 1; i < Path.Points.Count; i++)
            {
                Gizmos.DrawSphere(_Deswiz(Path.Points[i - 1]), Definition.radius / 2f);
                Gizmos.DrawSphere(_Deswiz(Path.Points[i]), Definition.radius / 2f);
                Gizmos.DrawLine(_Deswiz(Path.Points[i]), _Deswiz(Path.Points[i - 1]));
            }

            Gizmos.color = agentColor;
            Gizmos.DrawSphere(_Deswiz(Path.GetPositionAtTime(Path.Times[0] + m_animationTime)), Definition.radius);
        }
    }

    private Vector3 _Deswiz(Vector2 input)
    {
        return new Vector3(input.x, 0, input.y);
    }

    [Sirenix.OdinInspector.Button]
    public void RequestPath()
    {
        NavigationDispatcher.Instance.RequestPath(this, new List<ICostEvaluator<TimeBasedNavNode>>(), NavigationDispatcher.Instance.GoalCheckerForPosition(targetPositionObject.transform.position));
    }
}
