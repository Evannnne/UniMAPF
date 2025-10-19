using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class NavNode : MonoBehaviour, IGraphNode, INodePositionProperty
{
    public static List<NavNode> Nodes = new List<NavNode>();
    public List<NavNode> connections;
    public Vector3 position;
    public Color labelColor = Color.white;

    public void OnEnable()
    {
        Nodes.Add(this);
        position = transform.position;
    }
    public void OnDisable()
    {
        Nodes.Remove(this);
    }

    public void OnDrawGizmos()
    {
        Gizmos.color = labelColor;
        Gizmos.DrawSphere(transform.position, 0.25f);
        if(connections != null)
            foreach (var connection in connections)
                Debug.DrawLine(transform.position, connection.transform.position);
    }

    public IEnumerable<IGraphNode> GetAdjacents(float costBound)
    {
        return connections;
    }
    public Vector3 Position => position;

    public override string ToString()
    {
        return $"(pos={Position})";
    }
}
