using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class Tester_SegmentPathSampling : MonoBehaviour
{
    public Vector2[] points;
    public float speed = 1.0f;
    public float sampleTime = 1.0f;

    private Vector3 _Swiz(Vector2 input) => transform.position + new Vector3(input.x, 0, input.y);
    public void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.gray;
        SegmentedPath path = new SegmentedPath(1f, points, 0f, speed);
        for(int i = 0; i < path.Points.Count - 1; i++)
        {
            Gizmos.DrawSphere(_Swiz(path.Points[i]), 0.5f);
            Gizmos.DrawSphere(_Swiz(path.Points[i+1]), 0.5f);
            Gizmos.DrawLine(_Swiz(path.Points[i]), _Swiz(path.Points[i + 1]));
        }
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(_Swiz(path.GetPositionAtTime(sampleTime)), 0.75f);
    }
}
