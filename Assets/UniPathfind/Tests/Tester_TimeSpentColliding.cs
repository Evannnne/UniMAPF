using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class Tester_TimeSpentColliding : MonoBehaviour
{
    public Vector2[] pointsA;
    public float speedA;
    public float startTimeA;
    public float radiusA = 0.5f;

    public Vector2[] pointsB;
    public float speedB;
    public float startTimeB;
    public float radiusB = 0.5f;

    [Sirenix.OdinInspector.ReadOnly] public float totalCollisionDuration;

    private Vector3 _Swiz(Vector2 input) => transform.position + new Vector3(input.x, 0, input.y);
    public void OnDrawGizmosSelected()
    {
        SegmentedPath pathA = new SegmentedPath(radiusA, pointsA, startTimeA, speedA);
        SegmentedPath pathB = new SegmentedPath(radiusB, pointsB, startTimeB, speedB);

        Gizmos.color = Color.green;
        for(int i = 0; i < pathA.Points.Count - 1; i++)
        {
            Gizmos.DrawSphere(_Swiz(pathA.Points[i]), radiusA * 0.9f);
            Gizmos.DrawSphere(_Swiz(pathA.Points[i+1]), radiusA * 0.9f);
            Gizmos.DrawLine(_Swiz(pathA.Points[i]), _Swiz(pathA.Points[i + 1]));
        }

        Gizmos.color = Color.blue;
        for (int i = 0; i < pathB.Points.Count - 1; i++)
        {
            Gizmos.DrawSphere(_Swiz(pathB.Points[i]), radiusB * 0.9f);
            Gizmos.DrawSphere(_Swiz(pathB.Points[i + 1]), radiusB * 0.9f);
            Gizmos.DrawLine(_Swiz(pathB.Points[i]), _Swiz(pathB.Points[i + 1]));
        }

        totalCollisionDuration = UniMAPFPathfindingUtility.ComputeTotalCollisionTime(pathA, pathB);
    }
}
