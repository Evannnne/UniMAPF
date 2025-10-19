using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class Tester_CollisionMath : MonoBehaviour
{
    public Vector2 position1;
    public Vector2 velocity1;
    public float radius1;

    public Vector2 position2;
    public Vector2 velocity2;
    public float radius2;

    public float computedCollisionTime;

    public void Update()
    {
        if (velocity1 != Vector2.zero && velocity2 != Vector2.zero)
        {
            computedCollisionTime = UniMAPFPathfindingUtility.GetCollisionInterval2D(position1, velocity1, radius1, position2, velocity2, radius2).x;
        }
    }

    private Vector3 _Swiz(Vector2 input) => transform.position + new Vector3(input.x, 0, input.y);
    public void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(_Swiz(position1), radius1);
        Gizmos.DrawLine(_Swiz(position1), _Swiz(position1 + velocity1));

        Gizmos.color = Color.blue;
        Gizmos.DrawSphere(_Swiz(position2), radius2);
        Gizmos.DrawLine(_Swiz(position2), _Swiz(position2 + velocity2));

        if (!float.IsNaN(computedCollisionTime))
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(_Swiz(position1 + velocity1 * computedCollisionTime), radius1);
            Gizmos.DrawSphere(_Swiz(position2 + velocity2 * computedCollisionTime), radius2);
        }
    }
}
