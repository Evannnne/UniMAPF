using UniMAPF.Pathfinding;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Benchmarker_PathSegmentCollision : MonoBehaviour
{
    [Sirenix.OdinInspector.Button]
    public void Benchmark(int pathSize = 100, int iterations = 1000)
    {
        // Create a path where P(t) = (t, 0)
        Vector2[] path = new Vector2[pathSize];
        float[] times = new float[pathSize];
        for (int i = 0; i < pathSize; i++)
        {
            path[i] = new Vector2(i, 0);
            times[i] = i;
        }
        SegmentedPath segmentPath = new SegmentedPath(1f, path, times);

        // Test a random point
        System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
        watch.Start();
        for (int i = 0; i < iterations; i++)
        {
            // If P(t) = (t, 0)
            // We need a segment to check s.t. P(t-1) = (t, 1), and P(t) = (t, 0)
            int index = Random.Range(1, pathSize);
            float soonest = UniMAPFPathfindingUtility.ComputeSoonestCollision(segmentPath, new Vector2(index, 1), new Vector2(index, 0), 1f, 1f, index - 1f);
            if (float.IsNaN(soonest))
            {
                Debug.LogError("Something went wrong!");
                break;
            }
        }
        Debug.Log($"Elapsed: {watch.ElapsedTicks / (float)System.TimeSpan.TicksPerMillisecond}ms for {iterations} iterations.");
    }
}
