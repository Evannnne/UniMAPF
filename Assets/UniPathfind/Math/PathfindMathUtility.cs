using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace UniMAPF.Pathfinding
{
    public class UniMAPF
    {
        /// <summary>
        /// Uses Floyd Warshall to compute all-pairs shortest paths
        /// </summary>
        /// <typeparam name="NodeType"></typeparam>
        /// <param name="inputs"></param>
        /// <param name="givenCosts"></param>
        /// <returns></returns>
        public static FloydWarshallLookupTableHeuristic<NodeType> ComputeAPSP<NodeType>(NodeType[] inputs, ICostEvaluator<NodeType>[] givenCosts) where NodeType : IGraphNode
        {
            float[,] lookup = new float[inputs.Length, inputs.Length];
            Dictionary<NodeType, int> indexer = new Dictionary<NodeType, int>();
            for (int i = 0; i < inputs.Length; i++)
                indexer[inputs[i]] = i;

            // Initialize to infinity
            for (int i = 0; i < inputs.Length; i++)
                for (int j = 0; j < inputs.Length; j++)
                    lookup[i, j] = float.MaxValue;

            // Initialize distances to self and between singlular edges
            for (int i = 0; i < inputs.Length; i++)
            {
                lookup[i, i] = 0;

                foreach (var successor in inputs[i].GetAdjacents(float.MaxValue))
                {
                    NodeType casted = (NodeType)successor;
                    int otherIndex = indexer[casted];
                    float cost = 0;
                    foreach (var costSource in givenCosts) cost += costSource.CalculateCost(0, inputs[i], casted);
                    lookup[i, otherIndex] = cost;
                }
            }

            // Then, compute floyd warshall
            for (int k = 0; k < inputs.Length; k++)
            {
                for (int i = 0; i < inputs.Length; i++)
                {
                    for (int j = 0; j < inputs.Length; j++)
                    {
                        if (lookup[i, j] > lookup[i, k] + lookup[k, j] && lookup[k, j] != float.MaxValue && lookup[i, k] != float.MaxValue)
                            lookup[i, j] = lookup[i, k] + lookup[k, j];
                    }
                }
            }
            return new FloydWarshallLookupTableHeuristic<NodeType>(lookup, indexer);
        }
    }

    /// <summary>
    /// A path defined by an agent radius, the points it passes through, and the speed it moves at
    /// </summary>
    public class SegmentedPath
    {
        public float Radius { get; private set; }

        public List<Vector2> Points { get; private set; }
        public List<Vector2> Velocities { get; private set; }
        public List<float> Times { get; private set; }

        public override string ToString()
        {
            string result = $"R={Radius}, Path=[";
            for (int i = 0; i < Points.Count; i++)
                result += $"(t={Times[i]}, p={Points[i]}, v={Velocities[i]})" + (i < Points.Count - 1 ? ", " : "");
            result += "]";
            return result;
        }

        public SegmentedPath(float radius, List<TimeBasedNavNode> path, bool extendToInfinity = false)
        {
            Radius = radius;
            Points = new List<Vector2>(path.Count);
            Times = new List<float>(path.Count);
            for (int i = 0; i < path.Count; i++)
            {
                Vector3 p = path[i].Position;
                Points.Add(new Vector2(p.x, p.z));
                Times.Add(path[i].Time);
            }
            CalculateVelocities();
            if (extendToInfinity) ExtendToInfinity();
        }

        public SegmentedPath(float radius, Vector2[] path, float startTime, float speed, bool extendToInfinity = false)
        {
            // Allows for easy extension of the path if needed
            Points = new List<Vector2>(path.Length + 1);
            Velocities = new List<Vector2>(path.Length + 1);
            Times = new List<float>(path.Length + 1);

            // Compute points, velocities, times at each step
            float time = startTime;
            Times.Add(time);
            Points.Add(path[0]);
            for (int i = 1; i < path.Length; i++)
            {
                Vector2 delta = path[i] - path[i - 1];
                float t = delta.magnitude / speed;
                time += t;
                Times.Add(time);
                Velocities.Add(delta.normalized * speed);
                Points.Add(path[i]);
            }
            Velocities.Add(Vector3.zero);
            Radius = radius;

            if(extendToInfinity) ExtendToInfinity();
        }
        public SegmentedPath(float radius, Vector2[] path, float[] times, bool extendToInfinity = false)
        {
            Radius = radius;
            Points = new List<Vector2>(path);
            Times = new List<float>(times);
            CalculateVelocities();
            if (extendToInfinity) ExtendToInfinity();
        }
        public void CalculateVelocities()
        {
            Velocities = new List<Vector2>(Points.Count + 1);

            // Compute velocities at each timestep
            for (int i = 0; i < Points.Count - 1; i++)
            {
                float dt = Times[i + 1] - Times[i];
                Vector2 dp = Points[i + 1] - Points[i];
                if (dt != 0) Velocities.Add(dp / dt);
                else Velocities.Add(Vector2.zero);
            }
            Velocities.Add(Vector2.zero);
        }
        public void ExtendToInfinity()
        {
            Velocities.Add(Vector3.zero);
            Points.Add(Points[Points.Count - 1]);
            Times.Add(float.MaxValue);
        }

        /// <summary>
        /// Efficiently get the position at a given time
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public Vector2 GetPositionAtTime(float t)
        {
            int segmentIndex = GetSegmentIndexForTime(t);
            if (segmentIndex != -1)
            {
                if (segmentIndex < Points.Count - 1)
                {
                    Vector2 p0 = Points[segmentIndex];
                    Vector2 p1 = Points[segmentIndex + 1];
                    float t0 = Times[segmentIndex];
                    float t1 = Times[segmentIndex + 1];
                    return Vector2.Lerp(p0, p1, (t - t0) / (t1 - t0));
                }
                else return Points[segmentIndex];
            }
            else return Points[0];
        }

        /// <summary>
        /// Returns the lower index i for a segment, such that Times[i] <= time <= Times[i+1]
        /// </summary>
        /// <param name="time"></param>
        /// <returns></returns>
        public int GetSegmentIndexForTime(float time)
        {
            // For edge cases
            if (time >= Times[Times.Count - 1]) return Times.Count - 1;
            if (time <= Times[0]) return 0;

            // First, find the element t_[n] that satisfies
            // I.e., t_[n] <= t <= t_[n+1]
            // We can use a binary search since the time map is monotonically increasing
            int lower = 0;
            int upper = Times.Count - 2;
            int found = -1;
            while (lower <= upper)
            {
                int mid = lower + (upper - lower) / 2;
                if (Times[mid] <= time && time <= Times[mid + 1])
                {
                    found = mid;
                    break;
                }

                if (Times[mid] < time)
                    lower = mid + 1;
                else upper = mid - 1;
            }
            return found;
        }
    }

    public static class UniMAPFPathfindingUtility
    {
        /// <summary>
        /// Turn a list into a string
        /// </summary>
        public static string ListToString<T>(List<T> list)
        {
            string result = "[";
            for (int i = 0; i < list.Count; i++)
                result += list[i].ToString() + (i < list.Count - 1 ? "," : "");
            result += "]";
            return result;
        }

        // Find the collision time of two agents, one starting at p0 and moving at speed v0, the other starting at p1 and moving at speed v1
        public static Vector2 GetCollisionInterval2D(Vector2 p0, Vector2 v0, float r0, Vector2 p1, Vector2 v1, float r1)
        {
            const float EPSILON = 0.01f;

            // We want to see if there's any point on the lines defined by p0 + v0t, p1 + v1t
            // Such that the distance between those two lines is less than or equal to r0+r1 (sphere/sphere intersection)
            float R = r0 + r1;
            Vector2 p = p1 - p0;
            Vector2 v = v1 - v0;

            // If they're already colliding, immediately return true!
            if (p.sqrMagnitude <= R * R) return new Vector2(0f, 1f);

            // We are now treating agent 0 as stationary, and agent 1 as moving in its frame of reference
            // With this reformulation, we are now checking to see if it gets within R units of the origin
            //      (p + vt).magnitude < R
            //  =>  (p + vt).sqrMagnitude < R * R
            //  =>  (p + vt).sqrMagnitude == R * R - epsilon
            //  =>  (p_x + v_x * t)^2 + (p_y + v_y * t)^2 == thresh
            //
            //  =>  (p_x^2 + 2 * p_x * v_x * t + v_x^2 * t^2) +
            //      (p_y^2 + 2 * p_x * v_y * t + v_y^2 * t^2) - thresh =
            //      0
            //
            //  => (v_x^2 + v_y^2) * t^2 + 2(p_x * v_x + p_y * v_y) * t + (p_x^2 + p_y^2 - thresh)
            // This gives us a quadratic formula that we can solve

            float thresh = R * R - EPSILON;
            float a = v.x * v.x + v.y * v.y;
            float b = 2 * (p.x * v.x + p.y * v.y);
            float c = p.x * p.x + p.y * p.y - thresh;

            float term2_squared = b * b - 4 * a * c;
            if (term2_squared < 0) return new Vector2(float.NaN, float.NaN);

            float term1 = -b;
            float term2 = Mathf.Sqrt(term2_squared);
            float term3 = 2 * a;

            float t1 = (term1 + term2) / term3;
            float t2 = (term1 - term2) / term3;
            
            if(t1 > t2)
            {
                var tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            return new Vector2(t1, t2);
        }

        /// <summary>
        /// Gets the soonest collision for a limited trajectory, returning NaN if none is found.
        /// </summary>
        public static float ComputeSoonestCollision(SegmentedPath a, Vector2 p0, Vector2 p1, float radius, float speed, float startTime)
        {
            SegmentedPath b = new SegmentedPath(radius, new Vector2[] { p0, p1 }, startTime, speed);
            int segmentA = a.GetSegmentIndexForTime(startTime);
            return ComputeSoonestCollision(a, b, aStart: segmentA);
        }

        /// <summary>
        /// Computes the soonest possible collision between paths A and B, returning NaN if none is found.
        /// If you have specific times you want to check, you can also specify a segment start for A or B.
        /// </summary>
        public static float ComputeSoonestCollision(SegmentedPath A, SegmentedPath B, int aStart = 0, int bStart = 0, bool log = false)
        {
            if (A == null || B == null) return float.NaN;

            int a = aStart;
            int b = bStart;

            int EMERGENCY_BREAK = 0;
            int EMERGENCY_BREAK_THRESH = A.Points.Count + B.Points.Count;

            // While there are still segments to evaluate
            while (a < A.Times.Count - 1 && b < B.Times.Count - 1)
            {
                float aTimeStart = A.Times[a];
                float aTimeEnd = A.Times[a + 1];
                float bTimeStart = B.Times[b];
                float bTimeEnd = B.Times[b + 1];

                // If A's current segment ends before B's begins, iterate i
                if (aTimeEnd < bTimeStart)
                {
                    a++;
                }
                // If B's current segment ends before A's begins, iterate j
                else if (bTimeEnd < aTimeStart)
                {
                    b++;
                }
                // Otherwise, there is an intersection between the current segment of A and B
                else
                {
                    if (log) Debug.Log($"Comparing segments a={a}, ta={A.Times[a]}, pa={A.Points[a]}, va={A.Velocities[a]} | b={b}, tb={B.Times[b]}, pb={B.Points[b]}, vb={B.Velocities[b]}");

                    // We need to analyze the time where the segments overlap, i.e. between the latest start and soonest end
                    float realStartTime = Mathf.Max(aTimeStart, bTimeStart);
                    float realEndTime = Mathf.Min(aTimeEnd, bTimeEnd);
                    float realWindow = realEndTime - realStartTime;

                    // We now need to reformulate the problem into a form where both agents move at the same time
                    // So, we "Fast forward" both agents to be at the correct time
                    float aDt = Mathf.Max(0, realStartTime - A.Times[a]);
                    float bDt = Mathf.Max(0, realStartTime - B.Times[b]);

                    Vector2 realAStart = A.Points[a] + A.Velocities[a] * aDt;
                    Vector2 realBStart = B.Points[b] + B.Velocities[b] * bDt;

                    // Calculate the fast-forwarded collision
                    float localCollisionTime = GetCollisionInterval2D(realAStart, A.Velocities[a], A.Radius, realBStart, B.Velocities[b], B.Radius).x;

                    if (localCollisionTime != float.NaN && localCollisionTime <= realWindow && localCollisionTime >= 0)
                    {
                        if (log) Debug.Log($"Collision found between a={a}, b={b} at time {realStartTime + localCollisionTime}");
                        return realStartTime + localCollisionTime;
                    }

                    // Otherwise, fast forward one or both of the segments
                    if (aTimeEnd == realEndTime)
                    {
                        a++;
                    }
                    if (bTimeEnd == realEndTime)
                    {
                        b++;
                    }
                }

                // Each operation should either increase a or b
                // If we go past a or b, then something has gone very wrong!
                EMERGENCY_BREAK++;
                if (EMERGENCY_BREAK > EMERGENCY_BREAK_THRESH)
                {
                    Debug.LogError("Reached emergency break!");
                    return float.NaN;
                }
            }
            return float.NaN;
        }

        /// <summary>
        /// Computes the total time spent colliding
        /// </summary>
        public static float ComputeTotalCollisionTime(SegmentedPath A, SegmentedPath B, int aStart = 0, int bStart = 0, bool log = false)
        {
            int a = aStart;
            int b = bStart;

            int EMERGENCY_BREAK = 0;
            int EMERGENCY_BREAK_THRESH = A.Points.Count + B.Points.Count;

            float sum = 0;

            // While there are still segments to evaluate
            while (a < A.Times.Count - 1 && b < B.Times.Count - 1)
            {
                float aTimeStart = A.Times[a];
                float aTimeEnd = A.Times[a + 1];
                float bTimeStart = B.Times[b];
                float bTimeEnd = B.Times[b + 1];

                // If A's current segment ends before B's begins, iterate i
                if (aTimeEnd < bTimeStart)
                {
                    a++;
                }
                // If B's current segment ends before A's begins, iterate j
                else if (bTimeEnd < aTimeStart)
                {
                    b++;
                }
                // Otherwise, there is an intersection between the current segment of A and B
                else
                {
                    if (log) Debug.Log($"Comparing segments a={a}, ta={A.Times[a]}, pa={A.Points[a]}, va={A.Velocities[a]} | b={b}, tb={B.Times[b]}, pb={B.Points[b]}, vb={B.Velocities[b]}");

                    // We need to analyze the time where the segments overlap, i.e. between the latest start and soonest end
                    float realStartTime = Mathf.Max(aTimeStart, bTimeStart);
                    float realEndTime = Mathf.Min(aTimeEnd, bTimeEnd);
                    float realWindow = realEndTime - realStartTime;

                    // We now need to reformulate the problem into a form where both agents move at the same time
                    // So, we "Fast forward" both agents to be at the correct time
                    float aDt = Mathf.Max(0, realStartTime - A.Times[a]);
                    float bDt = Mathf.Max(0, realStartTime - B.Times[b]);

                    Vector2 realAStart = A.Points[a] + A.Velocities[a] * aDt;
                    Vector2 realBStart = B.Points[b] + B.Velocities[b] * bDt;

                    // Calculate the fast-forwarded collision
                    Vector2 localCollisionTime = GetCollisionInterval2D(realAStart, A.Velocities[a], A.Radius, realBStart, B.Velocities[b], B.Radius);
                    if(!float.IsNaN(localCollisionTime.y) && !float.IsNaN(localCollisionTime.x)) sum += Mathf.Abs(localCollisionTime.y - localCollisionTime.x);

                    if (aTimeEnd == realEndTime)
                    {
                        a++;
                    }
                    if (bTimeEnd == realEndTime)
                    {
                        b++;
                    }
                }

                // Each operation should either increase a or b
                // If we go past a or b, then something has gone very wrong!
                EMERGENCY_BREAK++;
                if (EMERGENCY_BREAK > EMERGENCY_BREAK_THRESH)
                {
                    Debug.LogError("Reached emergency break!");
                    return float.NaN;
                }
            }

            return sum;
        }
    }
}