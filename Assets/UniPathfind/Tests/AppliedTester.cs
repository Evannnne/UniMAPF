using UniMAPF.Pathfinding;
using Sirenix.Utilities;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AppliedTester : MonoBehaviour
{
    public GameObject baseAgent;
    public List<SimpleTestAgent> agents;

    public float acceptableRadius = 20;
    public int agentCount = 10;

    public bool runSequential = false;

    [Sirenix.OdinInspector.Button]
    public void RunTest()
    {
        StartCoroutine(Run());
    }

    private IEnumerator Run()
    {
        int start = runSequential ? 0 : agents.Count - 1;
        for (int i = start; i < agents.Count; i++)
        {
            NavigationDispatcher.Instance.FlushRequests();
            Debug.Log($"Requesting {i + 1} agents to be pathfound.");
            for (int j = 0; j <= i; j++)
            {
                agents[j].RequestPath();
            }
            yield return new WaitForSeconds(0.1f);
            yield return new WaitWhile(() => NavigationDispatcher.Instance.JobInProgress);
            yield return new WaitForSeconds(0.1f);
        }
    }

    private NavNode GetClosestNode(Vector3 pos)
    {
        var nodes = FindObjectsOfType<NavNode>();
        var closestNode = nodes[0];
        var bestDist = Vector3.Distance(pos, nodes[0].position);
        for(int i = 0; i < nodes.Length; i++)
        {
            float dist = Vector3.Distance(pos, nodes[i].position);
            if(dist < bestDist)
            {
                closestNode = nodes[i];
                bestDist = dist;
            }
        }
        return closestNode;
    }

    [Sirenix.OdinInspector.Button]
    public void GenerateAgents()
    {
        HashSet<NavNode> occupied = new HashSet<NavNode>();

        for(int i = 0; i < agentCount; i++)
        {
            for (int j = 0; j < 100; j++)
            {
                Vector3 pos1 = Random.insideUnitCircle * Random.Range(acceptableRadius / 2f, acceptableRadius);
                pos1 = new Vector3(pos1.x, 0, pos1.y);
                pos1 += transform.position;
                NavNode src = GetClosestNode(pos1);

                Vector3 pos2 = Random.insideUnitCircle * Random.Range(acceptableRadius / 2f, acceptableRadius);
                pos2 = new Vector3(pos2.x, 0, pos2.y);
                pos2 += transform.position;
                NavNode dst = GetClosestNode(pos2);

                if (Vector3.Distance(pos1, pos2) < acceptableRadius / 2f) continue;

                if(!occupied.Contains(src)  && !occupied.Contains(dst))
                {
                    occupied.Add(src);
                    occupied.Add(dst);

                    var inst = Instantiate(baseAgent);
                    inst.transform.SetParent(transform, false);
                    inst.transform.position = src.position;
                    inst.name = $"Start {i}";
                    var target = new GameObject();
                    target.transform.SetParent(transform, false);
                    target.transform.position = dst.position;
                    target.name = $"Goal {i}";
                    inst.GetComponent<SimpleTestAgent>().targetPositionObject = target;
                    inst.GetComponent<SimpleTestAgent>().agentColor = new Color(Random.Range(0, 1f), Random.Range(0, 1f), Random.Range(0, 1f)).NormalizeRGB();
                    inst.SetActive(true);

                    agents.Add(inst.GetComponent<SimpleTestAgent>());

                    break;
                }
            }
        }

    }
}
