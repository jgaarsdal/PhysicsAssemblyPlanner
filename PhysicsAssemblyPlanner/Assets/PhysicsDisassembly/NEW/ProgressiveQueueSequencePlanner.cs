using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class ProgressiveQueueSequencePlanner
{
    private List<string> _partIds;
    private Dictionary<string, Mesh> _partMeshes;
    private int _maxSeqCount;

    private readonly string[] successStatus = { "Success", "Start with goal" };
    private readonly string[] failureStatus = { "Timeout", "Failure" };
    private readonly string[] validStatus;

    public ProgressiveQueueSequencePlanner(Mesh[] partMeshes)
    {
        var numParts = partMeshes.Length;
        _maxSeqCount = (1 + numParts) * numParts / 2 - 1;

        validStatus = successStatus.Concat(failureStatus).ToArray();

        for (var index = 0; index < partMeshes.Length; index++)
        {
            var id = index.ToString();
            _partIds.Add(id);
            _partMeshes.Add(id, partMeshes[index]);
        }
    }

    public (string status, List<string> sequence, int seqCount, float tPlanAll) PlanSequence(
        bool useRotation, string bodyType, float sdfDx, float collisionTh,
        float forceMag, int frameSkip, float seqMaxTime, float pathMaxTime, int seed,
        bool verbose = false)
    {
        UnityEngine.Random.InitState(seed);

        var seqStatus = "Failure";
        var sequence = new List<string>();
        var seqCount = 0;
        var tPlanAll = 0f;

        var activeQueue = _partIds.Select(node => (node, 1)).ToList();
        Shuffle(activeQueue);
        var inactiveQueue = new List<(string id, int depth)>();

        while (true)
        {
            var allIds = new List<string>(_partIds);
            
            var (moveId, maxDepth) = activeQueue[0];
            activeQueue.RemoveAt(0);
            
            var stillIds = allIds.Where(id => id != moveId).ToList();

            var currSeed = UnityEngine.Random.Range(0, _maxSeqCount);

            var (status, tPlan, path) = PlanPath(moveId, stillIds, false, bodyType, sdfDx,
                collisionTh, forceMag, frameSkip, pathMaxTime, maxDepth, currSeed);

            if (failureStatus.Contains(status) && useRotation)
            {
                var (rotStatus, tPlanRot, pathRot) = PlanPath(moveId, stillIds, true, bodyType,
                    sdfDx, collisionTh, forceMag, frameSkip, pathMaxTime, maxDepth, currSeed);
                status = rotStatus;
                tPlan += tPlanRot;
            }

            tPlanAll += tPlan;
            seqCount++;

            if (verbose)
            {
                Debug.Log($"# trials: {seqCount} | Move id: {moveId} | Status: {status} | Current planning time: {tPlan} | Total planning time: {tPlanAll}");
            }

            if (successStatus.Contains(status))
            {
                _partIds.Remove(moveId);
                sequence.Add(moveId);
            }
            else
            {
                inactiveQueue.Add((moveId, maxDepth + 1));
            }

            if (verbose)
            {
                Debug.Log($"Active queue: {string.Join(", ", activeQueue)}");
                Debug.Log($"Inactive queue: {string.Join(", ", inactiveQueue)}");
            }

            if (_partIds.Count == 1)
            {
                seqStatus = "Success";
                break;
            }

            if (activeQueue.Count == 0)
            {
                activeQueue = new List<(string id, int depth)>(inactiveQueue);
                inactiveQueue.Clear();
            }

            if (tPlanAll > seqMaxTime)
            {
                seqStatus = "Timeout";
                break;
            }
        }

        if (verbose)
        {
            Debug.Log($"Result: {seqStatus} | Disassembled: {sequence.Count}/{_partIds.Count - 1} | Total # trials: {seqCount} | Total planning time: {tPlanAll}");
            Debug.Log($"Sequence: {string.Join(", ", sequence)}");
        }

        return (seqStatus, sequence, seqCount, tPlanAll);
    }

    private void Shuffle<T>(List<T> list)
    {
        var n = list.Count;
        while (n > 1)
        {
            n--;
            var k = UnityEngine.Random.Range(0, n + 1);
            T value = list[k];
            list[k] = list[n];
            list[n] = value;
        }
    }

    private (string status, float tPlan, List<Vector3> path) PlanPath(
        string moveId, List<string> stillIds,
        bool rotation, string bodyType, float sdfDx, float collisionTh,
        float forceMag, int frameSkip, float maxTime, int maxDepth, int seed)
    {
        var planner = new BFSPlanner(moveId, stillIds, rotation, bodyType, sdfDx, collisionTh, forceMag, frameSkip, _partMeshes);
        var (status, tPlan, path) = planner.Plan(maxTime, maxDepth, seed);

        return (status, tPlan, path);
    }
}