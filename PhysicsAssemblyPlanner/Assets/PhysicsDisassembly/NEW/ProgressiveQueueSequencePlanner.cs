using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System;

public class ProgressiveQueueSequencePlanner
{
    private string assetFolder;
    private string assemblyDir;
    private string assemblyId;
    private List<string> partIds;
    private int numParts;
    private int maxSeqCount;

    private readonly string[] successStatus = { "Success", "Start with goal" };
    private readonly string[] failureStatus = { "Timeout", "Failure" };
    private readonly string[] validStatus;

    public ProgressiveQueueSequencePlanner(string assetFolder, string assemblyDir)
    {
        this.assetFolder = assetFolder;
        this.assemblyDir = assemblyDir;
        this.assemblyId = System.IO.Path.GetFileName(assemblyDir);

        LoadAssembly();

        numParts = partIds.Count;
        maxSeqCount = (1 + numParts) * numParts / 2 - 1;

        validStatus = successStatus.Concat(failureStatus).ToArray();
    }

    private void LoadAssembly()
    {
        // Implementation of LoadAssembly method
        // This should populate the 'partIds' list with part IDs
    }

    public (string status, List<string> sequence, int seqCount, float tPlanAll) PlanSequence(
        string pathPlannerName, bool rotation, string bodyType, float sdfDx, float collisionTh,
        float forceMag, int frameSkip, float seqMaxTime, float pathMaxTime, int seed,
        bool render, string recordDir, string saveDir, int nSaveState, bool verbose = false)
    {
        UnityEngine.Random.InitState(seed);

        if (render && !string.IsNullOrEmpty(recordDir))
        {
            System.IO.Directory.CreateDirectory(recordDir);
        }

        string seqStatus = "Failure";
        List<string> sequence = new List<string>();
        int seqCount = 0;
        float tPlanAll = 0;

        List<(string id, int depth)> activeQueue = partIds.Select(node => (node, 1)).ToList();
        Shuffle(activeQueue);
        List<(string id, int depth)> inactiveQueue = new List<(string id, int depth)>();

        while (true)
        {
            List<string> allIds = new List<string>(partIds);
            var (moveId, maxDepth) = activeQueue[0];
            activeQueue.RemoveAt(0);
            List<string> stillIds = allIds.Where(id => id != moveId).ToList();

            string recordPath = null;
            if (!string.IsNullOrEmpty(recordDir) && render)
            {
                recordPath = System.IO.Path.Combine(recordDir, assemblyId, $"{seqCount}_{moveId}.gif");
            }

            string currSaveDir = null;
            if (!string.IsNullOrEmpty(saveDir))
            {
                currSaveDir = System.IO.Path.Combine(saveDir, assemblyId, $"{seqCount}_{moveId}");
            }

            int currSeed = UnityEngine.Random.Range(0, maxSeqCount);

            var (status, tPlan) = PlanPath(moveId, stillIds, pathPlannerName, false, bodyType, sdfDx,
                collisionTh, forceMag, frameSkip, pathMaxTime, maxDepth, currSeed, render, recordPath,
                currSaveDir, nSaveState);

            if (failureStatus.Contains(status) && rotation)
            {
                var (rotStatus, tPlanRot) = PlanPath(moveId, stillIds, pathPlannerName, true, bodyType,
                    sdfDx, collisionTh, forceMag, frameSkip, pathMaxTime, maxDepth, currSeed, render,
                    recordPath, currSaveDir, nSaveState);
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
                partIds.Remove(moveId);
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

            if (partIds.Count == 1)
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
            Debug.Log($"Result: {seqStatus} | Disassembled: {sequence.Count}/{numParts - 1} | Total # trials: {seqCount} | Total planning time: {tPlanAll}");
            Debug.Log($"Sequence: {string.Join(", ", sequence)}");
        }

        return (seqStatus, sequence, seqCount, tPlanAll);
    }

    private void Shuffle<T>(List<T> list)
    {
        int n = list.Count;
        while (n > 1)
        {
            n--;
            int k = UnityEngine.Random.Range(0, n + 1);
            T value = list[k];
            list[k] = list[n];
            list[n] = value;
        }
    }

    private (string status, float tPlan) PlanPath(string moveId, List<string> stillIds, string plannerName,
        bool rotation, string bodyType, float sdfDx, float collisionTh, float forceMag, int frameSkip,
        float maxTime, int maxDepth, int seed, bool render, string recordPath, string saveDir, int nSaveState)
    {
        // Implementation of PlanPath method
        throw new NotImplementedException();
    }
}