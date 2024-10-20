using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

public class ProgressiveQueueSequencePlanner
{
    private List<string> _partIds = new List<string>();
    private Dictionary<string, Mesh> _partMeshes = new Dictionary<string, Mesh>();
    private Dictionary<string, SignedDistanceField> _partSDFs = new Dictionary<string, SignedDistanceField>();

    private readonly string[] _successStatus = { "Success", "Start with goal" };
    private readonly string[] _failureStatus = { "Timeout", "Failure" };
    //private readonly string[] _validStatus;

    public ProgressiveQueueSequencePlanner((Mesh, Transform)[] parts)
    {
        //_validStatus = _successStatus.Concat(_failureStatus).ToArray();

        for (var i = 0; i < parts.Length; i++)
        {
            var id = i.ToString();
            _partIds.Add(id);
            _partMeshes.Add(id, parts[i].Item1);
            _partSDFs.Add(id, new SignedDistanceField(parts[i].Item1, parts[i].Item2, 224));
        }
    }

    public async Task InitializeSignedDistanceFields()
    {
        foreach (var partSDF in _partSDFs.Values)
        {
            await partSDF.ComputeSDF();
        }
    }
    
    public (string status, List<string> sequence, int seqCount, float totalDurationSecs) PlanSequence(
        bool useRotation, float simulationForce, int simulationFrameSkip, float totalTimeoutSecs, float pathTimeoutSecs, int randomSeed,
        bool verbose = false)
    {
        UnityEngine.Random.InitState(randomSeed);

        var planner = new BFSPlanner("0", new List<string>{ "1" }, useRotation, simulationForce, simulationFrameSkip, _partMeshes, _partSDFs);
        var (testStatus, testDuration, testPath) = planner.Plan(pathTimeoutSecs, 1);

        foreach (var pos in testPath)
        {
            Debug.Log("testPath: " + pos.x + "," + pos.y + "," + pos.z);
        }
        
        return (testStatus, new List<string>{ "0" }, 1, testDuration);
        
        
        
        
        
        
        var seqStatus = "Failure";
        var sequence = new List<string>();
        var seqCount = 0;
        var totalDurationSecs = 0f;

        var activeQueue = _partIds.Select(partId => (partId, 1)).ToList();
        Shuffle(activeQueue);
        var inactiveQueue = new List<(string id, int depth)>();

        while (true)
        {
            var allIds = new List<string>(_partIds);
            
            var (moveId, maxDepth) = activeQueue[0];
            activeQueue.RemoveAt(0);
            
            var stillIds = allIds.Where(id => id != moveId).ToList();
            
            var (status, durationSecs, path) = PlanPath(moveId, stillIds, false, simulationForce, simulationFrameSkip, pathTimeoutSecs, maxDepth);

            if (_failureStatus.Contains(status) && useRotation)
            {
                var (rotationStatus, durationRotationSecs, pathRotation) = PlanPath(moveId, stillIds, true, simulationForce, simulationFrameSkip, pathTimeoutSecs, maxDepth);
                status = rotationStatus;
                durationSecs += durationRotationSecs;
            }

            totalDurationSecs += durationSecs;
            seqCount++;

            if (verbose)
            {
                Debug.Log($"# trials: {seqCount} | Move id: {moveId} | Status: {status} | Current planning time: {durationSecs} | Total planning time: {totalDurationSecs}");
            }

            if (_successStatus.Contains(status))
            {
                allIds.Remove(moveId);
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

            if (allIds.Count == 1)
            {
                seqStatus = "Success";
                break;
            }

            if (activeQueue.Count == 0)
            {
                activeQueue = new List<(string id, int depth)>(inactiveQueue);
                inactiveQueue.Clear();
            }

            if (totalDurationSecs > totalTimeoutSecs)
            {
                seqStatus = "Timeout";
                break;
            }
        }

        if (verbose)
        {
            Debug.Log($"Result: {seqStatus} | Disassembled: {sequence.Count}/{_partIds.Count - 1} | Total # trials: {seqCount} | Total planning time: {totalDurationSecs}");
            Debug.Log($"Sequence: {string.Join(", ", sequence)}");
        }

        return (seqStatus, sequence, seqCount, totalDurationSecs);
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
        bool rotation, float simulationForce, int simulationFrameSkip, float pathTimeoutSecs, int maxDepth)
    {
        var planner = new BFSPlanner(moveId, stillIds, rotation, simulationForce, simulationFrameSkip, _partMeshes, _partSDFs);
        var (status, tPlan, path) = planner.Plan(pathTimeoutSecs, maxDepth);

        return (status, tPlan, path);
    }
}