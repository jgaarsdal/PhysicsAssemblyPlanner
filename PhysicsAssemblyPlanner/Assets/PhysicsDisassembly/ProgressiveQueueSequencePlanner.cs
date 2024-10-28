using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using DG.Tweening;
using PhysicsDisassembly.SDF;

namespace PhysicsDisassembly
{
    public class ProgressiveQueueSequencePlanner
    {
        private List<string> _partIds = new List<string>();
        private Dictionary<string, GameObject> _partObjects = new Dictionary<string, GameObject>();
        private Dictionary<string, SignedDistanceField> _partSDFs = new Dictionary<string, SignedDistanceField>();
        private AssemblyPlanningConfiguration _configuration;
        
        private readonly string[] _successStatus = { "Success", "Start with goal" };
        private readonly string[] _failureStatus = { "Timeout", "Failure" };

        public ProgressiveQueueSequencePlanner(GameObject[] parts, AssemblyPlanningConfiguration configuration)
        {
            _configuration = configuration;
            
            for (var i = 0; i < parts.Length; i++)
            {
                var id = i.ToString();
                _partIds.Add(id);
                _partObjects.Add(id, parts[i]);
                _partSDFs.Add(id, new SignedDistanceField(parts[i], _configuration.SDFCollisionConfiguration));
            }
        }

        public async Task InitializeSignedDistanceFields()
        {
            await Task.WhenAll(_partSDFs.Values.Select(p => p.ComputeSDF()));
        }

        public (string status, List<Path> sequence, int seqCount, float totalDurationSecs) PlanSequence(int randomSeed)
        {
            UnityEngine.Random.InitState(randomSeed);

            var planner = new BFSPlanner("0", new List<string> { "1" }, _configuration.DisassemblyUseRotation, 
                _partObjects, _partSDFs, _configuration.BFSPlannerConfiguration, _configuration.PhysicsSimulationConfiguration);
            var (testStatus, testDuration, testPath) = planner.Plan(_configuration.PartTimeoutSecs, 1);

            Debug.Log($"Finished with status '{testStatus}' in {testDuration} seconds and {testPath.Positions.Count} points");

            _partObjects["0"].transform.DOPath(testPath.Positions.ToArray(), 5f).SetEase(Ease.InOutCubic).SetLoops(-1, LoopType.Yoyo);
            
            return (testStatus, new List<Path> { testPath }, 1, testDuration);



            var seqStatus = "Failure";
            var sequence = new List<Path>();
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

                var (status, durationSecs, path) = PlanPath(moveId, stillIds, false, maxDepth);

                if (_failureStatus.Contains(status) && _configuration.DisassemblyUseRotation)
                {
                    var (rotationStatus, durationRotationSecs, pathRotation) = PlanPath(moveId, stillIds, true, maxDepth);
                    status = rotationStatus;
                    path = pathRotation;
                    durationSecs += durationRotationSecs;
                }

                totalDurationSecs += durationSecs;
                seqCount++;

                if (_configuration.Verbose)
                {
                    Debug.Log(
                        $"# trials: {seqCount} | Move id: {moveId} | Status: {status} | Current planning time: {durationSecs} | Total planning time: {totalDurationSecs}");
                }

                if (_successStatus.Contains(status))
                {
                    allIds.Remove(moveId);
                    sequence.Add(path);
                }
                else
                {
                    inactiveQueue.Add((moveId, maxDepth + 1));
                }

                if (_configuration.Verbose)
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

                if (totalDurationSecs > _configuration.AssemblyTimeoutSecs)
                {
                    seqStatus = "Timeout";
                    break;
                }
            }

            if (_configuration.Verbose)
            {
                Debug.Log(
                    $"Result: {seqStatus} | Disassembled: {sequence.Count}/{_partIds.Count - 1} | Total # trials: {seqCount} | Total planning time: {totalDurationSecs}");
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

        private (string status, float tPlan, Path path) PlanPath(string moveId, List<string> stillIds, bool rotation, int maxDepth)
        {
            var planner = new BFSPlanner(moveId, stillIds, rotation, _partObjects, _partSDFs, 
                _configuration.BFSPlannerConfiguration, _configuration.PhysicsSimulationConfiguration);
            
            var (status, tPlan, path) = planner.Plan(_configuration.PartTimeoutSecs, maxDepth);

            return (status, tPlan, path);
        }
    }
}