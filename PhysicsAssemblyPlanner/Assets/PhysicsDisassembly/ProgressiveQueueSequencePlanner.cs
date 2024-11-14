using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using PhysicsDisassembly.SDF;

namespace PhysicsDisassembly
{
    public class ProgressiveQueueSequencePlanner
    {
        public Dictionary<string, Transform> PartObjects => _partObjects;
        public Dictionary<string, SignedDistanceField> PartSDFs => _partSDFs;
        
        private List<string> _partIds = new List<string>();
        private Dictionary<string, Transform> _partObjects = new Dictionary<string, Transform>();
        private Dictionary<string, SignedDistanceField> _partSDFs = new Dictionary<string, SignedDistanceField>();
        private AssemblyPlanningConfiguration _configuration;
        
        private readonly string[] _successStatus = { "Success", "Already disassembled" };
        private readonly string[] _failureStatus = { "Timeout", "Failure" };

        private Dictionary<string, int> _attemptsPerPart = new Dictionary<string, int>();
        private Dictionary<string, HashSet<string>> _blockedByRelations = new Dictionary<string, HashSet<string>>();
        
        private const int _maxAttemptsPerPart = 10;
        
        public ProgressiveQueueSequencePlanner(Transform assemblyRoot, AssemblyPlanningConfiguration configuration)
        {
            _configuration = configuration;
            
            var assemblyParts = assemblyRoot.GetComponentsInChildren<MeshFilter>()
                .Select(mf => GetAssemblyPartRootRecursive(assemblyRoot, mf.transform))
                .ToArray();
            
            for (var i = 0; i < assemblyParts.Length; i++)
            {
                var id = i.ToString();
                _partIds.Add(id);
                _partObjects.Add(id, assemblyParts[i]);
                _partSDFs.Add(id, new SignedDistanceField(assemblyParts[i], _configuration.SDFCollisionConfiguration));
            }
        }

        public async Task InitializeSignedDistanceFields()
        {
            Debug.Log("Start Initialize Signed Distance Fields");

            foreach (var partSDF in _partSDFs.Values)
            {
                await partSDF.ComputeSDF();
            }

            Debug.Log("Finished Initialize Signed Distance Fields");
        }

        public (string status, List<Path> sequence, int seqCount, float totalDurationSecs) PlanSequence(int randomSeed)
        {
            UnityEngine.Random.InitState(randomSeed);
            
            var seqStatus = "Failure";
            var sequence = new List<Path>();
            var seqCount = 0;
            var totalDurationSecs = 0f;

            _attemptsPerPart.Clear();
            _blockedByRelations.Clear();
            foreach (var partId in _partIds)
            {
                _attemptsPerPart[partId] = 0;
                _blockedByRelations[partId] = new HashSet<string>();
            }
            
            var activeQueue = _partIds.Select(partId => (partId, 1)).ToList();
            Shuffle(activeQueue);
            var inactiveQueue = new List<(string id, int depth)>();

            var allIds = new HashSet<string>(_partIds);
            
            while (true)
            {
                if (activeQueue.Count == 0)
                {
                    activeQueue = new List<(string id, int depth)>(inactiveQueue);
                    inactiveQueue.Clear();

                    if (activeQueue.Count == 0)
                    {
                        break;
                    }
                    
                    // Sort active queue by most promising parts
                    activeQueue.Sort((a, b) => CompareParts(a.partId, b.partId, allIds));
                }
                
                var (moveId, maxDepth) = activeQueue[0];
                activeQueue.RemoveAt(0);
                
                // Check if we should try this part before doing any planning
                if (!ShouldTryPart(moveId))
                {
                    if (_configuration.Verbose)
                    {
                        Debug.Log($"Skipping part {moveId} due to too many attempts");
                    }
                    continue;  // Skip to next part in queue
                }
                
                var stillIds = allIds.Where(id => id != moveId).ToList();

                if (_configuration.Verbose)
                {
                    Debug.Log($"Attempting to remove part {moveId} (attempt #{_attemptsPerPart[moveId]})");
                    Debug.Log($"Still parts: {string.Join(", ", stillIds)}");
                }
                
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
                    // Record which parts are blocking this part
                    foreach (var stillId in stillIds)
                    {
                        _blockedByRelations[moveId].Add(stillId);
                    }
                    
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
                Debug.Log($"Sequence: {string.Join(", ", sequence.Select(p => p.PartID))}");
            }

            return (seqStatus, sequence, seqCount, totalDurationSecs);
        }

        private Transform GetAssemblyPartRootRecursive(Transform assemblyRoot, Transform assemblyPart)
        {
            if (assemblyPart.parent == assemblyRoot || 
                assemblyPart.parent.GetComponentsInChildren<MeshFilter>().Length > 1)
            {
                return assemblyPart;
            }

            return GetAssemblyPartRootRecursive(assemblyRoot, assemblyPart.parent);
        }
        
        private void Shuffle<T>(IList<T> list)
        {
            var count = list.Count;
            var last = count - 1;
            for (var i = 0; i < last; ++i)
            {
                var randIndex = UnityEngine.Random.Range(i, count);
                var tmpValue = list[i];
                list[i] = list[randIndex];
                list[randIndex] = tmpValue;
            }
        }

        private (string status, float tPlan, Path path) PlanPath(string moveId, List<string> stillIds, bool rotation, int maxDepth)
        {
            // Increment attempts when we actually try to plan a path
            _attemptsPerPart[moveId]++;
        
            if (_configuration.Verbose)
            {
                Debug.Log($"Attempt #{_attemptsPerPart[moveId]} for part {moveId}");
            }

            var currentPartObjects = new Dictionary<string, Transform>() { { moveId, _partObjects[moveId] } };
            var currentPartSDFs = new Dictionary<string, SignedDistanceField>() { { moveId, _partSDFs[moveId] } };

            foreach (var partId in stillIds)
            {
                currentPartObjects.Add(partId, _partObjects[partId]);
                currentPartSDFs.Add(partId, _partSDFs[partId]);
            }
            
            var planner = new BFSPlanner(moveId, stillIds, rotation, currentPartObjects, currentPartSDFs, 
                _configuration.BFSPlannerConfiguration, _configuration.PhysicsSimulationConfiguration);
            
            var (status, tPlan, path) = planner.Plan(_configuration.PartTimeoutSecs, maxDepth, _configuration.Verbose);

            return (status, tPlan, path);
        }

        private int CompareParts(string partA, string partB, HashSet<string> remainingParts)
        {
            // Try parts with fewer attempts first
            var attemptCompare = _attemptsPerPart[partA].CompareTo(_attemptsPerPart[partB]);
            if (attemptCompare != 0)
            {
                return attemptCompare;
            }
            
            // If attempts are equal, consider blocking relations
            var aBlockedCount = _blockedByRelations[partA].Count(id => remainingParts.Contains(id));
            var bBlockedCount = _blockedByRelations[partB].Count(id => remainingParts.Contains(id));
            
            return aBlockedCount.CompareTo(bBlockedCount);
        }
        
        private bool ShouldTryPart(string partId)
        {
            if (_attemptsPerPart[partId] >= _maxAttemptsPerPart)
            {
                if (_configuration.Verbose)
                {
                    Debug.Log($"Skipping part {partId} - reached maximum attempts ({_maxAttemptsPerPart})");
                }
                return false;
            }
            return true;
        }
    }
}