using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using PhysicsDisassembly.Simulation;
using PhysicsDisassembly.RRTConnect;
using PhysicsDisassembly.SDF;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class AssemblyPlanner : MonoBehaviour
    {
        [Header("Assembly Planning Settings")]
        [SerializeField] private bool _useRotation = false;
        [SerializeField] private bool _useRRTConnect = true;
        [SerializeField] private bool _usePathSimplifier = true;
        [SerializeField] private float _assemblyTimeoutSecs = 600f;
        [SerializeField] private float _partTimeoutSecs = 450f;
        [SerializeField] private bool _useRandomSeed = true;
        [SerializeField] private int _fixedSeed = 896;
        [SerializeField] private bool _verbose = true;

        [Header("BFSPlanner Settings")] 
        [SerializeField] private float _bfsStatePositionThreshold = 0.21f;
        [SerializeField] private float _bfsStateAngleThreshold = 0.5f;

        [Header("Physics Simulation Settings")] 
        [SerializeField] private float _simulationForce = 10f;
        [SerializeField] private float _simulationTorque = 200f;
        [SerializeField] private float _simulationTimeStep = 0.0005f;
        [SerializeField] private int _simulationFrameSkip = 90;
        [SerializeField] private float _simulationContactStiffness = 20f;
        [SerializeField] private float _simulationContactDamping = 1f;
        [SerializeField] private float _simulationMaxVelocity = 0.0025f;
        [SerializeField] private float _simulationMaxAngularVelocity = 0.05f;
        [SerializeField] private int _simulationContactPointCount = 1024;
        [SerializeField] private float _simulationCollisionThreshold = -0.03f;

        [Header("SDF Collision Settings")] 
        [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private bool _useGPU = true;

        [Header("RRT-Connect Settings")]
        [SerializeField] private bool _rrtUseRotation = true;
        [SerializeField] private float _rrtStepSize = 0.5f; // TODO: or 0.1f?
        [SerializeField] private float _rrtRotationStepSize = 5f; // TODO: or 15f?
        [SerializeField] private int _rrtMaxIterations = 10000;
        [SerializeField] private float _rrtConnectDistance = 1f;
        [SerializeField] private int _rrtRandomPointAttempts = 10;
        [SerializeField] private float _rrtWorkspaceBoundsBufferPercentage = 0.5f;
        [SerializeField] [Range(0f, 1f)] private float _rrtExplorationBias = 0.5f;
        
        [Header("Path Simplifier Settings")]  
        [SerializeField] private float _minimumProgressThreshold = 0.1f;
        [SerializeField] private float _maximumProgressThreshold = 0.5f;
        [SerializeField] private int _transitionTestSteps = 10;

        private int _randomSeed;
        private AssemblyPart[] _assemblyParts;
        private Dictionary<string, PartData> _partDataMap;
        private AssemblyPlanningConfiguration _configuration;
        
        public async Task InitializeAsync(AssemblyPart[] parts)
        {
            _randomSeed = _useRandomSeed ? (int)DateTime.Now.Ticks : _fixedSeed;
            UnityEngine.Random.InitState(_randomSeed);
            
            _configuration = new AssemblyPlanningConfiguration()
            {
                DisassemblyUseRotation = _useRotation,
                UseRRTConnect = _useRRTConnect,
                UsePathSimplifier = _usePathSimplifier,
                BFSPlannerConfiguration = new BFSPlannerConfiguration()
                {
                    BFSStatePositionThreshold = _bfsStatePositionThreshold,
                    BFSStateAngleThreshold = _bfsStateAngleThreshold
                },
                AssemblyTimeoutSecs = _assemblyTimeoutSecs,
                PartTimeoutSecs = _partTimeoutSecs,
                PhysicsSimulationConfiguration = new PhysicsSimulationConfiguration()
                {
                    SimulationForce = _simulationForce,
                    SimulationTorque = _simulationTorque,
                    SimulationTimeStep = _simulationTimeStep,
                    SimulationFrameSkip = _simulationFrameSkip,
                    SimulationContactStiffness = _simulationContactStiffness,
                    SimulationContactDamping = _simulationContactDamping,
                    SimulationMaxVelocity = _simulationMaxVelocity,
                    SimulationMaxAngularVelocity = _simulationMaxAngularVelocity,
                    SimulationContactPointCount = _simulationContactPointCount,
                    SimulationCollisionThreshold = _simulationCollisionThreshold
                },
                SDFCollisionConfiguration = new SDFCollisionConfiguration()
                {
                    SDFDefaultCellSize = _sdfDefaultCellSize,
                    SDFBoxPadding = _sdfBoxPadding,
                    SDFUseGPU = _useGPU
                },
                RRTConfiguration = new RRTConfiguration()
                {
                    RRTUseRotation = _rrtUseRotation,
                    RRTStepSize = _rrtStepSize,
                    RRTRotationStepSize = _rrtRotationStepSize,
                    RRTMaxIterations = _rrtMaxIterations,
                    RRTConnectDistance = _rrtConnectDistance,
                    RRTRandomPointAttempts = _rrtRandomPointAttempts,
                    RRTWorkspaceBoundsBufferPercentage = _rrtWorkspaceBoundsBufferPercentage,
                    RRTExplorationBias = _rrtExplorationBias
                },
                SimplifierConfiguration = new PathSimplifierConfiguration()
                {
                    SimplifierMinimumProgressThreshold = _minimumProgressThreshold,
                    SimplifierMaximumProgressThreshold = _maximumProgressThreshold,
                    SimplifierTransitionTestSteps = _transitionTestSteps
                },
                Verbose = _verbose
            };

            _assemblyParts = parts;
            
            _partDataMap = new Dictionary<string, PartData>();
            for (var i = 0; i < parts.Length; i++)
            {
                var id = i.ToString();
                var mesh = parts[i].PartObject.GetComponentInChildren<MeshFilter>().sharedMesh;
                var sdf = new SignedDistanceField(parts[i].PartObject, _configuration.SDFCollisionConfiguration);
                var pointCloud = PointCloudSampler.GetPointCloud(mesh.vertices, mesh.triangles, 
                    _configuration.PhysicsSimulationConfiguration.SimulationContactPointCount,
                    PointCloudSampler.SampleMethod.WeightedBarycentricCoordinates, 
                    false, new Matrix4x4(), _randomSeed);
                
                _partDataMap.Add(id, new PartData(id, parts[i].PartObject, sdf, pointCloud));
            }
            
            await InitializeSignedDistanceFields();
        }
        
        public List<Path> RunPlanner()
        {
            var assemblyPlanner = new ProgressiveQueueSequencePlanner(_partDataMap, _configuration); 
            var (status, sequence, seqCount, totalDuration) = assemblyPlanner.PlanDisassemblySequence();

            if (_configuration.Verbose)
            {
                Debug.Log($"AssemblyPlanner: Finished planning with status '{status}' and random seed '{_randomSeed}'", this);
            }
            
            if (_configuration.UsePathSimplifier)
            {
                var simplifiedSequence = SimplifySequence(sequence, _partDataMap, _configuration, _configuration.DisassemblyUseRotation, true);
                sequence.Clear();
                sequence = simplifiedSequence;
                GC.Collect(); // TODO
            }
            
            if (_configuration.UseRRTConnect)
            {
                PhysicsSimulation rrtPhysicsSimulation = null;
                if (_configuration.UsePathSimplifier)
                {
                    rrtPhysicsSimulation = new PhysicsSimulation(_partDataMap,
                        _configuration.RRTConfiguration.RRTUseRotation, new PhysicsSimulationConfiguration()
                        {
                            SimulationContactPointCount = 0
                        });
                }
                
                for (var i = 0; i < sequence.Count; i++)
                {
                    var rrtPath = PlanRRTConnectPath(i, sequence, _assemblyParts, _configuration, _randomSeed);
                    if (rrtPath == null)
                    {
                        Debug.LogError($"AssemblyPlanner: Couldn't find RRT-Connect path to final state for part '{sequence[i].PartID}'", this);
                        continue;
                    }

                    if (_configuration.UsePathSimplifier)
                    {
                        var simplifiedPath = SimplifyPath(rrtPath, rrtPhysicsSimulation, _configuration, _configuration.RRTConfiguration.RRTUseRotation, false);
                
                        sequence[i].Positions.AddRange(simplifiedPath.Positions);
                        sequence[i].Orientations.AddRange(simplifiedPath.Orientations);
                    }
                    else
                    {
                        sequence[i].Positions.AddRange(rrtPath.Positions);
                        sequence[i].Orientations.AddRange(rrtPath.Orientations);
                    }
                }
            }

            return sequence;
        }
        
        private async Task InitializeSignedDistanceFields()
        {
            Debug.Log("Start Initialize Signed Distance Fields");

            foreach (var partData in _partDataMap.Values)
            {
                await partData.SDF.ComputeSDF();
            }

            Debug.Log("Finished Initialize Signed Distance Fields");
        }
        
        private List<Path> SimplifySequence(List<Path> sequence, Dictionary<string, PartData> partDataMap, 
            AssemblyPlanningConfiguration configuration, bool useRotation = false, bool useSDFCollision = true)
        {
            var physicsSimulation = new PhysicsSimulation(partDataMap, useRotation, configuration.PhysicsSimulationConfiguration);
            
            var simplifiedSequence = new List<Path>();
            foreach (var partPath in sequence)
            {
                var simplifiedPath = SimplifyPath(partPath, physicsSimulation, configuration, useRotation, useSDFCollision);
                simplifiedSequence.Add(simplifiedPath);
            }

            return simplifiedSequence;
        }
        
        private Path SimplifyPath(Path partPath, PhysicsSimulation physicsSimulation, 
            AssemblyPlanningConfiguration configuration, bool useRotation = false, bool useSDFCollision = true)
        {
            var pathSimplifier = new PathSimplifier(physicsSimulation, partPath.PartID, configuration.SimplifierConfiguration);
            var simplifiedPath = pathSimplifier.SimplifyPath(partPath, useRotation, useSDFCollision, configuration.Verbose);

            if (configuration.Verbose)
            {
                Debug.Log($"AssemblyPlanner: Finished simplifying part '{partPath.PartID}' from {partPath.Positions.Count} states to {simplifiedPath.Positions.Count} states", this);
            }
            
            return simplifiedPath;
        }

        private Path PlanRRTConnectPath(int currentPartIndex, List<Path> sequence, AssemblyPart[] parts, 
            AssemblyPlanningConfiguration configuration, int randomSeed)
        {
            var otherObjects = new Transform[sequence.Count - 1];
            var otherObjectStates = new State[sequence.Count - 1];
            for (var j = 0; j < sequence.Count; j++)
            {
                if (j == currentPartIndex)
                {
                    continue;
                }
                
                if (j < currentPartIndex)
                {
                    otherObjects[j] = sequence[j].PartObject;
                    
                    var otherPart = parts.First(p => p.PartObject == sequence[j].PartObject);
                    otherObjectStates[j] = new State(
                        otherPart.PartFinalState.GetComponentInChildren<Renderer>().bounds.center,
                        otherPart.PartFinalState.position,
                        otherPart.PartFinalState.rotation,
                        Vector3.zero,
                        Vector3.zero);
                }
                else
                {
                    var otherObjectPath = sequence[j];
                    
                    otherObjects[j - 1] = otherObjectPath.PartObject;
                    
                    var otherPartPivot = otherObjectPath.PartObject.position;
                    var otherPartCenter = otherObjectPath.PartObject.GetComponentInChildren<Renderer>().bounds.center;
                    var otherPartCenterOffset = otherPartCenter - otherPartPivot;
                    var firstPartPivotPosition = otherObjectPath.Positions[0];
                    otherObjectStates[j - 1] = new State(
                        firstPartPivotPosition + otherPartCenterOffset,
                        firstPartPivotPosition,
                        otherObjectPath.Orientations[0],
                        Vector3.zero,
                        Vector3.zero);
                }
            }

            var partPath = sequence[currentPartIndex];
            
            var partPivot = partPath.PartObject.position;
            var partCenter = partPath.PartObject.GetComponentInChildren<Renderer>().bounds.center;
            var centerOffset = partCenter - partPivot;
            var lastPartPivotPosition = partPath.Positions.Last();
            var startState = new State(
                lastPartPivotPosition + centerOffset,
                lastPartPivotPosition,
                partPath.Orientations.Last(),
                Vector3.zero,
                Vector3.zero);

            var part = parts.First(p => p.PartObject == partPath.PartObject);
            var goalState = new State(
                part.PartFinalState.GetComponentInChildren<Renderer>().bounds.center,
                part.PartFinalState.position,
                part.PartFinalState.rotation,
                Vector3.zero,
                Vector3.zero);
            
            var rrtConnect = new RRTConnectPlanner(partPath.PartID, partPath.PartObject, otherObjects, configuration.RRTConfiguration);
            return rrtConnect.PlanPath(startState, goalState, otherObjectStates, randomSeed);
        }
    }
}