using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using PhysicsDisassembly.Simulation;
using PhysicsDisassembly.RRTConnect;
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
        [SerializeField] private float _minimumProgressThreshold = 0.1f; // TODO: or 0.01? Check TestRRTConnect with 0.1 to see
        [SerializeField] private int _transitionTestSteps = 10;
        
        public async Task<List<Path>> RunPlanner(AssemblyPart[] parts)
        {
            var configuration = new AssemblyPlanningConfiguration()
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
                    SimplifierTransitionTestSteps = _transitionTestSteps
                },
                Verbose = _verbose
            };

            return await RunPlanner(parts, configuration);
        }
        
        public async Task<List<Path>> RunPlanner(AssemblyPart[] parts, AssemblyPlanningConfiguration configuration)
        {
            var assemblyPlanner = new ProgressiveQueueSequencePlanner(parts, configuration);
            await assemblyPlanner.InitializeSignedDistanceFields();
            
            var randomSeed = _useRandomSeed ? DateTime.Now.Millisecond : _fixedSeed;
            var (status, sequence, seqCount, totalDuration) = assemblyPlanner.PlanSequence(randomSeed);

            if (configuration.Verbose)
            {
                Debug.Log($"AssemblyPlanner: Finished planning with status '{status}' and random seed '{randomSeed}'", this);
            }
            
            if (configuration.UsePathSimplifier)
            {
                var physicsSimulation = new PhysicsSimulation(assemblyPlanner.PartObjects, assemblyPlanner.PartSDFs,
                    _useRotation, configuration.PhysicsSimulationConfiguration);
                
                var simplifiedSequence = new List<Path>();
                foreach (var partPath in sequence)
                {
                    var pathSimplifier = new PathSimplifier(physicsSimulation, partPath.PartID, configuration.SimplifierConfiguration);
                    var simplifiedPath = pathSimplifier.SimplifyPath(partPath, configuration.DisassemblyUseRotation, true, configuration.Verbose);

                    simplifiedSequence.Add(simplifiedPath);

                    if (configuration.Verbose)
                    {
                        Debug.Log($"AssemblyPlanner: Finished simplifying part '{partPath.PartID}' from {partPath.Positions.Count} states to {simplifiedPath.Positions.Count} states", this);
                    }
                }

                sequence = simplifiedSequence;
            }
            
            if (configuration.UseRRTConnect)
            
                // TODO: Issue is that sequence count is 1 less than the total number of parts.
                // TODO: So if there are 2 parts sequence count will be 1
                
                Debug.LogError("sequence.Count is " + sequence.Count);
                
                for (var i = 0; i < sequence.Count; i++)
                {
                    var otherObjects = new Transform[sequence.Count - 1];
                    var otherObjectStates = new State[sequence.Count - 1];
                    for (var j = 0; j < sequence.Count; j++)
                    {
                        if (j == i)
                        {
                            continue;
                        }
                        
                        if (j < i)
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
                            
                            otherObjectStates[j - 1] = new State(
                                otherObjectPath.PartObject.GetComponentInChildren<Renderer>().bounds.center,
                                otherObjectPath.Positions[0],
                                otherObjectPath.Orientations[0],
                                Vector3.zero,
                                Vector3.zero);
                        }
                    }

                    var partPath = sequence[i];
                    
                    var startState = new State(
                        partPath.PartObject.GetComponentInChildren<Renderer>().bounds.center,
                        partPath.Positions.Last(),
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


                    var rrtConnect = new RRTConnectPlanner(partPath.PartID, partPath.PartObject, otherObjects,
                        configuration.RRTConfiguration);
                    var rrtPath = rrtConnect.PlanPath(startState, goalState, otherObjectStates, randomSeed);
                    if (rrtPath == null)
                    {
                        Debug.LogError($"AssemblyPlanner: Couldn't find RRT-Connect path to final state for part '{partPath.PartID}'", this);
                        continue;
                    }
                    
                    // TODO: Only use path simplifier once after both planning and RRT-Connect?
                    
                    if (configuration.UsePathSimplifier)
                    {
                        var physicsSimulation = new PhysicsSimulation(assemblyPlanner.PartObjects, null,
                            configuration.RRTConfiguration.RRTUseRotation, new PhysicsSimulationConfiguration()
                            {
                                SimulationContactPointCount = 0
                            });
                        
                        var pathSimplifier = new PathSimplifier(physicsSimulation, rrtPath.PartID,
                            configuration.SimplifierConfiguration);
                        var simplifiedPath = pathSimplifier.SimplifyPath(rrtPath, configuration.RRTConfiguration.RRTUseRotation, 
                            false, configuration.Verbose);

                        partPath.Positions.AddRange(simplifiedPath.Positions);
                        partPath.Orientations.AddRange(simplifiedPath.Orientations);

                        if (configuration.Verbose)
                        {
                            Debug.Log($"AssemblyPlanner: Finished simplifying part '{partPath.PartID}' from {partPath.Positions.Count} states to {simplifiedPath.Positions.Count} states", this);
                        }
                    }
                    else
                    {
                        partPath.Positions.AddRange(rrtPath.Positions);
                        partPath.Orientations.AddRange(rrtPath.Orientations);
                    }
                }
            }

            return sequence;
        }
    }
}