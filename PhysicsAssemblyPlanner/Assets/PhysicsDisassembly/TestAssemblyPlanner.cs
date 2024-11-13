using System;
using System.Collections.Generic;
using DG.Tweening;
using PhysicsDisassembly.Simulation;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class TestAssemblyPlanner : MonoBehaviour
    {
        [Header("Assembly Planning Settings")] [SerializeField]
        private GameObject _assemblyRoot = default;

        [SerializeField] private bool _useRotation = false;
        [SerializeField] private float _assemblyTimeoutSecs = 600f;
        [SerializeField] private float _partTimeoutSecs = 30f;
        [SerializeField] private bool _useRandomSeed = true;
        [SerializeField] private int _fixedSeed = 896;
        [SerializeField] private bool _verbose = true;

        [Header("BFSPlanner Settings")] [SerializeField]
        private float _bfsStatePositionThreshold = 0.05f;

        [SerializeField] private float _bfsStateAngleThreshold = 0.5f;

        [Header("Physics Simulation Settings")] [SerializeField]
        private float _simulationForce = 5f;

        [SerializeField] private float _simulationTorque = 1f;
        [SerializeField] private float _simulationTimeStep = 0.0005f;
        [SerializeField] private int _simulationFrameSkip = 90;
        [SerializeField] private float _simulationContactStiffness = 15f;
        [SerializeField] private float _simulationContactDamping = 0f;
        [SerializeField] private float _simulationMaxVelocity = 0.01f;
        [SerializeField] private float _simulationMaxAngularVelocity = 0.01f;
        [SerializeField] private int _simulationContactPointCount = 1024;
        [SerializeField] private float _simulationCollisionThreshold = 0f;

        [Header("SDF Collision Settings")] [SerializeField]
        private float _sdfDefaultCellSize = 0.05f;

        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private bool _useGPU = true;

        [Header("RRT-Connect Settings")] [SerializeField]
        private float _rrtStepSize = 0.1f;

        [SerializeField] private float _rrtRotationStepSize = 15f; // degrees
        [SerializeField] private int _rrtMaxIterations = 10000;
        [SerializeField] private float _rrtConnectDistance = 1.0f;
        [SerializeField] private int _rrtRandomPointAttempts = 10;

        [Header("Path Simplifier Settings")] 
        [SerializeField] private float _minimumProgressThreshold = 0.1f;
        [SerializeField] private int _transitionTestSteps = 10;

        private ProgressiveQueueSequencePlanner _assemblyPlanner;
        private Sequence _disassemblyTweenSequence;
        private List<Path> _disassemblySequence;

        private void Start()
        {
            DOTween.Init();
        }

        [ContextMenu("Run Planner")]
        public async void RunAssemblyPlannerButton()
        {
            var configuration = new AssemblyPlanningConfiguration()
            {
                DisassemblyUseRotation = _useRotation,
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
                    RRTStepSize = _rrtStepSize,
                    RRTRotationStepSize = _rrtRotationStepSize,
                    RRTMaxIterations = _rrtMaxIterations,
                    RRTConnectDistance = _rrtConnectDistance,
                    RRTRandomPointAttempts = _rrtRandomPointAttempts
                },
                Verbose = _verbose
            };

            _assemblyPlanner = new ProgressiveQueueSequencePlanner(_assemblyRoot, configuration);
            await _assemblyPlanner.InitializeSignedDistanceFields();

            var randomSeed = _useRandomSeed ? DateTime.Now.Millisecond : _fixedSeed;
            var (status, sequence, seqCount, totalDuration) = _assemblyPlanner.PlanSequence(randomSeed);

            Debug.Log($"Finished planning with status '{status}' and random seed '{randomSeed}'");

            var physicsSimulation = new PhysicsSimulation(_assemblyPlanner.PartObjects, _assemblyPlanner.PartSDFs,
                _useRotation, configuration.PhysicsSimulationConfiguration);
            var simplifiedSequence = new List<Path>();
            foreach (var partPath in sequence)
            {
                var pathSimplifier = new PathSimplifier(physicsSimulation, partPath.PartID, 
                    _minimumProgressThreshold, _transitionTestSteps, _verbose);
                var simplifiedPath = pathSimplifier.SimplifyPath(partPath, _useRotation, true);

                simplifiedSequence.Add(simplifiedPath);

                Debug.Log(
                    $"Finished simplifying part '{partPath.PartID}' from {partPath.Positions.Count} states to {simplifiedPath.Positions.Count} states");
            }

            _disassemblySequence = simplifiedSequence;

            ClearTweens();

            _disassemblyTweenSequence = DOTween.Sequence();

            for (var i = 0; i < _disassemblySequence.Count; i++)
            {
                var sequenceTransform = _disassemblySequence[i].PartObject.transform;
                var positions = _disassemblySequence[i].Positions;
                var rotations = _disassemblySequence[i].Orientations;

                Debug.Log($"Sequence #{i} has {positions.Count} positions/rotations");

                var stepInterval = positions.Count > 200
                    ? Mathf.FloorToInt(positions.Count / 100f)
                    : 1;

                var pointDuration = positions.Count > 200
                    ? 4f / (positions.Count / 100f)
                    : 4f / positions.Count;

                for (var j = 0; j < positions.Count; j += stepInterval)
                {
                    // Add position and rotation tweens to run in parallel
                    _disassemblyTweenSequence.Append(
                            sequenceTransform.DOMove(positions[j], pointDuration)
                                .SetEase(Ease.InOutCubic))
                        .Join(sequenceTransform.DORotateQuaternion(rotations[j], pointDuration)
                            .SetEase(Ease.InOutCubic));
                }
            }

            _disassemblyTweenSequence.AppendInterval(1f).SetLoops(-1, LoopType.Yoyo).Play();
        }

        [ContextMenu("Reset Planner")]
        public void ResetAssemblyPlannerButton()
        {
            ClearTweens();

            if (_disassemblySequence == null)
            {
                return;
            }

            foreach (var disassembly in _disassemblySequence)
            {
                disassembly.PartObject.transform.position = disassembly.Positions[0];
                disassembly.PartObject.transform.rotation = disassembly.Orientations[0];
            }
        }

        private void ClearTweens()
        {
            if (_disassemblyTweenSequence == null)
            {
                return;
            }

            _disassemblyTweenSequence.Kill();
            _disassemblyTweenSequence = null;
        }
    }
}