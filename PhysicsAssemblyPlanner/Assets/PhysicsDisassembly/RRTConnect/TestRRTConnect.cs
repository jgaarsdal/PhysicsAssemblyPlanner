using System;
using System.Collections.Generic;
using System.Linq;
using DG.Tweening;
using PhysicsDisassembly.Simulation;
using UnityEngine;
using Sequence = DG.Tweening.Sequence;

namespace PhysicsDisassembly.RRTConnect
{
    public class TestRRTConnect : MonoBehaviour
    {
        [Header("Test Objects")]
        [SerializeField] private Transform _testObject = default;
        [SerializeField] private Transform[] _otherObjects = default;
        [SerializeField] private Transform _testObjectEndState = default;
        
        [Header("RRT-Connect Settings")] 
        [SerializeField] private bool _rrtUseRotation = false;
        [SerializeField] private float _rrtStepSize = 0.1f;
        [SerializeField] private float _rrtRotationStepSize = 15f; // degrees
        [SerializeField] private int _rrtMaxIterations = 10000;
        [SerializeField] private float _rrtConnectDistance = 1.0f;
        [SerializeField] private int _rrtRandomPointAttempts = 10;
        [SerializeField] private float _rrtWorkspaceBoundsBufferPercentage = 0.5f;
        [SerializeField] [Range(0f, 1f)] private float _rrtExplorationBias = 0.5f;
        
        [Header("Path Simplifier Settings")]
        [SerializeField] private float _minimumProgressThreshold = 0.1f;
        [SerializeField] private int _transitionTestSteps = 10;

        private Sequence _rrtConnectSequence;
        
        [ContextMenu("Run Planner")]
        public async void RunAssemblyPlannerButton()
        {
            ClearTweens();
            
            var rrtConfiguration = new RRTConfiguration()
            {
                RRTUseRotation = _rrtUseRotation,
                RRTStepSize = _rrtStepSize,
                RRTRotationStepSize = _rrtRotationStepSize,
                RRTMaxIterations = _rrtMaxIterations,
                RRTConnectDistance = _rrtConnectDistance,
                RRTRandomPointAttempts = _rrtRandomPointAttempts,
                RRTWorkspaceBoundsBufferPercentage = _rrtWorkspaceBoundsBufferPercentage,
                RRTExplorationBias = _rrtExplorationBias
            };
            
            var startState = new State(
                _testObject.GetComponentInChildren<Renderer>().bounds.center, 
                _testObject.position, 
                _testObject.rotation, 
                Vector3.zero, 
                Vector3.zero);
            
            var goalState = new State(
                _testObjectEndState.GetComponentInChildren<Renderer>().bounds.center, 
                _testObjectEndState.position, 
                _testObjectEndState.rotation, 
                Vector3.zero, 
                Vector3.zero);

            var otherStates = _otherObjects.Select(o => 
                new State(
                    o.GetComponentInChildren<Renderer>().bounds.center,
                    o.position, 
                    o.rotation,
                    Vector3.zero, 
                    Vector3.zero))
                .ToArray();

            var randomSeed = DateTime.Now.Millisecond;
            var rrtConnect = new RRTConnectPlanner(_testObject.name, _testObject, _otherObjects, rrtConfiguration);
            var path = rrtConnect.PlanPath(startState, goalState, otherStates, randomSeed);
            if (path == null)
            {
                return;
            }

            Debug.Log($"TestRRTConnect found {path.Positions.Count} positions BEFORE simplification");
            
            var partObjects = new Dictionary<string, Transform>();
            partObjects.Add("0", _testObject);
            for (var i = 0; i < _otherObjects.Length; i++)
            {
                partObjects.Add((i + 1).ToString(), _otherObjects[i]);
            }
            
            var physicsSimulation = new PhysicsSimulation(partObjects, null, _rrtUseRotation, 
                new PhysicsSimulationConfiguration()
                {
                    SimulationContactPointCount = 0
                });
            
            var pathSimplifier = new PathSimplifier(physicsSimulation, path.PartID,
                new PathSimplifierConfiguration()
                {
                    SimplifierMinimumProgressThreshold = _minimumProgressThreshold, 
                    SimplifierTransitionTestSteps = _transitionTestSteps
                });
            
            path = pathSimplifier.SimplifyPath(path, _rrtUseRotation, false, true);
            
            var pathSize = path.Positions.Count;
            Debug.Log($"TestRRTConnect found {pathSize} positions AFTER simplification");
            
            _rrtConnectSequence = DOTween.Sequence();
            
            var pointDuration = 4f / pathSize;
            
            for (var i = 0; i < path.Positions.Count; i++)
            {
                _rrtConnectSequence.Append(
                        _testObject.DOMove(path.Positions[i], pointDuration)
                            .SetEase(Ease.InOutCubic))
                    .Join(_testObject.DORotateQuaternion(path.Orientations[i], pointDuration)
                        .SetEase(Ease.InOutCubic));
            }
            
            _rrtConnectSequence.SetLoops(-1, LoopType.Yoyo).Play();
        }

        private void OnDestroy()
        {
            ClearTweens();
        }

        private void ClearTweens()
        {
            if (_rrtConnectSequence == null)
            {
                return;
            }

            _rrtConnectSequence.Kill();
            _rrtConnectSequence = null;
        }
    }
}