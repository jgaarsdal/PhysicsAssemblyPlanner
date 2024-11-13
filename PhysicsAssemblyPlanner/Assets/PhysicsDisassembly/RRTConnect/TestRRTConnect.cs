using System;
using System.Collections.Generic;
using System.Linq;
using DG.Tweening;
using PhysicsDisassembly.Simulation;
using UnityEngine;
using Sequence = DG.Tweening.Sequence;

namespace PhysicsDisassembly.RRTConnect
{
    // TODO: Use OBB for collision
    
    public class TestRRTConnect : MonoBehaviour
    {
        [Header("Test Objects")]
        [SerializeField] private GameObject _testObject = default;
        [SerializeField] private GameObject[] _otherObjects = default;
        [SerializeField] private GameObject _testObjectEndState = default;
        
        [Header("RRT-Connect Settings")] 
        [SerializeField] private bool _rrtUseRotation = false;
        [SerializeField] private float _rrtStepSize = 0.1f;
        [SerializeField] private float _rrtRotationStepSize = 15f; // degrees
        [SerializeField] private int _rrtMaxIterations = 10000;
        [SerializeField] private float _rrtConnectDistance = 1.0f;
        [SerializeField] private int _rrtRandomPointAttempts = 10;
        [SerializeField] private float _rrtWorkspaceBoundsBuffer = 0.5f;
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
                RRTWorkspaceBoundsBuffer = _rrtWorkspaceBoundsBuffer,
                RRTExplorationBias = _rrtExplorationBias
            };
            
            var startState = new State(
                _testObject.GetComponentInChildren<Renderer>().bounds.center, 
                _testObject.transform.position, 
                _testObject.transform.rotation, 
                Vector3.zero, 
                Vector3.zero);
            
            var goalState = new State(
                _testObjectEndState.GetComponentInChildren<Renderer>().bounds.center, 
                _testObjectEndState.transform.position, 
                _testObjectEndState.transform.rotation, 
                Vector3.zero, 
                Vector3.zero);

            var otherStates = _otherObjects.Select(o => 
                new State(
                    o.GetComponentInChildren<Renderer>().bounds.center,
                    o.transform.position, 
                    o.transform.rotation, 
                    Vector3.zero, 
                    Vector3.zero))
                .ToArray();

            var randomSeed = DateTime.Now.Millisecond;
            var rrtConnect = new RRTConnect(_testObject.name, _testObject, _otherObjects, rrtConfiguration);
            var path = rrtConnect.PlanPath(startState, goalState, otherStates, randomSeed);
            if (path == null)
            {
                return;
            }

            Debug.Log($"TestRRTConnect found {path.Positions.Count} positions BEFORE simplification");
            
            var partObjects = new Dictionary<string, GameObject>();
            partObjects.Add("0", _testObject);
            for (var i = 0; i < _otherObjects.Length; i++)
            {
                partObjects.Add((i + 1).ToString(), _otherObjects[i]);
            }

            var useRotation = false;
            var physicsSimulation = new PhysicsSimulation(partObjects, null, _rrtUseRotation, 
                new PhysicsSimulationConfiguration() { SimulationContactPointCount = 0 });
            var pathSimplifier = new PathSimplifier(physicsSimulation, path.PartID, 
                _minimumProgressThreshold, _transitionTestSteps, true);
            
            path = pathSimplifier.SimplifyPath(path, _rrtUseRotation, false);
            
            var pathSize = path.Positions.Count;
            Debug.Log($"TestRRTConnect found {pathSize} positions AFTER simplification");
            
            _rrtConnectSequence = DOTween.Sequence();
            
            var pointDuration = 4f / pathSize;
            
            for (var i = 0; i < path.Positions.Count; i++)
            {
                _rrtConnectSequence.Append(
                        _testObject.transform.DOMove(path.Positions[i], pointDuration)
                            .SetEase(Ease.InOutCubic))
                    .Join(_testObject.transform.DORotateQuaternion(path.Orientations[i], pointDuration)
                        .SetEase(Ease.InOutCubic));
            }
            
            _rrtConnectSequence.SetLoops(-1, LoopType.Yoyo).Play();
        }
        
        private void DrawCell(Bounds b, Color color, float delay = 60f)
        {
            // bottom
            var p1 = new Vector3(b.min.x, b.min.y, b.min.z);
            var p2 = new Vector3(b.max.x, b.min.y, b.min.z);
            var p3 = new Vector3(b.max.x, b.min.y, b.max.z);
            var p4 = new Vector3(b.min.x, b.min.y, b.max.z);

            Debug.DrawLine(p1, p2, color, delay);
            Debug.DrawLine(p2, p3, color, delay);
            Debug.DrawLine(p3, p4, color, delay);
            Debug.DrawLine(p4, p1, color, delay);

            // top
            var p5 = new Vector3(b.min.x, b.max.y, b.min.z);
            var p6 = new Vector3(b.max.x, b.max.y, b.min.z);
            var p7 = new Vector3(b.max.x, b.max.y, b.max.z);
            var p8 = new Vector3(b.min.x, b.max.y, b.max.z);

            Debug.DrawLine(p5, p6, color, delay);
            Debug.DrawLine(p6, p7, color, delay);
            Debug.DrawLine(p7, p8, color, delay);
            Debug.DrawLine(p8, p5, color, delay);

            // sides
            Debug.DrawLine(p1, p5, color, delay);
            Debug.DrawLine(p2, p6, color, delay);
            Debug.DrawLine(p3, p7, color, delay);
            Debug.DrawLine(p4, p8, color, delay);
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