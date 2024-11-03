using System;
using System.Collections.Generic;
using System.Linq;
using DG.Tweening;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class TestAssemblyPlanner : MonoBehaviour
    {
        [Header("Assembly Planning Settings")]
        [SerializeField] private GameObject _assemblyRoot = default;
        [SerializeField] private bool _useRotation = false;
        [SerializeField] private float _assemblyTimeoutSecs = 600f;
        [SerializeField] private float _partTimeoutSecs = 30f;
        [SerializeField] private bool _useRandomSeed = true;
        [SerializeField] private int _fixedSeed = 896;
        [SerializeField] private bool _verbose = true;
        
        [Header("BFSPlanner Settings")]
        [SerializeField] private float _bfsStatePositionThreshold = 0.05f;
        [SerializeField] private float _bfsStateAngleThreshold = 0.5f;
        
        [Header("Physics Simulation Settings")]
        [SerializeField] private float _simulationForce = 5f;
        [SerializeField] private float _simulationTimeStep = 0.0005f; 
        [SerializeField] private int _simulationFrameSkip = 90;
        [SerializeField] private float _simulationContactStiffness = 15f;
        [SerializeField] private float _simulationContactDamping = 0f;
        
        [Header("SDF Collision Settings")]
        [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private float _sdfCollisionPenetrationThreshold = 0.01f;
        [SerializeField] private bool _useGPU = true;
        
        private ProgressiveQueueSequencePlanner _assemblyPlanner;
        private Tween[] _disassemblyTweens;
        private List<Path> _disassemblySequence;
    
        [ContextMenu("Run Planner")]
        public async void RunAssemblyPlannerButton()
        {
            var configuration = new AssemblyPlanningConfiguration()
            {
                DisassemblyUseRotation = _useRotation,
                BFSPlannerConfiguration = new BFSPlannerConfiguration()
                {
                    BFSStatePositionThreshold = _bfsStatePositionThreshold,
                    BFSStateAngleThreshold = _bfsStateAngleThreshold,
                },
                AssemblyTimeoutSecs = _assemblyTimeoutSecs,
                PartTimeoutSecs = _partTimeoutSecs,
                PhysicsSimulationConfiguration = new PhysicsSimulationConfiguration()
                {
                    SimulationForce = _simulationForce,
                    SimulationTimeStep = _simulationTimeStep,
                    SimulationFrameSkip = _simulationFrameSkip,
                    SimulationContactStiffness = _simulationContactStiffness,
                    SimulationContactDamping = _simulationContactDamping,
                },
                SDFCollisionConfiguration = new SDFCollisionConfiguration()
                {
                    SDFDefaultCellSize = _sdfDefaultCellSize,
                    SDFBoxPadding = _sdfBoxPadding,
                    SDFCollisionPenetrationThreshold = _sdfCollisionPenetrationThreshold,
                    SDFUseGPU = _useGPU,
                },
                Verbose = _verbose
            };
            
            _assemblyPlanner = new ProgressiveQueueSequencePlanner(_assemblyRoot, configuration);
            await _assemblyPlanner.InitializeSignedDistanceFields();

            var randomSeed = _useRandomSeed ? DateTime.Now.Millisecond : _fixedSeed;
            var (status, sequence, seqCount, totalDuration) = _assemblyPlanner.PlanSequence(randomSeed);

            Debug.Log($"Finished planning with status '{status}' and random seed '{randomSeed}'");
            
            _disassemblySequence = sequence;
            
            ClearTweens();
            
            _disassemblyTweens = new Tween[_disassemblySequence.Count];
            for (var i = 0; i < _disassemblySequence.Count; i++)
            {
                _disassemblyTweens[i] = _disassemblySequence[i].PartObject.transform
                    .DOPath(_disassemblySequence[i].Positions.ToArray(), 5f)
                    .SetEase(Ease.InOutCubic)
                    .SetLoops(-1, LoopType.Yoyo)
                    .SetDelay(i > 0 ? 5f : 0f);
            }
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
            if (_disassemblyTweens == null)
            {
                return;
            }
            
            foreach (var tween in _disassemblyTweens)
            {
                if (tween != null && tween.IsActive())
                {
                    tween.Kill();
                }
            }
            
            _disassemblyTweens = null;
        }
    }
}