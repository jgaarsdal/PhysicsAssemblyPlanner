using System.Linq;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class TestAssemblyPlanner : MonoBehaviour
    {
        [Header("Assembly Planning Settings")]
        [SerializeField] private GameObject _assemblyRoot = default;
        [SerializeField] private bool _useRotation = false;
        [SerializeField] private float _assemblyTimeoutSecs = 600f;
        [SerializeField] private float _partTimeoutSecs = 60f;
        [SerializeField] private bool _verbose = true;
        
        [Header("BFSPlanner Settings")]
        [SerializeField] private float _bfsStatePositionThreshold = 0.05f;
        [SerializeField] private float _bfsStateAngleThreshold = 0.5f;
        
        [Header("Physics Simulation Settings")]
        [SerializeField] private float _simulationForce = 50f;
        [SerializeField] private float _simulationTimeStep = 0.001f; //0.016f; 
        [SerializeField] private int _simulationFrameSkip = 100;
        [SerializeField] private float _simulationContactStiffness = 1f;
        [SerializeField] private float _simulationContactDamping = 0f;
        
        [Header("SDF Collision Settings")]
        [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private float _sdfCollisionPenetrationThreshold = 0.01f;
        [SerializeField] private bool _useGPU = true;
        
        
        private ProgressiveQueueSequencePlanner _assemblyPlanner;
    
        private async void Start()
        {
            var assemblyParts = _assemblyRoot.GetComponentsInChildren<MeshFilter>()
                .Select(p => p.gameObject)
                .ToArray();

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
            
            _assemblyPlanner = new ProgressiveQueueSequencePlanner(assemblyParts, configuration);
            await _assemblyPlanner.InitializeSignedDistanceFields();
        
            Debug.Log("Finished initializing");
        }
    
        [ContextMenu("Run Planner")]
        public void RunAssemblyPlannerButton()
        {
            var (status, sequence, seqCount, totalDuration) = _assemblyPlanner.PlanSequence(3);
            Debug.Log(status);
        }
    }
}