
namespace PhysicsDisassembly
{
    public struct AssemblyPlanningConfiguration
    {
        public bool DisassemblyUseRotation { get; set; }
        public bool UseRRTConnect { get; set; }
        public bool UsePathSimplifier { get; set; }
        public BFSPlannerConfiguration BFSPlannerConfiguration { get; set; }
        public float AssemblyTimeoutSecs { get; set; }
        public float PartTimeoutSecs { get; set; }
        public PhysicsSimulationConfiguration PhysicsSimulationConfiguration { get; set; }
        public SDFCollisionConfiguration SDFCollisionConfiguration { get; set; }
        public RRTConfiguration RRTConfiguration { get; set; }
        public PathSimplifierConfiguration SimplifierConfiguration { get; set; }
        public bool Verbose { get; set; }
    }
    
    public struct BFSPlannerConfiguration
    {
        public float BFSStatePositionThreshold;
        public float BFSStateAngleThreshold;
    }
    
    public struct PhysicsSimulationConfiguration
    {
        public float SimulationForce { get; set; }
        public float SimulationTorque { get; set; }
        public float SimulationTimeStep { get; set; }
        public int SimulationFrameSkip { get; set; }
        public float SimulationContactStiffness { get; set; }
        public float SimulationContactDamping { get; set; }
        public float SimulationMaxVelocity { get; set; }
        public float SimulationMaxAngularVelocity { get; set; }
        public int SimulationContactPointCount { get; set; }
        public float SimulationCollisionThreshold { get; set; }
    }
    
    public struct SDFCollisionConfiguration
    {
        public float SDFDefaultCellSize { get; set; }
        public float SDFBoxPadding { get; set; }
        public bool SDFUseGPU { get; set; }
    }

    public struct RRTConfiguration
    {
        public bool RRTUseRotation { get; set; }
        public float RRTStepSize { get; set; }
        public float RRTRotationStepSize { get; set; }
        public int RRTMaxIterations { get; set; }
        public float RRTConnectDistance { get; set; }
        public int RRTRandomPointAttempts { get; set; }
        public float RRTExplorationBias { get; set; } // 0 = pure exploitation, 1 = pure exploration
        public float RRTWorkspaceBoundsBufferPercentage { get; set; }
    }

    public struct PathSimplifierConfiguration
    {
        public float SimplifierMinimumProgressThreshold { get; set; }
        public float SimplifierMaximumProgressThreshold { get; set; }
        public int SimplifierTransitionTestSteps { get; set; }
    }
}