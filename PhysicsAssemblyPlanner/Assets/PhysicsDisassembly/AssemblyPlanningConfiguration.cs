
namespace PhysicsDisassembly
{
    public struct AssemblyPlanningConfiguration
    {
        public bool DisassemblyUseRotation { get; set; }
        public BFSPlannerConfiguration BFSPlannerConfiguration { get; set; }
        public float AssemblyTimeoutSecs { get; set; }
        public float PartTimeoutSecs { get; set; }
        public PhysicsSimulationConfiguration PhysicsSimulationConfiguration { get; set; }
        public SDFCollisionConfiguration SDFCollisionConfiguration { get; set; }
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
    }
    
    public struct SDFCollisionConfiguration
    {
        public float SDFDefaultCellSize { get; set; }
        public float SDFBoxPadding { get; set; }
        public float SDFCollisionPenetrationThreshold { get; set; }
        public bool SDFUseGPU { get; set; }
    }
}