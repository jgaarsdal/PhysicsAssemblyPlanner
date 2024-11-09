using PhysicsDisassembly.SDF;
using UnityEngine;

namespace PhysicsDisassembly.Simulation
{
    public class SDFCollisionPart
    {
        private SignedDistanceField _sdf;
        private PhysicsSimulation _simulation;
        private PhysicsSimulationConfiguration _configuration;
        private string _partId;
        private readonly Vector3 _gridOriginOffset; // Offset from object position to SDF grid origin

        public SDFCollisionPart(string partId, PhysicsSimulation simulation, SignedDistanceField sdf, PhysicsSimulationConfiguration configuration)
        {
            _partId = partId;
            _simulation = simulation;
            _sdf = sdf;
            _configuration = configuration;
        }
        
        public bool CheckCollision(SDFCollisionPart other)
        {
            var contactPoints = _simulation.GetContactPoints(_partId);
            foreach (var contactPoint in contactPoints)
            {
                // Transform vertex to other object's local space and get grid position
                var gridPos = other._sdf.WorldToGridPosition(contactPoint);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = other._sdf.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);

                // Only process if there's penetration
                if (penetrationDistance < _configuration.SimulationCollisionThreshold)
                {
                    return true;
                }
            }

            return false;
        }
        
        public (Vector3 force, Vector3 torque) CheckAndResolveCollision(SDFCollisionPart other, bool addTorque = false)
        {
            return (CheckAndResolveCollisionTranslationOnly(other), Vector3.zero);
            
            /*
            return addTorque
                ? CheckAndResolveCollisionWithTorque(other)
                : (CheckAndResolveCollisionTranslationOnly(other), Vector3.zero);
            */
        }
        
        private Vector3 CheckAndResolveCollisionTranslationOnly(SDFCollisionPart other)
        {
            var totalForce = Vector3.zero;
            
            var contactPoints = _simulation.GetContactPoints(_partId);
            var velocity = _simulation.GetVelocity(_partId);
            
            foreach (var contactPoint in contactPoints)
            {
                // Transform vertex to other object's local space and get grid position
                var gridPos = other._sdf.WorldToGridPosition(contactPoint);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = other._sdf.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);

                // Only process if there's penetration
                if (penetrationDistance < _configuration.SimulationCollisionThreshold)
                {
                    // Calculate contact normal (gradient of SDF)
                    var normal = CalculateSDFGradient(other, contactPoint);
                    
                    // Calculate penetration speed (ḋ = ∇g(x) · ẋ from paper)
                    var penetrationSpeed = Vector3.Dot(normal, velocity);

                    // Calculate contact force using penalty model from paper:
                    // f_c = (-k_n + k_d * ḋ)d * n
                    var contactForce = ((-_configuration.SimulationContactStiffness + _configuration.SimulationContactDamping 
                        * penetrationSpeed) * penetrationDistance) * normal;
                    
                    // Apply force
                    totalForce += contactForce;
                }
            }

            return totalForce;
        }
        
        private (Vector3 force, Vector3 torque) CheckAndResolveCollisionWithTorque(SDFCollisionPart other)
        {
            var totalForce = Vector3.zero;
            var totalTorque = Vector3.zero;
            
            var contactPoints = _simulation.GetContactPoints(_partId);
            var velocity = _simulation.GetVelocity(_partId);
            var angularVelocity = _simulation.GetAngularVelocity(_partId);
            
            // Center of mass position
            var centerPosition = _simulation.GetPosition(_partId);
            
            foreach (var contactPoint in contactPoints)
            {
                // Transform vertex to other object's local space and get grid position
                var gridPos = other._sdf.WorldToGridPosition(contactPoint);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = other._sdf.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);

                // Only process if there's penetration
                if (penetrationDistance < _configuration.SimulationCollisionThreshold)
                {
                    // Calculate contact normal (gradient of SDF)
                    var normal = CalculateSDFGradient(other, contactPoint);
                 
                    // Calculate moment arm from center to contact point
                    var r = contactPoint - centerPosition;
        
                    // Calculate point velocity including angular contribution
                    var pointVelocity = velocity + Vector3.Cross(angularVelocity, r);
                    
                    // Calculate penetration speed (ḋ = ∇g(x) · ẋ from paper)
                    var penetrationSpeed = Vector3.Dot(normal, pointVelocity);

                    // Calculate contact force using penalty model from paper:
                    // f_c = (-k_n + k_d * ḋ)d * n
                    var contactForce = ((-_configuration.SimulationContactStiffness + _configuration.SimulationContactDamping 
                        * penetrationSpeed) * penetrationDistance) * normal;

                    // Calculate torque for this contact point: τ = r × F
                    var contactTorque = Vector3.Cross(r, contactForce);
                    
                    // Apply force and torque
                    totalForce += contactForce;
                    totalTorque += contactTorque;
                }
            }

            return (totalForce, totalTorque);
        }

        private Vector3 CalculateSDFGradient(SDFCollisionPart other, Vector3 worldPos)
        {
            const float epsilon = 0.01f;

            // Sample the SDF at offset positions to compute gradient
            var dx = (SampleSDF(other, worldPos + Vector3.right * epsilon) -
                      SampleSDF(other, worldPos - Vector3.right * epsilon)) / (2f * epsilon);

            var dy = (SampleSDF(other, worldPos + Vector3.up * epsilon) -
                      SampleSDF(other, worldPos - Vector3.up * epsilon)) / (2f * epsilon);

            var dz = (SampleSDF(other, worldPos + Vector3.forward * epsilon) -
                      SampleSDF(other, worldPos - Vector3.forward * epsilon)) / (2f * epsilon);

            var gradient = new Vector3(dx, dy, dz);

            if (gradient.sqrMagnitude < 1e-6f)
            {
                // If gradient is too small, use direction to center as fallback
                var thisCenter = GetObjectCenter();
                var otherCenter = other.GetObjectCenter();
                return (thisCenter - otherCenter).normalized;
            }

            return gradient.normalized;
        }

        private float SampleSDF(SDFCollisionPart other, Vector3 worldPos)
        {
            var gridPos = other._sdf.WorldToGridPosition(worldPos);
            return other._sdf.GetDistance(gridPos);
        }

        private Vector3 GetObjectCenter()
        {
            // Get the actual geometric center of the object using vertices
            var vertices = _simulation.GetVertices(_partId);
            var sum = Vector3.zero;

            foreach (var vertex in vertices)
            {
                sum += vertex;
            }

            return sum / vertices.Length;
        }
    }
}