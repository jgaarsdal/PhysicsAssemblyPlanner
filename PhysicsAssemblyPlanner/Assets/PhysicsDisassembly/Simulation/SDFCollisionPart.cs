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

        // TODO: Try to return a force instead in order to parallelize (and possibly move to GPU)
        
        public Vector3 CheckAndResolveCollision(SDFCollisionPart other)
        {
            var force = Vector3.zero;
            
            // Sample contact points using vertices of this part
            var vertices = _simulation.GetVertices(_partId);
            var velocity = _simulation.GetVelocity(_partId);

            /*var triangles = _sdf.Triangles;
            var tris = new int[triangles.Length * 3];

            for (var i = 0; i < triangles.Length; i++)
            {
                tris[i * 3] = triangles[i].x;
                tris[i * 3 + 1] = triangles[i].y;
                tris[i * 3 + 2] = triangles[i].z;
            }
            
            var contactPoints = PointCloudSampler.GetPointCloud(vertices, tris, 1024,
                PointCloudSampler.SampleMethod.WeightedBarycentricCoordinates, false);*/
            
            
            foreach (var vertex in vertices)
            {
                // Get point velocity including angular contribution
                var pointVelocity = velocity;

                // Transform vertex to other object's local space and get grid position
                var gridPos = other._sdf.WorldToGridPosition(vertex);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = other._sdf.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);

                // Only process if there's penetration
                if (penetrationDistance < 0f)
                {
                    // Calculate contact normal (gradient of SDF)
                    var normal = CalculateSDFGradient(other, vertex);
                    
                    // Calculate penetration speed (ḋ = ∇g(x) · ẋ from paper)
                    var penetrationSpeed = Vector3.Dot(normal, pointVelocity);

                    // Calculate contact force using penalty model from paper:
                    // f_c = (-k_n + k_d * ḋ)d * n
                    var contactForce = ((-_configuration.SimulationContactStiffness + _configuration.SimulationContactDamping * penetrationSpeed) * penetrationDistance) * normal;

                    // Apply force and torque
                    force += contactForce;
                }
            }

            return force;

            /*
            // Sample contact points using vertices of this part
            var vertices = _simulation.GetVertices(_partId);
            var velocity = _simulation.GetVelocity(_partId);

            foreach (var vertex in vertices)
            {
                // Get point velocity including angular contribution
                var pointVelocity = velocity;

                // Transform vertex to other object's local space and get grid position
                var gridPos = other._sdf.WorldToGridPosition(vertex);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = other._sdf.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);

                // Only process if there's penetration
                if (penetrationDistance < 0f)
                {
                    // Calculate contact normal (gradient of SDF)
                    var normal = CalculateSDFGradient(other, vertex);

                    // Calculate penetration speed (ḋ = ∇g(x) · ẋ from paper)
                    var penetrationSpeed = Vector3.Dot(normal, pointVelocity);

                    // Calculate contact force using penalty model from paper:
                    // f_c = (-k_n + k_d * ḋ)d * n
                    var contactForce = ((-_configuration.SimulationContactStiffness + _configuration.SimulationContactDamping * penetrationSpeed) * penetrationDistance) * normal;

                    // Apply force and torque
                    _simulation.ApplyForceAtPoint(_partId, contactForce, vertex);
                }
            }
            */
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