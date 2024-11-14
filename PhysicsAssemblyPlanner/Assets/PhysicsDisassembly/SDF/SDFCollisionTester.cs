using System.Threading.Tasks;
using PhysicsDisassembly.Simulation;
using UnityEngine;

namespace PhysicsDisassembly.SDF
{
    public class SDFCollisionTester : MonoBehaviour
    {
        [SerializeField] private Transform _partA = default;
        [SerializeField] private Transform _partB = default;
        [Space] [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private float _collisionPenetrationThreshold = 0f;
        [SerializeField] private int _collisionContactPointCount = 1024;
        [SerializeField] private bool _useGPU = true;
        [SerializeField] private bool _visualize = true;

        private SignedDistanceField _sdfPartA;
        private SignedDistanceField _sdfPartB;

        [ContextMenu("Compute SDF")]
        public void ComputeSDFButton()
        {
            ComputeSDF();
        }

        [ContextMenu("Test SDF Collision")]
        public void TestCollisionButton()
        {
            TestCollision();
        }

        private async void ComputeSDF()
        {
            _sdfPartA = new SignedDistanceField(_partA, _sdfDefaultCellSize, _sdfBoxPadding, _useGPU);
            _sdfPartB = new SignedDistanceField(_partB, _sdfDefaultCellSize, _sdfBoxPadding, _useGPU);

            await Task.WhenAll(_sdfPartA.ComputeSDF(), _sdfPartB.ComputeSDF());
        }

        private void TestCollision()
        {
            var isColliding = false;
            
            _sdfPartA.UpdateVertices();
            _sdfPartB.UpdateVertices();

            var physicsSimulationConfiguration = new PhysicsSimulationConfiguration()
            {
                SimulationContactPointCount = _collisionContactPointCount,
                SimulationCollisionThreshold = _collisionPenetrationThreshold
            };

            var partA = new Part(_partA.GetComponentInChildren<MeshFilter>().sharedMesh, _partA.transform.position,
                _partA.transform.rotation, _partA.transform.localScale, physicsSimulationConfiguration);
            
            var contactPoints = partA.GetContactPoints();
            
            foreach (var contactPoint in contactPoints)
            {
                // Transform vertex to other object's local space and get grid position
                var gridPos = _sdfPartB.WorldToGridPosition(contactPoint);

                // Get penetration distance (equation from paper: d = min(g(x), 0))
                var distance = _sdfPartB.GetDistance(gridPos);
                var penetrationDistance = Mathf.Min(distance, 0f);
                
                if (penetrationDistance < _collisionPenetrationThreshold)
                {
                    isColliding = true;
                    
                    var collisionNormal = CalculateCollisionNormal(contactPoint);

                    Debug.DrawLine(contactPoint, contactPoint + collisionNormal * 0.5f, Color.red, 60f);
                    Debug.Log("collisionNormal: " + collisionNormal);
                }
            }
            
            /*
            var verts = _sdfPartA.WorldVertices;
            var tris = new int[_sdfPartA.Triangles.Length * 3];

            for (var i = 0; i < _sdfPartA.Triangles.Length; i++)
            {
                tris[i * 3] = _sdfPartA.Triangles[i].x;
                tris[i * 3 + 1] = _sdfPartA.Triangles[i].y;
                tris[i * 3 + 2] = _sdfPartA.Triangles[i].z;
            }
            
            var contactPoints = PointCloudSampler.GetPointCloud(verts, tris, _collisionContactPointCount,
                PointCloudSampler.SampleMethod.WeightedBarycentricCoordinates, false);
        
            var isColliding = _sdfPartA.CheckCollision(_sdfPartB, contactPoints);
            if (isColliding)
            {
                var contactPoint = FindContactPoint(contactPoints);
                var collisionNormal = CalculateCollisionNormal(contactPoint);

                Debug.DrawLine(contactPoint, contactPoint + collisionNormal * 0.5f, Color.red, 60f);
                Debug.Log("collisionNormal: " + collisionNormal);
            }*/

            Debug.Log($"SDFCollisionTester: IS COLLIDING = {isColliding}", this);
        }

        /*
        private Vector3 FindContactPoint(Vector3[] contactPoints)
        {
            var minDistance = float.MaxValue;
            var bestPoint = Vector3.zero;
            var bestSDF = 0f;
            
            //if (_visualize)
            //{
            //    var center = GetObjectCenter(_sdfPartA.WorldVertices);
            //    foreach (var contactPoint in contactPoints)
            //    {
            //        Debug.DrawLine(center, contactPoint, Color.blue, 60f);
            //    }
            //}
            
            // Sample vertices to find the point of deepest penetration
            foreach (var vertex in contactPoints)
            {
                // Transform vertex to other object's local space
                var gridPos = _sdfPartB.WorldToGridPosition(vertex);
                var distance = _sdfPartB.GetDistance(gridPos);

                // Keep track of the point with smallest (most negative) distance
                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestPoint = vertex;
                    bestSDF = distance;
                }
            }

            if (Mathf.Approximately(minDistance, float.MaxValue))
            {
                // Fallback if no penetrating points found
                Debug.LogWarning("No penetrating points found in collision!");
                return (GetObjectCenter(_sdfPartA.WorldVertices) + GetObjectCenter(_sdfPartB.WorldVertices)) * 0.5f;
            }

            // Move the contact point to the surface using the SDF value
            var normal = CalculateCollisionNormal(bestPoint);
            return bestPoint - normal * bestSDF;
        }

        private Vector3 GetObjectCenter(Vector3[] vertices)
        {
            // Get the actual geometric center of the object using vertices
            var sum = Vector3.zero;
            foreach (var vertex in vertices)
            {
                sum += vertex;
            }

            return sum / vertices.Length;
        }
        */

        private Vector3 CalculateCollisionNormal(Vector3 samplePoint)
        {
            const float epsilon = 0.01f;

            // Sample the SDF at offset positions
            var dx = (SampleSDF(_sdfPartB, samplePoint + Vector3.right * epsilon) -
                      SampleSDF(_sdfPartB, samplePoint - Vector3.right * epsilon)) / (2f * epsilon);

            var dy = (SampleSDF(_sdfPartB, samplePoint + Vector3.up * epsilon) -
                      SampleSDF(_sdfPartB, samplePoint - Vector3.up * epsilon)) / (2f * epsilon);

            var dz = (SampleSDF(_sdfPartB, samplePoint + Vector3.forward * epsilon) -
                      SampleSDF(_sdfPartB, samplePoint - Vector3.forward * epsilon)) / (2f * epsilon);

            var normal = new Vector3(dx, dy, dz);

            if (normal.sqrMagnitude < 1e-6f)
            {
                // If gradient is too small, use direction between centers of bounds
                var thisCenter = _sdfPartA.Origin + Vector3.one * (_sdfPartA.GridSize * _sdfPartA.CellSize * 0.5f);
                var otherCenter = _sdfPartB.Origin + Vector3.one * (_sdfPartB.GridSize * _sdfPartB.CellSize * 0.5f);
                normal = (thisCenter - otherCenter).normalized;
            }
            else
            {
                normal = normal.normalized;
            }

            return normal;
        }

        private float SampleSDF(SignedDistanceField otherSDF, Vector3 worldPos)
        {
            // Transform the world position to the other object's local space
            var gridPos = otherSDF.WorldToGridPosition(worldPos);
            return otherSDF.GetDistance(gridPos);
        }
    }
}