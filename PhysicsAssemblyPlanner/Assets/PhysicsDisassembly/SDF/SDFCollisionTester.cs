using System.Threading.Tasks;
using UnityEngine;

namespace PhysicsDisassembly.SDF
{
    public class SDFCollisionTester : MonoBehaviour
    {
        [SerializeField] private GameObject _partA = default;
        [SerializeField] private GameObject _partB = default;
        [Space] [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
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
            _sdfPartA = new SignedDistanceField(_partA, _sdfDefaultCellSize, _sdfBoxPadding, 0.01f, _useGPU);
            _sdfPartB = new SignedDistanceField(_partB, _sdfDefaultCellSize, _sdfBoxPadding, 0.01f, _useGPU);

            await Task.WhenAll(_sdfPartA.ComputeSDF(), _sdfPartB.ComputeSDF());
        }

        private void TestCollision()
        {
            _sdfPartA.UpdateVertices();
            _sdfPartB.UpdateVertices();



            var isColliding = _sdfPartA.CheckCollision(_sdfPartB);
            if (isColliding)
            {
                var contactPoint = FindContactPoint();
                var collisionNormal = CalculateCollisionNormal(contactPoint);

                Debug.DrawLine(contactPoint, contactPoint + collisionNormal * 0.5f, Color.red, 60f);
                Debug.Log("collisionNormal: " + collisionNormal);
            }

            Debug.Log($"SDFCollisionTester: IS COLLIDING = {isColliding}", this);
        }

        private Vector3 FindContactPoint()
        {
            var minDistance = float.MaxValue;
            var bestPoint = Vector3.zero;
            var bestSDF = 0f;

            // Sample vertices to find the point of deepest penetration
            foreach (var vertex in _sdfPartA.WorldVertices)
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