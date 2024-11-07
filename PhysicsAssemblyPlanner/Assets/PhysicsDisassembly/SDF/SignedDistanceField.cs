using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

namespace PhysicsDisassembly.SDF
{
    public static class Vector3Extensions
    {
        public static Vector3 Scale(this Vector3 v, float x, float y, float z)
        {
            return new Vector3(v.x * x, v.y * y, v.z * z);
        }
    }

    public class SignedDistanceField
    {
        public Transform ObjectTransform => _objectTransform;
        public Vector3[] WorldVertices => _vertices;
        //public Vector3[] ContactPoints => _contactPoints;
        public Vector3Int[] Triangles => _triangles;
        public float[,,] Distances => _distances;
        public int GridSize => _gridSize;
        public Vector3 Origin => _origin;
        public float CellSize => _cellSize;
        
        private MeshFilter[] _objectMeshes;
        private Vector3[] _vertices;
        private Vector3[] _contactPoints;
        private float[,,] _distances;
        private int[,,] _closestTriangles;
        private Vector3 _origin;
        private float _cellSize;
        private int _gridSize;
        private readonly Transform _objectTransform;
        private readonly Vector3Int[] _triangles;
        private readonly float _boundingBoxPadding;
        private readonly float _defaultCellSize;
        //private readonly float _collisionPenetrationThreshold;
        //private readonly int _collisionContactPointCount;
        private readonly bool _useGPU;
        
        // GPU variables
        private ComputeBuffer _distancesBuffer;
        private ComputeBuffer _closestTrianglesBuffer;
        private ComputeBuffer _trianglesBuffer;
        private readonly ComputeShader _computeShader;

        public SignedDistanceField(GameObject gameObj, SDFCollisionConfiguration configuration)
        : this(gameObj, configuration.SDFDefaultCellSize, configuration.SDFBoxPadding, /*configuration.SDFCollisionPenetrationThreshold,*/ configuration.SDFUseGPU)
        { }
        
        public SignedDistanceField(GameObject gameObj, float defaultCellSize = 0.05f, float boundingBoxPadding = 0.1f, /*float collisionPenetrationThreshold = 0.01f,*/ bool useGPU = true)
        {
            _objectTransform = gameObj.transform;
            _objectMeshes = _objectTransform.GetComponentsInChildren<MeshFilter>();
            _defaultCellSize = defaultCellSize;
            _boundingBoxPadding = boundingBoxPadding;
            //_collisionPenetrationThreshold = collisionPenetrationThreshold;
            //_collisionContactPointCount = collisionContactPointCount;
            _useGPU = useGPU;
            
            // TODO: Try with cell size that is different for x,y,z 
            // TODO: Same for grid size
            
            _triangles = GetAllTriangles();
            UpdateVertices();
            
            if (SupportsComputeShaders())
            {
                _computeShader = Resources.Load<ComputeShader>("SDFCompute");
            }
        }
        
        public Vector3Int WorldToGridPosition(Vector3 worldPosition)
        {
            var localPosition = worldPosition - _origin;
            var gridPosition = localPosition / _cellSize;
            
            return new Vector3Int(
                Mathf.RoundToInt(gridPosition.x),
                Mathf.RoundToInt(gridPosition.y),
                Mathf.RoundToInt(gridPosition.z));
        }
        
        public float GetDistance(Vector3Int gridPosition)
        {
            var x = Mathf.Clamp(gridPosition.x, 0, _gridSize - 1);
            var y = Mathf.Clamp(gridPosition.y, 0, _gridSize - 1);
            var z = Mathf.Clamp(gridPosition.z, 0, _gridSize - 1);
            return _distances[x, y, z];
        }
        
        // Using the Fast Sweeping Method
        public async Task ComputeSDF()
        {
            var platformStr = "CPU";
            
            var startTime = Time.realtimeSinceStartup;
            if (_useGPU && SupportsComputeShaders())
            {
                platformStr = "GPU";
                await ComputeSDFOnGPUAsync();
            }
            else
            {
                await ComputeSDFOnCPUAsync();
            }
            
            Debug.Log($"ComputeSDF time on {platformStr}: {Time.realtimeSinceStartup - startTime} seconds");
        }
        
        public void UpdateVertices()
        {
            var bounds = CalculateBoundsForAllRenderers(_objectTransform.gameObject);
            if (bounds.size == Vector3.zero)
            {
                _cellSize = _defaultCellSize;
            }
            else
            {
                var cellSizes = GetSDFCellSize(bounds);
                _cellSize = Mathf.Min(cellSizes.x, cellSizes.y, cellSizes.z);
            }
            
            bounds.Expand(_boundingBoxPadding);
            bounds = CalculateGrid(bounds);
            _origin = bounds.min;

            _vertices = GetAllVertices();
            
            /*
            // Get contact points for collision test
            var tris = new int[Triangles.Length * 3];

            for (var i = 0; i < Triangles.Length; i++)
            {
                tris[i * 3] = Triangles[i].x;
                tris[i * 3 + 1] = Triangles[i].y;
                tris[i * 3 + 2] = Triangles[i].z;
            }
            
            _contactPoints = PointCloudSampler.GetPointCloud(_vertices, tris, 1024,
                PointCloudSampler.SampleMethod.WeightedBarycentricCoordinates, false);
                */
        }
        
        /*public bool CheckCollision(SignedDistanceField otherSDF)
        {
            return CheckCollision(otherSDF, _contactPoints);
        }
        
        public bool CheckCollision(SignedDistanceField otherSDF, Vector3[] contactPoints)
        {
            var collisionFound = false;
            var lockObj = new object();

            // Check all contact points of this object against the other object's SDF
            Parallel.ForEach(contactPoints, 
                (contactPoint, state) =>
                {
                    // Convert to grid coordinates
                    var gridPos = otherSDF.WorldToGridPosition(contactPoint);

                    // Check if the point is inside the other object's grid
                    if (gridPos.x >= 0 && gridPos.x < otherSDF.GridSize &&
                        gridPos.y >= 0 && gridPos.y < otherSDF.GridSize &&
                        gridPos.z >= 0 && gridPos.z < otherSDF.GridSize)
                    {
                        // Get the SDF value at this point
                        var distance = otherSDF.GetDistance(gridPos);
                        
                        // If the distance is negative or very close to zero, we have a collision
                        if (distance <= _collisionPenetrationThreshold)
                        {
                            lock (lockObj)
                            {
                                collisionFound = true;
                            }
                            state.Stop();
                        }
                    }
                });

            return collisionFound;
        }
        */
        
        private Vector3 GetSDFCellSize(Bounds bounds)
        {
            var cellSizeX = Mathf.Min(_defaultCellSize, bounds.size.x / 20f);
            var cellSizeY = Mathf.Min(_defaultCellSize, bounds.size.y / 20f);
            var cellSizeZ = Mathf.Min(_defaultCellSize, bounds.size.z / 20f);
            return new Vector3(cellSizeX, cellSizeY, cellSizeZ);
        }
        
        private Bounds CalculateBoundsForAllRenderers(GameObject gameObj)
        {
            var renderers = gameObj.GetComponentsInChildren<Renderer>();
            if (renderers.Length == 0)
            {
                Debug.LogError($"SignedDistanceField: No renderers found on gameobject '{gameObj.name}'");
                return new Bounds();
            }

            var bounds = renderers[0].bounds;
            for (var i = 1; i < renderers.Length; i++)
            {
                bounds.Encapsulate(renderers[i].bounds);
            }

            return bounds;
        }
        
        private Bounds CalculateGrid(Bounds gridBounds)
        {
            // Find the largest dimension of the bounds
            var largestDimension = Mathf.Max(gridBounds.size.x, gridBounds.size.y, gridBounds.size.z);

            // Calculate the initial number of cells
            var numberOfCells = Mathf.CeilToInt(largestDimension / _cellSize);

            // Calculate the expanded size to fit whole cells
            var expandedSize = numberOfCells * _cellSize;

            _gridSize = numberOfCells;
            
            // Create the expanded bounds size
            return new Bounds(gridBounds.center, Vector3.one * expandedSize);
        }

        private async Task ComputeSDFOnGPUAsync()
        {
            await InitializeBuffers();

            // Compute Initial Distances
            var kernelInitialDistance = _computeShader.FindKernel("InitialDistance");
            var threadGroupSize = 8;
            var numThreadGroups = Mathf.CeilToInt(_gridSize / (float)threadGroupSize);
            _computeShader.Dispatch(kernelInitialDistance, numThreadGroups, numThreadGroups, numThreadGroups);
            
            // Perform Sweeps
            var kernelPositive = _computeShader.FindKernel("SweepPositive");
            var kernelNegative = _computeShader.FindKernel("SweepNegative");

            _computeShader.SetBuffer(kernelPositive, "_Distances", _distancesBuffer);
            _computeShader.SetBuffer(kernelPositive, "_ClosestTriangles", _closestTrianglesBuffer);
            _computeShader.SetBuffer(kernelNegative, "_Distances", _distancesBuffer);
            _computeShader.SetBuffer(kernelNegative, "_ClosestTriangles", _closestTrianglesBuffer);

            var sweepDirections = new Vector3Int[]
            {
                new (1, 1, 1),
                new (-1, -1, -1),
                new (1, 1, -1),
                new (-1, -1, 1),
                new (1, -1, 1),
                new (-1, 1, -1),
                new (1, -1, -1),
                new (-1, 1, 1)
            };

            for (var sweep = 0; sweep < 2; sweep++)
            {
                foreach (var direction in sweepDirections)
                {
                    _computeShader.SetInts("_SweepDirection", new[] { direction.x, direction.y, direction.z });
                    if (direction.x > 0 ||
                        (direction.x == 0 && direction.y > 0) ||
                        (direction.x == 0 && direction.y == 0 && direction.z > 0))
                    {
                        _computeShader.Dispatch(kernelPositive, numThreadGroups, numThreadGroups, numThreadGroups);
                    }
                    else
                    {
                        _computeShader.Dispatch(kernelNegative, numThreadGroups, numThreadGroups, numThreadGroups);
                    }
                }
            }

            // Retrieve GPU Results
            var distancesArray = new float[_gridSize * _gridSize * _gridSize];
            var closestTrianglesArray = new int[_gridSize * _gridSize * _gridSize];
            _distancesBuffer.GetData(distancesArray);
            _closestTrianglesBuffer.GetData(closestTrianglesArray);

            var intersectionCount = new int[_gridSize, _gridSize, _gridSize];

            await Task.Run(() =>
            {
                Parallel.For(0, _gridSize, (i) =>
                {
                    for (var j = 0; j < _gridSize; j++)
                    {
                        for (var k = 0; k < _gridSize; k++)
                        {
                            var index = i + j * _gridSize + k * _gridSize * _gridSize;
                            _distances[i, j, k] = distancesArray[index];
                            _closestTriangles[i, j, k] = closestTrianglesArray[index];
                        }
                    }
                });

                // Determine Inside/Outside

                var cellFactor = 1f / _cellSize;

                Parallel.For(0, _triangles.Length, (t) =>
                {
                    var p = _vertices[_triangles[t].x];
                    var q = _vertices[_triangles[t].y];
                    var r = _vertices[_triangles[t].z];

                    var fip = (p - _origin).Scale(cellFactor, cellFactor, cellFactor);
                    var fiq = (q - _origin).Scale(cellFactor, cellFactor, cellFactor);
                    var fir = (r - _origin).Scale(cellFactor, cellFactor, cellFactor);

                    var j0 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Min(fip.y, fiq.y, fir.y)), 0, _gridSize - 1);
                    var j1 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Max(fip.y, fiq.y, fir.y)), 0, _gridSize - 1);
                    var k0 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Min(fip.z, fiq.z, fir.z)), 0, _gridSize - 1);
                    var k1 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Max(fip.z, fiq.z, fir.z)), 0, _gridSize - 1);

                    for (var k = k0; k <= k1; k++)
                    {
                        for (var j = j0; j <= j1; j++)
                        {
                            if (PointInTriangle2D(j, k, fip.y, fip.z, fiq.y, fiq.z, fir.y, fir.z, out var a, out var b,
                                    out var c))
                            {
                                var fi = a * fip.x + b * fiq.x + c * fir.x;
                                var iInterval = Mathf.CeilToInt((float)fi);

                                if (iInterval < 0)
                                {
                                    intersectionCount[0, j, k]++;
                                }
                                else if (iInterval < _gridSize)
                                {
                                    intersectionCount[iInterval, j, k]++;
                                }
                            }
                        }
                    }
                });

                Parallel.For(0, _gridSize, (k) =>
                {
                    for (var j = 0; j < _gridSize; j++)
                    {
                        var totalCount = 0;
                        for (var i = 0; i < _gridSize; i++)
                        {
                            totalCount += intersectionCount[i, j, k];
                            if (totalCount % 2 == 1)
                            {
                                _distances[i, j, k] = -_distances[i, j, k];
                            }
                        }
                    }
                });
            });

            ReleaseBuffers();
        }

        private async Task InitializeBuffers()
        {
            var totalSize = _gridSize * _gridSize * _gridSize;
            _distancesBuffer = new ComputeBuffer(totalSize, sizeof(float));
            _closestTrianglesBuffer = new ComputeBuffer(totalSize, sizeof(int));
            _trianglesBuffer = new ComputeBuffer(_triangles.Length, 3 * sizeof(float) * 3);
            
            _distances = new float[_gridSize, _gridSize, _gridSize];
            _closestTriangles = new int[_gridSize, _gridSize, _gridSize];
            var triangleData = new Vector3[_triangles.Length * 3];
            
            await Task.Run(() =>
            {
                for (var i = 0; i < _triangles.Length; i++)
                {
                    triangleData[i * 3] = _vertices[_triangles[i].x];
                    triangleData[i * 3 + 1] = _vertices[_triangles[i].y];
                    triangleData[i * 3 + 2] = _vertices[_triangles[i].z];
                }
            });

            _trianglesBuffer.SetData(triangleData);

            _computeShader.SetBuffer(0, "_Distances", _distancesBuffer);
            _computeShader.SetBuffer(0, "_ClosestTriangles", _closestTrianglesBuffer);
            _computeShader.SetBuffer(0, "_Triangles", _trianglesBuffer);
            _computeShader.SetInt("_GridSize", _gridSize);
            _computeShader.SetFloat("_CellSize", _cellSize);
            _computeShader.SetVector("_Origin", _origin);
            _computeShader.SetInt("_NumTriangles", _triangles.Length);
        }

        private void ReleaseBuffers()
        {
            _distancesBuffer?.Release();
            _closestTrianglesBuffer?.Release();
            _trianglesBuffer?.Release();
        }

        private async Task ComputeSDFOnCPUAsync()
        {
            // Initialize grid values
            _distances = new float[_gridSize, _gridSize, _gridSize];
            _closestTriangles = new int[_gridSize, _gridSize, _gridSize];
            var distVal = (_gridSize * 3) * (_cellSize * 3);

            Parallel.For(0, _gridSize, (i) =>
            {
                for (var j = 0; j < _gridSize; j++)
                {
                    for (var k = 0; k < _gridSize; k++)
                    {
                        _distances[i, j, k] = distVal;
                        _closestTriangles[i, j, k] = -1;
                    }
                }
            });

            var intersectionCount = new int[_gridSize, _gridSize, _gridSize];
            var exactBand = 1;
            var cellFactor = 1f / _cellSize;

            await Task.Run(() =>
            {
                Parallel.For(0, _triangles.Length, (t) =>
                {
                    var p = _vertices[_triangles[t].x];
                    var q = _vertices[_triangles[t].y];
                    var r = _vertices[_triangles[t].z];

                    var fip = (p - _origin).Scale(cellFactor, cellFactor, cellFactor);
                    var fiq = (q - _origin).Scale(cellFactor, cellFactor, cellFactor);
                    var fir = (r - _origin).Scale(cellFactor, cellFactor, cellFactor);

                    var i0 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Min(fip.x, fiq.x, fir.x)) - exactBand, 0, _gridSize - 1);
                    var i1 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(fip.x, fiq.x, fir.x)) + exactBand + 1, 0, _gridSize - 1);
                    var j0 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Min(fip.y, fiq.y, fir.y)) - exactBand, 0, _gridSize - 1);
                    var j1 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(fip.y, fiq.y, fir.y)) + exactBand + 1, 0, _gridSize - 1);
                    var k0 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Min(fip.z, fiq.z, fir.z)) - exactBand, 0, _gridSize - 1);
                    var k1 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Max(fip.z, fiq.z, fir.z)) + exactBand + 1, 0, _gridSize - 1);
                    
                    for (var k = k0; k <= k1; k++)
                    {
                        for (var j = j0; j <= j1; j++)
                        {
                            for (var i = i0; i <= i1; i++)
                            {
                                var gx = new Vector3(i * _cellSize + _origin.x, j * _cellSize + _origin.y,
                                    k * _cellSize + _origin.z);
                                var d = PointTriangleDistance(gx, p, q, r);
                                if (d < _distances[i, j, k])
                                {
                                    _distances[i, j, k] = d;
                                    _closestTriangles[i, j, k] = t;
                                }
                            }
                        }
                    }

                    j0 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Min(fip.y, fiq.y, fir.y)), 0, _gridSize - 1);
                    j1 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Max(fip.y, fiq.y, fir.y)), 0, _gridSize - 1);
                    k0 = Mathf.Clamp(Mathf.CeilToInt(Mathf.Min(fip.z, fiq.z, fir.z)), 0, _gridSize - 1);
                    k1 = Mathf.Clamp(Mathf.FloorToInt(Mathf.Max(fip.z, fiq.z, fir.z)), 0, _gridSize - 1);
                    
                    for (var k = k0; k <= k1; k++)
                    {
                        for (var j = j0; j <= j1; j++)
                        {
                            if (PointInTriangle2D(j, k, fip.y, fip.z, fiq.y, fiq.z, fir.y, fir.z, out var a, out var b,
                                    out var c))
                            {
                                var fi = a * fip.x + b * fiq.x + c * fir.x;
                                var iInterval = Mathf.CeilToInt((float)fi);

                                if (iInterval < 0)
                                {
                                    intersectionCount[0, j, k]++;
                                }
                                else if (iInterval < _gridSize)
                                {
                                    intersectionCount[iInterval, j, k]++;
                                }
                            }
                        }
                    }
                });

                // Sweep twice in all 8 directions
                for (var pass = 0; pass < 2; pass++)
                {
                    Sweep(_triangles, _vertices, +1, +1, +1);
                    Sweep(_triangles, _vertices, -1, -1, -1);
                    Sweep(_triangles, _vertices, +1, +1, -1);
                    Sweep(_triangles, _vertices, -1, -1, +1);
                    Sweep(_triangles, _vertices, +1, -1, +1);
                    Sweep(_triangles, _vertices, -1, +1, -1);
                    Sweep(_triangles, _vertices, +1, -1, -1);
                    Sweep(_triangles, _vertices, -1, +1, +1);
                }

                Parallel.For(0, _gridSize, (k) =>
                {
                    for (var j = 0; j < _gridSize; j++)
                    {
                        var totalCount = 0;
                        for (var i = 0; i < _gridSize; i++)
                        {
                            totalCount += intersectionCount[i, j, k];
                            if (totalCount % 2 == 1)
                            {
                                _distances[i, j, k] = -_distances[i, j, k];
                            }
                        }
                    }
                });
            });
        }

        private void Sweep(Vector3Int[] triangles, Vector3[] vertices, int di, int dj, int dk)
        {
            var i0 = di > 0 ? 1 : _distances.GetLength(0) - 2;
            var i1 = di > 0 ? _distances.GetLength(0) : -1;
            var j0 = dj > 0 ? 1 : _distances.GetLength(1) - 2;
            var j1 = dj > 0 ? _distances.GetLength(1) : -1;
            var k0 = dk > 0 ? 1 : _distances.GetLength(2) - 2;
            var k1 = dk > 0 ? _distances.GetLength(2) : -1;

            for (var k = k0; k != k1; k += dk)
            {
                for (var j = j0; j != j1; j += dj)
                {
                    for (var i = i0; i != i1; i += di)
                    {
                        var gx = new Vector3(i * _cellSize + _origin.x, j * _cellSize + _origin.y, k * _cellSize + _origin.z);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i - di, j, k);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i, j - dj, k);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i - di, j - dj, k);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i, j, k - dk);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i - di, j, k - dk);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i, j - dj, k - dk);
                        CheckNeighbor(triangles, vertices, gx, i, j, k, i - di, j - dj, k - dk);
                    }
                }
            }
        }
        
        private void CheckNeighbor(Vector3Int[] triangles, Vector3[] vertices, Vector3 gx, 
            int i0, int j0, int k0, int i1, int j1, int k1)
        {
            if (_closestTriangles[i1, j1, k1] >= 0)
            {
                var tri = triangles[_closestTriangles[i1, j1, k1]];
                var dist = PointTriangleDistance(gx, vertices[tri.x], vertices[tri.y], vertices[tri.z]);
                
                if (dist < _distances[i0, j0, k0])
                {
                    _distances[i0, j0, k0] = dist;
                    _closestTriangles[i0, j0, k0] = _closestTriangles[i1, j1, k1];
                }
            }
        }

        private float PointTriangleDistance(Vector3 x0, Vector3 x1, Vector3 x2, Vector3 x3)
        {
            var x13 = x1 - x3;
            var x23 = x2 - x3;
            var x03 = x0 - x3;
            var m13 = x13.sqrMagnitude;
            var m23 = x23.sqrMagnitude;
            var d = Vector3.Dot(x13, x23);
            var invDet = 1f / Mathf.Max(m13 * m23 - d * d, 1e-30f);
            var a = Vector3.Dot(x13, x03);
            var b = Vector3.Dot(x23, x03);
            var w23 = invDet * (m23 * a - d * b);
            var w31 = invDet * (m13 * b - d * a);
            var w12 = 1 - w23 - w31;
            
            if (w23 >= 0 && w31 >= 0 && w12 >= 0)
            {
                return Vector3.Distance(x0, w23 * x1 + w31 * x2 + w12 * x3);
            }

            if (w23 > 0)
            {
                return Mathf.Min(PointSegmentDistance(x0, x1, x2), PointSegmentDistance(x0, x1, x3));
            }
            
            if (w31 > 0)
            {
                return Mathf.Min(PointSegmentDistance(x0, x1, x2), PointSegmentDistance(x0, x2, x3));
            }
            
            return Mathf.Min(PointSegmentDistance(x0, x1, x3), PointSegmentDistance(x0, x2, x3));
        }
        
        private float PointSegmentDistance(Vector3 x0, Vector3 x1, Vector3 x2)
        {
            return Vector3.Distance(x0, PointSegmentClosestPoint(x0, x1, x2));
        }
        
        private Vector3 PointSegmentClosestPoint(Vector3 x0, Vector3 x1, Vector3 x2)
        {
            var dx = x2 - x1;
            var m2 = dx.sqrMagnitude;
            
            var s12 = Vector3.Dot(x2 - x0, dx) / m2;
            s12 = Mathf.Clamp01(s12);
            
            return s12 * x1 + (1 - s12) * x2;
        }

        private bool PointInTriangle2D(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3,
            out double a, out double b, out double c)
        {
            x1 -= x0; x2 -= x0; x3 -= x0;
            y1 -= y0; y2 -= y0; y3 -= y0;
            
            var signa = Orientation(x2, y2, x3, y3, out a);
            if (signa == 0)
            {
                a = b = c = 0;
                return false;
            }
            
            var signb = Orientation(x3, y3, x1, y1, out b);
            if (signb != signa)
            {
                a = b = c = 0;
                return false;
            }
            
            var signc = Orientation(x1, y1, x2, y2, out c);
            if (signc != signa)
            {
                a = b = c = 0;
                return false;
            }
            
            var sum = a + b + c;
            a /= sum;
            b /= sum;
            c /= sum;
            return true;
        }
        
        private int Orientation(double x1, double y1, double x2, double y2, out double twiceSignedArea)
        {
            twiceSignedArea = y1 * x2 - x1 * y2;
            if (twiceSignedArea > 0)
            {
                return 1;
            }

            if (twiceSignedArea < 0)
            {
                return -1;
            }
            
            if (y2 > y1)
            {
                return 1;
            }
            
            if (y2 < y1)
            {
                return -1;
            }
            
            if (x1 > x2)
            {
                return 1;
            }
            
            if (x1 < x2)
            {
                return -1;
            }
            
            return 0;
        }
        
        private bool SupportsComputeShaders()
        {
            return SystemInfo.supportsComputeShaders;
        }

        private Vector3[] GetAllVertices()
        {
            return _objectMeshes
                .SelectMany(mf => mf.sharedMesh.vertices
                    .Select(v => mf.transform.TransformPoint(v))).ToArray();
        }

        private Vector3Int[] GetAllTriangles()
        {
            return _objectMeshes.SelectMany(mf => mf.sharedMesh.triangles
                .Select((value, index) => new { value, index })
                .GroupBy(x => x.index / 3)
                .Select(g => new Vector3Int(
                    g.ElementAt(0).value,
                    g.ElementAt(1).value,
                    g.ElementAt(2).value
                ))).ToArray();
        }
    }
}