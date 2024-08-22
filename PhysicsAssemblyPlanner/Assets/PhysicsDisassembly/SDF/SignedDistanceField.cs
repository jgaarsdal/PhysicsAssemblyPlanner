using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

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
    public float[,,] Distances => _distances;
    public int GridSize => _gridSize;
    public Vector3 Origin => _origin;
    public float CellSize => _cellSize;

    private Mesh _mesh;
    private Transform _objectTransform;
    private float[,,] _distances;
    private int[,,] _closestTriangles;
    private Vector3 _origin;
    private float _cellSize;
    private int _gridSize;

    // GPU variables
    private ComputeShader _computeShader;
    private ComputeBuffer _distancesBuffer;
    private ComputeBuffer _closestTrianglesBuffer;
    private ComputeBuffer _trianglesBuffer;
    
    public SignedDistanceField(Mesh mesh, Transform meshTransform, int gridSize = 32, float boundingBoxPadding = 0.1f)
    {
        var bounds = mesh.bounds;
        bounds.Expand(boundingBoxPadding);

        _mesh = mesh;
        _objectTransform = meshTransform;
        _origin = bounds.min;
        _cellSize = Mathf.Max(bounds.size.x, bounds.size.y, bounds.size.z) / (gridSize - 1);
        _gridSize = gridSize;
        
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
    public async Task ComputeSDF(bool useGpuIfPossible = true)
    {
        var platformStr = "CPU";
        
        var startTime = Time.realtimeSinceStartup;
        if (useGpuIfPossible && SupportsComputeShaders())
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
    
    public bool CheckCollision(SignedDistanceField part2Sdf, Vector3 position, Quaternion rotation)
    {
        // Get the SDFs for both parts
        var part2Distances = part2Sdf.Distances;

        // Get the transforms for both parts
        Transform part2Transform = part2Sdf.ObjectTransform;

        // Calculate the relative transform
        Matrix4x4 E0i = Matrix4x4.TRS(_objectTransform.position, _objectTransform.rotation, Vector3.one);
        Matrix4x4 part2E0i = Matrix4x4.TRS(part2Transform.position, part2Transform.rotation, Vector3.one);
        Matrix4x4 relativeTransform = part2E0i.inverse * E0i;

        // Apply the new position and rotation
        Matrix4x4 newTransform = Matrix4x4.TRS(position, rotation, Vector3.one) * relativeTransform;
        
        Vector3 part2Origin = part2Sdf.Origin;

        for (int i = 0; i < GridSize; i++)
        {
            for (int j = 0; j < GridSize; j++)
            {
                for (int k = 0; k < GridSize; k++)
                {
                    Vector3 x = part2Origin + new Vector3(i, j, k) * _cellSize;
                    Vector3 xMove = newTransform.inverse.MultiplyPoint3x4(x);
                    Vector3 xMoveGrid = (xMove - _origin) / _cellSize;

                    if (xMoveGrid.x >= 0 && xMoveGrid.x < GridSize - 1 &&
                        xMoveGrid.y >= 0 && xMoveGrid.y < GridSize - 1 &&
                        xMoveGrid.z >= 0 && xMoveGrid.z < GridSize - 1)
                    {
                        float d0 = part2Distances[i, j, k];
                        float d1 = TrilinearInterpolation(_distances, xMoveGrid.x, xMoveGrid.y, xMoveGrid.z);

                        if (d0 < 0 && d1 < 0)
                        {
                            // Collision detected
                            return true; 
                        }
                    }
                }
            }
        }

        // No collision detected
        return false; 
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
        var vertices = new List<Vector3>(_mesh.vertices);
        var triangles = new List<Vector3Int>();
        var meshTriangles = _mesh.triangles;
        
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
            for (var i = 0; i < meshTriangles.Length; i += 3)
            {
                triangles.Add(new Vector3Int(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]));
            }

            var cellFactor = 1f / _cellSize;

            Parallel.For(0, triangles.Count, (t) =>
            {
                var p = vertices[triangles[t].x];
                var q = vertices[triangles[t].y];
                var r = vertices[triangles[t].z];

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
        var vertices = new List<Vector3>(_mesh.vertices);
        var triangles = new List<Vector3Int>();
        var meshTriangles = _mesh.triangles;
        
        var totalSize = _gridSize * _gridSize * _gridSize;
        _distancesBuffer = new ComputeBuffer(totalSize, sizeof(float));
        _closestTrianglesBuffer = new ComputeBuffer(totalSize, sizeof(int));
        _trianglesBuffer = new ComputeBuffer(meshTriangles.Length / 3, 3 * sizeof(float) * 3);
        
        _distances = new float[_gridSize, _gridSize, _gridSize];
        _closestTriangles = new int[_gridSize, _gridSize, _gridSize];
        var triangleData = new Vector3[meshTriangles.Length];
        
        await Task.Run(() =>
        {
            for (var i = 0; i < meshTriangles.Length; i += 3)
            {
                triangles.Add(new Vector3Int(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]));
            }
            
            for (var i = 0; i < triangles.Count; i++)
            {
                triangleData[i * 3] = vertices[triangles[i].x];
                triangleData[i * 3 + 1] = vertices[triangles[i].y];
                triangleData[i * 3 + 2] = vertices[triangles[i].z];
            }
        });

        _trianglesBuffer.SetData(triangleData);

        _computeShader.SetBuffer(0, "_Distances", _distancesBuffer);
        _computeShader.SetBuffer(0, "_ClosestTriangles", _closestTrianglesBuffer);
        _computeShader.SetBuffer(0, "_Triangles", _trianglesBuffer);
        _computeShader.SetInt("_GridSize", _gridSize);
        _computeShader.SetFloat("_CellSize", _cellSize);
        _computeShader.SetVector("_Origin", _origin);
        _computeShader.SetInt("_NumTriangles", triangles.Count);
    }

    private void ReleaseBuffers()
    {
        _distancesBuffer?.Release();
        _closestTrianglesBuffer?.Release();
        _trianglesBuffer?.Release();
    }

    private async Task ComputeSDFOnCPUAsync()
    {
        var vertices = new List<Vector3>(_mesh.vertices);
        
        var triangles = new List<Vector3Int>();
        var meshTriangles = _mesh.triangles;
        for (var i = 0; i < meshTriangles.Length; i += 3)
        {
            triangles.Add(new Vector3Int(meshTriangles[i], meshTriangles[i + 1], meshTriangles[i + 2]));
        }

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
            Parallel.For(0, triangles.Count, (t) =>
            {
                var p = vertices[triangles[t].x];
                var q = vertices[triangles[t].y];
                var r = vertices[triangles[t].z];

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
                Sweep(triangles, vertices, +1, +1, +1);
                Sweep(triangles, vertices, -1, -1, -1);
                Sweep(triangles, vertices, +1, +1, -1);
                Sweep(triangles, vertices, -1, -1, +1);
                Sweep(triangles, vertices, +1, -1, +1);
                Sweep(triangles, vertices, -1, +1, -1);
                Sweep(triangles, vertices, +1, -1, -1);
                Sweep(triangles, vertices, -1, +1, +1);
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

    private void Sweep(List<Vector3Int> triangles, List<Vector3> vertices, int di, int dj, int dk)
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
    
    private void CheckNeighbor(List<Vector3Int> triangles, List<Vector3> vertices, Vector3 gx, 
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

    private float TrilinearInterpolation(float[,,] distances, float x, float y, float z)
    {
        int x0 = Mathf.FloorToInt(x);
        int y0 = Mathf.FloorToInt(y);
        int z0 = Mathf.FloorToInt(z);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        float xd = x - x0;
        float yd = y - y0;
        float zd = z - z0;

        float c000 = distances[x0, y0, z0];
        float c100 = distances[x1, y0, z0];
        float c010 = distances[x0, y1, z0];
        float c110 = distances[x1, y1, z0];
        float c001 = distances[x0, y0, z1];
        float c101 = distances[x1, y0, z1];
        float c011 = distances[x0, y1, z1];
        float c111 = distances[x1, y1, z1];

        return Mathf.Lerp(
            Mathf.Lerp(Mathf.Lerp(c000, c100, xd), Mathf.Lerp(c010, c110, xd), yd),
            Mathf.Lerp(Mathf.Lerp(c001, c101, xd), Mathf.Lerp(c011, c111, xd), yd),
            zd);
    }
}