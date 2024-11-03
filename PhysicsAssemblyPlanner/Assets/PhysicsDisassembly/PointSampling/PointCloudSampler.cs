using System;
using System.Collections.Generic;
using UnityEngine;

public static class PointCloudSampler
{
    public enum SampleMethod
    {
        BarycentricCoordinates,
        WeightedBarycentricCoordinates
    }
    
    public static Vector3[] GetPointCloud(GameObject objectRoot, int totalPointCount, SampleMethod sampleMethod, bool convertToWorldSpace = false)
    {
        var result = new List<Vector3>();

        var meshFilters = objectRoot.GetComponentsInChildren<MeshFilter>();
        var meshCount = meshFilters.Length;
        
        var sizeFactors = GetAssemblyPartSizeFactors(meshFilters);
        var pointCounter = 0;
        
        for (var i = 0; i < meshCount; i++)
        {
            var mesh = meshFilters[i].sharedMesh;
            var pointCount = Mathf.FloorToInt(sizeFactors[i] * totalPointCount);
            
            pointCounter += pointCount;
            if (i >= meshCount - 1 && pointCounter < totalPointCount)
            {
                pointCount += totalPointCount - pointCounter;
            }

            Vector3[] points;
            Vector3[] pointNormals;
            switch (sampleMethod)
            {
                case SampleMethod.BarycentricCoordinates:
                    points = GetPointCloud(mesh.vertices, mesh.triangles, pointCount, false, out pointNormals, convertToWorldSpace, meshFilters[i].transform.localToWorldMatrix);
                    break;
                case SampleMethod.WeightedBarycentricCoordinates:
                    points = GetWeightedPointCloud(mesh.vertices, mesh.triangles, pointCount, false, out pointNormals, convertToWorldSpace, meshFilters[i].transform.localToWorldMatrix);
                    break;
                default:
                    throw new NotImplementedException();
            }
            
            result.AddRange(points);
        }
        
        return result.ToArray();
    }
    
    public static Vector3[] GetPointCloudWithNormals(GameObject objectRoot, int totalPointCount, SampleMethod sampleMethod, out Vector3[] normals, bool convertToWorldSpace = false)
    {
        var result = new List<Vector3>();
        var normalsList = new List<Vector3>();
        
        var meshFilters = objectRoot.GetComponentsInChildren<MeshFilter>();
        var meshCount = meshFilters.Length;
        
        var sizeFactors = GetAssemblyPartSizeFactors(meshFilters);
        var pointCounter = 0;
        
        for (var i = 0; i < meshCount; i++)
        {
            var mesh = meshFilters[i].sharedMesh;
            var pointCount = Mathf.FloorToInt(sizeFactors[i] * totalPointCount);
            
            pointCounter += pointCount;
            if (i >= meshCount - 1 && pointCounter < totalPointCount)
            {
                pointCount += totalPointCount - pointCounter;
            }

            Vector3[] points;
            Vector3[] pointNormals;
            switch (sampleMethod)
            {
                case SampleMethod.BarycentricCoordinates:
                    points = GetPointCloud(mesh.vertices, mesh.triangles, pointCount, true, out pointNormals, convertToWorldSpace, meshFilters[i].transform.localToWorldMatrix);
                    break;
                case SampleMethod.WeightedBarycentricCoordinates:
                    points = GetWeightedPointCloud(mesh.vertices, mesh.triangles, pointCount, true, out pointNormals, convertToWorldSpace, meshFilters[i].transform.localToWorldMatrix);
                    break;
                default:
                    throw new NotImplementedException();
            }
            
            result.AddRange(points);
            normalsList.AddRange(pointNormals);
        }
        
        normals = normalsList.ToArray();
        return result.ToArray();
    }
    
    public static Vector3[] GetPointCloud(Vector3[] vertices, int[] triangles, int pointCount, SampleMethod sampleMethod, bool convertToWorldSpace = false, Matrix4x4 localToWorldMatrix = new Matrix4x4())
    {
        Vector3[] normals;
        switch (sampleMethod)
        {
            case SampleMethod.BarycentricCoordinates:
                return GetPointCloud(vertices, triangles, pointCount, false, out normals, convertToWorldSpace, localToWorldMatrix);
            case SampleMethod.WeightedBarycentricCoordinates:
                return GetWeightedPointCloud(vertices, triangles, pointCount, false, out normals, convertToWorldSpace, localToWorldMatrix);
            default:
                throw new NotImplementedException();
        }
    }
    
    public static Vector3[] GetPointCloudWithNormals(Vector3[] vertices, int[] triangles, int pointCount, SampleMethod sampleMethod, out Vector3[] normals, bool convertToWorldSpace = false, Matrix4x4 localToWorldMatrix = new Matrix4x4())
    {
        switch (sampleMethod)
        {
            case SampleMethod.BarycentricCoordinates:
                return GetPointCloud(vertices, triangles, pointCount, true, out normals, convertToWorldSpace, localToWorldMatrix);
            case SampleMethod.WeightedBarycentricCoordinates:
                return GetWeightedPointCloud(vertices, triangles, pointCount, true, out normals, convertToWorldSpace, localToWorldMatrix);
            default:
                throw new NotImplementedException();
        }
    }
    
    public static float[] GetAssemblyPartSizeFactors(MeshFilter[] meshList)
    {
        var totalSizeFactor = 0f;

        // Loop through all child gameobjects
        var sizeFactors = new float[meshList.Length];
        for (var i = 0; i < meshList.Length; i++)
        {
            var renderer = meshList[i].gameObject.GetComponent<Renderer>();
            var bounds = renderer.bounds;
            
            var sizeFactor = (bounds.max - bounds.min).magnitude;
            totalSizeFactor += sizeFactor;
            sizeFactors[i] = sizeFactor;
        }
        
        // Normalize each element by dividing it by the total
        for (var i = 0; i < sizeFactors.Length; i++)
        {
            sizeFactors[i] /= totalSizeFactor;
        }

        // Calculate the size of the bounding box
        return sizeFactors;
    }
    
    public static float[] GetAssemblyPartSizeFactors(List<MeshFilter> meshList)
    {
        var totalSizeFactor = 0f;

        // Loop through all child gameobjects
        var sizeFactors = new float[meshList.Count];
        for (var i = 0; i < meshList.Count; i++)
        {
            var renderer = meshList[i].gameObject.GetComponent<Renderer>();
            var bounds = renderer.bounds;
            
            var sizeFactor = (bounds.max - bounds.min).magnitude;
            totalSizeFactor += sizeFactor;
            sizeFactors[i] = sizeFactor;
        }
        
        // Normalize each element by dividing it by the total
        for (var i = 0; i < sizeFactors.Length; i++)
        {
            sizeFactors[i] /= totalSizeFactor;
        }

        // Calculate the size of the bounding box
        return sizeFactors;
    }
    
    // Using Barycentric Coordinates method
    private static Vector3[] GetPointCloud(Vector3[] vertices, int[] triangles, int pointCount, bool addNormals, out Vector3[] normals, bool convertToWorldSpace = false, Matrix4x4 localToWorldMatrix = new Matrix4x4())
    {
        var result = new Vector3[pointCount];
        
        normals = null;
        if (addNormals)
        {
            normals = new Vector3[pointCount];
        }

        var random = new System.Random(Guid.NewGuid().GetHashCode());
        for (var i = 0; i < pointCount; i++)
        {
            result[i] = SampleMeshPoint(triangles, vertices, addNormals, random, out var faceNormal);
            if (addNormals)
            {
                normals[i] = faceNormal;
            }
            
            if (convertToWorldSpace)
            {
                result[i] = localToWorldMatrix.MultiplyPoint3x4(result[i]);
                if (addNormals)
                {
                    normals[i] = (localToWorldMatrix * normals[i]).normalized;
                }
            }
        }

        return result;
    }

    // Using a weighted Barycentric Coordinates method to increase spread
    // Larger triangles have a larger weight and are more probable
    private static Vector3[] GetWeightedPointCloud(Vector3[] vertices, int[] triangles, int pointCount, bool addNormals, out Vector3[] normals, bool convertToWorldSpace = false, Matrix4x4 localToWorldMatrix = new Matrix4x4())
    {
        // Calculate areas of triangles and their CDF
        var totalArea = 0f;
        var triangleAreas = new float[triangles.Length / 3];
        
        for (var i = 0; i < triangles.Length / 3; i++)
        {
            var v0 = vertices[triangles[i * 3]];
            var v1 = vertices[triangles[i * 3 + 1]];
            var v2 = vertices[triangles[i * 3 + 2]];

            var area = Vector3.Cross(v1 - v0, v2 - v0).magnitude * 0.5f;
            triangleAreas[i] = area;
            totalArea += area;
        }
        
        // Convert areas to CDF (cumulative distribution function)
        for (var i = 1; i < triangleAreas.Length; i++)
        {
            triangleAreas[i] += triangleAreas[i - 1];
        }

        var result = new Vector3[pointCount];
        
        normals = null;
        if (addNormals)
        {
            normals = new Vector3[pointCount];
        }
        
        var random = new System.Random(Guid.NewGuid().GetHashCode());
        for (var i = 0; i < pointCount; i++)
        {
            result[i] = SampleMeshPointWithWeights(triangles, vertices, totalArea, triangleAreas, addNormals, random, out var faceNormal);
            if (addNormals)
            {
                normals[i] = faceNormal;
            }
            
            if (convertToWorldSpace)
            {
                result[i] = localToWorldMatrix.MultiplyPoint3x4(result[i]);
                if (addNormals)
                {
                    normals[i] = (localToWorldMatrix * normals[i]).normalized;
                }
            }
        }

        return result;
    }

    private static Vector3 SampleMeshPoint(int[] triangles, Vector3[] vertices, bool addNormal, System.Random random, out Vector3 faceNormal)
    {
        var index = Mathf.FloorToInt((float)random.NextDouble() * (triangles.Length / 3f));
        var v0 = vertices[triangles[index * 3]];
        var v1 = vertices[triangles[index * 3 + 1]];
        var v2 = vertices[triangles[index * 3 + 2]];
        
        if (addNormal)
        {
            faceNormal = GetFaceNormalOfTriangle(v0, v1, v2);
        }
        else
        {
            faceNormal = Vector3.zero;
        }

        return SamplePointInTriangle(v0, v1, v2, random);
    }

    private static Vector3 SampleMeshPointWithWeights(int[] triangles, Vector3[] vertices, float totalArea, float[] triangleAreas, bool addNormal, System.Random random, out Vector3 faceNormal)
    {
        // Pick a random triangle based on area
        var randomValue = (float)random.NextDouble() * totalArea;
        var triangleIndex = System.Array.BinarySearch(triangleAreas, randomValue);
        if (triangleIndex < 0)
        {
            triangleIndex = ~triangleIndex;
        }

        var v0 = vertices[triangles[triangleIndex * 3]];
        var v1 = vertices[triangles[triangleIndex * 3 + 1]];
        var v2 = vertices[triangles[triangleIndex * 3 + 2]];

        if (addNormal)
        {
            faceNormal = GetFaceNormalOfTriangle(v0, v1, v2);
        }
        else
        {
            faceNormal = Vector3.zero;
        }
        
        // Sample a point inside the triangle
        return SamplePointInTriangle(v0, v1, v2, random);
    }
    
    private static Vector3 SamplePointInTriangle(Vector3 v0, Vector3 v1, Vector3 v2, System.Random random)
    {
        var u = (float)random.NextDouble();
        var v = (float)random.NextDouble() * (1f - u);

        var w = 1 - (u + v);

        return v0 * u + v1 * v + v2 * w;
    }

    private static Vector3 GetFaceNormalOfTriangle(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        // Compute the cross product of two edges
        return Vector3.Cross((v1 - v0).normalized, (v2 - v0).normalized).normalized;
    }
}
