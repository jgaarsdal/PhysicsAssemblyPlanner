using UnityEngine;
using System.Collections.Generic;

public class PhysicsSimulation
{
    public int NdofR { get; private set; } // number of reduced DoFs
    public int NdofM { get; private set; } // number of maximal DoFs
    public int NdofU { get; private set; } // number of control variables
    public int NdofVar { get; private set; } // number of variables
    
    private Dictionary<string, Part> _parts = new Dictionary<string, Part>();
    private float _timeStep = 0.01f;

    public PhysicsSimulation(Dictionary<string, Mesh> partMeshes)
    {
        foreach (var kvp in partMeshes)
        {
            _parts[kvp.Key] = new Part(kvp.Value);
        }

        // Set degrees of freedom
        NdofR = 6 * _parts.Count; // 6 DoF per part (3 for position, 3 for rotation)
        NdofM = 6 * _parts.Count; // Same as NdofR in this case
        NdofU = 6 * _parts.Count; // 3 for force, 3 for torque per part
        NdofVar = 12 * _parts.Count; // position, rotation, velocity, angular velocity per part
    }

    public void Reset()
    {
        foreach (var part in _parts.Values)
        {
            part.Reset();
        }
    }

    public Vector3 GetPosition(string partId)
    {
        return _parts[partId].Position;
    }

    public Quaternion GetRotation(string partId)
    {
        return _parts[partId].Rotation;
    }

    public Vector3 GetVelocity(string partId)
    {
        return _parts[partId].Velocity;
    }

    public Vector3 GetAngularVelocity(string partId)
    {
        return _parts[partId].AngularVelocity;
    }

    public void SetPosition(string partId, Vector3 position)
    {
        _parts[partId].Position = position;
    }

    public void SetRotation(string partId, Quaternion rotation)
    {
        _parts[partId].Rotation = rotation;
    }

    public void SetVelocity(string partId, Vector3 velocity)
    {
        _parts[partId].Velocity = velocity;
    }

    public void SetAngularVelocity(string partId, Vector3 angularVelocity)
    {
        _parts[partId].AngularVelocity = angularVelocity;
    }

    public void ApplyForce(string partId, Vector3 force)
    {
        _parts[partId].ApplyForce(force);
    }

    public void ApplyForceAndTorque(string partId, Vector3 force, Vector3 torque)
    {
        _parts[partId].ApplyForce(force);
        _parts[partId].ApplyTorque(torque);
    }

    public Bounds GetBounds(string partId)
    {
        return _parts[partId].Bounds;
    }

    public void UpdateParts()
    {
        foreach (var part in _parts.Values)
        {
            part.Update();
        }
    }

    public void Forward(int steps)
    {
        for (var i = 0; i < steps; i++)
        {
            Step();
        }
    }

    public Vector3[] GetVertices(string partId, bool worldSpace = true)
    {
        if (_parts.TryGetValue(partId, out Part part))
        {
            return part.GetVertices(worldSpace);
        }
        throw new KeyNotFoundException($"Part with ID {partId} not found.");
    }

    private void Step()
    {
        foreach (var part in _parts.Values)
        {
            part.Step(_timeStep);
        }
    }
}