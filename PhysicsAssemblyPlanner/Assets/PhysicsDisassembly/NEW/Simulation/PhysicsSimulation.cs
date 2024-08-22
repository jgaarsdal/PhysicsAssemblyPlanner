using UnityEngine;
using System.Collections.Generic;

public class PhysicsSimulation
{
    private Dictionary<string, Part> parts = new Dictionary<string, Part>();
    private float timeStep = 0.01f;
    
    public int NdofR { get; private set; } // number of reduced DoFs
    public int NdofM { get; private set; } // number of maximal DoFs
    public int NdofU { get; private set; } // number of control variables
    public int NdofVar { get; private set; } // number of variables

    public PhysicsSimulation(Dictionary<string, Mesh> partMeshes)
    {
        foreach (var kvp in partMeshes)
        {
            parts[kvp.Key] = new Part(kvp.Value);
        }

        // Set degrees of freedom
        NdofR = 6 * parts.Count; // 6 DoF per part (3 for position, 3 for rotation)
        NdofM = 6 * parts.Count; // Same as NdofR in this case
        NdofU = 6 * parts.Count; // 3 for force, 3 for torque per part
        NdofVar = 12 * parts.Count; // position, rotation, velocity, angular velocity per part
    }

    public void Reset()
    {
        foreach (var part in parts.Values)
        {
            part.Reset();
        }
    }

    public Vector3 GetPosition(string partId)
    {
        return parts[partId].Position;
    }

    public Quaternion GetRotation(string partId)
    {
        return parts[partId].Rotation;
    }

    public Vector3 GetVelocity(string partId)
    {
        return parts[partId].Velocity;
    }

    public Vector3 GetAngularVelocity(string partId)
    {
        return parts[partId].AngularVelocity;
    }

    public void SetPosition(string partId, Vector3 position)
    {
        parts[partId].Position = position;
    }

    public void SetRotation(string partId, Quaternion rotation)
    {
        parts[partId].Rotation = rotation;
    }

    public void SetVelocity(string partId, Vector3 velocity)
    {
        parts[partId].Velocity = velocity;
    }

    public void SetAngularVelocity(string partId, Vector3 angularVelocity)
    {
        parts[partId].AngularVelocity = angularVelocity;
    }

    public void ApplyForce(string partId, Vector3 force)
    {
        parts[partId].ApplyForce(force);
    }

    public void ApplyForceAndTorque(string partId, Vector3 force, Vector3 torque)
    {
        parts[partId].ApplyForce(force);
        parts[partId].ApplyTorque(torque);
    }

    public Bounds GetBounds(string partId)
    {
        return parts[partId].Bounds;
    }

    public void UpdateParts(bool designGradient = false)
    {
        foreach (var part in parts.Values)
        {
            part.Update();
        }
    }

    public void Forward(int steps)
    {
        for (int i = 0; i < steps; i++)
        {
            Step();
        }
    }

    public Vector3[] GetVertices(string partId, bool worldSpace = true)
    {
        if (parts.TryGetValue(partId, out Part part))
        {
            return part.GetVertices(worldSpace);
        }
        throw new KeyNotFoundException($"Part with ID {partId} not found.");
    }

    private void Step()
    {
        foreach (var part in parts.Values)
        {
            part.Step(timeStep);
        }
    }

    private class Part
    {
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 AngularVelocity { get; set; }
        public Vector3 Force { get; private set; }
        public Vector3 Torque { get; private set; }
        public Bounds Bounds { get; private set; }

        private Vector3 initialPosition;
        private Quaternion initialRotation;
        private Mesh mesh;
        private Vector3[] localVertices;
        private Vector3[] worldVertices;

        public Part(Mesh mesh)
        {
            this.mesh = mesh;
            this.localVertices = mesh.vertices;
            this.worldVertices = new Vector3[localVertices.Length];
            this.Bounds = mesh.bounds;
            this.initialPosition = Vector3.zero;
            this.initialRotation = Quaternion.identity;
            Reset();
        }

        public void ApplyForce(Vector3 force)
        {
            Force += force;
        }

        public void ApplyTorque(Vector3 torque)
        {
            Torque += torque;
        }

        public void Update()
        {
            // Update world vertices and bounding box
            Bounds = new Bounds(Position, Vector3.zero);
            for (int i = 0; i < localVertices.Length; i++)
            {
                worldVertices[i] = Rotation * localVertices[i] + Position;
                Bounds.Encapsulate(worldVertices[i]);
            }
        }

        public void Step(float timeStep)
        {
            // Simple Euler integration
            Velocity += Force * timeStep;
            AngularVelocity += Torque * timeStep;
            Position += Velocity * timeStep;
            Rotation *= Quaternion.Euler(AngularVelocity * timeStep);

            // Reset forces and torques
            Force = Vector3.zero;
            Torque = Vector3.zero;

            // Update world vertices and bounds
            Update();
        }

        public void Reset()
        {
            Position = initialPosition;
            Rotation = initialRotation;
            Velocity = Vector3.zero;
            AngularVelocity = Vector3.zero;
            Force = Vector3.zero;
            Torque = Vector3.zero;
            Update();
        }

        public Vector3[] GetVertices(bool worldSpace = true)
        {
            return worldSpace ? worldVertices : localVertices;
        }
    }
}