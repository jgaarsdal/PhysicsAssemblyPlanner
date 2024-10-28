using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using PhysicsDisassembly.SDF;

namespace PhysicsDisassembly.Simulation
{
    public class PhysicsSimulation
    {
        private Dictionary<string, Part> _parts = new Dictionary<string, Part>();
        private Dictionary<string, SDFCollisionPart> _collisionParts = new Dictionary<string, SDFCollisionPart>();
        private PhysicsSimulationConfiguration _configuration;
        private bool _useRotation;

        public PhysicsSimulation(Dictionary<string, GameObject> partObjects,
            Dictionary<string, SignedDistanceField> partSDFs, bool useRotation, PhysicsSimulationConfiguration configuration)
        {
            _configuration = configuration;
            _useRotation = useRotation;

            foreach (var kvp in partObjects)
            {
                var partId = kvp.Key;
                var partObject = kvp.Value;
                var partTransform = partObject.transform;

                _parts[partId] = new Part(partObject.GetComponent<MeshFilter>().sharedMesh, partTransform.position,
                    partTransform.rotation, partTransform.localScale);
                _collisionParts[partId] = new SDFCollisionPart(partId, this, partSDFs[partId], _configuration);
            }
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

        public Vector3 GetPivotPosition(string partId)
        {
            return _parts[partId].PivotPosition;
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

        public void ApplyForceAtPoint(string partId, Vector3 contactForce, Vector3 vertex)
        {
            if (_useRotation)
            {
                _parts[partId].ApplyForceAtPoint(contactForce, vertex);
            }
            else
            {
                _parts[partId].ApplyForce(contactForce);
            }
        }

        public Bounds GetBounds(string partId)
        {
            return _parts[partId].Bounds;
        }

        public void UpdateParts()
        {
            // Update the physics simulation
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

        public void CheckCollisions(string movingPartId)
        {
            var stillCollisionParts = _collisionParts
                .Where(kvp => kvp.Key != movingPartId)
                .Select(kvp => kvp.Value)
                .ToArray();

            var movingPart = _collisionParts[movingPartId];

            // Check collision with all still parts
            foreach (var stillPart in stillCollisionParts)
            {
                movingPart.CheckAndResolveCollision(stillPart);
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
                part.Step(_configuration.SimulationTimeStep);
            }
        }
    }
}