using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace PhysicsDisassembly.Simulation
{
    public class PhysicsSimulation
    {
        private Dictionary<string, SimulationPart> _simulationParts = new Dictionary<string, SimulationPart>();
        private Dictionary<string, SDFCollisionPart> _collisionParts = new Dictionary<string, SDFCollisionPart>();
        private PhysicsSimulationConfiguration _configuration;
        private bool _useRotation;

        public PhysicsSimulation(Dictionary<string, PartData> partDataMap, bool useRotation, PhysicsSimulationConfiguration configuration)
        {
            _configuration = configuration;
            _useRotation = useRotation;

            foreach (var kvp in partDataMap)
            {
                var partId = kvp.Key;
                var partData = kvp.Value;
                var partObject = partData.PartObject;
                
                _simulationParts.Add(partId, new SimulationPart(partObject.GetComponentInChildren<MeshFilter>().sharedMesh, 
                    partData.LocalContactPoints, partObject.position, partObject.rotation, partObject.localScale, configuration));

                if (partData.SDF != null)
                {
                    _collisionParts.Add(partId, new SDFCollisionPart(partId, this, partData.SDF, _configuration));
                }
            }
        }

        public void Reset()
        {
            foreach (var part in _simulationParts.Values)
            {
                part.Reset();
            }
        }

        public SimulationPart GetPart(string partId)
        {
            return _simulationParts[partId];
        }
        
        public Vector3 GetPosition(string partId)
        {
            return _simulationParts[partId].CenterPosition;
        }

        public Vector3 GetPivotPosition(string partId)
        {
            return _simulationParts[partId].PivotPosition;
        }

        public Quaternion GetRotation(string partId)
        {
            return _simulationParts[partId].Rotation;
        }

        public Vector3 GetVelocity(string partId)
        {
            return _simulationParts[partId].Velocity;
        }

        public Vector3 GetAngularVelocity(string partId)
        {
            return _simulationParts[partId].AngularVelocity;
        }

        public void SetCenterPosition(string partId, Vector3 position)
        {
            _simulationParts[partId].CenterPosition = position;
        }
        
        public void SetPivotPosition(string partId, Vector3 position)
        {
            _simulationParts[partId].PivotPosition = position;
        }

        public void SetRotation(string partId, Quaternion rotation)
        {
            _simulationParts[partId].Rotation = rotation;
        }

        public void SetVelocity(string partId, Vector3 velocity)
        {
            _simulationParts[partId].Velocity = velocity;
        }

        public void SetAngularVelocity(string partId, Vector3 angularVelocity)
        {
            _simulationParts[partId].AngularVelocity = angularVelocity;
        }

        public void ApplyForce(string partId, Vector3 force)
        {
            _simulationParts[partId].ApplyForce(force);
        }

        public void ApplyForceAndTorque(string partId, Vector3 force, Vector3 torque)
        {
            _simulationParts[partId].ApplyForce(force);
            _simulationParts[partId].ApplyTorque(torque);
        }

        public void ApplyForceAtPoint(string partId, Vector3 contactForce, Vector3 vertex)
        {
            if (_useRotation)
            {
                _simulationParts[partId].ApplyForceAtPoint(contactForce, vertex);
            }
            else
            {
                _simulationParts[partId].ApplyForce(contactForce);
            }
        }

        public Bounds GetBounds(string partId)
        {
            return _simulationParts[partId].Bounds;
        }

        public void UpdateParts()
        {
            // Update the physics simulation
            foreach (var part in _simulationParts.Values)
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

        public bool CheckCollisions(string movingPartId, bool useSDFCollision = true)
        {
            var hasCollision = false;
            var lockObj = new object();
            
            if (!useSDFCollision)
            {
                var moveBounds = GetBounds(movingPartId);

                var stillPartBounds = _simulationParts
                    .Where(kvp => kvp.Key != movingPartId)
                    .Select(kvp => kvp.Value.Bounds)
                    .ToArray();

                Parallel.ForEach(stillPartBounds, (stillBounds, state) =>
                {
                    if (moveBounds.Intersects(stillBounds))
                    {
                        lock (lockObj)
                        {
                            hasCollision = true;
                        }
                        state.Stop();
                    }
                });
                
                return hasCollision;
            }
            
            var movingPart = _collisionParts[movingPartId];
            var stillCollisionParts = _collisionParts
                .Where(kvp => kvp.Key != movingPartId)
                .Select(kvp => kvp.Value)
                .ToArray();
            
            Parallel.ForEach(stillCollisionParts, (stillPart, state) =>
            {
                if (movingPart.CheckCollision(stillPart))
                {
                    lock (lockObj)
                    {
                        hasCollision = true;
                    }
                    state.Stop();
                }
            });

            return hasCollision;
        }
        
        public void CheckAndResolveCollisions(string movingPartId)
        {
            var stillCollisionParts = _collisionParts
                .Where(kvp => kvp.Key != movingPartId)
                .Select(kvp => kvp.Value)
                .ToArray();

            var movingPart = _collisionParts[movingPartId];
            var force = Vector3.zero;
            var torque = Vector3.zero;

            var lockObj = new object();
            Parallel.ForEach(stillCollisionParts, (stillPart) =>
            {
                var (tempForce, tempTorque) = movingPart.CheckAndResolveCollision(stillPart, _useRotation);

                lock (lockObj)
                {
                    force += tempForce;
                    torque += tempTorque;
                }
            });

            if (_useRotation)
            {
                ApplyForceAndTorque(movingPartId, force, torque);
            }
            else
            {
                ApplyForce(movingPartId, force);
            }
        }

        public Vector3[] GetVertices(string partId, bool worldSpace = true)
        {
            if (_simulationParts.TryGetValue(partId, out SimulationPart part))
            {
                return part.GetVertices(worldSpace);
            }

            throw new KeyNotFoundException($"Part with ID {partId} not found.");
        }
        
        public Vector3[] GetContactPoints(string partId)
        {
            if (_simulationParts.TryGetValue(partId, out SimulationPart part))
            {
                return part.GetContactPoints();
            }

            throw new KeyNotFoundException($"Part with ID {partId} not found.");
        }

        private void Step()
        {
            foreach (var part in _simulationParts.Values)
            {
                part.Step(_configuration.SimulationTimeStep);
            }
        }
    }
}