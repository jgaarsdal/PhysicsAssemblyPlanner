using UnityEngine;

namespace PhysicsDisassembly.Simulation
{
    public class SimulationPart
    {
        public Vector3 CenterPosition
        {
            get => _centerPosition;
            set
            {
                _centerPosition = value;
                _pivotPosition = _centerPosition - Rotation * _centerOffset; // TODO: Need rotation?
            } 
        }
        
        public Vector3 PivotPosition
        {
            get => _pivotPosition;
            set
            {
                _pivotPosition = value;
                _centerPosition = _pivotPosition + Rotation *_centerOffset; // TODO: Need rotation?
            }
        }
        
        public Vector3 PivotCenterOffset => _centerOffset;
        public Quaternion Rotation { get; set; }
        public Vector3 Scale { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 AngularVelocity { get; set; }
        public Vector3 Force { get; private set; }
        public Vector3 Torque { get; private set; }
        public Bounds Bounds { get; private set; }

        private Vector3 _centerPosition;
        private Vector3 _pivotPosition;
        private Vector3 _initialTransformPosition;
        private Quaternion _initialRotation;
        private Vector3 _initialScale;
        private Vector3[] _localVertices;
        private Vector3[] _worldVertices;
        private Vector3[] _localContactPoints;
        private Vector3[] _worldContactPoints;
        private int[] _triangles;
        private Vector3 _centerOffset; // Offset from local origin to geometric center
        
        private readonly float _maxVelocity = 0.01f;
        private readonly float _maxAngularVelocity = 0.01f;

        public SimulationPart(Mesh mesh, Vector3[] localContactPoints, PhysicsSimulationConfiguration configuration)
            : this(mesh, localContactPoints, Vector3.zero, Quaternion.identity, Vector3.one, configuration)
        {
        }

        public SimulationPart(Mesh mesh, Vector3[] localContactPoints, Vector3 initialTransformPosition, Quaternion initialRotation, Vector3 initialScale, PhysicsSimulationConfiguration configuration)
        {
            _maxVelocity = configuration.SimulationMaxVelocity;
            _maxAngularVelocity = configuration.SimulationMaxAngularVelocity;

            _triangles = mesh.triangles;
            _localVertices = mesh.vertices;
            _worldVertices = new Vector3[_localVertices.Length];
            _localContactPoints = localContactPoints;
            _worldContactPoints = new Vector3[_localContactPoints.Length];
            
            // Calculate the geometric center in local space
            var sum = Vector3.zero;
            foreach (var vertex in _localVertices)
            {
                sum += vertex;
            }

            var center = sum / _localVertices.Length;
            _centerOffset = center - _initialTransformPosition;

            _initialTransformPosition = initialTransformPosition;
            _initialRotation = initialRotation;
            _initialScale = initialScale;

            Reset();
        }

        public void ApplyForceAtPoint(Vector3 force, Vector3 worldPoint)
        {
            Force += force;

            // Calculate torque relative to current position
            var r = worldPoint - _centerPosition;
            Torque += Vector3.Cross(r, force);
        }

        public void ApplyForce(Vector3 force)
        {
            Force += force;
        }

        public void ApplyTorque(Vector3 torque)
        {
            Torque += torque;
        }

        private Quaternion NormalizeQuaternion(Quaternion input)
        {
            var sum = 0f;
            for (var i = 0; i < 4; ++i)
            {
                sum += input[i] * input[i];
            }

            var magnitudeInverse = 1 / Mathf.Sqrt(sum);
            for (var i = 0; i < 4; ++i)
            {
                input[i] *= magnitudeInverse;
            }

            return input;
        }

        public void Update()
        {
            Rotation = NormalizeQuaternion(Rotation);
            
            var transformMatrix = Matrix4x4.TRS(_pivotPosition, Rotation, Scale);

            _worldVertices[0] = transformMatrix.MultiplyPoint3x4(_localVertices[0]);
            var tempBounds = new Bounds(_worldVertices[0], Vector3.zero);
            for (var i = 1; i < _localVertices.Length; i++)
            {
                _worldVertices[i] = transformMatrix.MultiplyPoint3x4(_localVertices[i]);
                tempBounds.Encapsulate(_worldVertices[i]);
            }

            if (_localContactPoints != null && _localContactPoints.Length > 0)
            {
                _worldContactPoints[0] = transformMatrix.MultiplyPoint3x4(_localContactPoints[0]);
                //var contactBounds = new Bounds(_worldContactPoints[0], Vector3.zero);
                for (var i = 1; i < _localContactPoints.Length; i++)
                {
                    _worldContactPoints[i] = transformMatrix.MultiplyPoint3x4(_localContactPoints[i]);
                    //contactBounds.Encapsulate(_worldContactPoints[i]);
                }
            }
            
            Bounds = tempBounds;
        }
        
        public void Step(float timeStep)
        {
            // Calculate new velocities using timestep
            Velocity += Force * timeStep;
            AngularVelocity += Torque * timeStep;

            // Apply damping
            //Velocity = ApplyDamping(Velocity, 0.99f);
            //AngularVelocity = ApplyDamping(AngularVelocity, 0.98f);
            
            // Clamp linear velocity while preserving direction
            if (Velocity.magnitude > _maxVelocity)
            {
                Velocity = Velocity.normalized * _maxVelocity;
            }

            // Clamp angular velocity while preserving axis of rotation
            if (AngularVelocity.magnitude > _maxAngularVelocity)
            {
                AngularVelocity = AngularVelocity.normalized * _maxAngularVelocity;
            }
            
            // Update positions using new velocities
            _centerPosition += Velocity;
            Rotation *= Quaternion.Euler(AngularVelocity);
            
            // Reset forces and torques
            Force = Vector3.zero;
            Torque = Vector3.zero;

            // Update world vertices and bounds
            Update();
        }
        
        /*
         private Vector3 ApplyDamping(Vector3 velocity, float dampingFactor = 0.98f)
        {
            return velocity * dampingFactor;
        }
        */

        public void Reset()
        {
            PivotPosition = _initialTransformPosition;
            Rotation = _initialRotation;
            Scale = _initialScale;
            Velocity = Vector3.zero;
            AngularVelocity = Vector3.zero;
            Force = Vector3.zero;
            Torque = Vector3.zero;

            Update();
        }

        public Vector3[] GetVertices(bool worldSpace = true)
        {
            return worldSpace ? _worldVertices : _localVertices;
        }

        public Vector3[] GetContactPoints(bool worldSpace = true)
        {
            return worldSpace ? _worldContactPoints : _localContactPoints;
        }
    }
}