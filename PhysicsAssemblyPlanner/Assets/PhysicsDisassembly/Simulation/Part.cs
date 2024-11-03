using UnityEngine;

namespace PhysicsDisassembly.Simulation
{
    public class Part
    {
        public Vector3 Position { get; set; }
        public Vector3 PivotPosition => Position - (Rotation * _centerOffset);
        public Quaternion Rotation { get; set; }
        public Vector3 Scale { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 AngularVelocity { get; set; }
        public Vector3 Force { get; private set; }
        public Vector3 Torque { get; private set; }
        public Bounds Bounds { get; private set; }

        private Vector3 _initialTransformPosition;
        private Quaternion _initialRotation;
        private Vector3 _initialScale;
        private Vector3[] _localVertices;
        private Vector3[] _worldVertices;

        private Vector3 _centerOffset; // Offset from local origin to geometric center

        public Part(Mesh mesh)
            : this(mesh, Vector3.zero, Quaternion.identity, Vector3.one)
        {
        }

        public Part(Mesh mesh, Vector3 initialTransformPosition, Quaternion initialRotation, Vector3 initialScale)
        {
            _localVertices = mesh.vertices;
            _worldVertices = new Vector3[_localVertices.Length];

            // Calculate the geometric center in local space
            var sum = Vector3.zero;
            foreach (var vertex in _localVertices)
            {
                sum += vertex;
            }

            _centerOffset = sum / _localVertices.Length;

            _initialTransformPosition = initialTransformPosition;
            _initialRotation = initialRotation;
            _initialScale = initialScale;

            Reset();
        }

        public void ApplyForceAtPoint(Vector3 force, Vector3 worldPoint)
        {
            Force += force;

            // Calculate torque relative to current position
            var r = worldPoint - Position;
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

            // Calculate transform position (pivot point) from center position
            var pivotPosition = Position - Rotation * _centerOffset;
            var transformMatrix = Matrix4x4.TRS(pivotPosition, Rotation, Scale);

            _worldVertices[0] = transformMatrix.MultiplyPoint3x4(_localVertices[0]);
            var tempBounds = new Bounds(_worldVertices[0], Vector3.zero);
            for (var i = 1; i < _localVertices.Length; i++)
            {
                _worldVertices[i] = transformMatrix.MultiplyPoint3x4(_localVertices[i]);
                tempBounds.Encapsulate(_worldVertices[i]);
            }
            
            Bounds = tempBounds;
        }

        public void Step(float timeStep)
        {
            // Calculate new velocities using timestep
            Velocity += Force * timeStep;
            AngularVelocity += Torque * timeStep;

            // TODO Tweak settings
            
            // Apply damping
            //Velocity = ApplyDamping(Velocity, 0.99f);
            //AngularVelocity = ApplyDamping(AngularVelocity, 0.98f);
            
            // Clamp linear velocity while preserving direction
            Debug.Log("Velocity.magnitude 1: " + Velocity.magnitude);
            if (Velocity.magnitude > 0.01f)
            {
                Velocity = Velocity.normalized * 0.01f;
            }
            Debug.Log("Velocity.magnitude 2: " + Velocity.magnitude);

            // Clamp angular velocity while preserving axis of rotation
            //if (AngularVelocity.magnitude > 5f)
            //{
            //    AngularVelocity = AngularVelocity.normalized * 5f;
            //}
            
            // Update positions using new velocities
            Position += Velocity;
            Rotation *= Quaternion.Euler(AngularVelocity);
            
            // Reset forces and torques
            Force = Vector3.zero;
            Torque = Vector3.zero;

            // Update world vertices and bounds
            Update();
        }

        public void Reset()
        {
            Position = _initialTransformPosition + _initialRotation * _centerOffset;
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
        
        private Vector3 ApplyDamping(Vector3 velocity, float dampingFactor = 0.98f)
        {
            return velocity * dampingFactor;
        }
    }
}