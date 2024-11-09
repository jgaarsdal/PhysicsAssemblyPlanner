using UnityEngine;

namespace PhysicsDisassembly
{
    public class State
    {
        public Vector3 PivotPosition { get; set; }
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 AngularVelocity { get; set; }

        public State(Vector3 position, Vector3 pivotPosition, Quaternion rotation, Vector3 velocity, Vector3 angularVelocity)
        {
            Position = position;
            PivotPosition = pivotPosition;
            Rotation = rotation;
            Velocity = velocity;
            AngularVelocity = angularVelocity;
        }
    }
}