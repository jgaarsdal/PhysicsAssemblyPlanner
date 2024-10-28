using System;
using UnityEngine;

namespace PhysicsDisassembly
{
    public struct State : IEquatable<State>
    {
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Vector3 Velocity { get; set; }
        public Vector3 AngularVelocity { get; set; }

        public State(Vector3 position, Quaternion rotation, Vector3 velocity, Vector3 angularVelocity)
        {
            Position = position;
            Rotation = rotation;
            Velocity = velocity;
            AngularVelocity = angularVelocity;
        }

        public bool Equals(State other)
        {
            return Position.Equals(other.Position) &&
                   Rotation.Equals(other.Rotation) &&
                   Velocity.Equals(other.Velocity) &&
                   AngularVelocity.Equals(other.AngularVelocity);
        }

        public override bool Equals(object obj)
        {
            return obj is State other && Equals(other);
        }

        public override int GetHashCode()
        {
            return HashCode.Combine(Position, Rotation, Velocity, AngularVelocity);
        }
    }
}