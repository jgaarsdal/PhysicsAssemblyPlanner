using UnityEngine;

namespace PhysicsDisassembly.RRTConnect
{
    public class Node
    {
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Node Parent { get; set; }

        public Node(Vector3 position, Quaternion rotation, Node parent = null)
        {
            Position = position;
            Rotation = rotation;
            Parent = parent;
        }
    }
}