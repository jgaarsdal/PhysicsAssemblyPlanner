using UnityEngine;

namespace PhysicsDisassembly.RRTConnect
{
    public class SamplingCandidate
    {
        public Vector3 Position;
        public float NeighborDistance;
        public Node NearestNode;
        public bool IsValidPath;

        public SamplingCandidate(Vector3 position, float distance, Node nearest, bool validPath)
        {
            Position = position;
            NeighborDistance = distance;
            NearestNode = nearest;
            IsValidPath = validPath;
        }
    }
}