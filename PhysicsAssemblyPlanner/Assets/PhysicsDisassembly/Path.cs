using System.Collections.Generic;
using UnityEngine;

namespace PhysicsDisassembly
{
    public struct Path
    {
        public string PartID { get; private set; }
        public List<Vector3> Positions { get; set; }
        public List<Quaternion> Orientations { get; set; }

        public Path(string partID)
        {
            PartID = partID;
            Positions = new List<Vector3>();
            Orientations = new List<Quaternion>();
        }

        public void AddState(State newState)
        {
            Positions.Add(newState.Position);
            Orientations.Add(newState.Rotation);
        }
    }
}
