using System.Collections.Generic;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class Path
    {
        public string PartID { get; private set; }
        public Transform PartObject { get; set; }
        public List<Vector3> Positions { get; set; }
        public List<Quaternion> Orientations { get; set; }

        public Path(string partID, Transform partObject)
        {
            PartID = partID;
            PartObject = partObject;
            Positions = new List<Vector3>();
            Orientations = new List<Quaternion>();
        }

        public void AddState(State newState)
        {
            Positions.Add(newState.PivotPosition);
            Orientations.Add(newState.Rotation);
        }
    }
}
