using System;
using UnityEngine;

namespace PhysicsDisassembly
{
    [Serializable]
    public struct AssemblyPart
    {
        public Transform PartObject;
        public Transform PartFinalState;
    }
}