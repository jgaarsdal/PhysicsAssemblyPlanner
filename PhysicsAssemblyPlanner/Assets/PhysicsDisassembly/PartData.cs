using PhysicsDisassembly.SDF;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class PartData
    {
        public string PartId => _partId;
        public Transform PartObject => _partObject;
        public SignedDistanceField SDF => _sdf;
        public Vector3[] LocalContactPoints => _localContactPoints;
        
        private readonly string _partId;
        private readonly Transform _partObject;
        private readonly SignedDistanceField _sdf;
        private readonly Vector3[] _localContactPoints;

        public PartData(string partId, Transform partObject, SignedDistanceField sdf, Vector3[] localContactPoints)
        {
            _partId = partId;
            _partObject = partObject;
            _sdf = sdf;
            _localContactPoints = localContactPoints;
        }
    }
}