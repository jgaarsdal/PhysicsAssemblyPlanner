using System.Threading.Tasks;
using UnityEngine;

public class SDFCollisionTester : MonoBehaviour
{
    [SerializeField] private GameObject _partA = default;
    [SerializeField] private GameObject _partB = default;
    [Space]
    [SerializeField] private float _sdfDefaultCellSize = 0.05f;
    [SerializeField] private float _maxDistance = 1f;
    [SerializeField] private float _sdfBoxPadding = 0.1f;
    [SerializeField] private bool _useGPU = true;
    [SerializeField] private bool _visualize = true;

    private SignedDistanceField _sdfPartA;
    private SignedDistanceField _sdfPartB;
    
    [ContextMenu("Compute SDF")]
    public void ComputeSDFButton()
    {
        ComputeSDF();
    }
    
    [ContextMenu("Test SDF Collision")]
    public void TestCollisionButton()
    {
        TestCollision();
    }

    private async void ComputeSDF()
    {
        _sdfPartA = new SignedDistanceField(_partA, _sdfDefaultCellSize, _sdfBoxPadding);
        _sdfPartB = new SignedDistanceField(_partB, _sdfDefaultCellSize, _sdfBoxPadding);

        await Task.WhenAll(_sdfPartA.ComputeSDF(_useGPU), _sdfPartB.ComputeSDF(_useGPU));
    }

    private void TestCollision()
    {
        var relativePosition = _sdfPartA.ObjectTransform.InverseTransformPoint(_sdfPartB.ObjectTransform.position);
        var relativeRotation = Quaternion.Inverse(_sdfPartA.ObjectTransform.rotation) * _sdfPartB.ObjectTransform.rotation;
        
        _sdfPartA.UpdateVertices();
        _sdfPartB.UpdateVertices();
        
        var isColliding = _sdfPartA.CheckCollision(_sdfPartB, relativePosition, relativeRotation);
        Debug.Log($"SDFCollisionTester: IS COLLIDING = {isColliding}", this);
    }
}
