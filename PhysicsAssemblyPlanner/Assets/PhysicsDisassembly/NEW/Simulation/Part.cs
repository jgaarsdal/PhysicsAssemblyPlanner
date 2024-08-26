using UnityEngine;

public class Part
{
    public Vector3 Position { get; set; }
    public Quaternion Rotation { get; set; }
    public Vector3 Velocity { get; set; }
    public Vector3 AngularVelocity { get; set; }
    public Vector3 Force { get; private set; }
    public Vector3 Torque { get; private set; }
    public Bounds Bounds { get; private set; }

    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private Vector3[] _localVertices;
    private Vector3[] _worldVertices;

    public Part(Mesh mesh)
    {
        _localVertices = mesh.vertices;
        _worldVertices = new Vector3[_localVertices.Length];
        this.Bounds = mesh.bounds;
        _initialPosition = Vector3.zero;
        _initialRotation = Quaternion.identity;
        Reset();
    }

    public void ApplyForce(Vector3 force)
    {
        Force += force;
    }

    public void ApplyTorque(Vector3 torque)
    {
        Torque += torque;
    }

    public void Update()
    {
        // Update world vertices and bounding box
        Bounds = new Bounds(Position, Vector3.zero);
        for (int i = 0; i < _localVertices.Length; i++)
        {
            _worldVertices[i] = Rotation * _localVertices[i] + Position;
            Bounds.Encapsulate(_worldVertices[i]);
        }
    }

    public void Step(float timeStep)
    {
        // Simple Euler integration
        Velocity += Force * timeStep;
        AngularVelocity += Torque * timeStep;
        Position += Velocity * timeStep;
        Rotation *= Quaternion.Euler(AngularVelocity * timeStep);

        // Reset forces and torques
        Force = Vector3.zero;
        Torque = Vector3.zero;

        // Update world vertices and bounds
        Update();
    }

    public void Reset()
    {
        Position = _initialPosition;
        Rotation = _initialRotation;
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
}
