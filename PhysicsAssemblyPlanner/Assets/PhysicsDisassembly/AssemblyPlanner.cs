using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;

public class AssemblyPlanner : MonoBehaviour
{
    private struct PhysicsAction
    {
        public Vector3 Force;
        public Vector3 Torque;

        public PhysicsAction(Vector3 force, Vector3 torque)
        {
            Force = force;
            Torque = torque;
        }
    }
    
    [SerializeField] private GameObject[] _assemblyParts = default;
    [SerializeField] private float _forceMagnitude = 100f;
    [SerializeField] private float _torqueMagnitude = 10f;
    [SerializeField] private float _timeStep = 0.1f;
    [SerializeField] private float _rotationStep = 15f; // Degrees
    [SerializeField] private float _similarityThreshold = 0.05f;
    [SerializeField] private int _maxBFSDepth = 10;
    [SerializeField] private float _timeout = 300f; // 5 minutes
    [SerializeField] private float _collisionMargin = 0.01f;
    
    [SerializeField] private float _staticFrictionCoefficient = 0.6f;
    [SerializeField] private float _kineticFrictionCoefficient = 0.4f;
    [SerializeField] private Vector3 _gravity = new Vector3(0, -9.81f, 0);
    [SerializeField] private float _airDensity = 1.225f; // kg/m^3 (at sea level and 15°C)
    [SerializeField] private float _dragCoefficient = 0.47f; // Sphere drag coefficient
    
    [SerializeField] private float _heuristicWeight = 1.0f;
    [SerializeField] private int _maxIterations = 1000000;
    [SerializeField] private int _maxDepth = 50;
    [SerializeField] private int _parallelTasks = 8;
    
    private List<SignedDistanceField> _partSDFs;
    private List<Vector3> _initialPositions;
    private List<Quaternion> _initialRotations;

    private ConcurrentDictionary<(int, Vector3, Quaternion), bool> _collisionCache = new ConcurrentDictionary<(int, Vector3, Quaternion), bool>();
    
    [ContextMenu("Run Planner")]
    public void VisualizeSDFButton()
    {
        PlanDisassemblySequence();
    }
    
    private void Start()
    {
        InitializeAssembly();
    }

    private async Task InitializeAssembly()
    {
        _partSDFs = new List<SignedDistanceField>();
        _initialPositions = new List<Vector3>();
        _initialRotations = new List<Quaternion>();
        
        foreach (var part in _assemblyParts)
        {
            var meshFilter = part.GetComponent<MeshFilter>();
            var sdfCalculator = new SignedDistanceField(meshFilter.sharedMesh, 224, 0.1f);
            
            await sdfCalculator.ComputeSDF();
            
            _partSDFs.Add(sdfCalculator);
            _initialPositions.Add(part.transform.position);
            _initialRotations.Add(part.transform.rotation);
        }
        
        Debug.Log("Assembly Planner initialized!");
    }
    
    public async Task<List<DisassemblyStep>> PlanDisassemblySequence()
    {
        var disassemblySequence = new List<DisassemblyStep>();
        var remainingParts = new List<int>(Enumerable.Range(0, _assemblyParts.Length));

        while (remainingParts.Count > 0)
        {
            var step = await FindNextDisassemblyStep(remainingParts);
            if (step != null)
            {
                disassemblySequence.Add(step);
                remainingParts.Remove(step.PartIndex);
            }
            else
            {
                Debug.LogError("Failed to find a valid disassembly sequence.");
                break;
            }
        }
        
        Debug.Log("Assembly Planner finished!");

        return disassemblySequence;
    }
    
    private async Task<DisassemblyStep> FindNextDisassemblyStep(List<int> remainingParts)
    {
        foreach (var partIndex in remainingParts)
        {
            var path = await IterativeDeepeningSearch(partIndex);
            if (path != null)
            {
                return new DisassemblyStep { PartIndex = partIndex, Path = path };
            }
        }
        return null;
    }

    private async Task<List<(Vector3, Quaternion)>> IterativeDeepeningSearch(int partIndex)
    {
        for (var depth = 1; depth <= _maxDepth; depth++)
        {
            var result = await DisassemblyPathPlanning(partIndex, depth);
            if (result != null)
            {
                return result;
            }
        }
        return null;
    }

    private async Task<List<(Vector3, Quaternion)>> DisassemblyPathPlanning(int partIndex, int maxDepth)
    {
        var priorityQueue = new PriorityQueue<DisassemblyNode>();
        var initialNode = new DisassemblyNode
        {
            Position = _initialPositions[partIndex],
            Rotation = _initialRotations[partIndex],
            Path = new List<(Vector3, Quaternion)> { (_initialPositions[partIndex], _initialRotations[partIndex]) },
            Cost = 0
        };
        priorityQueue.Enqueue(initialNode, CalculatePriority(initialNode, partIndex));

        var visitedStates = new HashSet<(Vector3, Quaternion)>();
        var startTime = Time.realtimeSinceStartup;
        int iterations = 0;

        while (priorityQueue.Count > 0 && iterations < _maxIterations && Time.realtimeSinceStartup - startTime < _timeout)
        {
            var node = priorityQueue.Dequeue();
            iterations++;

            if (IsDisassembled(partIndex, node.Position))
            {
                return node.Path;
            }

            if (node.Path.Count >= maxDepth)
            {
                continue;
            }

            var tasks = new List<Task<DisassemblyNode>>();
            foreach (var action in GetActions())
            {
                tasks.Add(Task.Run(() => SimulateAndCheckAction(partIndex, node, action)));
            }

            var results = await Task.WhenAll(tasks);

            foreach (var newNode in results.Where(n => n != null))
            {
                if (!visitedStates.Contains((newNode.Position, newNode.Rotation)))
                {
                    float priority = CalculatePriority(newNode, partIndex);
                    priorityQueue.Enqueue(newNode, priority);
                    visitedStates.Add((newNode.Position, newNode.Rotation));
                }
            }

            if (iterations % 1000 == 0)
            {
                await Task.Yield();
            }
        }

        return null; // No valid path found within the current depth limit
    }

    private float CalculatePriority(DisassemblyNode node, int partIndex)
    {
        float heuristic = CalculateHeuristic(partIndex, node.Position);
        return node.Cost + _heuristicWeight * heuristic;
    }

    private float CalculateHeuristic(int partIndex, Vector3 position)
    {
        Vector3 assemblyCenter = GetAssemblyCenter(partIndex);
        return Vector3.Distance(position, assemblyCenter);
    }

    private Vector3 GetAssemblyCenter(int excludePartIndex)
    {
        Vector3 sum = Vector3.zero;
        int count = 0;
        for (int i = 0; i < _assemblyParts.Length; i++)
        {
            if (i != excludePartIndex)
            {
                sum += _assemblyParts[i].transform.position;
                count++;
            }
        }
        return sum / count;
    }

    private DisassemblyNode SimulateAndCheckAction(int partIndex, DisassemblyNode currentNode, PhysicsAction action)
    {
        var (newPosition, newRotation) = SimulateAction(partIndex, currentNode.Position, currentNode.Rotation, action);
        
        if (!CheckCollisionCached(partIndex, newPosition, newRotation))
        {
            var newPath = new List<(Vector3, Quaternion)>(currentNode.Path) { (newPosition, newRotation) };
            return new DisassemblyNode
            {
                Position = newPosition,
                Rotation = newRotation,
                Path = newPath,
                Cost = currentNode.Cost + 1
            };
        }

        return null;
    }

    private bool CheckCollisionCached(int partIndex, Vector3 position, Quaternion rotation)
    {
        var key = (partIndex, position, rotation);
        if (_collisionCache.TryGetValue(key, out bool collisionResult))
        {
            return collisionResult;
        }

        collisionResult = CheckCollision(partIndex, position, rotation);
        _collisionCache[key] = collisionResult;
        return collisionResult;
    }
    
    private (Vector3, Quaternion) SimulateAction(int partIndex, Vector3 position, Quaternion rotation, PhysicsAction action)
    {
        var rb = _assemblyParts[partIndex].GetComponent<Rigidbody>();
        var velocity = rb.velocity;
        var angularVelocity = rb.angularVelocity;

        // Apply gravity
        var gravityForce = _gravity * rb.mass;

        // Calculate total force (action force + gravity)
        var totalForce = action.Force + gravityForce;

        // Calculate air resistance
        var area = Mathf.PI * rb.transform.localScale.x * rb.transform.localScale.x; // Approximating area as if it were a sphere
        var dragForce = -0.5f * _airDensity * velocity.sqrMagnitude * _dragCoefficient * area * velocity.normalized;
        totalForce += dragForce;

        // Apply friction
        var normalForce = -Vector3.Project(totalForce, _gravity.normalized);
        var normalForceMagnitude = normalForce.magnitude;
        var frictionForce = Vector3.zero;

        if (velocity.sqrMagnitude < 0.001f) // Apply static friction
        {
            var tangentialForce = totalForce - Vector3.Project(totalForce, _gravity.normalized);
            var maxStaticFriction = _staticFrictionCoefficient * normalForceMagnitude;

            if (tangentialForce.magnitude < maxStaticFriction)
            {
                frictionForce = -tangentialForce;
            }
            else
            {
                frictionForce = -tangentialForce.normalized * maxStaticFriction;
            }
        }
        else // Apply kinetic friction
        {
            frictionForce = -velocity.normalized * _kineticFrictionCoefficient * normalForceMagnitude;
        }

        totalForce += frictionForce;

        // Calculate acceleration
        var acceleration = totalForce / rb.mass;
        var angularAcceleration = action.Torque / rb.inertiaTensor.magnitude;

        // Update velocity and position
        velocity += acceleration * _timeStep;
        position += velocity * _timeStep + 0.5f * acceleration * _timeStep * _timeStep;

        // Update angular velocity and rotation
        angularVelocity += angularAcceleration * _timeStep;
        var deltaRotation = Quaternion.Euler(angularVelocity * Mathf.Rad2Deg * _timeStep);
        rotation = deltaRotation * rotation;

        return (position, rotation);
    }

    private PhysicsAction[] GetActions()
    {
        return new PhysicsAction[]
        {
            new (Vector3.right * _forceMagnitude, Vector3.zero),
            new (Vector3.left * _forceMagnitude, Vector3.zero),
            new (Vector3.up * _forceMagnitude, Vector3.zero),
            new (Vector3.down * _forceMagnitude, Vector3.zero),
            new (Vector3.forward * _forceMagnitude, Vector3.zero),
            new (Vector3.back * _forceMagnitude, Vector3.zero),
            new (Vector3.zero, Vector3.right * _torqueMagnitude),
            new (Vector3.zero, Vector3.left * _torqueMagnitude),
            new (Vector3.zero, Vector3.up * _torqueMagnitude),
            new (Vector3.zero, Vector3.down * _torqueMagnitude),
            new (Vector3.zero, Vector3.forward * _torqueMagnitude),
            new (Vector3.zero, Vector3.back * _torqueMagnitude)
        };
    }
    
    private bool IsDisassembled(int partIndex, Vector3 position)
    {
        // Check if the part is outside the bounding box of the assembly
        var assemblyBounds = GetAssemblyBounds(partIndex);
        return !assemblyBounds.Contains(position);
    }
    
    private Bounds GetAssemblyBounds(int excludePartIndex)
    {
        var bounds = new Bounds();
        for (var i = 0; i < _assemblyParts.Length; i++)
        {
            if (i != excludePartIndex)
            {
                bounds.Encapsulate(_assemblyParts[i].GetComponent<Renderer>().bounds);
            }
        }
        return bounds;
    }
    
    private bool CheckCollision(int partIndex, Vector3 position, Quaternion rotation)
    {
        for (var i = 0; i < _assemblyParts.Length; i++)
        {
            if (i != partIndex)
            {
                // First, check bounding box collision
                var partBounds = _assemblyParts[partIndex].GetComponent<Renderer>().bounds;
                partBounds.center = position;
                
                var otherBounds = _assemblyParts[i].GetComponent<Renderer>().bounds;

                if (partBounds.Intersects(otherBounds))
                {
                    // If bounding boxes intersect, perform detailed SDF check
                    var partToWorldMatrix = Matrix4x4.TRS(position, rotation, Vector3.one);
                    var worldToOtherMatrix = _assemblyParts[i].transform.worldToLocalMatrix;
                    var partToOtherMatrix = worldToOtherMatrix * partToWorldMatrix;

                    // Check multiple points on the part's bounding box
                    var checkPoints = GetBoundingBoxCheckPoints(partBounds);

                    foreach (var point in checkPoints)
                    {
                        var localPoint = partToOtherMatrix.MultiplyPoint3x4(point);
                        var gridPosition = _partSDFs[i].WorldToGridPosition(localPoint);

                        if (_partSDFs[i].GetDistance(gridPosition) <= _collisionMargin)
                        {
                            // Collision detected
                            return true; 
                        }
                    }
                }
            }
        }
        
        // No collision
        return false; 
    }
    
    private Vector3[] GetBoundingBoxCheckPoints(Bounds bounds)
    {
        var min = bounds.min;
        var max = bounds.max;
        return new Vector3[]
        {
            new (min.x, min.y, min.z),
            new (max.x, min.y, min.z),
            new (min.x, max.y, min.z),
            new (max.x, max.y, min.z),
            new (min.x, min.y, max.z),
            new (max.x, min.y, max.z),
            new (min.x, max.y, max.z),
            new (max.x, max.y, max.z),
            bounds.center
        };
    }
}
