using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class BFSPlanner
{
    private const float _statePosDistThreshold = 0.05f;
    private const float _stateAngleDistThreshold = 0.5f;
    
    private string _moveId;
    private List<string> _stillIds;
    private bool _useRotation;
    private float _simulationForce;
    private int _simulationFrameSkip;
    
    private Dictionary<string, SignedDistanceField> _partSDFs;
    private PhysicsSimulation _simulation;
    
    //private Vector3 _minBoxMove;
    //private Vector3 _maxBoxMove;
    //private Vector3 _sizeBoxMove;
    private Vector3 _minBoxStill;
    private Vector3 _maxBoxStill;
    //private Vector3 _sizeBoxStill;
    //private Vector3 _stateLowerBound;
    //private Vector3 _stateUpperBound;

    public BFSPlanner(string moveId, List<string> stillIds, bool useRotation, 
        float simulationForce, int simulationFrameSkip, Dictionary<string, Mesh> partMeshes, Dictionary<string, SignedDistanceField> partSDFs)
    {
        _moveId = moveId;
        _stillIds = stillIds;
        _partSDFs = partSDFs;
        _useRotation = useRotation;
        _simulationForce = simulationForce;
        _simulationFrameSkip = simulationFrameSkip;

        _simulation = new PhysicsSimulation(partMeshes);
        CalculateBounds();
    }

    public (string status, float totalDurationSecs, List<Vector3> path) Plan(float timeoutSecs, int maxDepth)
    {
        _simulation.Reset();

        var tree = new Tree();
        var initState = GetState();
        
        tree.AddNode(initState);

        if (IsDisassembled())
        {
            return ("Start with goal", 0f, new List<Vector3>());
        }

        var status = "Failure";
        var startTime = Time.realtimeSinceStartup;
        var step = 0;
        List<Vector3> path = null;

        var stateQueue = new Queue<State>();
        stateQueue.Enqueue(initState);

        while (stateQueue.Count > 0 && step < maxDepth)
        {
            var state = stateQueue.Dequeue();
            
            foreach (var action in GetActions())
            {
                SetState(state);
                ApplyAction(action);
                
                _simulation.UpdateParts();

                var statesBetween = new List<State>();
                for (var i = 0; i < _simulationFrameSkip; i++)
                {
                    _simulation.Forward(1);
                    
                    var stateBetween = GetState();
                    statesBetween.Add(stateBetween);

                    var durationSecs = Time.realtimeSinceStartup - startTime;
                    if (durationSecs > timeoutSecs)
                    {
                        status = "Timeout";
                        return (status, durationSecs, null);
                    }
                }

                var newState = statesBetween[statesBetween.Count - 1];

                if (!AnyStateSimilar(tree.GetNodes(), newState))
                {
                    tree.AddNode(newState);
                    tree.AddEdge(state, newState, action, statesBetween);
                    stateQueue.Enqueue(newState);

                    if (IsDisassembled())
                    {
                        status = "Success";
                        path = GetPath(tree, newState);
                        return (status, Time.realtimeSinceStartup - startTime, path);
                    }
                }
            }

            step++;
        }
        
        return (status, Time.realtimeSinceStartup - startTime, path);
    }

    private State GetState()
    {
        var position = _simulation.GetPosition(_moveId);
        var rotation = _simulation.GetRotation(_moveId);
        var velocity = _simulation.GetVelocity(_moveId);
        var angularVelocity = _simulation.GetAngularVelocity(_moveId);
        return new State(position, rotation, velocity, angularVelocity);
    }

    private void SetState(State state)
    {
        _simulation.SetPosition(_moveId, state.Position);
        _simulation.SetRotation(_moveId, state.Rotation);
        _simulation.SetVelocity(_moveId, state.Velocity);
        _simulation.SetAngularVelocity(_moveId, state.AngularVelocity);
    }

    private void ApplyAction(Vector3 action)
    {
        var force = action.normalized * _simulationForce;
        if (_useRotation)
        {
            var torque = new Vector3(force.x * 3, force.y * 3, force.z);
            _simulation.ApplyForceAndTorque(_moveId, force, torque);
        }
        else
        {
            _simulation.ApplyForce(_moveId, force);
        }
    }

    private bool IsDisassembled()
    {
        var movePartSDF = _partSDFs[_moveId];
        var position = _simulation.GetPosition(_moveId);
        var rotation = _simulation.GetRotation(_moveId);

        foreach (var stillId in _stillIds)
        {
            if (movePartSDF.CheckCollision(_partSDFs[stillId], position, rotation))
            {
                return false;
            }
        }

        var moveVertices = _simulation.GetVertices(_moveId, true);
        var minMove = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        var maxMove = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        foreach (var vertex in moveVertices)
        {
            minMove = Vector3.Min(minMove, vertex);
            maxMove = Vector3.Max(maxMove, vertex);
        }

        var moveContainStill = Vector3.Min(minMove, _minBoxStill) == minMove && Vector3.Max(maxMove, _maxBoxStill) == maxMove;
        var stillContainMove = Vector3.Min(minMove, _minBoxStill) == _minBoxStill && Vector3.Max(maxMove, _maxBoxStill) == _maxBoxStill;

        return !(moveContainStill || stillContainMove);
    }

    private Vector3[] GetActions()
    {
        if (_useRotation)
        {
            return new Vector3[]
            {
                new Vector3(0, 0, 0),
                new Vector3(0, 0, 1),
                new Vector3(0, 0, -1),
                new Vector3(0, 1, 0),
                new Vector3(0, -1, 0),
                new Vector3(1, 0, 0),
                new Vector3(-1, 0, 0)
            };
        }
        else
        {
            return new Vector3[]
            {
                new Vector3(0, 0, 1),
                new Vector3(0, 0, -1),
                new Vector3(0, 1, 0),
                new Vector3(0, -1, 0),
                new Vector3(1, 0, 0),
                new Vector3(-1, 0, 0)
            };
        }
    }

    private bool AnyStateSimilar(List<State> states, State newState)
    {
        foreach (var state in states)
        {
            if (StateSimilar(state, newState))
            {
                return true;
            }
        }
        return false;
    }

    private bool StateSimilar(State state1, State state2)
    {
        var statePositionDist = Vector3.Distance(state1.Position, state2.Position);
        if (statePositionDist >= _statePosDistThreshold)
        {
            return false;
        }
        
        if (_useRotation)
        {
            var stateAngleDist = Quaternion.Angle(state1.Rotation, state2.Rotation);
            return stateAngleDist < _stateAngleDistThreshold;
        }

        return true;
    }

    private List<Vector3> GetPath(Tree tree, State endState)
    {
        var statePath = tree.GetRootPath(endState);
        return statePath.Select(state => state.Position).ToList();
    }

    private void CalculateBounds()
    {
        //_minBoxMove = _simulation.GetBounds(_moveId).min;
        //_maxBoxMove = _simulation.GetBounds(_moveId).max;
        //_sizeBoxMove = _maxBoxMove - _minBoxMove;

        _minBoxStill = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        _maxBoxStill = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        foreach (var stillId in _stillIds)
        {
            var minBounds = _simulation.GetBounds(stillId).min;
            var maxBounds = _simulation.GetBounds(stillId).max;
            _minBoxStill = Vector3.Min(_minBoxStill, minBounds);
            _maxBoxStill = Vector3.Max(_maxBoxStill, maxBounds);
        }
        //_sizeBoxStill = _maxBoxStill - _minBoxStill;

        //_stateLowerBound = (_minBoxStill - _maxBoxMove) - 0.5f * _sizeBoxMove;
        //_stateUpperBound = (_maxBoxStill - _minBoxMove) + 0.5f * _sizeBoxMove;
    }
}