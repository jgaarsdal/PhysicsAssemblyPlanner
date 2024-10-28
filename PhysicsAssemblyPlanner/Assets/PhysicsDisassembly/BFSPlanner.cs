using System.Collections.Generic;
using PhysicsDisassembly.SDF;
using PhysicsDisassembly.Simulation;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class BFSPlanner
    {
        private string _moveId;
        private List<string> _stillIds;
        private bool _useRotation;
        private PhysicsSimulation _simulation;
        private Bounds _stillBounds;
        private BFSPlannerConfiguration _plannerConfiguration;
        private PhysicsSimulationConfiguration _physicsConfiguration;
        
        public BFSPlanner(string moveId, List<string> stillIds, bool useRotation,
            Dictionary<string, GameObject> partObjects, Dictionary<string, SignedDistanceField> partSDFs, 
            BFSPlannerConfiguration plannerConfiguration, PhysicsSimulationConfiguration physicsConfiguration)
        {
            _moveId = moveId;
            _stillIds = stillIds;
            _useRotation = useRotation;
            _physicsConfiguration = physicsConfiguration;
            _plannerConfiguration = plannerConfiguration;
            _simulation = new PhysicsSimulation(partObjects, partSDFs, useRotation, _physicsConfiguration);
            
            CalculateBounds();
        }

        public (string status, float totalDurationSecs, Path path) Plan(float timeoutSecs, int maxDepth)
        {
            _simulation.Reset();

            var initState = GetState();
            var path = GetPath(new List<State> { initState });

            if (IsDisassembled())
            {
                return ("Already disassembled", 0f, path);
            }

            var status = "Failure";
            var startTime = Time.realtimeSinceStartup;
            var step = 0;
            var actions = GetActions(); // TODO shuffle

            var stateQueue = new Queue<(State, Path)>();
            stateQueue.Enqueue((initState, new Path(_moveId)));

            while (stateQueue.Count > 0 && step < maxDepth)
            {
                var (state, currentPath) = stateQueue.Dequeue();

                foreach (var action in actions)
                {
                    var newState = state;
                    var tempPath = currentPath;

                    _simulation.Reset();

                    SetState(state);
                    ApplyAction(action);
                    
                    while (true)
                    {
                        SetState(GetState());

                        for (var i = 0; i < _physicsConfiguration.SimulationFrameSkip; i++)
                        {
                            _simulation.Forward(1);
                            _simulation.CheckCollisions(_moveId);

                            newState = GetState();
                            tempPath.AddState(newState);

                            var durationSecs = Time.realtimeSinceStartup - startTime;
                            if (durationSecs > timeoutSecs)
                            {
                                status = "Timeout";
                                break;
                            }
                        }

                        if (IsDisassembled())
                        {
                            status = "Success";
                            path = tempPath;
                            break;
                        }

                        if (status == "Timeout")
                        {
                            break;
                        }

                        if (AnyStateSimilar(tempPath, newState, _physicsConfiguration.SimulationFrameSkip))
                        {
                            break;
                        }
                    }

                    if (status == "Success" || status == "Timeout")
                    {
                        break;
                    }

                    stateQueue.Enqueue((newState, tempPath));
                }

                if (status == "Success" || status == "Timeout")
                {
                    break;
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
            var force = action.normalized * _physicsConfiguration.SimulationForce;
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
            var moveBounds = _simulation.GetBounds(_moveId);
            return !moveBounds.Intersects(_stillBounds);
        }

        private Vector3[] GetActions()
        {
            if (_useRotation)
            {
                return new Vector3[]
                {
                    new Vector3(0, 0, 0),
                    new Vector3(-1, 0, 0),
                    new Vector3(1, 0, 0),
                    new Vector3(0, 0, 1),
                    new Vector3(0, 0, -1),
                    new Vector3(0, 1, 0),
                    new Vector3(0, -1, 0)
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

        private bool AnyStateSimilar(Path path, State newState, int frameSkip)
        {
            for (var i = 0; i < path.Positions.Count - frameSkip; i++)
            {
                var position = path.Positions[i];
                var statePositionDist = Vector3.Distance(position, newState.Position);
                if (statePositionDist >= _plannerConfiguration.BFSStatePositionThreshold)
                {
                    continue;
                }

                if (_useRotation)
                {
                    var rotation = path.Orientations[i];
                    var stateAngleDist = Quaternion.Angle(rotation, newState.Rotation);
                    if (stateAngleDist >= _plannerConfiguration.BFSStateAngleThreshold)
                    {
                        continue;
                    }
                }

                return true;
            }

            return false;
        }

        private Path GetPath(List<State> states)
        {
            var statePath = new Path(_moveId);
            foreach (var state in states)
            {
                statePath.Positions.Add(state.Position);
                statePath.Orientations.Add(state.Rotation);
            }

            return statePath;
        }

        private void CalculateBounds()
        {
            var tempStillBounds = _simulation.GetBounds(_stillIds[0]);
            for (var i = 1; i < _stillIds.Count; i++)
            {
                tempStillBounds.Encapsulate(_simulation.GetBounds(_stillIds[i]));
            }

            _stillBounds = tempStillBounds;
        }
    }
}