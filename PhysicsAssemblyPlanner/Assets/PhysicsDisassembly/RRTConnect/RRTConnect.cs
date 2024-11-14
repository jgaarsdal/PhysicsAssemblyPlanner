using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PhysicsDisassembly.RRTConnect
{
    public class RRTConnect
    {
        private readonly bool _useRotation = false;
        private readonly float _stepSize = 0.1f;
        private readonly float _rotationStepSize = 15f; // degrees
        private readonly int _maxIterations = 10000;
        private readonly float _connectDistance = 1.0f;
        private readonly int _randomPointAttempts = 10; // Number of random samples to test
        private readonly float _explorationBias = 0.5f;
        private readonly float _workspaceBoundsBufferPercentage = 0.5f;
        private readonly string _partId;
        private readonly Transform _partObject;
        private readonly Transform[] _otherObjects;
        
        private List<Node> _treeStart = new List<Node>();
        private List<Node> _treeGoal = new List<Node>();
        private Bounds _partBounds;
        private Bounds[] _otherPartBounds;
        private Bounds _workspaceBounds;
        private bool _treesAreSwapped = false;

        public RRTConnect(string partId, Transform partObject, Transform[] otherObjects, RRTConfiguration configuration)
        {
            _partId = partId;
            _partObject = partObject;
            _otherObjects = otherObjects;

            _useRotation = configuration.RRTUseRotation;
            _stepSize = configuration.RRTStepSize;
            _rotationStepSize = configuration.RRTRotationStepSize;
            _maxIterations = configuration.RRTMaxIterations;
            _connectDistance = configuration.RRTConnectDistance;
            _randomPointAttempts = configuration.RRTRandomPointAttempts;
            _explorationBias = configuration.RRTExplorationBias;
            _workspaceBoundsBufferPercentage = configuration.RRTWorkspaceBoundsBufferPercentage;
        }
        
        public Path PlanPath(State startState, State goalState, State[] otherPartStates, int randomSeed)
        {
            UnityEngine.Random.InitState(randomSeed);
            
            // Get the moving part bounds at its start state
            _partBounds = GetPartBounds(_partObject, startState, true);
            
            // Get the other parts bounds at their start states
            _otherPartBounds = GetPartBounds(_otherObjects, otherPartStates, false);
            
            // Update the bounds of other parts
            for (var i = 0; i < _otherPartBounds.Length; i++)
            {
                _otherPartBounds[i].center = otherPartStates[i].Position;
            }
            
            // Initialize trees
            _treeStart.Clear();
            _treeGoal.Clear();
            _treeStart.Add(new Node(startState.Position, startState.Rotation));
            _treeGoal.Add(new Node(goalState.Position, goalState.Rotation));
            
            _workspaceBounds = GetWorkspaceBounds(startState, goalState);
            
            for (var i = 0; i < _maxIterations; i++)
            {
                // Get random configuration with Voronoi bias
                var randomPos = GetRandomConfigWithVoronoiBias(_treeStart);
                var randomRot = _useRotation ? Random.rotation : Quaternion.identity;

                var newNode = GrowTree(_treeStart, randomPos, randomRot);
                if (newNode != null)
                {
                    // Try to connect to goal tree
                    if (TryConnect(newNode, out List<Node> connectionPath))
                    {
                        Debug.Log($"RRTConnect: Succeeded in finding a path within {i + 1}/{_maxIterations} iterations");
                        
                        // Path found - Reconstruct the complete path
                        return ReconstructPath(newNode, connectionPath);
                    }
                }

                // Swap trees for next iteration
                SwapTrees();
            }

            Debug.Log($"RRTConnect: Failed to find a path within {_maxIterations} iterations");
            return null;
        }
        
        private Bounds GetWorkspaceBounds(State startState, State goalState)
        {
            var workspaceBounds = new Bounds(startState.Position, _partBounds.size);
            workspaceBounds.Encapsulate(new Bounds(goalState.Position, _partBounds.size));
            
            foreach (var otherBounds in _otherPartBounds)
            {
                workspaceBounds.Encapsulate(otherBounds);
            }
            
            // Apply a buffer
            workspaceBounds.Expand(_partBounds.size);
            workspaceBounds.Expand(workspaceBounds.size * _workspaceBoundsBufferPercentage);

            return workspaceBounds;
        }

        private Node GrowTree(List<Node> tree, Vector3 targetPos, Quaternion targetRot)
        {
            var nearest = FindNearestNode(tree, targetPos);

            // Calculate step direction
            var direction = (targetPos - nearest.Position).normalized;
            var newPos = nearest.Position + direction * _stepSize;

            // Interpolate rotation
            var newRot = _useRotation
                ? Quaternion.RotateTowards(nearest.Rotation, targetRot, _rotationStepSize)
                : nearest.Rotation;

            // Check for collision
            if (HasCollision(newPos))
            {
                return null;
            }

            var newNode = new Node(newPos, newRot, nearest);
            tree.Add(newNode);
            
            return newNode;
        }

        private bool TryConnect(Node sourceNode, out List<Node> connectionPath)
        {
            connectionPath = new List<Node>();
            var current = sourceNode;
            var nearestTarget = FindNearestNode(_treeGoal, sourceNode.Position);

            while (true)
            {
                var direction = (nearestTarget.Position - current.Position).normalized;
                var newPos = current.Position + direction * _stepSize;
                var newRot = _useRotation 
                    ? Quaternion.RotateTowards(current.Rotation, nearestTarget.Rotation, _rotationStepSize)
                    : current.Rotation;

                if (HasCollision(newPos))
                {
                    return false;
                }

                var newNode = new Node(newPos, newRot, current);
                connectionPath.Add(newNode);

                if (Vector3.Distance(newPos, nearestTarget.Position) < _connectDistance &&
                    Quaternion.Angle(newRot, nearestTarget.Rotation) < _rotationStepSize)
                {
                    return true;
                }

                current = newNode;
            }
        }

        private Node FindNearestNode(List<Node> tree, Vector3 position)
        {
            Node nearest = null;
            var minDistance = float.MaxValue;

            foreach (var node in tree)
            {
                var distance = Vector3.Distance(node.Position, position);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearest = node;
                }
            }

            return nearest;
        }

        private Path ReconstructPath(Node meetingPointStart, List<Node> connectionPath)
        {
            var path = new Path(_partId, _partObject);
            var partPivotOffset = _partObject.position - _partBounds.center;
            
            // Add path from start to meeting point
            var startPath = new List<Node>();
            var current = meetingPointStart;
            
            while (current != null)
            {
                startPath.Add(current);
                current = current.Parent;
            }

            // Add paths in correct order
            for (var i = startPath.Count - 1; i >= 0; i--)
            {
                path.Positions.Add(startPath[i].Position + startPath[i].Rotation * partPivotOffset);
                path.Orientations.Add(startPath[i].Rotation);
            }

            // Add connection path
            foreach (var node in connectionPath)
            {
                path.Positions.Add(node.Position + node.Rotation * partPivotOffset);
                path.Orientations.Add(node.Rotation);
            }
            
            // Add the goal tree in reverse
            for (var i = _treeGoal.Count - 1; i >= 0; i--)
            {
                var node = _treeGoal[i];
                path.Positions.Add(node.Position + node.Rotation * partPivotOffset);
                path.Orientations.Add(node.Rotation);
            }

            if (_treesAreSwapped)
            {
                path.Positions.Reverse();
                path.Orientations.Reverse();
            }
            
            return path;
        }

        private void SwapTrees()
        {
            var temp = _treeStart;
            _treeStart = _treeGoal;
            _treeGoal = temp;

            _treesAreSwapped = !_treesAreSwapped;
        }

        // Implement Voronoi bias by selecting random points and choosing
        // the one that's furthest from its nearest neighbor in the tree
        private Vector3 GetRandomConfigWithVoronoiBias(List<Node> tree)
        {
            SamplingCandidate bestValidCandidate = null;
            SamplingCandidate bestOverallCandidate = null;
            var maxValidDistance = float.MinValue;
            var maxOverallDistance = float.MinValue;

            for (var i = 0; i < _randomPointAttempts; i++)
            {
                var randomPoint = new Vector3(
                    Random.Range(_workspaceBounds.min.x, _workspaceBounds.max.x),
                    Random.Range(_workspaceBounds.min.y, _workspaceBounds.max.y),
                    Random.Range(_workspaceBounds.min.z, _workspaceBounds.max.z)
                );
                
                var nearest = FindNearestNode(tree, randomPoint);
                var distance = Vector3.Distance(randomPoint, nearest.Position);
                var canReachPoint = IsValidMovement(nearest.Position, randomPoint);
                
                var candidate = new SamplingCandidate(randomPoint, distance, nearest, canReachPoint);
                
                // Update best valid candidate
                if (canReachPoint && distance > maxValidDistance)
                {
                    maxValidDistance = distance;
                    bestValidCandidate = candidate;
                }
                
                // Update best overall candidate
                if (distance > maxOverallDistance)
                {
                    maxOverallDistance = distance;
                    bestOverallCandidate = candidate;
                }
            }
            
            // Probabilistically choose between exploration and exploitation
            if (Random.value < _explorationBias)
            {
                // Exploration: Try to use the overall best candidate
                if (bestOverallCandidate != null)
                {
                    // If the best overall point isn't reachable, try to move in its general direction
                    if (!bestOverallCandidate.IsValidPath)
                    {
                        var direction = (bestOverallCandidate.Position - bestOverallCandidate.NearestNode.Position).normalized;
                        return FindBestValidPoint(bestOverallCandidate.NearestNode.Position, direction);
                    }
                    return bestOverallCandidate.Position;
                }
            }
        
            // Exploitation: Use the best valid candidate
            if (bestValidCandidate != null)
            {
                return bestValidCandidate.Position;
            }

            // If no valid candidates found, try to make progress toward the goal
            return GetProgressiveRandomPoint(tree);
        }
        
        private bool IsValidMovement(Vector3 fromPosition, Vector3 toPosition)
        {
            var direction = toPosition - fromPosition;
            var distance = direction.magnitude;
            direction.Normalize();

            // Check if we can move at least a minimal distance toward the target
            var minProgress = _stepSize * 0.1f; // 10% of step size
            var minimalStep = fromPosition + direction * minProgress;

            if (HasCollision(minimalStep))
            {
                return false;
            }

            // Do a more detailed check if trying to move further
            if (distance > _stepSize)
            {
                var checks = Mathf.CeilToInt(distance / _stepSize);
                var checkStepSize = distance / checks;

                for (var i = 1; i <= checks; i++)
                {
                    var checkPoint = fromPosition + direction * (checkStepSize * i);
                    if (HasCollision(checkPoint))
                    {
                        return false;
                    }
                }
            }

            return true;
        }
        
        private Vector3 GetProgressiveRandomPoint(List<Node> tree)
        {
            var goalPosition = _treeGoal[0].Position;
            
            // Try to make progress toward the goal
            var nearest = FindNearestNode(tree, goalPosition);
            var directionToGoal = (goalPosition - nearest.Position).normalized;
        
            // Add some randomness to avoid getting stuck
            var randomOffset = Random.insideUnitSphere * _stepSize;
            var direction = (directionToGoal + randomOffset.normalized * 0.5f).normalized;
        
            return FindBestValidPoint(nearest.Position, direction);
        }

        private Vector3 FindBestValidPoint(Vector3 startPoint, Vector3 direction)
        {
            var maxDistance = _stepSize * 2f; // Try up to 2x step size
            var currentDistance = _stepSize;

            while (currentDistance <= maxDistance)
            {
                var testPoint = startPoint + direction * currentDistance;
                if (_workspaceBounds.Contains(testPoint) && IsValidMovement(startPoint, testPoint))
                {
                    return testPoint;
                }
                currentDistance += _stepSize * 0.5f;
            }

            // If no valid point found, return a small step in the desired direction
            return startPoint + direction * _stepSize * 0.5f;
        }

        private bool HasCollision(Vector3 position)
        {
            var updatedBounds = new Bounds(position, _partBounds.size);
            return _otherPartBounds.Any(pb => updatedBounds.Intersects(pb));
        }
        
        private Bounds GetPartBounds(Transform part, State state, bool addRotationBuffer)
        {
            var bounds = part.GetComponentInChildren<Renderer>().bounds;

            if (addRotationBuffer)
            {
                var maxSize = GetBoundsLargestAxisSize(bounds);
                bounds.size = Vector3.one * maxSize;
            }
            
            bounds.center = state.Position;

            return bounds;
        }
        
        private Bounds[] GetPartBounds(Transform[] parts, State[] states, bool addRotationBuffer)
        {
            var bounds = new Bounds[parts.Length];
            for (var i = 0; i < parts.Length; i++)
            {
                bounds[i] = GetPartBounds(parts[i], states[i], addRotationBuffer);
            }
            
            return bounds;
        }

        private float GetBoundsLargestAxisSize(Bounds bounds)
        {
            return Mathf.Max(bounds.size.x, Mathf.Max(bounds.size.y, bounds.size.z));
        }
    }
}