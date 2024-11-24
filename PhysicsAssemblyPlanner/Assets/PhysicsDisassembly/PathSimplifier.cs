using PhysicsDisassembly.Simulation;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class PathSimplifier
    {
        private readonly string _partId;
        private readonly Vector3 _partPivotCenterOffset;
        private readonly PathSimplifierConfiguration _configuration;
        
        private PhysicsSimulation _simulation;
        private bool _useRotation;
        private bool _useSDFCollision;
        private bool _verbose;
        
        public PathSimplifier(PhysicsSimulation simulation, string partId, PathSimplifierConfiguration configuration)
        {
            _simulation = simulation;
            _partId = partId;
            _configuration = configuration;
            
            _partPivotCenterOffset = _simulation.GetPart(partId).PivotCenterOffset;
        }

        public Path SimplifyPath(Path originalPath, bool useRotation, bool useSDFCollision = true, bool verbose = false)
        {
            _useRotation = useRotation;
            _useSDFCollision = useSDFCollision;
            _verbose = verbose;
            
            _simulation.Reset();
            
            var simplifiedPath = new Path(_partId, originalPath.PartObject);

            // Always keep first and last states
            simplifiedPath.AddState(GetStateAt(originalPath, 0));

            // Find key states that represent significant progress
            var currentIndex = 0;
            while (currentIndex < originalPath.Positions.Count - 1)
            {
                var nextKeyIndex = FindNextKeyState(originalPath, currentIndex);
                if (nextKeyIndex == -1 || nextKeyIndex >= originalPath.Positions.Count - 1)
                {
                    break;
                }

                simplifiedPath.AddState(GetStateAt(originalPath, nextKeyIndex));
                currentIndex = nextKeyIndex;
            }

            // Add final state
            simplifiedPath.AddState(GetStateAt(originalPath, originalPath.Positions.Count - 1));

            if (_verbose)
            {
                Debug.Log($"PathSimplifier: Simplified path from {originalPath.Positions.Count} to {simplifiedPath.Positions.Count} states");
            }

            return simplifiedPath;
        }

        private int FindNextKeyState(Path path, int startIndex)
        {
            var lastIndex = path.Positions.Count - 1;
            var startPos = path.Positions[startIndex];
            var startRot = path.Orientations[startIndex];
            var endPos = path.Positions[lastIndex];
            var endRot = path.Orientations[lastIndex];

            var bestProgress = 0f;
            var bestIndex = -1;
            var previousProgress = 0f;

            // Look through subsequent states
            for (var i = startIndex + 1; i < path.Positions.Count; i++)
            {
                var currentPos = path.Positions[i];
                var currentRot = path.Orientations[i];

                // Calculate progress metrics
                 var totalProgress = CalculateDistanceProgress(startPos, currentPos, endPos);

                if (_useRotation)
                {
                    // TODO: Test this
                    var rotationProgress = CalculateRotationProgress(startRot, currentRot, endRot);
                    totalProgress = (totalProgress + rotationProgress) * 0.5f;
                }

                // TODO: 
                
                // Check if this state represents significant progress
                if (/*totalProgress > bestProgress && */totalProgress > /*previousProgress +*/ _configuration.SimplifierMinimumProgressThreshold)
                {
                    

                    // Verify we can move directly to this state
                    if (IsValidTransition(path.Positions[startIndex], path.Orientations[startIndex],
                            path.Positions[i], path.Orientations[i]))
                    {
                        bestProgress = totalProgress;
                        bestIndex = i;
                    }
                }

                previousProgress = totalProgress;
            }

            return bestIndex;
        }

        private float CalculateDistanceProgress(Vector3 start, Vector3 current, Vector3 goal)
        {
            var totalDistance = Vector3.Distance(start, goal);
            if (totalDistance < float.Epsilon)
            {
                return 1f;
            }

            var startToGoal = goal - start;
            var startToCurrent = current - start;

            // Project current position onto start-goal vector
            var projection = Vector3.Project(startToCurrent, startToGoal.normalized);
            var progressDistance = projection.magnitude;

            // Check if we're moving in the right direction
            //if (Vector3.Dot(startToGoal, projection) < 0f)
            if (Vector3.Dot(startToGoal.normalized, startToCurrent.normalized) < -0.25f)
            {
                progressDistance = 0f;
            }

            return Mathf.Clamp01(progressDistance / totalDistance);
        }

        private float CalculateRotationProgress(Quaternion start, Quaternion current, Quaternion goal)
        {
            var totalAngle = Quaternion.Angle(start, goal);
            if (totalAngle < float.Epsilon)
            {
                return 1f;
            }

            var currentAngle = Quaternion.Angle(start, current);
            var remainingAngle = Quaternion.Angle(current, goal);

            // Check if we're rotating in the right direction
            if (currentAngle > totalAngle || remainingAngle > totalAngle)
            {
                return 0f;
            }

            return Mathf.Clamp01(currentAngle / totalAngle);
        }

        private bool IsValidTransition(Vector3 startPos, Quaternion startRot, Vector3 endPos, Quaternion endRot)
        {
            // Test if we can move directly between these states
            for (var i = 1; i <= _configuration.SimplifierTransitionTestSteps; i++)
            {
                var t = i / (float)_configuration.SimplifierTransitionTestSteps;
                var testPos = Vector3.Lerp(startPos, endPos, t);
                
                _simulation.SetPivotPosition(_partId, testPos);

                if (_useRotation)
                {
                    var testRot = Quaternion.Lerp(startRot, endRot, t);
                    _simulation.SetRotation(_partId, testRot);
                }
                
                _simulation.UpdateParts();
                
                if (_simulation.CheckCollisions(_partId, _useSDFCollision))
                {
                    return false;
                }
            }

            return true;
        }
        
        private State GetStateAt(Path path, int index)
        {
            var pivotPosition = path.Positions[index];
            return new State(
                pivotPosition + /*(path.Orientations[index] **/ _partPivotCenterOffset/*)*/,
                pivotPosition,
                path.Orientations[index],
                Vector3.zero,
                Vector3.zero
            );
        }
    }
}