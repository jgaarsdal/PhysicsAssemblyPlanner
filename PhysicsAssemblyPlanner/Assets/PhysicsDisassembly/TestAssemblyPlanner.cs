using System.Collections.Generic;
using DG.Tweening;
using UnityEngine;

namespace PhysicsDisassembly
{
    [RequireComponent(typeof(AssemblyPlanner))]
    public class TestAssemblyPlanner : MonoBehaviour
    {
        [SerializeField] private AssemblyPart[] _assemblyParts = default;
        
        private Sequence _disassemblyTweenSequence;
        private List<Path> _disassemblySequence;

        private void Start()
        {
            DOTween.Init();
        }

        [ContextMenu("Run Planner")]
        public async void RunAssemblyPlannerButton()
        {
            var assemblyPlanner = this.GetComponent<AssemblyPlanner>();
            _disassemblySequence = await assemblyPlanner.RunPlanner(_assemblyParts);

            ClearTweens();

            _disassemblyTweenSequence = DOTween.Sequence();

            for (var i = 0; i < _disassemblySequence.Count; i++)
            {
                var sequenceTransform = _disassemblySequence[i].PartObject;
                var positions = _disassemblySequence[i].Positions;
                var rotations = _disassemblySequence[i].Orientations;

                Debug.Log($"Sequence #{i} has {positions.Count} positions/rotations");

                var stepInterval = positions.Count > 200
                    ? Mathf.FloorToInt(positions.Count / 100f)
                    : 1;

                var pointDuration = positions.Count > 200
                    ? 4f / (positions.Count / 100f)
                    : 4f / positions.Count;

                for (var j = 0; j < positions.Count; j += stepInterval)
                {
                    // Add position and rotation tweens to run in parallel
                    _disassemblyTweenSequence.Append(
                            sequenceTransform.DOMove(positions[j], pointDuration)
                                .SetEase(Ease.InOutCubic))
                        .Join(sequenceTransform.DORotateQuaternion(rotations[j], pointDuration)
                            .SetEase(Ease.InOutCubic));
                }
            }

            _disassemblyTweenSequence.AppendInterval(1f).SetLoops(-1, LoopType.Yoyo).Play();
        }

        [ContextMenu("Reset Planner")]
        public void ResetAssemblyPlannerButton()
        {
            ClearTweens();

            if (_disassemblySequence == null)
            {
                return;
            }

            foreach (var disassembly in _disassemblySequence)
            {
                disassembly.PartObject.position = disassembly.Positions[0];
                disassembly.PartObject.rotation = disassembly.Orientations[0];
            }
        }

        private void ClearTweens()
        {
            if (_disassemblyTweenSequence == null)
            {
                return;
            }

            _disassemblyTweenSequence.Kill();
            _disassemblyTweenSequence = null;
        }
    }
}