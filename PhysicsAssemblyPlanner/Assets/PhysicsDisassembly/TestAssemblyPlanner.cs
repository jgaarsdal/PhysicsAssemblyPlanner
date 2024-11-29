using System;
using System.Collections.Generic;
using System.IO;
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
            
            await assemblyPlanner.InitializeAsync(_assemblyParts);
            
            _disassemblySequence = assemblyPlanner.RunPlanner(); // TODO: Do coroutine or async?

            ClearTweens();

            _disassemblyTweenSequence = DOTween.Sequence();

            for (var i = 0; i < _disassemblySequence.Count; i++)
            {
                var sequenceTransform = _disassemblySequence[i].PartObject;
                var positions = _disassemblySequence[i].Positions;
                var rotations = _disassemblySequence[i].Orientations;

                Debug.Log($"Sequence #{i} has {positions.Count} positions/rotations");
                
                var pointDuration = Mathf.Clamp(4f / positions.Count, 0.0005f, 4f);
                
                for (var j = 0; j < positions.Count; j++)
                {
                    // Add position and rotation tweens to run in parallel
                    _disassemblyTweenSequence.Append(
                            sequenceTransform.DOMove(positions[j], pointDuration)
                                .SetEase(Ease.InOutCubic))
                        .Join(sequenceTransform.DORotateQuaternion(rotations[j], pointDuration)
                            .SetEase(Ease.InOutCubic));
                }
            }

            _disassemblyTweenSequence/*.AppendInterval(1f)*/.SetLoops(-1, LoopType.Yoyo).Play();
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
        
        [ContextMenu("Write Plan")]
        public void WriteAssemblyPlanButton()
        {
            if (_disassemblySequence == null)
            {
                return;
            }
            
            foreach (var disassembly in _disassemblySequence)
            {
                var content = "";
                for (var i = 0; i < disassembly.Positions.Count; i++)
                {
                    content += $"{disassembly.Positions[i]} - {disassembly.Orientations[i].eulerAngles}\n";
                }
                
                var filePath = System.IO.Path.Join(Application.dataPath, $"plan_{disassembly.PartID}_{DateTime.Now.Millisecond}.txt");
                File.WriteAllText(filePath, content);
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