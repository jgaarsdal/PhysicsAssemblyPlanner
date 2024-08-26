using System.Linq;
using UnityEngine;

public class TestAssemblyPlanner : MonoBehaviour
{
    [SerializeField] private GameObject _assemblyRoot = default;
    
    private ProgressiveQueueSequencePlanner _assemblyPlanner;
    
    private void Start()
    {
        var assemblyParts = _assemblyRoot.GetComponentsInChildren<MeshFilter>()
            .Select(p => p.sharedMesh)
            .ToArray();

        _assemblyPlanner = new ProgressiveQueueSequencePlanner(assemblyParts);
    }
    
    [ContextMenu("Run Planner")]
    public void RunAssemblyPlannerButton()
    {
        _assemblyPlanner.PlanSequence();
    }
}
