using System.Linq;
using UnityEngine;

public class TestAssemblyPlanner : MonoBehaviour
{
    [SerializeField] private GameObject _assemblyRoot = default;
    
    private ProgressiveQueueSequencePlanner _assemblyPlanner;
    
    private async void Start()
    {
        var assemblyParts = _assemblyRoot.GetComponentsInChildren<MeshFilter>()
            .Select(p => (p.sharedMesh, p.transform))
            .ToArray();

        _assemblyPlanner = new ProgressiveQueueSequencePlanner(assemblyParts);
        await _assemblyPlanner.InitializeSignedDistanceFields();
        
        Debug.Log("Finished initializing");
    }
    
    [ContextMenu("Run Planner")]
    public void RunAssemblyPlannerButton()
    {
        _assemblyPlanner.PlanSequence(false, 0.1f, 6, 600f, 30f, 3, true);
    }
}
