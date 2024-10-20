using UnityEngine;

public class SDFVisualizer : MonoBehaviour
{
    // TODO: Make cell size similar to the paper, and no need to make grid larger than the bounds (plus padding)
    // TODO: So instead of specifying number of cells, number of cells is determined by the cell size and the bounds

    // TODO: Also make a script to test the collision between SDFs
    
    [SerializeField] private float _maxDistance = 1f;
    [SerializeField] private int _sdfGridSize = 8;
    [SerializeField] private float _sdfBoxPadding = 0.1f;
    [SerializeField] private Material _material;
    [SerializeField] private bool _useGPU = true;
    [SerializeField] private bool _visualize = true;
    
    private GameObject[,,] _visualizationCubes;

    [ContextMenu("Visualize SDF")]
    public void VisualizeSDFButton()
    {
        VisualizeSDF();
    }
    
    [ContextMenu("Clear SDF")]
    public void ClearSDFButton()
    {
        ClearVisualization();
    }
    
    private void OnDestroy()
    {
        ClearVisualization();
    }

    public async void VisualizeSDF()
    {
        var mesh = this.GetComponent<MeshFilter>().sharedMesh;
        var sdf = new SignedDistanceField(mesh, this.transform, _sdfGridSize, _sdfBoxPadding);
        await sdf.ComputeSDF(_useGPU);
        
        ClearVisualization();

        if (!_visualize)
        {
            return;
        }
        
        var gridSize = sdf.GridSize;
        _visualizationCubes = new GameObject[gridSize, gridSize, gridSize];

        for (var x = 0; x < gridSize; x++)
        {
            for (var y = 0; y < gridSize; y++)
            {
                for (var z = 0; z < gridSize; z++)
                {
                    var distance = sdf.GetDistance(new Vector3Int(x, y, z));
                    CreateVisualizationCube(x, y, z, distance, sdf.Origin, sdf.CellSize);
                }
            }
        }
    }

    private void CreateVisualizationCube(int x, int y, int z, float distance, Vector3 origin, float cellSize)
    {
        var cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.transform.SetParent(transform);

        var position = origin + new Vector3(x * cellSize, y * cellSize, z * cellSize);
        cube.transform.position = position;
        
        //cube.transform.localScale = Vector3.one * _visualizationScale;
        var globalScale = Vector3.one * cellSize;
        cube.transform.localScale = Vector3.one;
        cube.transform.localScale = new Vector3 (globalScale.x / cube.transform.lossyScale.x, globalScale.y / cube.transform.lossyScale.y, globalScale.z / cube.transform.lossyScale.z);

        var renderer = cube.GetComponent<Renderer>();
        renderer.material = new Material(_material);
        renderer.material.color = GetDistanceColor(distance);

        _visualizationCubes[x, y, z] = cube;
    }
    
    public static void SetGlobalScale (Transform transform, Vector3 globalScale)
    {
        transform.localScale = Vector3.one;
        transform.localScale = new Vector3 (globalScale.x/transform.lossyScale.x, globalScale.y/transform.lossyScale.y, globalScale.z/transform.lossyScale.z);
    }

    private void ClearVisualization()
    {
        if (_visualizationCubes != null)
        {
            foreach (var cube in _visualizationCubes)
            {
                if (cube != null)
                {
                    DestroyImmediate(cube);
                }
            }

            _visualizationCubes = null;
        }
    }
    
    private Color GetDistanceColor(float distance)
    {
        var t = Mathf.Clamp01((distance + _maxDistance) / (2f * _maxDistance));
        return Color.Lerp(Color.red, Color.blue, t);
    }
}
