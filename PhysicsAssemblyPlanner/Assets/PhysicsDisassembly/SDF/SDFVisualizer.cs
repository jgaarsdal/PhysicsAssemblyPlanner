using UnityEngine;

namespace PhysicsDisassembly.SDF
{
    public class SDFVisualizer : MonoBehaviour
    {
        // TODO: Make cell size similar to the paper, and no need to make grid larger than the bounds (plus padding)
        // TODO: So instead of specifying number of cells, number of cells is determined by the cell size and the bounds

        // TODO: Also make a script to test the collision between SDFs

        [SerializeField] private float _sdfDefaultCellSize = 0.05f;
        [SerializeField] private float _maxDistance = 1f;
        [SerializeField] private float _sdfBoxPadding = 0.1f;
        [SerializeField] private bool _useGPU = true;
        [SerializeField] private bool _visualize = true;

        [ContextMenu("Visualize SDF")]
        public void VisualizeSDFButton()
        {
            VisualizeSDF();
        }

        public async void VisualizeSDF()
        {
            var sdf = new SignedDistanceField(this.gameObject, _sdfDefaultCellSize, _sdfBoxPadding, 0.01f, _useGPU);
            await sdf.ComputeSDF();

            if (!_visualize)
            {
                return;
            }

            var gridSize = sdf.GridSize;

            for (var x = 0; x < gridSize; x++)
            {
                for (var y = 0; y < gridSize; y++)
                {
                    for (var z = 0; z < gridSize; z++)
                    {
                        var distance = sdf.GetDistance(new Vector3Int(x, y, z));
                        if (distance >= _maxDistance)
                        {
                            continue;
                        }

                        DrawCell(x, y, z, distance, sdf.Origin, sdf.CellSize);
                    }
                }
            }
        }

        private void DrawCell(int x, int y, int z, float distance, Vector3 origin, float cellSize, float delay = 60f)
        {
            var position = origin + new Vector3(x * cellSize, y * cellSize, z * cellSize);
            var color = GetDistanceColor(distance);

            var b = new Bounds(position, Vector3.one * cellSize);

            // bottom
            var p1 = new Vector3(b.min.x, b.min.y, b.min.z);
            var p2 = new Vector3(b.max.x, b.min.y, b.min.z);
            var p3 = new Vector3(b.max.x, b.min.y, b.max.z);
            var p4 = new Vector3(b.min.x, b.min.y, b.max.z);

            Debug.DrawLine(p1, p2, color, delay);
            Debug.DrawLine(p2, p3, color, delay);
            Debug.DrawLine(p3, p4, color, delay);
            Debug.DrawLine(p4, p1, color, delay);

            // top
            var p5 = new Vector3(b.min.x, b.max.y, b.min.z);
            var p6 = new Vector3(b.max.x, b.max.y, b.min.z);
            var p7 = new Vector3(b.max.x, b.max.y, b.max.z);
            var p8 = new Vector3(b.min.x, b.max.y, b.max.z);

            Debug.DrawLine(p5, p6, color, delay);
            Debug.DrawLine(p6, p7, color, delay);
            Debug.DrawLine(p7, p8, color, delay);
            Debug.DrawLine(p8, p5, color, delay);

            // sides
            Debug.DrawLine(p1, p5, color, delay);
            Debug.DrawLine(p2, p6, color, delay);
            Debug.DrawLine(p3, p7, color, delay);
            Debug.DrawLine(p4, p8, color, delay);
        }

        private Color GetDistanceColor(float distance)
        {
            var t = Mathf.Clamp01((distance + _maxDistance) / (2f * _maxDistance));
            return Color.Lerp(Color.red, Color.blue, t);
        }
    }
}