using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class BFSPlanner
{
    private const float TransDistTh = 0.05f;
    private const float QuatDistTh = 0.5f;

    private string assetFolder;
    private string assemblyDir;
    private string moveId;
    private List<string> stillIds;
    private bool rotation;
    private string bodyType;
    private float sdfDx;
    private float collisionTh;
    private float forceMag;
    private int frameSkip;
    private int ndof;

    private Dictionary<string, SignedDistanceField> _partSDFs = new Dictionary<string, SignedDistanceField>();
    private PhysicsSimulation simulation;

    private Vector3 minBoxMove;
    private Vector3 maxBoxMove;
    private Vector3 sizeBoxMove;
    private Vector3 minBoxStill;
    private Vector3 maxBoxStill;
    private Vector3 sizeBoxStill;
    private Vector3 stateLowerBound;
    private Vector3 stateUpperBound;

    public BFSPlanner(string assetFolder, string assemblyDir, string moveId, List<string> stillIds,
        bool rotation, string bodyType, float sdfDx, float collisionTh, float forceMag, int frameSkip)
    {
        this.assetFolder = assetFolder;
        this.assemblyDir = assemblyDir;
        this.moveId = moveId;
        this.stillIds = stillIds;
        this.rotation = rotation;
        this.bodyType = bodyType;
        this.sdfDx = sdfDx;
        this.collisionTh = collisionTh;
        this.forceMag = forceMag;
        this.frameSkip = frameSkip;
        this.ndof = rotation ? 6 : 3;

        InitializeSimulation();
        CalculateBounds();
    }

    private void InitializeSimulation()
    {
        // Initialize SignedDistanceField and PhysicsSimulation
        //simulation = new PhysicsSimulation(/* parameters */);
    }

    private void CalculateBounds()
    {
        // Get the vertices of the moving part
        Vector3[] verticesMove = simulation.GetVertices(moveId);
    
        // Calculate bounds for the moving part
        minBoxMove = Vector3.positiveInfinity;
        maxBoxMove = Vector3.negativeInfinity;
        foreach (Vector3 vertex in verticesMove)
        {
            minBoxMove = Vector3.Min(minBoxMove, vertex);
            maxBoxMove = Vector3.Max(maxBoxMove, vertex);
        }
        sizeBoxMove = maxBoxMove - minBoxMove;

        // Calculate bounds for the still parts
        minBoxStill = Vector3.positiveInfinity;
        maxBoxStill = Vector3.negativeInfinity;
        foreach (string stillId in stillIds)
        {
            Vector3[] verticesStill = simulation.GetVertices(stillId);
            foreach (Vector3 vertex in verticesStill)
            {
                minBoxStill = Vector3.Min(minBoxStill, vertex);
                maxBoxStill = Vector3.Max(maxBoxStill, vertex);
            }
        }
        sizeBoxStill = maxBoxStill - minBoxStill;

        // Calculate state bounds
        stateLowerBound = (minBoxStill - maxBoxMove) - 0.5f * sizeBoxMove;
        stateUpperBound = (maxBoxStill - minBoxMove) + 0.5f * sizeBoxMove;
    }

    public (string status, float tPlan, List<Vector3> path) Plan(float maxTime, int maxDepth, int seed, 
        bool returnPath = true, bool render = false, string recordPath = null)
    {
        UnityEngine.Random.InitState(seed);

        simulation.Reset();

        Tree tree = new Tree();
        State initState = GetState();
        tree.AddNode(initState);

        if (IsDisassembled())
        {
            return ("Start with goal", 0f, new List<Vector3>());
        }

        string status = "Failure";
        List<Vector3> path = null;
        float tStart = Time.realtimeSinceStartup;
        int step = 0;

        Queue<State> stateQueue = new Queue<State>();
        stateQueue.Enqueue(initState);

        while (stateQueue.Count > 0 && step < maxDepth)
        {
            State state = stateQueue.Dequeue();
            
            foreach (Vector3 action in GetActions())
            {
                SetState(state);
                ApplyAction(action);
                simulation.UpdateParts();

                List<State> statesBetween = new List<State>();
                for (int i = 0; i < frameSkip; i++)
                {
                    simulation.Forward(1);
                    State stateBetween = GetState();
                    statesBetween.Add(stateBetween);

                    float tPlan = Time.realtimeSinceStartup - tStart;
                    if (tPlan > maxTime)
                    {
                        status = "Timeout";
                        return (status, tPlan, null);
                    }
                }

                State newState = statesBetween[statesBetween.Count - 1];

                if (!AnyStateSimilar(tree.GetNodes(), newState))
                {
                    tree.AddNode(newState);
                    tree.AddEdge(state, newState, action, statesBetween);
                    stateQueue.Enqueue(newState);

                    if (IsDisassembled())
                    {
                        status = "Success";
                        path = GetPath(tree, newState);
                        return (status, Time.realtimeSinceStartup - tStart, path);
                    }
                }
            }

            step++;
        }

        if (render)
        {
            Render(path, recordPath);
        }

        return (status, Time.realtimeSinceStartup - tStart, path);
    }

    private State GetState()
    {
        Vector3 position = simulation.GetPosition(moveId);
        Quaternion rotation = simulation.GetRotation(moveId);
        Vector3 velocity = simulation.GetVelocity(moveId);
        Vector3 angularVelocity = simulation.GetAngularVelocity(moveId);
        return new State(position, rotation, velocity, angularVelocity);
    }

    private void SetState(State state)
    {
        simulation.SetPosition(moveId, state.Position);
        simulation.SetRotation(moveId, state.Rotation);
        simulation.SetVelocity(moveId, state.Velocity);
        simulation.SetAngularVelocity(moveId, state.AngularVelocity);
    }

    private void ApplyAction(Vector3 action)
    {
        Vector3 force = action.normalized * forceMag;
        if (rotation)
        {
            Vector3 torque = new Vector3(force.x * 3, force.y * 3, force.z);
            simulation.ApplyForceAndTorque(moveId, force, torque);
        }
        else
        {
            simulation.ApplyForce(moveId, force);
        }
    }

    private bool IsDisassembled()
    {
        Vector3 position = simulation.GetPosition(moveId);
        Quaternion rotation = simulation.GetRotation(moveId);

        foreach (string stillId in stillIds)
        {
            var moveSdf = _partSDFs[moveId];
            var stillSdf = _partSDFs[stillId];
            if (moveSdf.CheckCollision(stillSdf, position, rotation))
            {
                return false;
            }
        }

        Vector3 minBox = position + rotation * minBoxMove;
        Vector3 maxBox = position + rotation * maxBoxMove;

        bool moveContainStill = Vector3.Min(minBox, minBoxStill) == minBox && Vector3.Max(maxBox, maxBoxStill) == maxBox;
        bool stillContainMove = Vector3.Min(minBox, minBoxStill) == minBoxStill && Vector3.Max(maxBox, maxBoxStill) == maxBoxStill;

        return !(moveContainStill || stillContainMove);
    }

    private Vector3[] GetActions()
    {
        if (rotation)
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
        foreach (State state in states)
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
        float transDist = Vector3.Distance(state1.Position, state2.Position);
        if (rotation)
        {
            float quatDist = Quaternion.Angle(state1.Rotation, state2.Rotation);
            return transDist < TransDistTh && quatDist < QuatDistTh;
        }
        else
        {
            return transDist < TransDistTh;
        }
    }

    private List<Vector3> GetPath(Tree tree, State endState)
    {
        List<State> statePath = tree.GetRootPath(endState);
        return statePath.Select(state => state.Position).ToList();
    }

    private void Render(List<Vector3> path, string recordPath)
    {
        // This method would be used to visualize the path in the Unity scene
        // and optionally record it if recordPath is provided
        if (path == null || path.Count == 0)
        {
            Debug.LogWarning("No path to render.");
            return;
        }

        // Visualize the path in the scene
        for (int i = 0; i < path.Count - 1; i++)
        {
            Debug.DrawLine(path[i], path[i + 1], Color.green, 5f);
        }

        // If recordPath is provided, we could use Unity's built-in recording tools
        // or a custom recording solution to save the visualization
        if (!string.IsNullOrEmpty(recordPath))
        {
            Debug.Log($"Recording path to: {recordPath}");
            // Implement recording logic here
        }
    }

    public void SavePath(List<Vector3> path, string saveDir, int nSaveState)
    {
        if (path == null || path.Count == 0)
        {
            Debug.LogWarning("No path to save.");
            return;
        }

        // Ensure the save directory exists
        System.IO.Directory.CreateDirectory(saveDir);

        // Calculate the step size to save nSaveState points
        int stepSize = Mathf.Max(1, path.Count / nSaveState);

        // Save the path points
        string pathFile = System.IO.Path.Combine(saveDir, "path.txt");
        using (System.IO.StreamWriter writer = new System.IO.StreamWriter(pathFile))
        {
            for (int i = 0; i < path.Count; i += stepSize)
            {
                Vector3 point = path[i];
                writer.WriteLine($"{point.x} {point.y} {point.z}");
            }
        }

        Debug.Log($"Path saved to: {pathFile}");
    }
}