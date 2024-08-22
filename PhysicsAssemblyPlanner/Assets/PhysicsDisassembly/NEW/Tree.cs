using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Tree
{
    private Dictionary<State, List<(State, Vector3, List<State>)>> edges = new Dictionary<State, List<(State, Vector3, List<State>)>>();
    private List<State> nodes = new List<State>();
    private State rootNode;

    public void AddNode(State state)
    {
        if (!nodes.Contains(state))
        {
            nodes.Add(state);
            edges[state] = new List<(State, Vector3, List<State>)>();
            if (nodes.Count == 1)
            {
                rootNode = state;
            }
        }
    }

    public void AddEdge(State from, State to, Vector3 action, List<State> statesBetween)
    {
        if (!edges.ContainsKey(from))
        {
            edges[from] = new List<(State, Vector3, List<State>)>();
        }

        edges[from].Add((to, action, statesBetween));
    }

    public List<State> GetNodes()
    {
        return new List<State>(nodes);
    }

    public List<(State, State, Vector3, List<State>)> GetEdges()
    {
        List<(State, State, Vector3, List<State>)> allEdges = new List<(State, State, Vector3, List<State>)>();
        foreach (var kvp in edges)
        {
            foreach (var (to, action, statesBetween) in kvp.Value)
            {
                allEdges.Add((kvp.Key, to, action, statesBetween));
            }
        }

        return allEdges;
    }

    public List<State> GetRootPath(State endState)
    {
        List<State> path = new List<State>();
        State currentState = endState;

        while (currentState != null)
        {
            path.Add(currentState);
            if (currentState == rootNode)
            {
                break;
            }

            currentState = GetPredecessor(currentState);
        }

        path.Reverse();
        return path;
    }

    public State GetPredecessor(State state)
    {
        foreach (var kvp in edges)
        {
            foreach (var (to, _, _) in kvp.Value)
            {
                if (to == state)
                {
                    return kvp.Key;
                }
            }
        }

        return null;
    }

    public List<State> GetSuccessors(State state)
    {
        if (edges.TryGetValue(state, out var successors))
        {
            return successors.Select(s => s.Item1).ToList();
        }

        return new List<State>();
    }

    public int GetInDegree(State state)
    {
        return edges.Values.Sum(list => list.Count(edge => edge.Item1 == state));
    }

    public int GetOutDegree(State state)
    {
        return edges.TryGetValue(state, out var successors) ? successors.Count : 0;
    }

    public bool HasPath(State startState, State endState)
    {
        HashSet<State> visited = new HashSet<State>();
        Queue<State> queue = new Queue<State>();
        queue.Enqueue(startState);

        while (queue.Count > 0)
        {
            State current = queue.Dequeue();
            if (current == endState)
            {
                return true;
            }

            if (!visited.Contains(current))
            {
                visited.Add(current);
                foreach (var successor in GetSuccessors(current))
                {
                    queue.Enqueue(successor);
                }
            }
        }

        return false;
    }

    public List<State> GetPath(State startState, State endState)
    {
        if (!HasPath(startState, endState))
        {
            return null;
        }

        List<State> path = new List<State>();
        State current = endState;

        while (current != startState)
        {
            path.Add(current);
            current = GetPredecessor(current);
        }

        path.Add(startState);
        path.Reverse();
        return path;
    }
}