using System.Collections.Generic;
using UnityEngine;

public class Tree
{
    private Dictionary<State, List<(State, Vector3, List<State>)>> _edges = new Dictionary<State, List<(State, Vector3, List<State>)>>();
    private List<State> _nodes = new List<State>();

    public void AddNode(State state)
    {
        if (!_nodes.Contains(state))
        {
            _nodes.Add(state);
            _edges[state] = new List<(State, Vector3, List<State>)>();
        }
    }

    public void AddEdge(State from, State to, Vector3 action, List<State> statesBetween)
    {
        _edges[from].Add((to, action, statesBetween));
    }

    public List<State> GetNodes()
    {
        return _nodes;
    }

    public List<State> GetRootPath(State endState)
    {
        var path = new List<State>();
        var currentState = endState;

        while (currentState != null)
        {
            path.Add(currentState);
            currentState = GetPredecessor(currentState);
        }

        path.Reverse();
        return path;
    }

    private State GetPredecessor(State state)
    {
        foreach (var kvp in _edges)
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
}