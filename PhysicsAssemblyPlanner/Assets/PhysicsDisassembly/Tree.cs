using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PhysicsDisassembly
{
    public class Tree
    {
        private Dictionary<State, int> _nodes = new();
        private Dictionary<State, List<Edge>> _outEdges = new();
        private Dictionary<State, Edge> _inEdges = new();
        private int _nodeIdx = 0;
        private State _rootNode = default;

        public void AddNode(State state)
        {
            if (!_nodes.ContainsKey(state))
            {
                _nodes[state] = _nodeIdx;
                _outEdges[state] = new List<Edge>();
                _nodeIdx++;

                if (_nodeIdx == 1)
                {
                    _rootNode = state;
                }
            }
        }

        public void AddEdge(State stateSrc, State stateTarget, float[] action, Stack<State> statesBetween)
        {
            if (!_nodes.ContainsKey(stateSrc) || !_nodes.ContainsKey(stateTarget))
            {
                Debug.LogError("Tree: Source or target node doesn't exist");
                return;
            }

            var edge = new Edge(stateSrc, stateTarget, action, statesBetween);
            _outEdges[stateSrc].Add(edge);
            _inEdges[stateTarget] = edge;
        }

        public Edge GetInEdge(State state)
        {
            if (!_nodes.ContainsKey(state))
            {
                Debug.LogError("Tree: Node doesn't exist");
                return null;
            }

            return _inEdges.GetValueOrDefault(state);
        }

        public List<State> GetPath(State startState, State endState)
        {
            var path = new List<State>();
            var state = endState;

            while (true)
            {
                path.Add(state);
                if (state.Equals(startState))
                {
                    break;
                }

                var inEdge = GetInEdge(state);
                if (inEdge == null)
                {
                    return null;
                }

                var revStatesBetween = inEdge.StatesBetween.ToArray().Reverse();
                path.AddRange(revStatesBetween);
                state = inEdge.Source;
            }

            path.Reverse();
            return path;
        }

        public List<State> GetRootPath(State endState)
        {
            return GetPath(_rootNode, endState);
        }
    }
}