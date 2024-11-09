using System.Collections.Generic;

namespace PhysicsDisassembly
{
    public class Edge
    {
        public State Source { get; set; }
        public State Target { get; set; }
        public float[] Action { get; set; }
        public Stack<State> StatesBetween { get; set; }

        public Edge(State source, State target, float[] action, Stack<State> statesBetween)
        {
            Source = source;
            Target = target;
            Action = action;
            StatesBetween = statesBetween;
        }
    }
}