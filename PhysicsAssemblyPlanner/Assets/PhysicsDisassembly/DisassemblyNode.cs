using System.Collections.Generic;
using UnityEngine;

public class DisassemblyNode
{
    public Vector3 Position { get; set; }
    public Quaternion Rotation { get; set; }
    public List<(Vector3, Quaternion)> Path { get; set; }
    public float Cost { get; set; }
}