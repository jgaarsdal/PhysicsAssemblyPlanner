using System.Collections.Generic;
using UnityEngine;

public class DisassemblyStep
{
    public int PartIndex { get; set; }
    public List<(Vector3, Quaternion)> Path { get; set; }
}
