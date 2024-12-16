# Physics Assembly Planner
Physics-based automatic disassembly of composite 3D models in Unity (work in progress).

## Methods
This solution is based on the paper [Assemble Them All: Physics-Based Planning for Generalizable Assembly by Disassembly](https://github.com/yunshengtian/Assemble-Them-All).
It is able to plan the assembly and disassembly of real-world assemblies, determining the physically realistic assembly sequence and motion. The method utilizes an assembly-by-disassembly approach as well as simulation of physics with precise collision detection using Signed Distance Fields (SDF). This enables it to determine not only the translation but also any rotation required in order to remove a part from the assembly.

## Implementation
The solution is a Unity project using C# with parallelization when possible for faster computation. For the computation of SDFs compute shaders are available to run this on the GPU.