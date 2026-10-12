1. Where moduleIndex maps to the sphere
The main configuration is in:
software/src/common/include/DASPi-shape-config.h
Your current configuration is:
using ConfiguredSphereSpace =
    IcosahedronModuleSphereSpace<9, 2, 19, 18>;

using tpgy_t =
    IcosahedronSphereTopology<ConfiguredSphereSpace>;


This establishes the following mapping:
moduleIndex	Icosahedron face index	Raspberry Pi IP
0	9	10.0.2.4
1	2	10.0.2.5
2	19	10.0.2.6
3	18	10.0.2.7
The IP addresses reflect your existing node assignments. They are not assigned by the sphere topology itself.
The order inside IcosahedronModuleSphereSpace<...> is significant. Each position corresponds to a moduleIndex, while the number at that position identifies one of the 20 triangular faces on the sphere.
For example, moduleIndex=2 selects global face 19.
2. How the four faces are positioned
I also checked the icosahedron's adjacency table.
Your selected faces are arranged as one central face with three surrounding faces:
Schematic of shared face edges, unfolded into a plane. Not the final 3D orientation.
Face 9 has exactly these three neighbours:
facetAdjacencies_[9] = {2, 19, 18};


This matches your intended four-camera-module arrangement: one central module with three surrounding modules. Adjacent face normals are approximately 41.81° apart.
The face definitions are in DASPi-icosahedronspace.h.
3. How the mapping is applied
The relevant execution path is:
DASPi-shape-config.h
ConfiguredSphereSpace = 9, 2, 19, 18

DASPi-icosahedronspherespace.h
ModuleFaceIndex<ModuleIndex>()

ApertureComputeModule
Selects the compile-time face geometry for capture


ComputeModule
Assigns peer streams to the corresponding sphere faces




The actual lookup is:
template<std::size_t ModuleIndex>
static consteval std::size_t ModuleFaceIndex()
{
    return moduleFaceIndices_[ModuleIndex];
}


Both capture and stitching now use ConfiguredSphereSpace, so they share the same compile-time mapping.
4. How to verify it is working
When you start each ApertureComputeModule, it prints:
ModuleIndex: 0 -> Global FaceIndex: 9
ModuleIndex: 1 -> Global FaceIndex: 2
ModuleIndex: 2 -> Global FaceIndex: 19
ModuleIndex: 3 -> Global FaceIndex: 18


The ComputeModule also prints [rig] module assignments and [main] Creating AperturePeer, including each module's face and network address.
Make sure it is launched with:
--nApertureComputeModules=4 \
--usbBaseIp=10.0.2.1 \
--serverIps=10.0.2.4,10.0.2.5,10.0.2.6,10.0.2.7


One important detail: the existing --reverseModuleOrder option can reverse how received streams are assigned to module positions. Leave it disabled for the standard mapping above.
Conclusion: DASPi-shape-config.h is now the central place to change your sphere assignments. Since the mapping is compile-time, rebuild both the ApertureComputeModule and ComputeModule software after making changes.
