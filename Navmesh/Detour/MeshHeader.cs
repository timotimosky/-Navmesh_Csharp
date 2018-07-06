/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
Recast4J Copyright (c) 2015 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
namespace org.recast4j.detour
{

	/// <summary>
	/// Provides high level information related to a dtMeshTile object. </summary>
	public class MeshHeader
	{
		/// <summary>
		/// A magic number used to detect compatibility of navigation tile data. </summary>
		public static readonly int DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';
		/// <summary>
		/// A version number used to detect compatibility of navigation tile data. </summary>
		public const int DT_NAVMESH_VERSION = 7;
		public const int DT_NAVMESH_VERSION_RECAST4J = 0x8807;
		/// <summary>
		/// A magic number used to detect the compatibility of navigation tile states. </summary>
		public static readonly int DT_NAVMESH_STATE_MAGIC = 'D' << 24 | 'N' << 16 | 'M' << 8 | 'S';
		/// <summary>
		/// A version number used to detect compatibility of navigation tile states. </summary>
		public const int DT_NAVMESH_STATE_VERSION = 1;

		/// <summary>
		/// < Tile magic number. (Used to identify the data format.) </summary>
		public int magic;
		/// <summary>
		/// < Tile data format version number. </summary>
		public int version;
		/// <summary>
		/// < The x-position of the tile within the dtNavMesh tile grid. (x, y, layer) </summary>
		public int x;
		/// <summary>
		/// < The y-position of the tile within the dtNavMesh tile grid. (x, y, layer) </summary>
		public int y;
		/// <summary>
		/// < The layer of the tile within the dtNavMesh tile grid. (x, y, layer) </summary>
		public int layer;
		/// <summary>
		/// < The user defined id of the tile. </summary>
		public int userId;
		/// <summary>
		/// < The number of polygons in the tile. </summary>
		public int polyCount;
		/// <summary>
		/// < The number of vertices in the tile. </summary>
		public int vertCount;
		/// <summary>
		/// < The number of allocated links. </summary>
		public int maxLinkCount;
		/// <summary>
		/// < The number of sub-meshes in the detail mesh. </summary>
		public int detailMeshCount;
		/// <summary>
		/// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.) </summary>
		public int detailVertCount;
		/// <summary>
		/// < The number of triangles in the detail mesh. </summary>
		public int detailTriCount;
		/// <summary>
		/// < The number of bounding volume nodes. (Zero if bounding volumes are disabled.) </summary>
		public int bvNodeCount;
		/// <summary>
		/// < The number of off-mesh connections. </summary>
		public int offMeshConCount;
		/// <summary>
		/// < The index of the first polygon which is an off-mesh connection. </summary>
		public int offMeshBase;
		/// <summary>
		/// < The height of the agents using the tile. </summary>
		public float walkableHeight;
		/// <summary>
		/// < The radius of the agents using the tile. </summary>
		public float walkableRadius;
		/// <summary>
		/// < The maximum climb height of the agents using the tile. </summary>
		public float walkableClimb;
		/// <summary>
		/// < The minimum bounds of the tile's AABB. [(x, y, z)] </summary>
		public readonly float[] bmin = new float[3];
		/// <summary>
		/// < The maximum bounds of the tile's AABB. [(x, y, z)] </summary>
		public readonly float[] bmax = new float[3];
		/// <summary>
		/// The bounding volume quantization factor. </summary>
		public float bvQuantFactor;
	}

}