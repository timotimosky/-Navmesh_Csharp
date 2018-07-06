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
	/// Defines a polyogn within a dtMeshTile object. </summary>
	public class Poly
	{

		internal readonly int index;
		/// <summary>
		/// The polygon is a standard convex polygon that is part of the surface of the mesh. </summary>
		public const int DT_POLYTYPE_GROUND = 0;
		/// <summary>
		/// The polygon is an off-mesh connection consisting of two vertices. </summary>
		public const int DT_POLYTYPE_OFFMESH_CONNECTION = 1;
		/// <summary>
		/// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.) </summary>
		public int firstLink;
		/// <summary>
		/// The indices of the polygon's vertices. The actual vertices are located in MeshTile::verts. </summary>
		public readonly int[] verts;
		/// <summary>
		/// Packed data representing neighbor polygons references and flags for each edge. </summary>
		public readonly int[] neis;
		/// <summary>
		/// The user defined polygon flags. </summary>
		public int flags;
		/// <summary>
		/// The number of vertices in the polygon. </summary>
		public int vertCount;
		/// <summary>
		/// The bit packed area id and polygon type.
		/// 
		/// @note Use the structure's set and get methods to access this value.
		/// </summary>
		public int areaAndtype;

		public Poly(int index, int maxVertsPerPoly)
		{
			this.index = index;
			verts = new int[maxVertsPerPoly];
			neis = new int[maxVertsPerPoly];
		}

		/// <summary>
		/// Sets the user defined area id. [Limit: < #DT_MAX_AREAS] </summary>
		internal virtual int Area
		{
			set
			{
				areaAndtype = (areaAndtype & 0xc0) | (value & 0x3f);
			}
			get
			{
				return areaAndtype & 0x3f;
			}
		}

		/// <summary>
		/// Sets the polygon type. (See: #dtPolyTypes.) </summary>
		internal virtual int Type
		{
			set
			{
				areaAndtype = (areaAndtype & 0x3f) | (value << 6);
			}
			get
			{
				return areaAndtype >> 6;
			}
		}



	}

}