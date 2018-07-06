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
namespace org.recast4j.recast
{

	/// <summary>
	/// Contains triangle meshes that represent detailed height data associated with the polygons in its associated polygon mesh object. </summary>
	public class PolyMeshDetail
	{

		/// <summary>
		/// The sub-mesh data. [Size: 4*#nmeshes] </summary>
		public int[] meshes;
		/// <summary>
		/// The mesh vertices. [Size: 3*#nverts] </summary>
		public float[] verts;
		/// <summary>
		/// The mesh triangles. [Size: 4*#ntris] </summary>
		public int[] tris;
		/// <summary>
		/// The number of sub-meshes defined by #meshes. </summary>
		public int nmeshes;
		/// <summary>
		/// The number of vertices in #verts. </summary>
		public int nverts;
		/// <summary>
		/// The number of triangles in #tris. </summary>
		public int ntris;

	}
}