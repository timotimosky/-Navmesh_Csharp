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
	///  Configuration parameters used to define multi-tile navigation meshes.
	///  The values are used to allocate space during the initialization of a navigation mesh. </summary>
	///  <seealso cref= NavMesh </seealso>
	public class NavMeshParams
	{
		/// <summary>
		/// The world space origin of the navigation mesh's tile space. [(x, y, z)] </summary>
		public readonly float[] orig = new float[3];
		/// <summary>
		/// The width of each tile. (Along the x-axis.) </summary>
		public float tileWidth;
		/// <summary>
		/// The height of each tile. (Along the z-axis.) </summary>
		public float tileHeight;
		/// <summary>
		/// The maximum number of tiles the navigation mesh can contain. </summary>
		public int maxTiles;
		/// <summary>
		/// The maximum number of polygons each tile can contain. </summary>
		public int maxPolys;
	}

}