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
	/// A compact, static heightfield representing unobstructed space. </summary>
	public class CompactHeightfield
	{

		/// <summary>
		/// The width of the heightfield. (Along the x-axis in cell units.) </summary>
		public int width;
		/// <summary>
		/// The height of the heightfield. (Along the z-axis in cell units.) </summary>
		public int height;
		/// <summary>
		/// The number of spans in the heightfield. </summary>
		public int spanCount;
		/// <summary>
		/// The walkable height used during the build of the field.  (See: RecastConfig::walkableHeight) </summary>
		public int walkableHeight;
		/// <summary>
		/// The walkable climb used during the build of the field. (See: RecastConfig::walkableClimb) </summary>
		public int walkableClimb;
		/// <summary>
		/// The AABB border size used during the build of the field. (See: RecastConfig::borderSize) </summary>
		public int borderSize;
		/// <summary>
		/// The maximum distance value of any span within the field. </summary>
		public int maxDistance;
		/// <summary>
		/// The maximum region id of any span within the field. </summary>
		public int maxRegions;
		/// <summary>
		/// The minimum bounds in world space. [(x, y, z)] </summary>
		public readonly float[] bmin = new float[3];
		/// <summary>
		/// The maximum bounds in world space. [(x, y, z)] </summary>
		public readonly float[] bmax = new float[3];
		/// <summary>
		/// The size of each cell. (On the xz-plane.) </summary>
		public float cs;
		/// <summary>
		/// The height of each cell. (The minimum increment along the y-axis.) </summary>
		public float ch;
		/// <summary>
		/// Array of cells. [Size: #width*#height] </summary>
		public CompactCell[] cells;
		/// <summary>
		/// Array of spans. [Size: #spanCount] </summary>
		public CompactSpan[] spans;
		/// <summary>
		/// Array containing border distance data. [Size: #spanCount] </summary>
		public int[] dist;
		/// <summary>
		/// Array containing area id data. [Size: #spanCount] </summary>
		public int[] areas;

	}

}