using System.Collections.Generic;

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
	/// Represents a group of related contours. </summary>
	public class ContourSet
	{

		/// <summary>
		/// A list of the contours in the set. </summary>
		public readonly List<Contour> conts = new List<Contour>();
		/// <summary>
		/// The minimum bounds in world space. [(x, y, z)] </summary>
		internal readonly float[] bmin = new float[3];
		/// <summary>
		/// The maximum bounds in world space. [(x, y, z)] </summary>
		internal readonly float[] bmax = new float[3];
		/// <summary>
		/// The size of each cell. (On the xz-plane.) </summary>
		public float cs;
		/// <summary>
		/// The height of each cell. (The minimum increment along the y-axis.) </summary>
		public float ch;
		/// <summary>
		/// The width of the set. (Along the x-axis in cell units.) </summary>
		public int width;
		/// <summary>
		/// The height of the set. (Along the z-axis in cell units.) </summary>
		public int height;
		/// <summary>
		/// The AABB border size used to generate the source data from which the contours were derived. </summary>
		public int borderSize;
	}

}