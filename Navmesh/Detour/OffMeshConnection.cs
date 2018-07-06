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
	/// Defines an navigation mesh off-mesh connection within a dtMeshTile object. An off-mesh connection is a user defined
	/// traversable connection made up to two vertices.
	/// </summary>
	public class OffMeshConnection
	{
		/// <summary>
		/// The endpoints of the connection. [(ax, ay, az, bx, by, bz)] </summary>
		public float[] pos = new float[6];
		/// <summary>
		/// The radius of the endpoints. [Limit: >= 0] </summary>
		public float rad;
		/// <summary>
		/// The polygon reference of the connection within the tile. </summary>
		public int poly;
		/// <summary>
		/// Link flags.
		/// 
		/// @note These are not the connection's user defined flags. Those are assigned via the connection's Poly definition.
		///       These are link flags used for internal purposes.
		/// </summary>
		public int flags;
		/// <summary>
		/// End point side. </summary>
		public int side;
		/// <summary>
		/// The id of the offmesh connection. (User assigned when the navigation mesh is built.) </summary>
		public int userId;
	}
}