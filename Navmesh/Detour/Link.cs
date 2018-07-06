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
	/// Defines a link between polygons.
	/// 
	/// @note This structure is rarely if ever used by the end user. </summary>
	/// <seealso cref= MeshTile </seealso>
	public class Link
	{
		/// <summary>
		/// Neighbour reference. (The neighbor that is linked to.) </summary>
		internal long @ref;
		/// <summary>
		/// Index of the next link. </summary>
		internal int next;
		/// <summary>
		/// Index of the polygon edge that owns this link. </summary>
		internal int edge;
		/// <summary>
		/// If a boundary link, defines on which side the link is. </summary>
		internal int side;
		/// <summary>
		/// If a boundary link, defines the minimum sub-edge area. </summary>
		internal int bmin;
		/// <summary>
		/// If a boundary link, defines the maximum sub-edge area. </summary>
		internal int bmax;

	}

}