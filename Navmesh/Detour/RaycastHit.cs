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
using System;
using System.Collections.Generic;

namespace org.recast4j.detour
{


	/// <summary>
	/// Provides information about raycast hit. Filled by NavMeshQuery::raycast
	/// </summary>
	public class RaycastHit
	{
		/// <summary>
		/// The hit parameter. (Float.MAX_VALUE if no wall hit.) </summary>
		public float t;
		/// <summary>
		/// hitNormal The normal of the nearest wall hit. [(x, y, z)] </summary>
		public readonly float[] hitNormal = new float[3];
		/// <summary>
		/// Visited polygons. </summary>
		public readonly List<long> path = new List<long>();
		/// <summary>
		/// The cost of the path until hit. </summary>
		public float pathCost;
		/// <summary>
		/// The index of the edge on the final polygon where the wall was hit. </summary>
		public int hitEdgeIndex;



		/// <summary>
		/// �Ƿ����赲������ײ
		/// @return
		/// </summary>
		public virtual bool Collide
		{
			get
			{
				return t >= 0 && t < 1;
			}
		}

	}

}