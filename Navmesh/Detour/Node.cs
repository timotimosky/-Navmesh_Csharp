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

	public class Node
	{

		internal static int DT_NODE_OPEN = 0x01;
		internal static int DT_NODE_CLOSED = 0x02;
		/// <summary>
		/// parent of the node is not adjacent. Found using raycast. </summary>
		internal static int DT_NODE_PARENT_DETACHED = 0x04;

		public readonly int index;

		/// <summary>
		/// Position of the node. </summary>
		private float[] pos;
		/// <summary>
		/// Cost from previous node to current node. </summary>
		internal float cost;
		/// <summary>
		/// Cost up to the node. </summary>
		internal float total;
		/// <summary>
		/// Index to parent node. </summary>
		internal int pidx;
		/// <summary>
		/// extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE </summary>
		internal int state;
		/// <summary>
		/// Node flags. A combination of dtNodeFlags. </summary>
		internal int flags;
		/// <summary>
		/// Polygon ref the node corresponds to. </summary>
		internal long id;

		public Node(int index) : base()
		{
			this.index = index;
		}

		public virtual float[] Pos
		{
			get
			{
				if (pos == null)
				{
					pos = new float[3];
				}
				return pos;
			}
			set
			{
				this.pos = value;
			}
		}


		public override string ToString()
		{
			return "[index:" + index + ",pos:" + pos[0] + " " + pos[1] + " " + pos[2] + "]";
		}




	}

}