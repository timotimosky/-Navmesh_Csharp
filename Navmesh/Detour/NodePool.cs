using System;
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
namespace org.recast4j.detour
{


	public class NodePool
	{

        internal Dictionary<long, IList<Node>> m_map = new Dictionary<long, IList<Node>>();
		internal List<Node> m_nodes = new List<Node>();

		public NodePool()
		{

		}

		public virtual void clear()
		{
			m_nodes.Clear();
			m_map.Clear();
		}

		internal virtual IList<Node> findNodes(long id, int maxNodes)
		{
			IList<Node> nodes = null;
           
			if (!m_map.TryGetValue(id, out nodes))
			{
                nodes = new List<Node>();
			}
			return nodes;
		}

		internal virtual Node findNode(long id)
		{
			IList<Node> nodes = null;
            m_map.TryGetValue(id, out nodes);

			if (nodes != null && nodes.Count > 0)
			{
				return nodes[0];
			}
			return null;
		}

		internal virtual Node findNode(long id, int state)
		{
            IList<Node> nodes = null;
            if (m_map.TryGetValue(id, out nodes))
			{
				foreach (Node node in nodes)
				{
					if (node.state == state)
					{
						return node;
					}
				}
			}
			return null;
		}

		internal virtual Node getNode(long id, int state)
		{
			IList<Node> nodes = null;
            if (m_map.TryGetValue(id, out nodes))
			{
				foreach (Node node in nodes)
				{
					if (node.state == state)
					{
						return node;
					}
				}
			}
			return create(id, state);
		}

		protected internal virtual Node create(long id, int state)
		{
			Node node = new Node(m_nodes.Count + 1);
			node.id = id;
			node.state = state;
			m_nodes.Add(node);
			IList<Node> nodes = null;
            if (!m_map.TryGetValue(id, out nodes))
			{
                nodes = new List<Node>();
				m_map.Add(id, nodes);
			}
			nodes.Add(node);
			return node;
		}

		public virtual int getNodeIdx(Node node)
		{
			return node != null ? node.index : 0;
		}

		public virtual Node getNodeAtIdx(int idx)
		{
			return idx != 0 ? m_nodes[idx - 1] : null;
		}

		public virtual int NodeCount
		{
			get
			{
				return m_nodes.Count;
			}
		}

		public virtual Node getNode(long @ref)
		{
			return getNode(@ref, 0);
		}

		/*
		
		inline int getMaxNodes() const { return m_maxNodes; }
		inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
		inline dtNodeIndex getNext(int i) const { return m_next[i]; }
		*/
	}

}