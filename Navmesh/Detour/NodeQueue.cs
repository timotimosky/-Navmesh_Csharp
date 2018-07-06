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

	public class NodeQueue
	{

//JAVA TO C# CONVERTER TODO TASK: Java lambdas satisfy functional interfaces, while .NET lambdas satisfy delegates - change the appropriate interface to a delegate:
		private readonly PriorityQueue<Node> m_heap = new PriorityQueue<Node>(new QueueComparator());

		public virtual void clear()
		{
			m_heap.Clear();
		}

		public virtual Node top()
		{
			return m_heap.Peek();
		}

		public virtual Node pop()
		{
			return m_heap.Poll();
		}

		public virtual void push(Node node)
		{
			m_heap.Offer(node);
		}

		public virtual void modify(Node node)
		{
			m_heap.Remove(node);
			m_heap.Offer(node);
		}

		public virtual bool Empty
		{
			get
			{
				return m_heap.Count <= 0;
			}
		}
	}



    public class QueueComparator : IComparer<Node>
    {
        public virtual int Compare(Node n1, Node n2)
        {
            return n1.total.CompareTo(n2.total);
        }
    }

}