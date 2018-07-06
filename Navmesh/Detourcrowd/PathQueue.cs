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


namespace org.recast4j.detour.crowd
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vCopy;



	public class PathQueue
	{

		private const int MAX_QUEUE = 8;
		internal const int DT_PATHQ_INVALID = 0;
		private const int MAX_KEEP_ALIVE = 2; // in update ticks.

		private PathQuery[] m_queue = new PathQuery[MAX_QUEUE];
		private long m_nextHandle = 1;
		private int m_queueHead;
		private NavMeshQuery m_navquery;

		protected internal PathQueue(int maxSearchNodeCount, NavMesh nav)
		{
			m_navquery = new NavMeshQuery(nav);
			for (int i = 0; i < MAX_QUEUE; ++i)
			{
				m_queue[i] = new PathQuery();
				m_queue[i].@ref = DT_PATHQ_INVALID;
				m_queue[i].path = new List<long>(50);
			}
			m_queueHead = 0;
		}

		protected internal virtual void update(int maxIters)
		{
			// Update path request until there is nothing to update
			// or upto maxIters pathfinder iterations has been consumed.
			int iterCount = maxIters;
			PathQuery q;
			int iters;
			UpdateSlicedPathResult res;
			FindPathResult path;
			for (int i = 0; i < MAX_QUEUE; ++i)
			{
				q = m_queue[m_queueHead % MAX_QUEUE];

				// Skip inactive requests.
				if (q.@ref == DT_PATHQ_INVALID)
				{
					m_queueHead++;
					continue;
				}

				// Handle completed request.
				if (q.status != null && (q.status.Success || q.status.Failed))
				{
					// If the path result has not been read in few frames, free the slot.
					q.keepAlive++;
					if (q.keepAlive > MAX_KEEP_ALIVE)
					{
						q.@ref = DT_PATHQ_INVALID;
						q.status = null;
					}

					m_queueHead++;
					continue;
				}

				// Handle query start.
				if (q.status == null)
				{
					q.status = m_navquery.initSlicedFindPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter, 0);
				}
				// Handle query in progress.
				if (q.status.InProgress)
				{
					iters = 0;
					res = m_navquery.updateSlicedFindPath(iterCount);
					iters = res.Iterations;
					q.status = res.Status;
					iterCount -= iters;
				}
				if (q.status.Success)
				{
					path = m_navquery.finalizeSlicedFindPath();
					q.status = path.Status;
					q.path = path.Refs;
				}

				if (iterCount <= 0)
				{
					break;
				}

				m_queueHead++;
			}

		}

		protected internal virtual long request(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter)
		{
			// Find empty slot
			int slot = -1;
			for (int i = 0; i < MAX_QUEUE; ++i)
			{
				if (m_queue[i].@ref == DT_PATHQ_INVALID)
				{
					slot = i;
					break;
				}
			}
			// Could not find slot.
			if (slot == -1)
			{
				return DT_PATHQ_INVALID;
			}

			long @ref = m_nextHandle++;
			if (m_nextHandle == DT_PATHQ_INVALID)
			{
				m_nextHandle++;
			}

			PathQuery q = m_queue[slot];
			q.@ref = @ref;
			DetourCommon.vCopy(q.startPos, startPos);
			q.startRef = startRef;
            DetourCommon.vCopy(q.endPos, endPos);
			q.endRef = endRef;
			q.status = null;
			q.filter = filter;
			q.keepAlive = 0;
			return @ref;

		}

		internal virtual Status getRequestStatus(long @ref)
		{
			for (int i = 0; i < MAX_QUEUE; ++i)
			{
				if (m_queue[i].@ref == @ref)
				{
					return m_queue[i].status;
				}
			}
			return Status.FAILURE;

		}

		internal virtual FindPathResult getPathResult(long @ref)
		{
			PathQuery q;
			for (int i = 0; i < MAX_QUEUE; ++i)
			{
				if (m_queue[i].@ref == @ref)
				{
					q = m_queue[i];
					// Free request for reuse.
					q.@ref = DT_PATHQ_INVALID;
					q.status = null;
					return new FindPathResult(Status.SUCCSESS, q.path);
				}
			}
			return FindPathResult.EMPTY_RESULT;
		}
	}

}