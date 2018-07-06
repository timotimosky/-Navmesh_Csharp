using System;

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
namespace org.recast4j.detour.crowd
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.distancePtSegSqr2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.sqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vCopy;



	public class LocalBoundary
	{


		private const int MAX_LOCAL_SEGS = 8;
        private const int MAX_LOCAL_POLYS = 16;

		private class Segment
		{
			/// <summary>
			/// Segment start/end </summary>
			internal float[] s = new float[6];
			/// <summary>
			/// Distance for pruning. </summary>
			internal float d;
		}

	/*	List<Segment> m_segs = new ArrayList<>();
		List<long> m_polys = new List<long>();*/

		private float[] m_center = new float[3];
		private Segment[] m_segs = new Segment[MAX_LOCAL_SEGS];
		private int m_nsegs;

		private long[] m_polys = new long[MAX_LOCAL_POLYS];
        private int m_npolys = 0;


		private int MAX_SEGS_PER_POLY = 6 * 3;
        private float[] segs;
		private float[] s = new float[6];
		/// <summary>
		///************** </summary>

		protected internal LocalBoundary()
		{
			m_center[0] = m_center[1] = m_center[2] = float.MaxValue;

            segs = new float[MAX_SEGS_PER_POLY * 6];

			for (int i = 0; i < m_segs.Length; i++)
			{
				m_segs[i] = new Segment();
			}

			m_nsegs = 0;
		}

		protected internal virtual void reset()
		{
			m_center[0] = m_center[1] = m_center[2] = float.MaxValue;
			m_npolys = 0;
			m_nsegs = 0;

			/*m_polys.clear();
			m_segs.clear();*/
		}

		protected internal virtual void addSegment(float dist, float[] s)
		{
			// Insert neighbour based on the distance.
			Segment seg;
			if (m_nsegs == 0)
			{
				seg = m_segs[0];
			}
			else if (dist >= m_segs[m_nsegs - 1].d)
			{
				if (m_nsegs >= MAX_LOCAL_SEGS)
				{
					return;
				}
				seg = m_segs[m_nsegs];
			}
			else
			{
				// Insert inbetween.
				int i;
				for (i = 0; i < m_nsegs; ++i)
				{
					if (dist <= m_segs[i].d)
					{
						break;
					}
				}
				if (i < MAX_LOCAL_SEGS - 1)
				{
					Array.Copy(m_segs, i, m_segs, i + 1, m_segs.Length - i - 1);
				}
				seg = m_segs[i];
			}
			seg.d = dist;
			Array.Copy(s, 0, seg.s, 0, 6);
			if (m_nsegs < MAX_LOCAL_SEGS)
			{
				m_nsegs += 1;
			}
		}


	/*	protected void addSegment(float dist, float[] s) {
			// Insert neighbour based on the distance.
			Segment seg = new Segment();
			System.arraycopy(s, 0, seg.s, 0, 6);
			seg.d = dist;
			if (m_segs.isEmpty()) {
				m_segs.add(seg);
			} else if (dist >= m_segs.get(m_segs.size() - 1).d) {
				if (m_segs.size() >= MAX_LOCAL_SEGS) {
					return;
				}
				m_segs.add(seg);
			} else {
				// Insert inbetween.
				int i;
				for (i = 0; i < m_segs.size(); ++i)
					if (dist <= m_segs.get(i).d)
						break;
				m_segs.add(i, seg);
			}
			while (m_segs.size() > MAX_LOCAL_SEGS) {
				m_segs.remove(m_segs.size() - 1);
			}
		}*/



		public virtual void update(long @ref, float[] pos, float collisionQueryRange, NavMeshQuery navquery, QueryFilter filter)
		{

			if (@ref == 0)
			{
				reset();
				return;
			}
			DetourCommon.vCopy(m_center, pos);
			// First query non-overlapping polygons.
			navquery.findLocalNeighbourhood(@ref, pos, collisionQueryRange, filter, m_polys, null, ref m_npolys, MAX_LOCAL_POLYS);

			// Secondly, store all polygon edges.
			m_nsegs = 0;
			int nsegs = 0;

            float t = 0;
			for (int j = 0; j < m_npolys; ++j)
			{
				navquery.getPolyWallSegments(m_polys[j], filter, segs, null, ref nsegs, MAX_SEGS_PER_POLY);
                for (int k = 0; k < nsegs; ++k)
				{
					Array.Copy(segs, k * 6, s, 0, 6);
					// Skip too distant segments.
                    float dist = DetourCommon.distancePtSegSqr2D(pos, s, 0, 3, ref t);
                    if (dist > DetourCommon.sqr(collisionQueryRange))
					{
						continue;
					}
                    addSegment(dist, s);
				}
			}
		}



		public virtual bool isValid(NavMeshQuery navquery, QueryFilter filter)
		{
			if (m_npolys <= 0)
			{
				return false;
			}

			// Check that all polygons still pass query filter.
			long @ref;
			for (int i = 0; i < m_npolys; i++)
			{
				@ref = m_polys[i];
				if (!navquery.isValidPolyRef(@ref, filter))
				{
					return false;
				}
			}
			return true;
		}
	/*	
		public void update(long ref, float[] pos, float collisionQueryRange, NavMeshQuery navquery, QueryFilter filter) {
			if (ref == 0) {
				reset();
				return;
			}
			vCopy(m_center, pos);
			// First query non-overlapping polygons.
			FindLocalNeighbourhoodResult res = navquery.findLocalNeighbourhood(ref, pos, collisionQueryRange, filter);
			this.m_polys = res.getRefs();
			m_segs.clear();
			// Secondly, store all polygon edges.
			float[] s;
			FloatFloatTupple2 distseg = new FloatFloatTupple2();
			GetPolyWallSegmentsResult gpws;
			for (int j = 0; j < m_polys.size(); ++j) {
				gpws = navquery.getPolyWallSegments(m_polys.get(j), false, filter);
				for (int k = 0; k < gpws.getSegmentRefs().size(); ++k) {
					s = gpws.getSegmentVerts().get(k);
					// Skip too distant segments.
					distseg.first = 0f;
					distseg.second = 0;
					distancePtSegSqr2D(pos, s, 0, 3, distseg);
					if (distseg.first > sqr(collisionQueryRange))
						continue;
					addSegment(distseg.first, s);
				}
			}
		}*/


	/*	public boolean isValid(NavMeshQuery navquery, QueryFilter filter) 
		{
			if (m_polys.isEmpty())
				return false;
	
			// Check that all polygons still pass query filter.
			long ref;
			for(int i = 0; i < m_polys.size(); i++)
			{
				ref = m_polys.get(i);
				if (!navquery.isValidPolyRef(ref, filter))
					return false;
			}
			return true;
		}*/


		public virtual float[] Center
		{
			get
			{
				return m_center;
			}
		}

		public virtual float[] getSegment(int j)
		{
			return m_segs[j].s;
		}

		public virtual int SegmentCount
		{
			get
			{
				return m_nsegs;
			}
		}

	/*	public float[] getSegment(int j) 
		{
			return m_segs.get(j).s;
		}
	
		public int getSegmentCount()
		{
			return m_segs.size();
		}*/
	}



}