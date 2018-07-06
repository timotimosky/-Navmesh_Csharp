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


	public class NavMeshQuery
	{
		private bool InstanceFieldsInitialized = false;

		private void InitializeInstanceFields()
		{
			qpneis = new MeshTile[QP_MAX_NEIS];
			masneis = new long[MAX_NEIS];
		}


		public const int DT_FINDPATH_LOW_QUALITY_FAR = 0x01; /// < [provisional] trade quality for performance far
																	/// from the origin. The idea is that by then a new
																	/// query will be issued
		public const int DT_FINDPATH_ANY_ANGLE = 0x02; /// < use raycasts during pathfind to "shortcut" (raycast
																/// still consider costs)

		/// <summary>
		/// Raycast should calculate movement cost along the ray and fill RaycastHit::cost </summary>
		public const int DT_RAYCAST_USE_COSTS = 0x01;

		/// Vertex flags returned by findStraightPath.
		/// <summary>
		/// The vertex is the start position in the path. </summary>
		public const int DT_STRAIGHTPATH_START = 0x01;
		/// <summary>
		/// The vertex is the end position in the path. </summary>
		public const int DT_STRAIGHTPATH_END = 0x02;
		/// <summary>
		/// The vertex is the start of an off-mesh connection. </summary>
		public const int DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04;

		/// Options for findStraightPath.
		public const int DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01; ///< Add a vertex at every polygon edge crossing where area changes.
		public const int DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02; ///< Add a vertex at every polygon edge crossing.

		internal static float H_SCALE = 0.999f; // Search heuristic scale.

		private readonly NavMesh m_nav;
		private readonly NodePool m_nodePool;
		private readonly NodePool m_tinyNodePool;
		private readonly NodeQueue m_openList;
		private QueryData m_query; /// < Sliced query state.

		/// <summary>
		///************������ʱ����******************* </summary>
		private readonly float[] frpverts;
		private readonly float[] frpareas;

		private readonly float[] frpacva = new float[3];
		private readonly float[] frpacvb = new float[3];
		private readonly float[] frpacverts;
		private readonly float[] frpacareas;

		private readonly float[] cpopverts;
		private readonly float[] cpopedged;
		private readonly float[] cpopedget;

		private readonly float[] cpopbverts;
		private readonly float[] cpopbedged;
		private readonly float[] cpopbedget;

		private readonly float[] fnpdiff = new float[3];
		private readonly long[] fnppolys = new long[128];
		private readonly ClosesPointOnPolyResult fnpclosest = new ClosesPointOnPolyResult();

		private readonly int[] qpitbmin1 = new int[3];
		private readonly int[] qpitbmax1 = new int[3];
		private readonly float[] qpitbmin2 = new float[3];
		private readonly float[] qpitbmax2 = new float[3];

		private readonly float[] qpbmin = new float[3];
		private readonly float[] qpbmax = new float[3];
		private readonly int[] qpminxy = new int[2];
		private readonly int[] qpmaxxy = new int[2];
		private readonly int QP_MAX_NEIS = 32;
		private MeshTile[] qpneis;

		private readonly float[] appt = new float[3];
		private readonly float[] apleft = new float[3];
		private readonly float[] apright = new float[3];

		private readonly float[] fspportalApex = new float[3];
		private readonly float[] fspportalLeft = new float[3];
		private readonly float[] fspportalRight = new float[3];
		private readonly float[] fspleft = new float[3];
		private readonly float[] fspright = new float[3];

		private readonly float[] masverts;
		private readonly int MAX_NEIS = 8;
		private long[] masneis;
		private readonly float[] massearchPos = new float[3];

		private readonly float[] rcverts;
		private readonly float[] rccurPos = new float[3];
		private readonly float[] rclastPos = new float[3];
		private readonly float[] rcdir = new float[3];
		private readonly float[] rceDir = new float[3];
		private readonly float[] rcdiff = new float[3];
        private DetourCommon.IntersectResult rciresult = new DetourCommon.IntersectResult();

		private readonly float[] fpacva = new float[3];
		private readonly float[] fpacvb = new float[3];

		private readonly float[] flnpa;
		private readonly float[] flnpb;
		private readonly float[] flnva = new float[3];
		private readonly float[] flnvb = new float[3];

		private SegInterval[] gpwsInts = new SegInterval[MAX_INTERVAL];
		internal float[] gpwstemp1 = new float[3];
		internal const int MAX_INTERVAL = 16;


		/// <summary>
		///******************************* </summary>

		public NavMeshQuery(NavMesh nav)
		{
			if (!InstanceFieldsInitialized)
			{
				InitializeInstanceFields();
				InstanceFieldsInitialized = true;
			}
			m_nav = nav;
			m_nodePool = new NodePool();
			m_tinyNodePool = new NodePool();
			m_openList = new NodeQueue();

			frpverts = new float[3 * m_nav.MaxVertsPerPoly];
			frpareas = new float[m_nav.MaxVertsPerPoly];
			frpacverts = new float[3 * m_nav.MaxVertsPerPoly];
			frpacareas = new float[m_nav.MaxVertsPerPoly];

			cpopverts = new float[m_nav.MaxVertsPerPoly * 3];
			cpopedged = new float[m_nav.MaxVertsPerPoly];
			cpopedget = new float[m_nav.MaxVertsPerPoly];

			cpopbverts = new float[m_nav.MaxVertsPerPoly * 3];
			cpopbedged = new float[m_nav.MaxVertsPerPoly];
			cpopbedget = new float[m_nav.MaxVertsPerPoly];

			masverts = new float[m_nav.MaxVertsPerPoly * 3];
			rcverts = new float[m_nav.MaxVertsPerPoly * 3 + 3];

			flnpa = new float[m_nav.MaxVertsPerPoly * 3];
			flnpb = new float[m_nav.MaxVertsPerPoly * 3];

			for (int i = 0; i < gpwsInts.Length; i++)
			{
				gpwsInts[i] = new SegInterval(0, 0, 0);
			}
		}

		public class FRand
		{
			internal Random r = new Random();

			public virtual float frand()
			{
				return (float)r.NextDouble();
			}
		}

		/// <summary>
		/// Returns random location on navmesh.
		/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon. </summary>
		/// <param name="filter"> The polygon filter to apply to the query. </param>
		/// <param name="frand"> Function returning a random number [0..1). </param>
		/// <returns> Random location </returns>
		public virtual FindRandomPointResult findRandomPoint(QueryFilter filter, FRand frand)
		{
			// Randomly pick one tile. Assume that all tiles cover roughly the same area.
			MeshTile tile = null;
			float tsum = 0.0f;
			MeshTile meshTile;
			for (int i = 0; i < m_nav.MaxTiles; i++)
			{
				meshTile = m_nav.getTile(i);
				if (meshTile == null || meshTile.data == null || meshTile.data.header == null)
				{
					continue;
				}

				// Choose random tile using reservoi sampling.
				float area = 1.0f; // Could be tile area too.
				tsum += area;
				float u1 = frand.frand();
				if (u1 * tsum <= area)
				{
					tile = meshTile;
				}
			}
			if (tile == null)
			{
				return FindRandomPointResult.EMPTY_RESULT;
			}

			// Randomly pick one polygon weighted by polygon area.
			Poly poly = null;
			long polyRef = 0;
			long @base = m_nav.getPolyRefBase(tile);

			float areaSum = 0.0f;
			Poly p;
			float polyArea, u;
			int va, vb, vc;
			for (int i = 0; i < tile.data.header.polyCount; ++i)
			{
				p = tile.data.polys[i];
				// Do not return off-mesh connection polygons.
				if (p.Type != Poly.DT_POLYTYPE_GROUND)
				{
					continue;
				}
				// Must pass filter
				long @ref = @base | i;
				if (!filter.passFilter(@ref, tile, p))
				{
					continue;
				}

				// Calc area of the polygon.
				polyArea = 0.0f;
				for (int j = 2; j < p.vertCount; ++j)
				{
					va = p.verts[0] * 3;
					vb = p.verts[j - 1] * 3;
					vc = p.verts[j] * 3;
					polyArea += DetourCommon.triArea2D(tile.data.verts, va, vb, vc);
				}

				// Choose random polygon weighted by area, using reservoi sampling.
				areaSum += polyArea;
				u = frand.frand();
				if (u * areaSum <= polyArea)
				{
					poly = p;
					polyRef = @ref;
				}
			}

			if (poly == null)
			{
				return FindRandomPointResult.EMPTY_RESULT;
			}

			// Randomly pick point on polygon.
			//float[] verts = new float[3 * m_nav.getMaxVertsPerPoly()];
			//float[] areas = new float[m_nav.getMaxVertsPerPoly()];
			DetourCommon.vResetArray(frpverts);
			DetourCommon.vResetArray(frpareas);

			Array.Copy(tile.data.verts, poly.verts[0] * 3, frpverts, 0, 3);
			for (int j = 1; j < poly.vertCount; ++j)
			{
				Array.Copy(tile.data.verts, poly.verts[j] * 3, frpverts, j * 3, 3);
			}

			float s = frand.frand();
			float t = frand.frand();

			float[] pt = DetourCommon.randomPointInConvexPoly(frpverts, poly.vertCount, frpareas, s, t);

			pt[1] = getPolyHeight(polyRef, pt, 0, pt[1]);

			return new FindRandomPointResult(Status.SUCCSESS, polyRef, pt);
		}



		/// <summary>
		/// Returns random location on navmesh within the reach of specified location.
		/// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
		/// The location is not exactly constrained by the circle, but it limits the visited polygons.
		/// </summary>
		/// <param name="startRef"> The reference id of the polygon where the search starts. </param>
		/// <param name="centerPos"> The center of the search circle. [(x, y, z)] </param>
		/// <param name="maxRadius"> </param>
		/// <param name="filter"> The polygon filter to apply to the query. </param>
		/// <param name="frand"> Function returning a random number [0..1). </param>
		/// <returns> Random location </returns>
		public virtual FindRandomPointResult findRandomPointAroundCircle(long startRef, float[] centerPos, float maxRadius, QueryFilter filter, FRand frand)
		{

			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}

            MeshTile startTile = null;
            Poly startPoly = null;

            m_nav.getTileAndPolyByRefUnsafe(startRef, ref startTile, ref startPoly);

			
			if (!filter.passFilter(startRef, startTile, startPoly))
			{
				throw new System.ArgumentException("Invalid start");
			}

			m_nodePool.clear();
			m_openList.clear();

			Node startNode = m_nodePool.getNode(startRef);
			DetourCommon.vCopy(startNode.Pos, centerPos);
			startNode.pidx = 0;
			startNode.cost = 0;
			startNode.total = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

			float radiusSqr = maxRadius * maxRadius;
			float areaSum = 0.0f;

			MeshTile randomTile = null;
			Poly randomPoly = null;
			long randomPolyRef = 0;

			Node bestNode;
			MeshTile bestTile = null;
			Poly bestPoly = null;
			float polyArea = 0.0f;
			float u = 0f;

			Link link;
			long neighbourRef;
			MeshTile neighbourTile;
			Poly neighbourPoly;
			int va, vb, vc;
			PortalResult portalpoints;
			float distt = 0;
			Node neighbourNode;
			while (!m_openList.Empty)
			{
				bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;
				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				long bestRef = bestNode.id;
                bestTile = null;
                bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, ref bestTile, ref bestPoly);

				// Place random locations on on ground.
				if (bestPoly.Type == Poly.DT_POLYTYPE_GROUND)
				{
					// Calc area of the polygon.
					polyArea = 0.0f;
					for (int j = 2; j < bestPoly.vertCount; ++j)
					{
						va = bestPoly.verts[0] * 3;
						vb = bestPoly.verts[j - 1] * 3;
						vc = bestPoly.verts[j] * 3;
						polyArea += DetourCommon.triArea2D(bestTile.data.verts, va, vb, vc);
					}
					// Choose random polygon weighted by area, using reservoi sampling.
					areaSum += polyArea;
					u = frand.frand();
					if (u * areaSum <= polyArea)
					{
						randomTile = bestTile;
						randomPoly = bestPoly;
						randomPolyRef = bestRef;
					}
				}

				// Get parent poly and tile.
				long parentRef = 0;
				if (bestNode.pidx != 0)
				{
					parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
				}

				//û��ִ������
	/*			if (parentRef != 0) {
					Tupple2<MeshTile, Poly> parentTilePoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
					MeshTile parentTile = parentTilePoly.first;
					Poly parentPoly = parentTilePoly.second;
				}*/

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					link = bestTile.links[i];
					neighbourRef = link.@ref;
					// Skip invalid neighbours and do not follow back to parent.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

                    neighbourTile = null;
                    neighbourPoly = null;
					// Expand to neighbour
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					// Do not advance if the polygon is excluded by the filter.
					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// Find edge and calc distance to the edge.
					portalpoints = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, 0, 0);
                    DetourCommon.vCopy(frpacva, portalpoints.left);
                    DetourCommon.vCopy(frpacvb, portalpoints.right);

					// If the circle is not touching the next polygon, skip it.
					distt = 0;
                    float distSqr = DetourCommon.distancePtSegSqr2D(centerPos, frpacva, frpacvb, ref distt);
					//float distSqr = distseg.first;
					if (distSqr > radiusSqr)
					{
						continue;
					}

					neighbourNode = m_nodePool.getNode(neighbourRef);

					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					{
						continue;
					}

					// Cost
					if (neighbourNode.flags == 0)
					{
                        DetourCommon.vLerp(neighbourNode.Pos, frpacva, frpacvb, 0.5f);
					}

                    float total = bestNode.total + DetourCommon.vDist(bestNode.Pos, neighbourNode.Pos);

					// The node is already in open list and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
					neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
					neighbourNode.total = total;

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						m_openList.modify(neighbourNode);
					}
					else
					{
						neighbourNode.flags = Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}
				}
			}

			if (randomPoly == null)
			{
				return new FindRandomPointResult(Status.FAILURE, 0, null);
			}

			// Randomly pick point on polygon.
			//float[] verts = new float[3 * m_nav.getMaxVertsPerPoly()];
			//float[] areas = new float[m_nav.getMaxVertsPerPoly()];
			DetourCommon.vResetArray(frpacverts);
			DetourCommon.vResetArray(frpacareas);

			Array.Copy(randomTile.data.verts, randomPoly.verts[0] * 3, frpacverts, 0, 3);
			for (int j = 1; j < randomPoly.vertCount; ++j)
			{
				Array.Copy(randomTile.data.verts, randomPoly.verts[j] * 3, frpacverts, j * 3, 3);
			}

			float s = frand.frand();
			float t = frand.frand();

            float[] pt = DetourCommon.randomPointInConvexPoly(frpacverts, randomPoly.vertCount, frpacareas, s, t);

			pt[1] = getPolyHeight(randomPolyRef, pt, 0, pt[1]);

			return new FindRandomPointResult(Status.SUCCSESS, randomPolyRef, pt);
		}

		//////////////////////////////////////////////////////////////////////////////////////////
		/// @par
		///
		/// Uses the detail polygons to find the surface height. (Most accurate.)
		///
		/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
		///
		/// See closestPointOnPolyBoundary() for a limited but faster option.
		///
		/// Finds the closest point on the specified polygon.
		///  @param[in]		ref			The reference id of the polygon.
		///  @param[in]		pos			The position to check. [(x, y, z)]
		///  @param[out]	closest		
		///  @param[out]	posOverPoly	
		/// @returns The status flags for the query.
		public virtual Status closestPointOnPoly(long @ref, float[] pos, ClosesPointOnPolyResult result)
		{
            MeshTile tile = null;
            Poly poly = null;
            if (m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly) == Status.FAILURE)
			{
				return Status.FAILURE;
			}
			

			// Off-mesh connections don't have detail polygons.
			if (poly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				int v0 = poly.verts[0] * 3;
				int v1 = poly.verts[1] * 3;
				float d0 = DetourCommon.vDist(pos, tile.data.verts, v0);
                float d1 = DetourCommon.vDist(pos, tile.data.verts, v1);
				float u = d0 / (d0 + d1);
                DetourCommon.vLerp(result.Closest, tile.data.verts, v0, v1, u);
				result.posOverPoly = false;
				return Status.SUCCSESS;
			}

			// Clamp point to be inside the polygon.
			DetourCommon.vResetArray(cpopverts);
			DetourCommon.vResetArray(cpopedged);
			DetourCommon.vResetArray(cpopedget);

			int nv = poly.vertCount;
			for (int i = 0; i < nv; ++i)
			{
				Array.Copy(tile.data.verts, poly.verts[i] * 3, cpopverts, i * 3, 3);
			}

            DetourCommon.vCopy(result.closest, pos);
			if (!m_nav.distancePtPolyEdgesSqr(pos, cpopverts, nv, cpopedged, cpopedget))
			{
				// Point is outside the polygon, dtClamp to nearest edge.
				float dmin = float.MaxValue;
				int imin = -1;
				for (int i = 0; i < nv; ++i)
				{
					if (cpopedged[i] < dmin)
					{
						dmin = cpopedged[i];
						imin = i;
					}
				}
				int va = imin * 3;
				int vb = ((imin + 1) % nv) * 3;
                DetourCommon.vLerp(result.closest, cpopverts, va, vb, cpopedget[imin]);
				result.posOverPoly = false;
			}
			else
			{
				result.posOverPoly = true;
			}
			int ip = poly.index;
			if (tile.data.detailMeshes != null && tile.data.detailMeshes.Length > ip)
			{
				PolyDetail pd = tile.data.detailMeshes[ip];
				float[] v1, v2, v3;
				int v1i, v2i, v3i;
				int t;
				// Find height at the location.
				for (int j = 0; j < pd.triCount; ++j)
				{
					t = (pd.triBase + j) * 4;
					if (tile.data.detailTris[t + 0] < poly.vertCount)
					{
						v1 = tile.data.verts;
						v1i = poly.verts[tile.data.detailTris[t + 0]] * 3;
					}
					else
					{
						v1 = tile.data.detailVerts;
						v1i = (pd.vertBase + (tile.data.detailTris[t + 0] - poly.vertCount)) * 3;
					}

					if (tile.data.detailTris[t + 1] < poly.vertCount)
					{
						v2 = tile.data.verts;
						v2i = poly.verts[tile.data.detailTris[t + 1]] * 3;
					}
					else
					{
						v2 = tile.data.detailVerts;
						v2i = (pd.vertBase + (tile.data.detailTris[t + 1] - poly.vertCount)) * 3;
					}

					if (tile.data.detailTris[t + 2] < poly.vertCount)
					{
						v3 = tile.data.verts;
						v3i = poly.verts[tile.data.detailTris[t + 2]] * 3;
					}
					else
					{
						v3 = tile.data.detailVerts;
						v3i = (pd.vertBase + (tile.data.detailTris[t + 2] - poly.vertCount)) * 3;
					}
					float h = 0;
					if (m_nav.closestHeightPointTriangle(pos, 0, v1, v1i, v2, v2i, v3, v3i, ref h))
					{
						result.closest[1] = h;
						break;
					}
				}
			}
			return Status.SUCCSESS;
		}

		/// @par
		///
		/// Much faster than closestPointOnPoly().
		///
		/// If the provided position lies within the polygon's xz-bounds (above or below),
		/// then @p pos and @p closest will be equal.
		///
		/// The height of @p closest will be the polygon boundary. The height detail is not used.
		///
		/// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
		///
		/// Returns a point on the boundary closest to the source point if the source point is outside the 
		/// polygon's xz-bounds.
		///  @param[in]		ref			The reference id to the polygon.
		///  @param[in]		pos			The position to check. [(x, y, z)]
		///  @param[out]	closest		The closest point. [(x, y, z)]
		/// @returns The status flags for the query.
		public virtual void closestPointOnPolyBoundary(long @ref, float[] pos, float[] closest)
		{
            MeshTile tile = null;
            Poly poly = null;
            m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly);
			

			// Collect vertices.
			//float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3];
			//float[] edged = new float[m_nav.getMaxVertsPerPoly()];
			//float[] edget = new float[m_nav.getMaxVertsPerPoly()];
			DetourCommon.vResetArray(cpopbverts);
			DetourCommon.vResetArray(cpopbedged);
			DetourCommon.vResetArray(cpopbedget);

			int nv = poly.vertCount;
			for (int i = 0; i < nv; ++i)
			{
				Array.Copy(tile.data.verts, poly.verts[i] * 3, cpopbverts, i * 3, 3);
			}

			if (m_nav.distancePtPolyEdgesSqr(pos, cpopbverts, nv, cpopbedged, cpopbedget))
			{
				DetourCommon.vCopy(closest, pos);
			}
			else
			{
				// Point is outside the polygon, dtClamp to nearest edge.
				float dmin = float.MaxValue;
				int imin = -1;
				for (int i = 0; i < nv; ++i)
				{
					if (cpopbedged[i] < dmin)
					{
						dmin = cpopbedged[i];
						imin = i;
					}
				}
				int va = imin * 3;
				int vb = ((imin + 1) % nv) * 3;
                DetourCommon.vLerp(closest, cpopbverts, va, vb, cpopbedget[imin]);
			}
		}

		/// @par
		///
		/// Will return #DT_FAILURE if the provided position is outside the xz-bounds
		/// of the polygon.
		///
		/// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
		///  @param[in]		ref			The reference id of the polygon.
		///  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
		///  @param[out]	height		The height at the surface of the polygon.
		/// @returns The status flags for the query.
		public virtual float getPolyHeight(long @ref, float[] pos, int index, float height)
		{
            MeshTile tile = null;
            Poly poly = null;
            m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly);
			
			if (poly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				int v0i = poly.verts[0] * 3;
				int v1i = poly.verts[1] * 3;
				float[] v0 = tile.data.verts;
				float[] v1 = tile.data.verts;
				//VectorPtr v0 = new VectorPtr(tile.data.verts, poly.verts[0] * 3);
				//VectorPtr v1 = new VectorPtr(tile.data.verts, poly.verts[1] * 3);
				float d0 = DetourCommon.vDist2D(pos, index, v0, v0i);
                float d1 = DetourCommon.vDist2D(pos, index, v1, v1i);
				float u = d0 / (d0 + d1);
				return v0[v0i + 1] + (v1[v1i + 1] - v0[v0i + 1]) * u;
				//return v0.get(1) + (v1.get(1) - v0.get(1)) * u;
			}
			else
			{
				int ip = poly.index;
				PolyDetail pd = tile.data.detailMeshes[ip];
				int t;
				float[] v1, v2, v3;
				int v1i, v2i, v3i;
				for (int j = 0; j < pd.triCount; ++j)
				{
					t = (pd.triBase + j) * 4;
					if (tile.data.detailTris[t + 0] < poly.vertCount)
					{
						v1 = tile.data.verts;
						v1i = poly.verts[tile.data.detailTris[t + 0]] * 3;
					}
					else
					{
						v1 = tile.data.detailVerts;
						v1i = (pd.vertBase + (tile.data.detailTris[t + 0] - poly.vertCount)) * 3;
					}

					if (tile.data.detailTris[t + 1] < poly.vertCount)
					{
						v2 = tile.data.verts;
						v2i = poly.verts[tile.data.detailTris[t + 1]] * 3;
					}
					else
					{
						v2 = tile.data.detailVerts;
						v2i = (pd.vertBase + (tile.data.detailTris[t + 1] - poly.vertCount)) * 3;
					}

					if (tile.data.detailTris[t + 2] < poly.vertCount)
					{
						v3 = tile.data.verts;
						v3i = poly.verts[tile.data.detailTris[t + 2]] * 3;
					}
					else
					{
						v3 = tile.data.detailVerts;
						v3i = (pd.vertBase + (tile.data.detailTris[t + 2] - poly.vertCount)) * 3;
					}
					float h = 0;
					if (m_nav.closestHeightPointTriangle(pos, index, v1, v1i, v2, v2i, v3, v3i, ref h))
					{
                        return h;
					}
				}
			}
			return height;
		}

		/// @par
		///
		/// @note If the search box does not intersect any polygons the search will
		/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
		/// @p nearestRef before using @p nearestPt.
		///
		/// @warning This function is not suitable for large area searches. If the search
		/// extents overlaps more than MAX_SEARCH (128) polygons it may return an invalid result.
		///
		/// @}
		/// @name Local Query Functions
		///@{

		/// Finds the polygon nearest to the specified center point.
		///  @param[in]		center		The center of the search box. [(x, y, z)]
		///  @param[in]		extents		The search distance along each axis. [(x, y, z)]
		///  @param[in]		filter		The polygon filter to apply to the query.
		/// @returns The status flags for the query.
		public virtual FindNearestPolyResult findNearestPoly(float[] center, float[] extents, QueryFilter filter)
		{

			FindNearestPolyResult result = new FindNearestPolyResult();

			// Get nearby polygons from proximity grid.
			//List<long> polys = queryPolygons(center, extents, filter);
//JAVA TO C# CONVERTER WARNING: The original Java variable was marked 'final':
//ORIGINAL LINE: final long[] polys = fnppolys;
			long[] polys = fnppolys;
			int polyCount = queryPolygons(center, extents, filter, polys, 128);

			DetourCommon.vSet(fnpdiff, 0, 0, 0);
			// Find nearest polygon amongst the nearby polygons.
			float nearestDistanceSqr = float.MaxValue;
			long @ref;
			bool posOverPoly;
			float[] closestPtPoly;
			float d;
			//for (int i = 0; i < polys.size(); ++i)
			for (int i = 0; i < polyCount; ++i)
			{
				//ref = polys.get(i);
				@ref = polys[i];
				fnpclosest.reset();
				closestPointOnPoly(@ref, center, fnpclosest);
				posOverPoly = fnpclosest.PosOverPoly;
				closestPtPoly = fnpclosest.Closest;

				// If a point is directly over a polygon and closer than
				// climb height, favor that instead of straight line nearest point.
				d = 0;
				//float[] diff = vSub(center, closestPtPoly);
                DetourCommon.vSub(fnpdiff, center, closestPtPoly);
				if (posOverPoly)
				{
                    MeshTile tile = null;
                    Poly poly = null;
                    m_nav.getTileAndPolyByRefUnsafe(polys[i], ref tile, ref poly);
					
					d = Math.Abs(fnpdiff[1]) - tile.data.header.walkableClimb;
					d = d > 0 ? d * d : 0;
				}
				else
				{
                    d = DetourCommon.vLenSqr(fnpdiff);
				}

				if (d < nearestDistanceSqr)
				{
					DetourCommon.vCopy(result.nearestPos, closestPtPoly);
					nearestDistanceSqr = d;
					result.nearestRef = @ref;
				}
			}

			return result;
		}



		/// <summary>
		/// �µ�queryPolygonsInTileʵ�� </summary>
		/// <param name="tile"> </param>
		/// <param name="qmin"> </param>
		/// <param name="qmax"> </param>
		/// <param name="filter"> </param>
		/// <param name="polys"> </param>
		/// <param name="npolys"> </param>
		/// <param name="maxPolys">
		/// @return </param>
//JAVA TO C# CONVERTER WARNING: 'final' parameters are not available in .NET:
//ORIGINAL LINE: protected int queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax, QueryFilter filter, long[] polys, int npolys, final int maxPolys)
		protected internal virtual int queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax, QueryFilter filter, long[] polys, int npolys, int maxPolys)
		{
			int n = 0;
			if (tile.data.bvTree != null)
			{
				int nodeIndex = 0;
				float[] tbmin = tile.data.header.bmin;
				float[] tbmax = tile.data.header.bmax;
				float qfac = tile.data.header.bvQuantFactor;
				// Calculate quantized box
				//int[] bmin = new int[3];
				//int[] bmax = new int[3];
				// dtClamp query box to world box.
                float minx = DetourCommon.clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
                float miny = DetourCommon.clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
                float minz = DetourCommon.clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
                float maxx = DetourCommon.clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
                float maxy = DetourCommon.clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
                float maxz = DetourCommon.clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
				// Quantize
				qpitbmin1[0] = (int)(qfac * minx) & 0xfffe;
				qpitbmin1[1] = (int)(qfac * miny) & 0xfffe;
				qpitbmin1[2] = (int)(qfac * minz) & 0xfffe;
				qpitbmax1[0] = (int)(qfac * maxx + 1) | 1;
				qpitbmax1[1] = (int)(qfac * maxy + 1) | 1;
				qpitbmax1[2] = (int)(qfac * maxz + 1) | 1;

				// Traverse tree
				long @base = m_nav.getPolyRefBase(tile);
				int end = tile.data.header.bvNodeCount;
				while (nodeIndex < end)
				{
					BVNode node = tile.data.bvTree[nodeIndex];
                    bool overlap = DetourCommon.overlapQuantBounds(qpitbmin1, qpitbmax1, node.bmin, node.bmax);
					bool isLeafNode = node.i >= 0;

					if (isLeafNode && overlap)
					{
						long @ref = @base | node.i;

						if (filter.passFilter(@ref, tile, tile.data.polys[node.i]))
						{
							polys[npolys + n] = @ref;
							n += 1;
						}
					}

					if (overlap || isLeafNode)
					{
						nodeIndex++;
					}
					else
					{
						int escapeIndex = -node.i;
						nodeIndex += escapeIndex;
					}
				}
				return n;
			}
			else
			{
				//float[] bmin = new float[3];
				//float[] bmax = new float[3];
				long @base = m_nav.getPolyRefBase(tile);
				for (int i = 0; i < tile.data.header.polyCount; ++i)
				{
					Poly p = tile.data.polys[i];
					// Do not return off-mesh connection polygons.
					if (p.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}
					long @ref = @base | i;
					if (!filter.passFilter(@ref, tile, p))
					{
						continue;
					}
					// Calc polygon bounds.
					int v = p.verts[0] * 3;
                    DetourCommon.vCopy(qpitbmin2, tile.data.verts, v);
                    DetourCommon.vCopy(qpitbmax2, tile.data.verts, v);
					for (int j = 1; j < p.vertCount; ++j)
					{
						v = p.verts[j] * 3;
                        DetourCommon.vMin(qpitbmin2, tile.data.verts, v);
                        DetourCommon.vMax(qpitbmax2, tile.data.verts, v);
					}
                    if (DetourCommon.overlapBounds(qmin, qmax, qpitbmin2, qpitbmax2))
					{
						polys[npolys + n] = @ref;
						n += 1;
					}
				}
				return n;
			}
		}

		/// <summary>
		/// �ϰ�queryPolygonsInTileʵ�� </summary>
		/// <param name="tile"> </param>
		/// <param name="qmin"> </param>
		/// <param name="qmax"> </param>
		/// <param name="filter">
		/// @return </param>
		protected internal virtual List<long> queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax, QueryFilter filter)
		{
			List<long> polys = new List<long>();

			if (tile.data.bvTree != null)
			{
				int nodeIndex = 0;
				float[] tbmin = tile.data.header.bmin;
				float[] tbmax = tile.data.header.bmax;
				float qfac = tile.data.header.bvQuantFactor;
				// Calculate quantized box
				//int[] bmin = new int[3];
				//int[] bmax = new int[3];
				// dtClamp query box to world box.
                float minx = DetourCommon.clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
                float miny = DetourCommon.clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
                float minz = DetourCommon.clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
                float maxx = DetourCommon.clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
                float maxy = DetourCommon.clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
                float maxz = DetourCommon.clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
				// Quantize
				qpitbmin1[0] = (int)(qfac * minx) & 0xfffe;
				qpitbmin1[1] = (int)(qfac * miny) & 0xfffe;
				qpitbmin1[2] = (int)(qfac * minz) & 0xfffe;
				qpitbmax1[0] = (int)(qfac * maxx + 1) | 1;
				qpitbmax1[1] = (int)(qfac * maxy + 1) | 1;
				qpitbmax1[2] = (int)(qfac * maxz + 1) | 1;

				// Traverse tree
				long @base = m_nav.getPolyRefBase(tile);
				int end = tile.data.header.bvNodeCount;
				while (nodeIndex < end)
				{
					BVNode node = tile.data.bvTree[nodeIndex];
                    bool overlap = DetourCommon.overlapQuantBounds(qpitbmin1, qpitbmax1, node.bmin, node.bmax);
					bool isLeafNode = node.i >= 0;

					if (isLeafNode && overlap)
					{
						long @ref = @base | node.i;

						if (filter.passFilter(@ref, tile, tile.data.polys[node.i]))
						{
							polys.Add(@ref);
						}
					}

					if (overlap || isLeafNode)
					{
						nodeIndex++;
					}
					else
					{
						int escapeIndex = -node.i;
						nodeIndex += escapeIndex;
					}
				}
				return polys;
			}
			else
			{
				//float[] bmin = new float[3];
				//float[] bmax = new float[3];
				long @base = m_nav.getPolyRefBase(tile);
				for (int i = 0; i < tile.data.header.polyCount; ++i)
				{
					Poly p = tile.data.polys[i];
					// Do not return off-mesh connection polygons.
					if (p.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}
					long @ref = @base | i;
					if (!filter.passFilter(@ref, tile, p))
					{
						continue;
					}
					// Calc polygon bounds.
					int v = p.verts[0] * 3;
                    DetourCommon.vCopy(qpitbmin2, tile.data.verts, v);
                    DetourCommon.vCopy(qpitbmax2, tile.data.verts, v);
					for (int j = 1; j < p.vertCount; ++j)
					{
						v = p.verts[j] * 3;
                        DetourCommon.vMin(qpitbmin2, tile.data.verts, v);
                        DetourCommon.vMax(qpitbmax2, tile.data.verts, v);
					}
                    if (DetourCommon.overlapBounds(qmin, qmax, qpitbmin2, qpitbmax2))
					{
						polys.Add(@ref);
					}
				}
				return polys;
			}
		}

		/// <summary>
		/// Finds polygons that overlap the search box.
		/// 
		/// If no polygons are found, the function will return with a polyCount of zero.
		/// </summary>
		/// <param name="center">
		///            The center of the search box. [(x, y, z)] </param>
		/// <param name="extents">
		///            The search distance along each axis. [(x, y, z)] </param>
		/// <param name="filter">
		///            The polygon filter to apply to the query. </param>
		/// <returns> The reference ids of the polygons that overlap the query box. </returns>
		public virtual List<long> queryPolygons(float[] center, float[] extents, QueryFilter filter)
		{
			//float[] bmin = vSub(center, extents);
			//float[] bmax = vAdd(center, extents);
            DetourCommon.vSub(qpbmin, center, extents);
            DetourCommon.vAdd(qpbmax, center, extents);
			// Find tiles the query touches.
			//int[] minxy = m_nav.calcTileLoc(bmin);
			m_nav.calcTileLoc(qpminxy, qpbmin);
			int minx = qpminxy[0];
			int miny = qpminxy[1];
			//int[] maxxy = m_nav.calcTileLoc(bmax);
			m_nav.calcTileLoc(qpmaxxy, qpbmax);
			int maxx = qpmaxxy[0];
			int maxy = qpmaxxy[1];
			List<long> polys = new List<long>();
			List<long> polysInTile;
			IList<MeshTile> neis;
			for (int y = miny; y <= maxy; ++y)
			{
				for (int x = minx; x <= maxx; ++x)
				{
					neis = m_nav.getTilesAt(x, y);
					for (int j = 0; j < neis.Count; ++j)
					{
						polysInTile = queryPolygonsInTile(neis[j], qpbmin, qpbmax, filter);
						polys.AddRange(polysInTile);
					}
				}
			}
			return polys;
		}



		/// <summary>
		/// �°�queryPolygonsʵ�� </summary>
		/// <param name="center"> </param>
		/// <param name="extents"> </param>
		/// <param name="filter"> </param>
		/// <param name="polys"> </param>
		/// <param name="polyCount"> </param>
		/// <param name="maxPolys">
		/// @return </param>
		/// <seealso cref= #queryPolygons(float[], float[], QueryFilter) </seealso>
//JAVA TO C# CONVERTER WARNING: 'final' parameters are not available in .NET:
//ORIGINAL LINE: public int queryPolygons(float[] center, float[] extents, QueryFilter filter, long[] polys, final int maxPolys)
		public virtual int queryPolygons(float[] center, float[] extents, QueryFilter filter, long[] polys, int maxPolys)
		{
			//float[] bmin = vSub(center, extents);
			//float[] bmax = vAdd(center, extents);
            DetourCommon.vSub(qpbmin, center, extents);
            DetourCommon.vAdd(qpbmax, center, extents);
			// Find tiles the query touches.
			//int[] minxy = m_nav.calcTileLoc(bmin);
			m_nav.calcTileLoc(qpminxy, qpbmin);
			int minx = qpminxy[0];
			int miny = qpminxy[1];
			//int[] maxxy = m_nav.calcTileLoc(bmax);
			m_nav.calcTileLoc(qpmaxxy, qpbmax);
			int maxx = qpmaxxy[0];
			int maxy = qpmaxxy[1];

//JAVA TO C# CONVERTER WARNING: The original Java variable was marked 'final':
//ORIGINAL LINE: final MeshTile[] neis = qpneis;
			MeshTile[] neis = qpneis;
			int n = 0;
			for (int y = miny; y <= maxy; ++y)
			{
				for (int x = minx; x <= maxx; ++x)
				{
					int neiss = m_nav.getTilesAt(x, y, neis, QP_MAX_NEIS);
					for (int j = 0; j < neiss; ++j)
					{
						n += queryPolygonsInTile(neis[j], qpbmin, qpbmax, filter, polys, n, maxPolys - n);
						if (n >= maxPolys)
						{
							return n;
						}
					}
				}
			}
			return n;
		}

		/// <summary>
		/// Finds a path from the start polygon to the end polygon.
		/// 
		/// If the end polygon cannot be reached through the navigation graph, the last polygon in the path will be the
		/// nearest the end polygon.
		/// 
		/// The start and end positions are used to calculate traversal costs. (The y-values impact the result.)
		/// </summary>
		/// <param name="startRef">
		///            The refrence id of the start polygon. </param>
		/// <param name="endRef">
		///            The reference id of the end polygon. </param>
		/// <param name="startPos">
		///            A position within the start polygon. [(x, y, z)] </param>
		/// <param name="endPos">
		///            A position within the end polygon. [(x, y, z)] </param>
		/// <param name="filter">
		///            The polygon filter to apply to the query. </param>
		/// <returns> Found path </returns>
		public virtual FindPathResult findPath(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter, bool debug)
		{
			if (startRef == 0 || endRef == 0)
			{
				throw new System.ArgumentException("Start or end ref = 0");
			}

			// Validate input
			if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef))
			{
				throw new System.ArgumentException("Invalid start or end ref");
			}

			List<long> path = new List<long>(20);
			if (startRef == endRef)
			{
				path.Add(startRef);
				return new FindPathResult(Status.SUCCSESS, path);
			}

			m_nodePool.clear();
			m_openList.clear();

			Node startNode = m_nodePool.getNode(startRef);
            DetourCommon.vCopy(startNode.Pos, startPos);
			startNode.pidx = 0;
			startNode.cost = 0;
            startNode.total = DetourCommon.vDist(startPos, endPos) * H_SCALE;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

			Node lastBestNode = startNode;
			float lastBestNodeCost = startNode.total;

			Status status = Status.SUCCSESS;

			Node bestNode, neighbourNode;
			long bestRef, neighbourRef;
			MeshTile bestTile, parentTile, neighbourTile;
			Poly bestPoly, parentPoly, neighbourPoly;
			int crossSide;
			float cost, heuristic, curCost, endCost, total;
            //if (debug)
            //{
            //    UnityEngine.Debug.Log("findPath while start");
            //}
            int count = 0;
			while (!m_openList.Empty)
			{
                if (count>=1000)
                {
                    return null;
                }
                count += 1;
				// Remove node from open list and put it in closed list.
				bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;
                //if (debug)
                //{
                //    UnityEngine.Debug.Log("findPath while run : " + (count++));
                //}
				// Reached the goal, stop searching.
				if (bestNode.id == endRef)
				{
					lastBestNode = bestNode;
					break;
				}

				// Get current poly and tile.
				// The API input has been cheked already, skip checking internal data.
				bestRef = bestNode.id;
                bestTile = null;
                bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, ref bestTile, ref bestPoly);
				

				// Get parent poly and tile.
				long parentRef = 0;
				parentTile = null;
				parentPoly = null;
				if (bestNode.pidx != 0)
				{
					parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
				}
				if (parentRef != 0)
				{
                    parentTile = null;
                    parentPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, ref parentTile, ref parentPoly);
				}

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					neighbourRef = bestTile.links[i].@ref;

					// Skip invalid ids and do not expand back to where we came from.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

					// Get neighbour poly and tile.
					// The API input has been cheked already, skip checking internal data.
                    neighbourTile = null;
                    neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// deal explicitly with crossing tile boundaries
					crossSide = 0;
					if (bestTile.links[i].side != 0xff)
					{
						crossSide = bestTile.links[i].side >> 1;
					}

					// get the node
					neighbourNode = m_nodePool.getNode(neighbourRef, crossSide);

					// If the node is visited the first time, calculate node position.
					if (neighbourNode.flags == 0)
					{
						DetourCommon.vCopy(neighbourNode.Pos, getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef,neighbourPoly, neighbourTile));
					}

					// Calculate cost and heuristic.
					cost = 0;
					heuristic = 0;
					curCost = 0;
					endCost = 0;
					// Special case for last node.
					if (neighbourRef == endRef)
					{
						// Cost
						curCost = filter.getCost(bestNode.Pos, neighbourNode.Pos, parentRef, parentTile, parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
						endCost = filter.getCost(neighbourNode.Pos, endPos, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0L, null, null);

						cost = bestNode.cost + curCost + endCost;
						heuristic = 0;
					}
					else
					{
						// Cost
						curCost = filter.getCost(bestNode.Pos, neighbourNode.Pos, parentRef, parentTile, parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
						cost = bestNode.cost + curCost;
                        heuristic = DetourCommon.vDist(neighbourNode.Pos, endPos) * H_SCALE;
					}

					total = cost + heuristic;

					// The node is already in open list and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}
					// The node is already visited and process, and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					// Add or update the node.
					neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
					neighbourNode.cost = cost;
					neighbourNode.total = total;

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						// Already in open, update node location.
						m_openList.modify(neighbourNode);
					}
					else
					{
						// Put the node in open list.
						neighbourNode.flags |= Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}

					// Update nearest node to target so far.
					if (heuristic < lastBestNodeCost)
					{
						lastBestNodeCost = heuristic;
						lastBestNode = neighbourNode;
					}
				}
			}
            //if (debug)
            //{
            //    UnityEngine.Debug.Log("findPath while start");
            //}
			if (lastBestNode.id != endRef)
			{
				status = Status.PARTIAL_RESULT;
			}

			// Reverse the path.
			Node prev = null;
			Node node = lastBestNode;
			Node next;
			do
			{
				next = m_nodePool.getNodeAtIdx(node.pidx);
				node.pidx = m_nodePool.getNodeIdx(prev);
				prev = node;
				node = next;
			} while (node != null);

			// Store path
			node = prev;
			do
			{
				path.Add(node.id);
				node = m_nodePool.getNodeAtIdx(node.pidx);
			} while (node != null);

			return new FindPathResult(status, path);
		}

		/// <summary>
		/// Intializes a sliced path query.
		/// 
		/// Common use case: -# Call initSlicedFindPath() to initialize the sliced path query. -# Call updateSlicedFindPath()
		/// until it returns complete. -# Call finalizeSlicedFindPath() to get the path.
		/// </summary>
		/// <param name="startRef">
		///            The reference id of the start polygon. </param>
		/// <param name="endRef">
		///            The reference id of the end polygon. </param>
		/// <param name="startPos">
		///            A position within the start polygon. [(x, y, z)] </param>
		/// <param name="endPos">
		///            A position within the end polygon. [(x, y, z)] </param>
		/// <param name="filter">
		///            The polygon filter to apply to the query. </param>
		/// <param name="options">
		///            query options (see: #FindPathOptions)
		/// @return </param>
		public virtual Status initSlicedFindPath(long startRef, long endRef, float[] startPos, float[] endPos, QueryFilter filter, int options)
		{
			// Init path state.
			m_query = new QueryData();
			m_query.status = Status.FAILURE;
			m_query.startRef = startRef;
			m_query.endRef = endRef;
            DetourCommon.vCopy(m_query.startPos, startPos);
            DetourCommon.vCopy(m_query.endPos, endPos);
			m_query.filter = filter;
			m_query.options = options;
			m_query.raycastLimitSqr = float.MaxValue;

			if (startRef == 0 || endRef == 0)
			{
				throw new System.ArgumentException("Start or end ref = 0");
			}

			// Validate input
			if (!m_nav.isValidPolyRef(startRef) || !m_nav.isValidPolyRef(endRef))
			{
				throw new System.ArgumentException("Invalid start or end ref");
			}

			// trade quality with performance?
			if ((options & DT_FINDPATH_ANY_ANGLE) != 0)
			{
				// limiting to several times the character radius yields nice results. It is not sensitive
				// so it is enough to compute it from the first tile.
				MeshTile tile = m_nav.getTileByRef(startRef);
				float agentRadius = tile.data.header.walkableRadius;
                m_query.raycastLimitSqr = DetourCommon.sqr(agentRadius * NavMesh.DT_RAY_CAST_LIMIT_PROPORTIONS);
			}

			if (startRef == endRef)
			{
				m_query.status = Status.SUCCSESS;
				return Status.SUCCSESS;
			}

			m_nodePool.clear();
			m_openList.clear();

			Node startNode = m_nodePool.getNode(startRef);
            DetourCommon.vCopy(startNode.Pos, startPos);
			startNode.pidx = 0;
			startNode.cost = 0;
            startNode.total = DetourCommon.vDist(startPos, endPos) * H_SCALE;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

			m_query.status = Status.IN_PROGRESS;
			m_query.lastBestNode = startNode;
			m_query.lastBestNodeCost = startNode.total;

			return m_query.status;
		}

		/// <summary>
		/// Updates an in-progress sliced path query.
		/// </summary>
		/// <param name="maxIter">
		///            The maximum number of iterations to perform. </param>
		/// <returns> The status flags for the query. </returns>
		public virtual UpdateSlicedPathResult updateSlicedFindPath(int maxIter)
		{
			if (!m_query.status.InProgress)
			{
				return new UpdateSlicedPathResult(m_query.status, 0);
			}

			// Make sure the request is still valid.
			if (!m_nav.isValidPolyRef(m_query.startRef) || !m_nav.isValidPolyRef(m_query.endRef))
			{
				m_query.status = Status.FAILURE;
				return new UpdateSlicedPathResult(m_query.status, 0);
			}

			int iter = 0;
			Node bestNode, parentNode, neighbourNode;
			long bestRef, neighbourRef;
			MeshTile bestTile, parentTile, neighbourTile;
			Poly bestPoly, parentPoly, neighbourPoly;
			bool invalidParent, tryLOS, foundShortCut;
			float cost, heuristic, curCost, endCost, total;
			while (iter < maxIter && !m_openList.Empty)
			{
				iter++;

				// Remove node from open list and put it in closed list.
				bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;

				// Reached the goal, stop searching.
				if (bestNode.id == m_query.endRef)
				{
					m_query.lastBestNode = bestNode;
					m_query.status = Status.SUCCSESS;
					return new UpdateSlicedPathResult(m_query.status, iter);
				}

				// Get current poly and tile.
				// The API input has been cheked already, skip checking internal
				// data.
				bestRef = bestNode.id;
                bestTile = null;
                bestPoly = null;
                if (m_nav.getTileAndPolyByRef(bestRef, ref bestTile, ref bestPoly) == Status.FAILURE)
				{
					m_query.status = Status.FAILURE;
					return new UpdateSlicedPathResult(m_query.status, iter);
				}

				
				// Get parent and grand parent poly and tile.
				long parentRef = 0, grandpaRef = 0;
				parentTile = null;
				parentPoly = null;
				parentNode = null;
				if (bestNode.pidx != 0)
				{
					parentNode = m_nodePool.getNodeAtIdx(bestNode.pidx);
					parentRef = parentNode.id;
					if (parentNode.pidx != 0)
					{
						grandpaRef = m_nodePool.getNodeAtIdx(parentNode.pidx).id;
					}
				}
				if (parentRef != 0)
				{
					invalidParent = false;

                    parentTile = null;
                    parentPoly = null;

                    if (m_nav.getTileAndPolyByRef(parentRef, ref parentTile, ref parentPoly) == Status.FAILURE)
					{
						invalidParent = true;
					}

					if (invalidParent || (grandpaRef != 0 && !m_nav.isValidPolyRef(grandpaRef)))
					{
						// The polygon has disappeared during the sliced query,
						// fail.
						m_query.status = Status.FAILURE;
						return new UpdateSlicedPathResult(m_query.status, iter);
					}
				}

				// decide whether to test raycast to previous nodes
				tryLOS = false;
				if ((m_query.options & DT_FINDPATH_ANY_ANGLE) != 0)
				{
					if ((parentRef != 0) && (DetourCommon.vDistSqr(parentNode.Pos, bestNode.Pos) < m_query.raycastLimitSqr))
					{
						tryLOS = true;
					}
				}

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					neighbourRef = bestTile.links[i].@ref;

					// Skip invalid ids and do not expand back to where we came
					// from.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

					// Get neighbour poly and tile.
					// The API input has been cheked already, skip checking internal
					// data.
                    neighbourTile = null;
                    neighbourPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					if (!m_query.filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// get the neighbor node
					neighbourNode = m_nodePool.getNode(neighbourRef, 0);

					// do not expand to nodes that were already visited from the
					// same parent
					if (neighbourNode.pidx != 0 && neighbourNode.pidx == bestNode.pidx)
					{
						continue;
					}

					// If the node is visited the first time, calculate node
					// position.
					if (neighbourNode.flags == 0)
					{
                        DetourCommon.vCopy(neighbourNode.Pos, getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile));
					}

					// Calculate cost and heuristic.
					cost = 0;
					heuristic = 0;

					// raycast parent
					foundShortCut = false;
					if (tryLOS)
					{
						RaycastHit rayHit = raycast(parentRef, parentNode.Pos, neighbourNode.Pos, m_query.filter, DT_RAYCAST_USE_COSTS, grandpaRef);
						foundShortCut = rayHit.t >= 1.0f;
						if (foundShortCut)
						{
							// shortcut found using raycast. Using shorter cost
							// instead
							cost = parentNode.cost + rayHit.pathCost;
						}
					}

					// update move cost
					if (!foundShortCut)
					{
						// No shortcut found.
						curCost = m_query.filter.getCost(bestNode.Pos, neighbourNode.Pos, parentRef, parentTile, parentPoly, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly);
						cost = bestNode.cost + curCost;
					}

					// Special case for last node.
					if (neighbourRef == m_query.endRef)
					{
						endCost = m_query.filter.getCost(neighbourNode.Pos, m_query.endPos, bestRef, bestTile, bestPoly, neighbourRef, neighbourTile, neighbourPoly, 0, null, null);

						cost = cost + endCost;
						heuristic = 0;
					}
					else
					{
                        heuristic = DetourCommon.vDist(neighbourNode.Pos, m_query.endPos) * H_SCALE;
					}

					total = cost + heuristic;

					// The node is already in open list and the new result is worse,
					// skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}
					// The node is already visited and process, and the new result
					// is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					// Add or update the node.
					neighbourNode.pidx = foundShortCut ? bestNode.pidx : m_nodePool.getNodeIdx(bestNode);
					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~(Node.DT_NODE_CLOSED | Node.DT_NODE_PARENT_DETACHED));
					neighbourNode.cost = cost;
					neighbourNode.total = total;
					if (foundShortCut)
					{
						neighbourNode.flags = (neighbourNode.flags | Node.DT_NODE_PARENT_DETACHED);
					}

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						// Already in open, update node location.
						m_openList.modify(neighbourNode);
					}
					else
					{
						// Put the node in open list.
						neighbourNode.flags |= Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}

					// Update nearest node to target so far.
					if (heuristic < m_query.lastBestNodeCost)
					{
						m_query.lastBestNodeCost = heuristic;
						m_query.lastBestNode = neighbourNode;
					}
				}
			}

			// Exhausted all nodes, but could not find path.
			if (m_openList.Empty)
			{
				m_query.status = Status.PARTIAL_RESULT;
			}

			return new UpdateSlicedPathResult(m_query.status, iter);
		}

		/// Finalizes and returns the results of a sliced path query.
		///  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.) 
		///  							[(polyRef) * @p pathCount]
		/// @returns The status flags for the query.
		public virtual FindPathResult finalizeSlicedFindPath()
		{
			List<long> path = new List<long>(20);
			if (m_query.status.Failed)
			{
				// Reset query.
				m_query = new QueryData();
				return new FindPathResult(Status.FAILURE, path);
			}

			if (m_query.startRef == m_query.endRef)
			{
				// Special case: the search starts and ends at same poly.
				path.Add(m_query.startRef);
			}
			else
			{
				// Reverse the path.
				if (m_query.lastBestNode.id != m_query.endRef)
				{
					m_query.status = Status.PARTIAL_RESULT;
				}

				Node prev = null;
				Node node = m_query.lastBestNode;
				int prevRay = 0;
				do
				{
					Node next = m_nodePool.getNodeAtIdx(node.pidx);
					node.pidx = m_nodePool.getNodeIdx(prev);
					prev = node;
					int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
					node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
					prevRay = nextRay;
					node = next;
				} while (node != null);

				// Store path
				node = prev;
				do
				{
					Node next = m_nodePool.getNodeAtIdx(node.pidx);
					if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0)
					{
						RaycastHit iresult = raycast(node.id, node.Pos, next.Pos, m_query.filter, 0, 0);
						path.AddRange(iresult.path);
						// raycast ends on poly boundary and the path might include the next poly boundary.
						if (path[path.Count - 1] == next.id)
						{
							path.RemoveAt(path.Count - 1); // remove to avoid duplicates
						}
					}
					else
					{
						path.Add(node.id);
					}

					node = next;
				} while (node != null);
			}

			Status status = m_query.status;
			// Reset query.
			m_query = new QueryData();

			return new FindPathResult(status, path);
		}

		/// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
		/// polygon on the existing path that was visited during the search.
		///  @param[in]		existing		An array of polygon references for the existing path.
		///  @param[in]		existingSize	The number of polygon in the @p existing array.
		///  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.) 
		///  								[(polyRef) * @p pathCount]
		/// @returns The status flags for the query.
		public virtual FindPathResult finalizeSlicedFindPathPartial(List<long> existing)
		{

			List<long> path = new List<long>(20);
			if (existing.Count == 0)
			{
				return new FindPathResult(Status.FAILURE, path);
			}
			if (m_query.status.Failed)
			{
				// Reset query.
				m_query = new QueryData();
				return new FindPathResult(Status.FAILURE, path);
			}
			if (m_query.startRef == m_query.endRef)
			{
				// Special case: the search starts and ends at same poly.
				path.Add(m_query.startRef);
			}
			else
			{
				// Find furthest existing node that was visited.
				Node prev = null;
				Node node = null;
				for (int i = existing.Count - 1; i >= 0; --i)
				{
					node = m_nodePool.findNode(existing[i]);
					if (node != null)
					{
						break;
					}
				}

				if (node == null)
				{
					m_query.status = Status.PARTIAL_RESULT;
					node = m_query.lastBestNode;
				}

				// Reverse the path.
				int prevRay = 0;
				do
				{
					Node next = m_nodePool.getNodeAtIdx(node.pidx);
					node.pidx = m_nodePool.getNodeIdx(prev);
					prev = node;
					int nextRay = node.flags & Node.DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
					node.flags = (node.flags & ~Node.DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
					prevRay = nextRay;
					node = next;
				} while (node != null);

				// Store path
				node = prev;
				do
				{
					Node next = m_nodePool.getNodeAtIdx(node.pidx);
					if ((node.flags & Node.DT_NODE_PARENT_DETACHED) != 0)
					{
						RaycastHit iresult = raycast(node.id, node.Pos, next.Pos, m_query.filter, 0, 0);
						path.AddRange(iresult.path);
						// raycast ends on poly boundary and the path might include the next poly boundary.
						if (path[path.Count - 1] == next.id)
						{
							path.RemoveAt(path.Count - 1); // remove to avoid duplicates
						}
					}
					else
					{
						path.Add(node.id);
					}

					node = next;
				} while (node != null);
			}
			Status status = m_query.status;
			// Reset query.
			m_query = new QueryData();

			return new FindPathResult(status, path);
		}

		protected internal virtual Status appendVertex(float[] pos, int flags, long @ref, IList<StraightPathItem> straightPath, int maxStraightPath)
		{
            if (straightPath.Count > 0 && DetourCommon.vEqual(straightPath[straightPath.Count - 1].pos, pos))
			{
				// The vertices are equal, update flags and poly.
				straightPath[straightPath.Count - 1].flags = flags;
				straightPath[straightPath.Count - 1].@ref = @ref;
			}
			else
			{
				if (straightPath.Count < maxStraightPath)
				{
					// Append new vertex.
					straightPath.Add(new StraightPathItem(pos, flags, @ref));
				}
				// If reached end of path or there is no space to append more vertices, return.
				if (flags == DT_STRAIGHTPATH_END || straightPath.Count >= maxStraightPath)
				{
					return Status.SUCCSESS;
				}
			}
			return Status.IN_PROGRESS;
		}

		protected internal virtual Status appendPortals(int startIdx, int endIdx, float[] endPos, List<long> path, IList<StraightPathItem> straightPath, int maxStraightPath, int options)
		{
			float[] startPos = straightPath[straightPath.Count - 1].pos;
			// Append or update last vertex
			//float[] pt = new float[3];
			DetourCommon.vSet(appt, 0, 0, 0);
			Status stat = null;
			MeshTile fromTile, toTile;
			Poly fromPoly, toPoly;
			long from, to;
			PortalResult portals;
            float s, t;
			for (int i = startIdx; i < endIdx; i++)
			{
				// Calculate portal
				from = path[i];
                fromTile = null;
                fromPoly = null;
                m_nav.getTileAndPolyByRef(from, ref fromTile, ref fromPoly);
				

				to = path[i + 1];
                toTile = null;
                toPoly = null;
                m_nav.getTileAndPolyByRef(to, ref toTile, ref toPoly);
				

				portals = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
                DetourCommon.vCopy(apleft, portals.left);
                DetourCommon.vCopy(apright, portals.right);

				if ((options & DT_STRAIGHTPATH_AREA_CROSSINGS) != 0)
				{
					// Skip intersection if only area crossings are requested.
					if (fromPoly.Area == toPoly.Area)
					{
						continue;
					}
				}

				// Append intersection
                s = 0;
                t = 0;
				if (intersectSegSeg2D(startPos, endPos, apleft, apright, ref s, ref t))
				{
					DetourCommon.vLerp(appt, apleft, apright, t);
					stat = appendVertex(appt, 0, path[i + 1], straightPath, maxStraightPath);
					if (!stat.InProgress)
					{
						return stat;
					}
				}
			}
			return Status.IN_PROGRESS;
		}

		/// @par
		/// Finds the straight path from the start to the end position within the polygon corridor.
		/// 
		/// This method peforms what is often called 'string pulling'.
		///
		/// The start position is clamped to the first polygon in the path, and the 
		/// end position is clamped to the last. So the start and end positions should 
		/// normally be within or very near the first and last polygons respectively.
		///
		/// The returned polygon references represent the reference id of the polygon 
		/// that is entered at the associated path position. The reference id associated 
		/// with the end point will always be zero.  This allows, for example, matching 
		/// off-mesh link points to their representative polygons.
		///
		/// If the provided result buffers are too small for the entire result set, 
		/// they will be filled as far as possible from the start toward the end 
		/// position.
		///
		///  @param[in]		startPos			Path start position. [(x, y, z)]
		///  @param[in]		endPos				Path end position. [(x, y, z)]
		///  @param[in]		path				An array of polygon references that represent the path corridor.
		///  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
		///  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
		///  @param[in]		options				Query options. (see: #dtStraightPathOptions)
		/// @returns The status flags for the query.
		public virtual IList<StraightPathItem> findStraightPath(float[] startPos, float[] endPos, List<long> path, int maxStraightPath, int options)
		{
			if (path.Count == 0)
			{
				throw new System.ArgumentException("Empty path");
			}
			// TODO: Should this be callers responsibility?
			float[] closestStartPos = new float[3];
			float[] closestEndPos = new float[3];
			closestPointOnPolyBoundary(path[0], startPos, closestStartPos);
			closestPointOnPolyBoundary(path[path.Count - 1], endPos, closestEndPos);
			IList<StraightPathItem> straightPath = new List<StraightPathItem>();
			// Add start point.
			Status stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[0], straightPath, maxStraightPath);
			if (!stat.InProgress)
			{
				return straightPath;
			}

			if (path.Count > 1)
			{
				//float[] portalApex = vCopy(closestStartPos);
				//float[] portalLeft = vCopy(portalApex);
				//float[] portalRight = vCopy(portalApex);
				DetourCommon.vCopy(fspportalApex, closestStartPos);
                DetourCommon.vCopy(fspportalLeft, fspportalApex);
                DetourCommon.vCopy(fspportalRight, fspportalApex);
				int apexIndex = 0;
				int leftIndex = 0;
				int rightIndex = 0;

				int leftPolyType = 0;
				int rightPolyType = 0;

				long leftPolyRef = path[0];
				long rightPolyRef = path[0];

				//float[] left;
				//float[] right;
				int fromType;
				int toType;
				float distt = 0;
				for (int i = 0; i < path.Count; ++i)
				{

					if (i + 1 < path.Count)
					{
						// Next portal.
						try
						{
							PortalResult portalPoints = getPortalPoints(path[i], path[i + 1]);
							//left = portalPoints.left;
							//right = portalPoints.right;
							DetourCommon.vCopy(fspleft, portalPoints.left);
                            DetourCommon.vCopy(fspright, portalPoints.right);
							fromType = portalPoints.fromType;
							toType = portalPoints.toType;
						}
						catch (Exception)
						{
							closestPointOnPolyBoundary(path[i], endPos, closestEndPos);
							// Append portals along the current straight path segment.
							if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
							{
								stat = appendPortals(apexIndex, i, closestEndPos, path, straightPath, options, maxStraightPath);
								if (!stat.InProgress)
								{
									return straightPath;
								}
							}
							appendVertex(closestEndPos, 0, path[i], straightPath, maxStraightPath);
							return straightPath;
						}

						// If starting really close the portal, advance.
						if (i == 0)
						{
                            distt = 0;
							float dt = DetourCommon.distancePtSegSqr2D(fspportalApex, fspleft, fspright, ref distt);
                            if (dt < DetourCommon.sqr(0.001f))
							{
								continue;
							}
						}
					}
					else
					{
						// End of the path.
						//left = vCopy(closestEndPos);
						//right = vCopy(closestEndPos);
                        DetourCommon.vCopy(fspleft, closestEndPos);
                        DetourCommon.vCopy(fspright, closestEndPos);
						fromType = toType = Poly.DT_POLYTYPE_GROUND;
					}

					// Right vertex.
                    if (DetourCommon.triArea2D(fspportalApex, fspportalRight, fspright) <= 0.0f)
					{
                        if (DetourCommon.vEqual(fspportalApex, fspportalRight) || DetourCommon.triArea2D(fspportalApex, fspportalLeft, fspright) > 0.0f)
						{
							//portalRight = vCopy(right);
                            DetourCommon.vCopy(fspportalRight, fspright);
							rightPolyRef = (i + 1 < path.Count) ? path[i + 1] : 0;
							rightPolyType = toType;
							rightIndex = i;
						}
						else
						{
							// Append portals along the current straight path segment.
							if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
							{
								stat = appendPortals(apexIndex, leftIndex, fspportalLeft, path, straightPath, options, maxStraightPath);
								if (!stat.InProgress)
								{
									return straightPath;
								}
							}

							//portalApex = vCopy(portalLeft);
                            DetourCommon.vCopy(fspportalApex, fspportalLeft);
							apexIndex = leftIndex;

							int flags = 0;
							if (leftPolyRef == 0)
							{
								flags = DT_STRAIGHTPATH_END;
							}
							else if (leftPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
							{
								flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
							}
							long @ref = leftPolyRef;

							// Append or update vertex
							stat = appendVertex(fspportalApex, flags, @ref, straightPath, maxStraightPath);
							if (!stat.InProgress)
							{
								return straightPath;
							}

							//portalLeft = vCopy(portalApex);
							//portalRight = vCopy(portalApex);
                            DetourCommon.vCopy(fspportalLeft, fspportalApex);
                            DetourCommon.vCopy(fspportalRight, fspportalApex);
							leftIndex = apexIndex;
							rightIndex = apexIndex;

							// Restart
							i = apexIndex;

							continue;
						}
					}

					// Left vertex.
                    if (DetourCommon.triArea2D(fspportalApex, fspportalLeft, fspleft) >= 0.0f)
					{
                        if (DetourCommon.vEqual(fspportalApex, fspportalLeft) || DetourCommon.triArea2D(fspportalApex, fspportalRight, fspleft) < 0.0f)
						{
							//portalLeft = vCopy(left);
                            DetourCommon.vCopy(fspportalLeft, fspleft);
							leftPolyRef = (i + 1 < path.Count) ? path[i + 1] : 0;
							leftPolyType = toType;
							leftIndex = i;
						}
						else
						{
							// Append portals along the current straight path segment.
							if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
							{
								stat = appendPortals(apexIndex, rightIndex, fspportalRight, path, straightPath, options, maxStraightPath);
								if (!stat.InProgress)
								{
									return straightPath;
								}
							}

							//portalApex = vCopy(portalRight);
                            DetourCommon.vCopy(fspportalApex, fspportalRight);
							apexIndex = rightIndex;

							int flags = 0;
							if (rightPolyRef == 0)
							{
								flags = DT_STRAIGHTPATH_END;
							}
							else if (rightPolyType == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
							{
								flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
							}
							long @ref = rightPolyRef;

							// Append or update vertex
							stat = appendVertex(fspportalApex, flags, @ref, straightPath, maxStraightPath);
							if (!stat.InProgress)
							{
								return straightPath;
							}

							//portalLeft = vCopy(portalApex);
							//portalRight = vCopy(portalApex);
                            DetourCommon.vCopy(fspportalLeft, fspportalApex);
                            DetourCommon.vCopy(fspportalRight, fspportalApex);
							leftIndex = apexIndex;
							rightIndex = apexIndex;

							// Restart
							i = apexIndex;

							continue;
						}
					}
				}

				// Append portals along the current straight path segment.
				if ((options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS)) != 0)
				{
					stat = appendPortals(apexIndex, path.Count - 1, closestEndPos, path, straightPath, options, maxStraightPath);
					if (!stat.InProgress)
					{
						return straightPath;
					}
				}
			}

			appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0, straightPath, maxStraightPath);

			return straightPath;
		}

		/// @par
		///
		/// This method is optimized for small delta movement and a small number of 
		/// polygons. If used for too great a distance, the result set will form an 
		/// incomplete path.
		///
		/// @p resultPos will equal the @p endPos if the end is reached. 
		/// Otherwise the closest reachable position will be returned.
		/// 
		/// @p resultPos is not projected onto the surface of the navigation 
		/// mesh. Use #getPolyHeight if this is needed.
		///
		/// This method treats the end position in the same manner as 
		/// the #raycast method. (As a 2D point.) See that method's documentation 
		/// for details.
		/// 
		/// If the @p visited array is too small to hold the entire result set, it will 
		/// be filled as far as possible from the start position toward the end 
		/// position.
		///
		/// Moves from the start to the end position constrained to the navigation mesh.
		///  @param[in]		startRef		The reference id of the start polygon.
		///  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
		///  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
		///  @param[in]		filter			The polygon filter to apply to the query.
		/// @returns Path
		public virtual MoveAlongSurfaceResult moveAlongSurface(long startRef, float[] startPos, float[] endPos, QueryFilter filter)
		{

			// Validate input
			if (startRef == 0)
			{
				throw new System.ArgumentException("Start ref = 0");
			}
			if (!m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}


			m_tinyNodePool.clear();

			Node startNode = m_tinyNodePool.getNode(startRef);
			startNode.pidx = 0;
			startNode.cost = 0;
			startNode.total = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_CLOSED;
			LinkedList<Node> stack = new LinkedList<Node>();
			stack.AddLast(startNode);

			float[] bestPos = new float[3];
			float bestDist = float.MaxValue;
			Node bestNode = null;
            DetourCommon.vCopy(bestPos, startPos);

			// Search constraints
			//float[] searchPos = vLerp(startPos, endPos, 0.5f);
            DetourCommon.vLerp(massearchPos, startPos, endPos, 0.5f);
            float searchRadSqr = DetourCommon.sqr(DetourCommon.vDist(startPos, endPos) / 2.0f + 0.001f);

			//float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3];
			DetourCommon.vResetArray(masverts);

			int nneis;
			Node curNode;
			MeshTile curTile;
			Poly curPoly;
			Link link;
			MeshTile neiTile;
			Poly neiPoly;
			long curRef;

			while (stack.Count > 0)
			{
				// Pop front.
                curNode = stack.First.Value;
                stack.RemoveFirst();

				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				curRef = curNode.id;
                curTile = null;
                curPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(curRef, ref curTile, ref curPoly);
				

				// Collect vertices.
				int nverts = curPoly.vertCount;
				for (int i = 0; i < nverts; ++i)
				{
					Array.Copy(curTile.data.verts, curPoly.verts[i] * 3, masverts, i * 3, 3);
				}

				// If target is inside the poly, stop search.
				if (DetourCommon.pointInPolygon(endPos, masverts, nverts))
				{
					bestNode = curNode;
					DetourCommon.vCopy(bestPos, endPos);
					break;
				}

				// Find wall edges and find nearest point inside the walls.
				for (int i = 0, j = curPoly.vertCount - 1; i < curPoly.vertCount; j = i++)
				{
					// Find links to neighbours.
					//int MAX_NEIS = 8;
					DetourCommon.vResetArray(masneis);
					nneis = 0;


					if ((curPoly.neis[j] & NavMesh.DT_EXT_LINK) != 0)
					{
						// Tile border.
						for (int k = curPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = curTile.links[k].next)
						{
							link = curTile.links[k];
							if (link.edge == j)
							{
								if (link.@ref != 0)
								{
                                    neiTile = null;
                                    neiPoly = null;
                                    m_nav.getTileAndPolyByRefUnsafe(link.@ref, ref neiTile, ref neiPoly);
									
									if (filter.passFilter(link.@ref, neiTile, neiPoly))
									{
										if (nneis < MAX_NEIS)
										{
											masneis[nneis++] = link.@ref;
										}
									}
								}
							}
						}
					}
					else if (curPoly.neis[j] != 0)
					{
						int idx = curPoly.neis[j] - 1;
						long @ref = m_nav.getPolyRefBase(curTile) | idx;
						if (filter.passFilter(@ref, curTile, curTile.data.polys[idx]))
						{
							// Internal edge, encode id.
							masneis[nneis++] = @ref;
						}
					}

					if (nneis == 0)
					{
						// Wall edge, calc distance.
						int vj = j * 3;
						int vi = i * 3;
                        float tseg = 0;
                        float distSqr = DetourCommon.distancePtSegSqr2D(endPos, masverts, vj, vi, ref tseg);
						if (distSqr < bestDist)
						{
							// Update nearest distance.
                            DetourCommon.vLerp(bestPos, masverts, vj, vi, tseg);
							bestDist = distSqr;
							bestNode = curNode;
						}
					}
					else
					{
						for (int k = 0; k < nneis; ++k)
						{
							// Skip if no node can be allocated.
							Node neighbourNode = m_tinyNodePool.getNode(masneis[k]);
							if (neighbourNode == null)
							{
								continue;
							}
							// Skip if already visited.
							if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
							{
								continue;
							}

							// Skip the link if it is too far from search constraint.
							// TODO: Maybe should use getPortalPoints(), but this one is way faster.
							int vj = j * 3;
							int vi = i * 3;
                            float t = 0;
                            float distSqr = DetourCommon.distancePtSegSqr2D(massearchPos, masverts, vj, vi, ref t);
							if (distSqr > searchRadSqr)
							{
								continue;
							}

							// Mark as the node as visited and push to queue.
							neighbourNode.pidx = m_tinyNodePool.getNodeIdx(curNode);
							neighbourNode.flags |= Node.DT_NODE_CLOSED;
							stack.AddLast(neighbourNode);
						}
					}
				}
			}

			List<long> visited = new List<long>();
			if (bestNode != null)
			{
				// Reverse the path.
				Node prev = null;
				Node node = bestNode;
				Node next;
				do
				{
					next = m_tinyNodePool.getNodeAtIdx(node.pidx);
					node.pidx = m_tinyNodePool.getNodeIdx(prev);
					prev = node;
					node = next;
				} while (node != null);

				// Store result
				node = prev;
				do
				{
					visited.Add(node.id);
					node = m_tinyNodePool.getNodeAtIdx(node.pidx);
				} while (node != null);
			}
			return new MoveAlongSurfaceResult(bestPos, visited);
		}

		public class PortalResult
		{
			internal readonly float[] left;
			internal readonly float[] right;
			internal readonly int fromType;
			internal readonly int toType;

			public PortalResult(float[] left, float[] right, int fromType, int toType)
			{
				this.left = left;
				this.right = right;
				this.fromType = fromType;
				this.toType = toType;
			}

		}

		public virtual PortalResult getPortalPoints(long from, long to)
		{
            MeshTile fromTile = null;
            Poly fromPoly = null;
            m_nav.getTileAndPolyByRef(from, ref fromTile, ref fromPoly);
			int fromType = fromPoly.Type;

            MeshTile toTile = null;
            Poly toPoly = null;
            m_nav.getTileAndPolyByRef(to, ref toTile, ref toPoly);		
			int toType = toPoly.Type;

			return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, fromType, toType);
		}

		private readonly float[] gppleft = new float[3];
		private readonly float[] gppright = new float[3];

		// Returns portal points between two polygons.
		public virtual PortalResult getPortalPoints(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly, MeshTile toTile, int fromType, int toType)
		{
			//float[] left = new float[3];
			//float[] right = new float[3];
			DetourCommon.vSet(gppleft, 0, 0, 0);
			DetourCommon.vSet(gppright, 0, 0, 0);
			// Find the link that points to the 'to' polygon.
			Link link = null;
			for (int i = fromPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = fromTile.links[i].next)
			{
				if (fromTile.links[i].@ref == to)
				{
					link = fromTile.links[i];
					break;
				}
			}
			if (link == null)
			{
				throw new System.ArgumentException("Null link");
			}

			// Handle off-mesh connections.
			if (fromPoly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				// Find link that points to first vertex.
				for (int i = fromPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = fromTile.links[i].next)
				{
					if (fromTile.links[i].@ref == to)
					{
						int v = fromTile.links[i].edge;
						Array.Copy(fromTile.data.verts, fromPoly.verts[v] * 3, gppleft, 0, 3);
						Array.Copy(fromTile.data.verts, fromPoly.verts[v] * 3, gppright, 0, 3);
						return new PortalResult(gppleft, gppright, fromType, toType);
					}
				}
				throw new System.ArgumentException("Invalid offmesh from connection");
			}

			if (toPoly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				for (int i = toPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = toTile.links[i].next)
				{
					if (toTile.links[i].@ref == from)
					{
						int v = toTile.links[i].edge;
						Array.Copy(toTile.data.verts, toPoly.verts[v] * 3, gppleft, 0, 3);
						Array.Copy(toTile.data.verts, toPoly.verts[v] * 3, gppright, 0, 3);
						return new PortalResult(gppleft, gppright, fromType, toType);
					}
				}
				throw new System.ArgumentException("Invalid offmesh to connection");
			}

			// Find portal vertices.
			int v0 = fromPoly.verts[link.edge];
			int v1 = fromPoly.verts[(link.edge + 1) % fromPoly.vertCount];
			Array.Copy(fromTile.data.verts, v0 * 3, gppleft, 0, 3);
			Array.Copy(fromTile.data.verts, v1 * 3, gppright, 0, 3);

			// If the link is at tile boundary, dtClamp the vertices to
			// the link width.
			if (link.side != 0xff)
			{
				// Unpack portal limits.
				if (link.bmin != 0 || link.bmax != 255)
				{
					float s = 1.0f / 255.0f;
					float tmin = link.bmin * s;
					float tmax = link.bmax * s;
					DetourCommon.vLerp(gppleft, fromTile.data.verts, v0 * 3, v1 * 3, tmin);
                    DetourCommon.vLerp(gppright, fromTile.data.verts, v0 * 3, v1 * 3, tmax);
				}
			}

			return new PortalResult(gppleft, gppright, fromType, toType);
		}

		// Returns edge mid point between two polygons.
		protected internal virtual float[] getEdgeMidPoint(long from, long to)
		{
			PortalResult ppoints = getPortalPoints(from, to);
			float[] left = ppoints.left;
			float[] right = ppoints.right;
			float[] mid = new float[3];
			mid[0] = (left[0] + right[0]) * 0.5f;
			mid[1] = (left[1] + right[1]) * 0.5f;
			mid[2] = (left[2] + right[2]) * 0.5f;
			return mid;
		}

		protected internal virtual float[] getEdgeMidPoint(long from, Poly fromPoly, MeshTile fromTile, long to, Poly toPoly, MeshTile toTile)
		{
			PortalResult ppoints = getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, 0, 0);
			float[] left = ppoints.left;
			float[] right = ppoints.right;
			float[] mid = new float[3];
			mid[0] = (left[0] + right[0]) * 0.5f;
			mid[1] = (left[1] + right[1]) * 0.5f;
			mid[2] = (left[2] + right[2]) * 0.5f;
			return mid;
		}

		private static float s = 1.0f / 255.0f;

		/// @par
		///
		/// This method is meant to be used for quick, short distance checks.
		///
		/// If the path array is too small to hold the result, it will be filled as 
		/// far as possible from the start postion toward the end position.
		///
		/// <b>Using the Hit Parameter t of RaycastHit</b>
		/// 
		/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
		/// the end position. In this case the path represents a valid corridor to the 
		/// end position and the value of @p hitNormal is undefined.
		///
		/// If the hit parameter is zero, then the start position is on the wall that 
		/// was hit and the value of @p hitNormal is undefined.
		///
		/// If 0 < t < 1.0 then the following applies:
		///
		/// @code
		/// distanceToHitBorder = distanceToEndPosition * t
		/// hitPoint = startPos + (endPos - startPos) * t
		/// @endcode
		///
		/// <b>Use Case Restriction</b>
		///
		/// The raycast ignores the y-value of the end position. (2D check.) This 
		/// places significant limits on how it can be used. For example:
		///
		/// Consider a scene where there is a main floor with a second floor balcony 
		/// that hangs over the main floor. So the first floor mesh extends below the 
		/// balcony mesh. The start position is somewhere on the first floor. The end 
		/// position is on the balcony.
		///
		/// The raycast will search toward the end position along the first floor mesh. 
		/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
		/// (no wall hit), meaning it reached the end position. This is one example of why
		/// this method is meant for short distance checks.
		///
		/// Casts a 'walkability' ray along the surface of the navigation mesh from 
		/// the start position toward the end position.
		/// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
		///  @param[in]		startRef	The reference id of the start polygon.
		///  @param[in]		startPos	A position within the start polygon representing 
		///  							the start of the ray. [(x, y, z)]
		///  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
		///  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
		///  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
		///  @param[in]		filter		The polygon filter to apply to the query.
		///  @param[out]	path		The reference ids of the visited polygons. [opt]
		///  @param[out]	pathCount	The number of visited polygons. [opt]
		///  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
		/// @returns The status flags for the query.
		public virtual RaycastHit raycast(long startRef, float[] startPos, float[] endPos, QueryFilter filter, int options, long prevRef)
		{
			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}
			if (prevRef != 0 && !m_nav.isValidPolyRef(prevRef))
			{
				throw new System.ArgumentException("Invalid pref ref");
			}

			RaycastHit hit = new RaycastHit();

			//float[] verts = new float[m_nav.getMaxVertsPerPoly() * 3 + 3];
			DetourCommon.vResetArray(rcverts);

			//float[] curPos = new float[3], lastPos = new float[3];
			DetourCommon.vResetArray(rccurPos);
			DetourCommon.vResetArray(rclastPos);

			//VectorPtr curPosV = new VectorPtr(rccurPos);
			int curPosV = 0;
            DetourCommon.vCopy(rccurPos, startPos);

			//float[] dir = vSub(endPos, startPos);
            DetourCommon.vSub(rcdir, endPos, startPos);

			MeshTile prevTile, tile, nextTile;
			Poly prevPoly, poly, nextPoly;

			// The API input has been checked already, skip checking internal data.
			long curRef = startRef;
            tile = null;
            poly = null;
            m_nav.getTileAndPolyByRefUnsafe(curRef, ref tile, ref poly);
			nextTile = prevTile = tile;
			nextPoly = prevPoly = poly;
			if (prevRef != 0)
			{
                prevTile = null;
                prevPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(prevRef, ref prevTile, ref prevPoly);
				
			}
			int nv;

			while (curRef != 0)
			{
				// Cast ray against current polygon.

				// Collect vertices.
				nv = 0;
				for (int i = 0; i < poly.vertCount; ++i)
				{
					Array.Copy(tile.data.verts, poly.verts[i] * 3, rcverts, nv * 3, 3);
					nv++;
				}

                rciresult.reset();
				intersectSegmentPoly2D(startPos, endPos, rcverts, nv, rciresult);
				if (!rciresult.intersects)
				{
					// Could not hit the polygon, keep the old t and report hit.
					return hit;
				}
				hit.hitEdgeIndex = rciresult.segMax;

				// Keep track of furthest t so far.
				if (rciresult.tmax > hit.t)
				{
					hit.t = rciresult.tmax;
				}

				// Store visited polygons.
				hit.path.Add(curRef);

				// Ray end is completely inside the polygon.
				if (rciresult.segMax == -1)
				{
					hit.t = float.MaxValue;

					// add the cost
					if ((options & DT_RAYCAST_USE_COSTS) != 0)
					{
						hit.pathCost += filter.getCost(rcverts, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly);
					}
					return hit;
				}

				// Follow neighbours.
				long nextRef = 0;

				for (int i = poly.firstLink; i != NavMesh.DT_NULL_LINK; i = tile.links[i].next)
				{
					Link link = tile.links[i];

					// Find link which contains this edge.
					if (link.edge != rciresult.segMax)
					{
						continue;
					}

					// Get pointer to the next polygon.
                    nextTile = null;
                    nextPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(link.@ref, ref nextTile, ref nextPoly);
					// Skip off-mesh connections.
					if (nextPoly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}

					// Skip links based on filter.
					if (!filter.passFilter(link.@ref, nextTile, nextPoly))
					{
						continue;
					}

					// If the link is internal, just return the ref.
					if (link.side == 0xff)
					{
						nextRef = link.@ref;
						break;
					}

					// If the link is at tile boundary,

					// Check if the link spans the whole edge, and accept.
					if (link.bmin == 0 && link.bmax == 255)
					{
						nextRef = link.@ref;
						break;
					}

					// Check for partial edge links.
					int v0 = poly.verts[link.edge];
					int v1 = poly.verts[(link.edge + 1) % poly.vertCount];
					int left = v0 * 3;
					int right = v1 * 3;

					// Check that the intersection lies inside the link portal.
					if (link.side == 0 || link.side == 4)
					{
						// Calculate link size.
						float lmin = tile.data.verts[left + 2] + (tile.data.verts[right + 2] - tile.data.verts[left + 2]) * (link.bmin * s);
						float lmax = tile.data.verts[left + 2] + (tile.data.verts[right + 2] - tile.data.verts[left + 2]) * (link.bmax * s);
						if (lmin > lmax)
						{
							float temp = lmin;
							lmin = lmax;
							lmax = temp;
						}

						// Find Z intersection.
						float z = startPos[2] + (endPos[2] - startPos[2]) * rciresult.tmax;
						if (z >= lmin && z <= lmax)
						{
							nextRef = link.@ref;
							break;
						}
					}
					else if (link.side == 2 || link.side == 6)
					{
						// Calculate link size.
						float lmin = tile.data.verts[left] + (tile.data.verts[right] - tile.data.verts[left]) * (link.bmin * s);
						float lmax = tile.data.verts[left] + (tile.data.verts[right] - tile.data.verts[left]) * (link.bmax * s);
						if (lmin > lmax)
						{
							float temp = lmin;
							lmin = lmax;
							lmax = temp;
						}

						// Find X intersection.
						float x = startPos[0] + (endPos[0] - startPos[0]) * rciresult.tmax;
						if (x >= lmin && x <= lmax)
						{
							nextRef = link.@ref;
							break;
						}
					}
				}

				// add the cost
				if ((options & DT_RAYCAST_USE_COSTS) != 0)
				{
					// compute the intersection point at the furthest end of the polygon
					// and correct the height (since the raycast moves in 2d)
					DetourCommon.vCopy(rclastPos, rccurPos);
					//curPos = vMad(startPos, dir, hit.t);
                    DetourCommon.vMad(rccurPos, startPos, rcdir, hit.t);
					int e1 = rciresult.segMax * 3;
					int e2 = ((rciresult.segMax + 1) % nv) * 3;
					//VectorPtr e1 = new VectorPtr(rcverts, rciresult.segMax * 3);
					//VectorPtr e2 = new VectorPtr(rcverts, ((rciresult.segMax + 1) % nv) * 3);
					//float[] eDir = vSub(e2, e1);
					//float[] diff = vSub(curPosV, e1);
                    DetourCommon.vSub(rceDir, rcverts, e2, rcverts, e1);
                    DetourCommon.vSub(rcdiff, rccurPos, curPosV, rcverts, e1);
                    float s = DetourCommon.sqr(rceDir[0]) > DetourCommon.sqr(rceDir[2]) ? rcdiff[0] / rceDir[0] : rcdiff[2] / rceDir[2];
					rccurPos[1] = rcverts[e1 + 1] + rceDir[1] * s;

					hit.pathCost += filter.getCost(rclastPos, rccurPos, prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly);
				}

				if (nextRef == 0)
				{
					// No neighbour, we hit a wall.

					// Calculate hit normal.
					int a = rciresult.segMax;
					int b = rciresult.segMax + 1 < nv ? rciresult.segMax + 1 : 0;
					int va = a * 3;
					int vb = b * 3;
					float dx = rcverts[vb] - rcverts[va];
					float dz = rcverts[vb + 2] - rcverts[va + 2];
					hit.hitNormal[0] = dz;
					hit.hitNormal[1] = 0;
					hit.hitNormal[2] = -dx;
                    DetourCommon.vNormalize(hit.hitNormal);
					return hit;
				}

				// No hit, advance to neighbour polygon.
				prevRef = curRef;
				curRef = nextRef;
				prevTile = tile;
				tile = nextTile;
				prevPoly = poly;
				poly = nextPoly;
			}

			return hit;
		}

		/// @par
		///
		/// At least one result array must be provided.
		///
		/// The order of the result set is from least to highest cost to reach the polygon.
		///
		/// A common use case for this method is to perform Dijkstra searches. 
		/// Candidate polygons are found by searching the graph beginning at the start polygon.
		///
		/// If a polygon is not found via the graph search, even if it intersects the 
		/// search circle, it will not be included in the result set. For example:
		///
		/// polyA is the start polygon.
		/// polyB shares an edge with polyA. (Is adjacent.)
		/// polyC shares an edge with polyB, but not with polyA
		/// Even if the search circle overlaps polyC, it will not be included in the 
		/// result set unless polyB is also in the set.
		/// 
		/// The value of the center point is used as the start position for cost 
		/// calculations. It is not projected onto the surface of the mesh, so its 
		/// y-value will effect the costs.
		///
		/// Intersection tests occur in 2D. All polygons and the search circle are 
		/// projected onto the xz-plane. So the y-value of the center point does not 
		/// effect intersection tests.
		///
		/// If the result arrays are to small to hold the entire result set, they will be 
		/// filled to capacity.
		/// 
		///@}
		/// @name Dijkstra Search Functions
		/// @{ 

		/// Finds the polygons along the navigation graph that touch the specified circle.
		///  @param[in]		startRef		The reference id of the polygon where the search starts.
		///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
		///  @param[in]		radius			The radius of the search circle.
		///  @param[in]		filter			The polygon filter to apply to the query.
		///  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
		///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
		///  								Zero if a result polygon has no parent. [opt]
		///  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
		///  @param[out]	resultCount		The number of polygons found. [opt]
		///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
		/// @returns The status flags for the query.
		public virtual FindPolysAroundResult findPolysAroundCircle(long startRef, float[] centerPos, float radius, QueryFilter filter)
		{

			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}

			List<long> resultRef = new List<long>();
			List<long> resultParent = new List<long>();
			List<float> resultCost = new List<float>();

			m_nodePool.clear();
			m_openList.clear();

			Node startNode = m_nodePool.getNode(startRef);
            DetourCommon.vCopy(startNode.Pos, centerPos);
			startNode.pidx = 0;
			startNode.cost = 0;
			startNode.total = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

			resultRef.Add(startNode.id);
			resultParent.Add(0L);
			resultCost.Add(0f);

			float radiusSqr = DetourCommon.sqr(radius);

			Node bestNode;
			long bestRef, parentRef, neighbourRef;
			MeshTile bestTile, parentTile, neighbourTile;
			Poly bestPoly, parentPoly, neighbourPoly;
			Link link;
			while (!m_openList.Empty)
			{
				bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;

				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				bestRef = bestNode.id;
                bestTile = null;
                bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, ref bestTile, ref bestPoly);
				

				// Get parent poly and tile.
				parentRef = 0;
				parentTile = null;
				parentPoly = null;
				if (bestNode.pidx != 0)
				{
					parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
				}
				if (parentRef != 0)
				{
                    parentTile = null;
                    parentPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, ref parentTile, ref parentPoly);
					
				}

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					link = bestTile.links[i];
					neighbourRef = link.@ref;
					// Skip invalid neighbours and do not follow back to parent.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

                    neighbourTile = null;
                    neighbourPoly = null;
					// Expand to neighbour
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					// Do not advance if the polygon is excluded by the filter.
					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// Find edge and calc distance to the edge.
					PortalResult pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, 0, 0);
					DetourCommon.vCopy(fpacva, pp.left);
                    DetourCommon.vCopy(fpacvb, pp.right);

					// If the circle is not touching the next polygon, skip it.
					float t = 0;
					float distSqr = DetourCommon.distancePtSegSqr2D(centerPos, fpacva, fpacvb, ref t);
					//float distSqr = distseg.first;
					if (distSqr > radiusSqr)
					{
						continue;
					}

					Node neighbourNode = m_nodePool.getNode(neighbourRef);

					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					{
						continue;
					}

					// Cost
					if (neighbourNode.flags == 0)
					{
                        DetourCommon.vLerp(neighbourNode.Pos, fpacva, fpacvb, 0.5f);
					}


                    float total = bestNode.total + DetourCommon.vDist(bestNode.Pos, neighbourNode.Pos);

					// The node is already in open list and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
					neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
					neighbourNode.total = total;

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						m_openList.modify(neighbourNode);
					}
					else
					{
						resultRef.Add(neighbourNode.id);
						resultParent.Add(m_nodePool.getNodeAtIdx(neighbourNode.pidx).id);
						resultCost.Add(neighbourNode.total);
						neighbourNode.flags = Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}
				}
			}

			return new FindPolysAroundResult(resultRef, resultParent, resultCost);
		}

		/// @par
		///
		/// The order of the result set is from least to highest cost.
		/// 
		/// At least one result array must be provided.
		///
		/// A common use case for this method is to perform Dijkstra searches. 
		/// Candidate polygons are found by searching the graph beginning at the start 
		/// polygon.
		/// 
		/// The same intersection test restrictions that apply to findPolysAroundCircle()
		/// method apply to this method.
		/// 
		/// The 3D centroid of the search polygon is used as the start position for cost 
		/// calculations.
		/// 
		/// Intersection tests occur in 2D. All polygons are projected onto the 
		/// xz-plane. So the y-values of the vertices do not effect intersection tests.
		/// 
		/// If the result arrays are is too small to hold the entire result set, they will 
		/// be filled to capacity.
		///
		/// Finds the polygons along the naviation graph that touch the specified convex polygon.
		///  @param[in]		startRef		The reference id of the polygon where the search starts.
		///  @param[in]		verts			The vertices describing the convex polygon. (CCW) 
		///  								[(x, y, z) * @p nverts]
		///  @param[in]		nverts			The number of vertices in the polygon.
		///  @param[in]		filter			The polygon filter to apply to the query.
		///  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
		///  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a 
		///  								result polygon has no parent. [opt]
		///  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
		///  @param[out]	resultCount		The number of polygons found.
		///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
		/// @returns The status flags for the query.
		public virtual FindPolysAroundResult findPolysAroundShape(long startRef, float[] verts, int nverts, QueryFilter filter)
		{
			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}

			List<long> resultRef = new List<long>();
			List<long> resultParent = new List<long>();
			List<float> resultCost = new List<float>();

			m_nodePool.clear();
			m_openList.clear();

			float[] centerPos = new float[] {0, 0, 0};
			for (int i = 0; i < nverts; ++i)
			{
				centerPos[0] += verts[i * 3];
				centerPos[1] += verts[i * 3 + 1];
				centerPos[2] += verts[i * 3 + 2];
			}
			float scale = 1.0f / nverts;
			centerPos[0] *= scale;
			centerPos[1] *= scale;
			centerPos[2] *= scale;

			Node startNode = m_nodePool.getNode(startRef);
            DetourCommon.vCopy(startNode.Pos, centerPos);
			startNode.pidx = 0;
			startNode.cost = 0;
			startNode.total = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

			resultRef.Add(startNode.id);
			resultParent.Add(0L);
			resultCost.Add(0f);
			DetourCommon.IntersectResult ir = new DetourCommon.IntersectResult();
			while (!m_openList.Empty)
			{
				Node bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;

				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				long bestRef = bestNode.id;
                MeshTile bestTile = null;
                Poly bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, ref bestTile, ref bestPoly);
				

				// Get parent poly and tile.
				long parentRef = 0;
				MeshTile parentTile = null;
				Poly parentPoly = null;
				if (bestNode.pidx != 0)
				{
					parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
				}
				if (parentRef != 0)
				{
                    parentTile = null;
                    parentPoly = null;
                    m_nav.getTileAndPolyByRefUnsafe(parentRef, ref parentTile, ref parentPoly);
					
				}

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					Link link = bestTile.links[i];
					long neighbourRef = link.@ref;
					// Skip invalid neighbours and do not follow back to parent.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

                    MeshTile neighbourTile = null;
                    Poly neighbourPoly = null;
					// Expand to neighbour
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					// Do not advance if the polygon is excluded by the filter.
					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// Find edge and calc distance to the edge.
					PortalResult pp = getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, 0, 0);
					float[] va = pp.left;
					float[] vb = pp.right;

					// If the poly is not touching the edge to the next polygon, skip the connection it.
					intersectSegmentPoly2D(va, vb, verts, nverts, ir);
					if (!ir.intersects)
					{
						continue;
					}
					if (ir.tmin > 1.0f || ir.tmax < 0.0f)
					{
						continue;
					}

					Node neighbourNode = m_nodePool.getNode(neighbourRef);

					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					{
						continue;
					}

					// Cost
					if (neighbourNode.flags == 0)
					{
						DetourCommon.vLerp(neighbourNode.Pos, va, vb, 0.5f);
					}

                    float total = bestNode.total + DetourCommon.vDist(bestNode.Pos, neighbourNode.Pos);

					// The node is already in open list and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
					neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
					neighbourNode.total = total;

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						m_openList.modify(neighbourNode);
					}
					else
					{
						resultRef.Add(neighbourNode.id);
						resultParent.Add(m_nodePool.getNodeAtIdx(neighbourNode.pidx).id);
						resultCost.Add(neighbourNode.total);
						neighbourNode.flags = Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}

				}
			}

			return new FindPolysAroundResult(resultRef, resultParent, resultCost);
		}

		/// @par
		///
		/// This method is optimized for a small search radius and small number of result 
		/// polygons.
		///
		/// Candidate polygons are found by searching the navigation graph beginning at 
		/// the start polygon.
		///
		/// The same intersection test restrictions that apply to the findPolysAroundCircle 
		/// mehtod applies to this method.
		///
		/// The value of the center point is used as the start point for cost calculations. 
		/// It is not projected onto the surface of the mesh, so its y-value will effect 
		/// the costs.
		/// 
		/// Intersection tests occur in 2D. All polygons and the search circle are 
		/// projected onto the xz-plane. So the y-value of the center point does not 
		/// effect intersection tests.
		/// 
		/// If the result arrays are is too small to hold the entire result set, they will 
		/// be filled to capacity.
		/// 
		/// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
		///  @param[in]		startRef		The reference id of the polygon where the search starts.
		///  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
		///  @param[in]		radius			The radius of the query circle.
		///  @param[in]		filter			The polygon filter to apply to the query.
		///  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
		///  @param[out]	resultParent	The reference ids of the parent polygons for each result. 
		///  								Zero if a result polygon has no parent. [opt]
		///  @param[out]	resultCount		The number of polygons found.
		///  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
		/// @returns The status flags for the query.
		public virtual int findLocalNeighbourhood(long startRef, float[] centerPos, float radius, QueryFilter filter, long[] resultRef, long[] resultParent, ref int resultCount, int maxResult)
		{

			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}

			//List<long> resultRef = new List<long>();
			//List<long> resultParent = new List<long>();

			m_tinyNodePool.clear();

			int status = Status.SUCCSESS.Mask;

			Node startNode = m_tinyNodePool.getNode(startRef);
			startNode.pidx = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_CLOSED;
			LinkedList<Node> stack = new LinkedList<Node>();
			stack.AddLast(startNode);

			int n = 0;
			if (n < maxResult)
			{
				resultRef[n] = startNode.id;
				if (resultParent != null)
				{
					resultParent[n] = 0;
				}
				n += 1;
			}

            float radiusSqr = DetourCommon.sqr(radius);

			//float[] pa = new float[m_nav.getMaxVertsPerPoly() * 3];
			//float[] pb = new float[m_nav.getMaxVertsPerPoly() * 3];
			DetourCommon.vResetArray(flnpa);
			DetourCommon.vResetArray(flnpb);


			Node curNode, neighbourNode;
			MeshTile curTile, neighbourTile, pastTile;
			Poly curPoly, neighbourPoly, pastPoly;
			long curRef, neighbourRef, pastRef;
			Link link;
			bool overlap, connected;
			int npa, npb;

			while (stack.Count > 0)
			{
				// Pop front.
                curNode = stack.First.Value;
                stack.RemoveFirst();

				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				curRef = curNode.id;
                curTile = null;
                curPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(curRef, ref curTile, ref curPoly);
				

				for (int i = curPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = curTile.links[i].next)
				{
					link = curTile.links[i];
					neighbourRef = link.@ref;
					// Skip invalid neighbours.
					if (neighbourRef == 0)
					{
						continue;
					}

					// Skip if cannot alloca more nodes.
					neighbourNode = m_tinyNodePool.getNode(neighbourRef);
					if (neighbourNode == null)
					{
						continue;
					}
					// Skip visited.
					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					{
						continue;
					}

                    neighbourTile = null;
                    neighbourPoly = null;
					// Expand to neighbour
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					// Skip off-mesh connections.
					if (neighbourPoly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}

					// Do not advance if the polygon is excluded by the filter.
					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					// Find edge and calc distance to the edge.
					PortalResult pp = getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, 0, 0);
                    DetourCommon.vCopy(flnva, pp.left);
                    DetourCommon.vCopy(flnvb, pp.right);

					// If the circle is not touching the next polygon, skip it.
					float t = 0;
                    float distSqr = DetourCommon.distancePtSegSqr2D(centerPos, flnva, flnvb, ref t);
					//float distSqr = distseg.first;
					if (distSqr > radiusSqr)
					{
						continue;
					}

					// Mark node visited, this is done before the overlap test so that
					// we will not visit the poly again if the test fails.
					neighbourNode.flags |= Node.DT_NODE_CLOSED;
					neighbourNode.pidx = m_tinyNodePool.getNodeIdx(curNode);

					// Check that the polygon does not collide with existing polygons.

					// Collect vertices of the neighbour poly.
					npa = neighbourPoly.vertCount;
					for (int k = 0; k < npa; ++k)
					{
						Array.Copy(neighbourTile.data.verts, neighbourPoly.verts[k] * 3, flnpa, k * 3, 3);
					}

					overlap = false;
					for (int j = 0; j < n; ++j)
					{
						pastRef = resultRef[j];

						// Connected polys do not overlap.
						connected = false;
						for (int k = curPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = curTile.links[k].next)
						{
							if (curTile.links[k].@ref == pastRef)
							{
								connected = true;
								break;
							}
						}
						if (connected)
						{
							continue;
						}

                        pastTile = null;
                        pastPoly = null;
						// Potentially overlapping.
                        m_nav.getTileAndPolyByRefUnsafe(pastRef, ref pastTile, ref pastPoly);
						

						// Get vertices and test overlap
						npb = pastPoly.vertCount;
						for (int k = 0; k < npb; ++k)
						{
							Array.Copy(pastTile.data.verts, pastPoly.verts[k] * 3, flnpb, k * 3, 3);
						}

						if (overlapPolyPoly2D(flnpa, npa, flnpb, npb))
						{
							overlap = true;
							break;
						}
					}
					if (overlap)
					{
						continue;
					}

					if (n < maxResult)
					{
						resultRef[n] = neighbourRef;
						if (resultParent != null)
						{
							resultParent[n] = curRef;
						}
						n += 1;
					}
					else
					{
						status |= Status.BUFFER_TOO_SMALL.Mask;
					}
					stack.AddLast(neighbourNode);
				}
			}
			resultCount = n;
			return status;
		}




		public class SegInterval
		{
			internal long @ref;
			internal int tmin, tmax;

			public SegInterval(long @ref, int tmin, int tmax)
			{
				this.@ref = @ref;
				this.tmin = tmin;
				this.tmax = tmax;
			}

		}

		public virtual int insertInterval(SegInterval[] ints, int nints, int tmin, int tmax, long @ref)
		{
			if (nints + 1 > MAX_INTERVAL)
			{
				return nints;
			}

			// Find insertion point.
			int idx = 0;
			while (idx < nints)
			{
				if (tmax <= ints[idx].tmin)
				{
					break;
				}
				idx++;
			}
			// Store
			if (idx != 0)
			{
				Array.Copy(ints, idx, ints, idx + 1, ints.Length - idx - 1);
			}
			ints[idx].@ref = @ref;
			ints[idx].tmin = tmin;
			ints[idx].tmax = tmax;
			nints++;
			return nints;
		}

		public virtual void insertInterval(IList<SegInterval> ints, int tmin, int tmax, long @ref)
		{
			// Find insertion point.
			int idx = 0;
			while (idx < ints.Count)
			{
				if (tmax <= ints[idx].tmin)
				{
					break;
				}
				idx++;
			}
			// Store
			ints.Insert(idx, new SegInterval(@ref, tmin, tmax));
		}

		/// @par
		///
		/// If the @p segmentRefs parameter is provided, then all polygon segments will be returned. 
		/// Otherwise only the wall segments are returned.
		/// 
		/// A segment that is normally a portal will be included in the result set as a 
		/// wall if the @p filter results in the neighbor polygon becoomming impassable.
		/// 
		/// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the 
		/// maximum segments per polygon of the source navigation mesh.
		/// 
		/// Returns the segments for the specified polygon, optionally including portals.
		///  @param[in]		ref				The reference id of the polygon.
		///  @param[in]		filter			The polygon filter to apply to the query.
		///  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
		///  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon. 
		///  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
		///  @param[out]	segmentCount	The number of segments returned.
		///  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
		/// @returns The status flags for the query.
		public virtual Status getPolyWallSegments(long @ref, QueryFilter filter, float[] segmentVerts, long[] segmentRefs, ref int segmentCount, int maxSegments)
		{

            MeshTile tile = null;
            Poly poly = null;
            if (m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly) == Status.FAILURE)
			{
				return Status.FAILURE;
			}
			

			bool storePortals = segmentRefs != null;

			int nints = 0;
			segmentCount = 0;
			int n = 0;

			Link link;
			MeshTile neiTile;
			Poly neiPoly;
			long neiRef;
			int idx;

			for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++)
			{
				// Skip non-solid edges.
				nints = 0;
				if ((poly.neis[j] & NavMesh.DT_EXT_LINK) != 0)
				{
					// Tile border.
					for (int k = poly.firstLink; k != NavMesh.DT_NULL_LINK; k = tile.links[k].next)
					{
						link = tile.links[k];
						if (link.edge == j)
						{
							if (link.@ref != 0)
							{
                                neiTile = null;
								neiPoly = null;
								m_nav.getTileAndPolyByRefUnsafe(link.@ref, ref neiTile, ref neiPoly);							
								if (filter.passFilter(link.@ref, neiTile, neiPoly))
								{
									nints = insertInterval(gpwsInts, nints, link.bmin, link.bmax, link.@ref);
								}
							}
						}
					}
				}
				else
				{
					// Internal edge
					neiRef = 0;
					if (poly.neis[j] != 0)
					{
						idx = (poly.neis[j] - 1);
						neiRef = m_nav.getPolyRefBase(tile) | idx;
						if (!filter.passFilter(neiRef, tile, tile.data.polys[idx]))
						{
							neiRef = 0;
						}
					}
					// If the edge leads to another polygon and portals are not stored, skip.
					if (neiRef != 0 && !storePortals)
					{
						continue;
					}

					if (n < maxSegments)
					{
						int vj = poly.verts[j] * 3;
						int vi = poly.verts[i] * 3;
						Array.Copy(tile.data.verts, vj, segmentVerts, n * 6, 3);
						Array.Copy(tile.data.verts, vi, segmentVerts, n * 6 + 3, 3);
						if (segmentRefs != null)
						{
							segmentRefs[n] = neiRef;
						}
						n++;
					}
					else
					{

					}
					continue;
				}

				// Add sentinels
				nints = insertInterval(gpwsInts, nints, -1, 0, 0);
				nints = insertInterval(gpwsInts, nints, 255, 256, 0);

				// Store segments.
				int vj1 = poly.verts[j] * 3;
				int vi1 = poly.verts[i] * 3;
				for (int k = 1; k < nints; ++k)
				{
					// Portal segment.
					if (storePortals && gpwsInts[k].@ref != 0)
					{
						float tmin = gpwsInts[k].tmin / 255.0f;
						float tmax = gpwsInts[k].tmax / 255.0f;
						if (n < maxSegments)
						{
                            DetourCommon.vLerp(gpwstemp1, tile.data.verts, vj1, vi1, tmin);
							Array.Copy(gpwstemp1, 0, segmentVerts, n * 6, 3);
                            DetourCommon.vLerp(gpwstemp1, tile.data.verts, vj1, vi1, tmax);
							Array.Copy(gpwstemp1, 0, segmentVerts, n * 6 + 3, 3);
							if (segmentRefs != null)
							{
								segmentRefs[n] = gpwsInts[k].@ref;
							}
							n++;
						}
					}
					// Wall segment.
					int imin = gpwsInts[k - 1].tmax;
					int imax = gpwsInts[k].tmin;
					if (imin != imax)
					{
						float tmin = imin / 255.0f;
						float tmax = imax / 255.0f;
						if (n < maxSegments)
						{
                            DetourCommon.vLerp(gpwstemp1, tile.data.verts, vj1, vi1, tmin);
							Array.Copy(gpwstemp1, 0, segmentVerts, n * 6, 3);
                            DetourCommon.vLerp(gpwstemp1, tile.data.verts, vj1, vi1, tmax);
							Array.Copy(gpwstemp1, 0, segmentVerts, n * 6 + 3, 3);
							if (segmentRefs != null)
							{
								segmentRefs[n] = 0;
							}
							n++;
						}

					}
				}
			}

			segmentCount = n;

			return Status.SUCCSESS;
		}

		public virtual GetPolyWallSegmentsResult getPolyWallSegments(long @ref, bool storePortals, QueryFilter filter)
		{
            MeshTile tile = null;
			Poly poly = null;
            m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly);
			

			List<long> segmentRefs = new List<long>();
			IList<float[]> segmentVerts = new List<float[]>();
			IList<SegInterval> ints = new List<SegInterval>(16);

			Link link;
			MeshTile neiTile;
			Poly neiPoly;
			long neiRef;
			int idx;
			float[] pwstemp1 = new float[3];

			for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++)
			{
				// Skip non-solid edges.
				ints.Clear();
				if ((poly.neis[j] & NavMesh.DT_EXT_LINK) != 0)
				{
					// Tile border.
					for (int k = poly.firstLink; k != NavMesh.DT_NULL_LINK; k = tile.links[k].next)
					{
						link = tile.links[k];
						if (link.edge == j)
						{
							if (link.@ref != 0)
							{
                                neiTile = null;
                                neiPoly = null;
                                m_nav.getTileAndPolyByRefUnsafe(link.@ref, ref neiTile, ref neiPoly);
								
								if (filter.passFilter(link.@ref, neiTile, neiPoly))
								{
									insertInterval(ints, link.bmin, link.bmax, link.@ref);
								}
							}
						}
					}
				}
				else
				{
					// Internal edge
					neiRef = 0;
					if (poly.neis[j] != 0)
					{
						idx = (poly.neis[j] - 1);
						neiRef = m_nav.getPolyRefBase(tile) | idx;
						if (!filter.passFilter(neiRef, tile, tile.data.polys[idx]))
						{
							neiRef = 0;
						}
					}
					// If the edge leads to another polygon and portals are not stored, skip.
					if (neiRef != 0 && !storePortals)
					{
						continue;
					}

					int vj = poly.verts[j] * 3;
					int vi = poly.verts[i] * 3;
					float[] seg = new float[6];
					Array.Copy(tile.data.verts, vj, seg, 0, 3);
					Array.Copy(tile.data.verts, vi, seg, 3, 3);
					segmentVerts.Add(seg);
					segmentRefs.Add(neiRef);
					continue;
				}

				// Add sentinels
				insertInterval(ints, -1, 0, 0);
				insertInterval(ints, 255, 256, 0);

				// Store segments.
				int vj1 = poly.verts[j] * 3;
				int vi1 = poly.verts[i] * 3;
				for (int k = 1; k < ints.Count; ++k)
				{
					// Portal segment.
					if (storePortals && ints[k].@ref != 0)
					{
						float tmin = ints[k].tmin / 255.0f;
						float tmax = ints[k].tmax / 255.0f;
						float[] seg = new float[6];
						DetourCommon.vLerp(pwstemp1, tile.data.verts, vj1, vi1, tmin);
						Array.Copy(pwstemp1, 0, seg, 0, 3);
                        DetourCommon.vLerp(pwstemp1, tile.data.verts, vj1, vi1, tmax);
						Array.Copy(pwstemp1, 0, seg, 3, 3);
						segmentVerts.Add(seg);
						segmentRefs.Add(ints[k].@ref);
					}

					// Wall segment.
					int imin = ints[k - 1].tmax;
					int imax = ints[k].tmin;
					if (imin != imax)
					{
						float tmin = imin / 255.0f;
						float tmax = imax / 255.0f;
						float[] seg = new float[6];
                        DetourCommon.vLerp(pwstemp1, tile.data.verts, vj1, vi1, tmin);
						Array.Copy(pwstemp1, 0, seg, 0, 3);
                        DetourCommon.vLerp(pwstemp1, tile.data.verts, vj1, vi1, tmax);
						Array.Copy(pwstemp1, 0, seg, 3, 3);
						segmentVerts.Add(seg);
						segmentRefs.Add(0L);
					}
				}
			}

			return new GetPolyWallSegmentsResult(segmentVerts, segmentRefs);
		}


		/// @par
		///
		/// @p hitPos is not adjusted using the height detail data.
		///
		/// @p hitDist will equal the search radius if there is no wall within the 
		/// radius. In this case the values of @p hitPos and @p hitNormal are
		/// undefined.
		///
		/// The normal will become unpredicable if @p hitDist is a very small number.
		///
		/// Finds the distance from the specified position to the nearest polygon wall.
		///  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
		///  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
		///  @param[in]		maxRadius		The radius of the search circle.
		///  @param[in]		filter			The polygon filter to apply to the query.
		///  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
		///  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
		///  @param[out]	hitNormal		The normalized ray formed from the wall point to the 
		///  								source point. [(x, y, z)]
		/// @returns The status flags for the query. 
		public virtual FindDistanceToWallResult findDistanceToWall(long startRef, float[] centerPos, float maxRadius, QueryFilter filter)
		{

			// Validate input
			if (startRef == 0 || !m_nav.isValidPolyRef(startRef))
			{
				throw new System.ArgumentException("Invalid start ref");
			}

			m_nodePool.clear();
			m_openList.clear();

			Node startNode = m_nodePool.getNode(startRef);
			DetourCommon.vCopy(startNode.Pos, centerPos);
			startNode.pidx = 0;
			startNode.cost = 0;
			startNode.total = 0;
			startNode.id = startRef;
			startNode.flags = Node.DT_NODE_OPEN;
			m_openList.push(startNode);

            float radiusSqr = DetourCommon.sqr(maxRadius);
			float[] hitPos = new float[3];
			long bestRef;
			MeshTile bestTile, neiTile;
			Poly bestPoly, neiPoly;
			long parentRef, neighbourRef;
			bool solid;
			Link link;

			while (!m_openList.Empty)
			{
				Node bestNode = m_openList.pop();
				bestNode.flags &= ~Node.DT_NODE_OPEN;
				bestNode.flags |= Node.DT_NODE_CLOSED;

				// Get poly and tile.
				// The API input has been cheked already, skip checking internal data.
				bestRef = bestNode.id;
                bestTile = null;
                bestPoly = null;
                m_nav.getTileAndPolyByRefUnsafe(bestRef, ref bestTile, ref bestPoly);
				

				// Get parent poly and tile.
				parentRef = 0;
				//MeshTile parentTile = null;
				//Poly parentPoly = null;
				if (bestNode.pidx != 0)
				{
					parentRef = m_nodePool.getNodeAtIdx(bestNode.pidx).id;
				}
	/*			if (parentRef != 0) {
					tileAndPoly = m_nav.getTileAndPolyByRefUnsafe(parentRef);
					parentTile = tileAndPoly.first;
					parentPoly = tileAndPoly.second;
				}*/

				// Hit test walls.
				for (int i = 0, j = bestPoly.vertCount - 1; i < bestPoly.vertCount; j = i++)
				{
					// Skip non-solid edges.
					if ((bestPoly.neis[j] & NavMesh.DT_EXT_LINK) != 0)
					{
						// Tile border.
						solid = true;
						for (int k = bestPoly.firstLink; k != NavMesh.DT_NULL_LINK; k = bestTile.links[k].next)
						{
							link = bestTile.links[k];
							if (link.edge == j)
							{
								if (link.@ref != 0)
								{
                                    neiTile = null;
                                    neiPoly = null;
                                    m_nav.getTileAndPolyByRefUnsafe(link.@ref, ref neiTile, ref neiPoly);
									
									if (filter.passFilter(link.@ref, neiTile, neiPoly))
									{
										solid = false;
									}
								}
								break;
							}
						}
						if (!solid)
						{
							continue;
						}
					}
					else if (bestPoly.neis[j] != 0)
					{
						// Internal edge
						int idx = (bestPoly.neis[j] - 1);
						long @ref = m_nav.getPolyRefBase(bestTile) | idx;
						if (filter.passFilter(@ref, bestTile, bestTile.data.polys[idx]))
						{
							continue;
						}
					}

					// Calc distance to the edge.
					int vj = bestPoly.verts[j] * 3;
					int vi = bestPoly.verts[i] * 3;
                    float tseg = 0;
                    float distSqr = DetourCommon.distancePtSegSqr2D(centerPos, bestTile.data.verts, vj, vi, ref tseg);

					// Edge is too far, skip.
					if (distSqr > radiusSqr)
					{
						continue;
					}

					// Hit wall, update radius.
					radiusSqr = distSqr;
					// Calculate hit pos.
					hitPos[0] = bestTile.data.verts[vj] + (bestTile.data.verts[vi] - bestTile.data.verts[vj]) * tseg;
					hitPos[1] = bestTile.data.verts[vj + 1] + (bestTile.data.verts[vi + 1] - bestTile.data.verts[vj + 1]) * tseg;
					hitPos[2] = bestTile.data.verts[vj + 2] + (bestTile.data.verts[vi + 2] - bestTile.data.verts[vj + 2]) * tseg;
				}

				for (int i = bestPoly.firstLink; i != NavMesh.DT_NULL_LINK; i = bestTile.links[i].next)
				{
					link = bestTile.links[i];
					neighbourRef = link.@ref;
					// Skip invalid neighbours and do not follow back to parent.
					if (neighbourRef == 0 || neighbourRef == parentRef)
					{
						continue;
					}

                    MeshTile neighbourTile = null;
                    Poly neighbourPoly = null;
					// Expand to neighbour.
                    m_nav.getTileAndPolyByRefUnsafe(neighbourRef, ref neighbourTile, ref neighbourPoly);
					

					// Skip off-mesh connections.
					if (neighbourPoly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}

					// Calc distance to the edge.
					int va = bestPoly.verts[link.edge] * 3;
					int vb = bestPoly.verts[(link.edge + 1) % bestPoly.vertCount] * 3;
                    float t = 0;
                    float distSqr = DetourCommon.distancePtSegSqr2D(centerPos, bestTile.data.verts, va, vb, ref t);
					// If the circle is not touching the next polygon, skip it.
					if (distSqr > radiusSqr)
					{
						continue;
					}

					if (!filter.passFilter(neighbourRef, neighbourTile, neighbourPoly))
					{
						continue;
					}

					Node neighbourNode = m_nodePool.getNode(neighbourRef);

					if ((neighbourNode.flags & Node.DT_NODE_CLOSED) != 0)
					{
						continue;
					}

					// Cost
					if (neighbourNode.flags == 0)
					{
                        DetourCommon.vCopy(neighbourNode.Pos, getEdgeMidPoint(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile));
					}

                    float total = bestNode.total + DetourCommon.vDist(bestNode.Pos, neighbourNode.Pos);

					// The node is already in open list and the new result is worse, skip.
					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0 && total >= neighbourNode.total)
					{
						continue;
					}

					neighbourNode.id = neighbourRef;
					neighbourNode.flags = (neighbourNode.flags & ~Node.DT_NODE_CLOSED);
					neighbourNode.pidx = m_nodePool.getNodeIdx(bestNode);
					neighbourNode.total = total;

					if ((neighbourNode.flags & Node.DT_NODE_OPEN) != 0)
					{
						m_openList.modify(neighbourNode);
					}
					else
					{
						neighbourNode.flags |= Node.DT_NODE_OPEN;
						m_openList.push(neighbourNode);
					}
				}
			}

			// Calc hit normal.
			float[] hitNormal = new float[3];
            DetourCommon.vSub(hitNormal, centerPos, hitPos);
            DetourCommon.vNormalize(hitNormal);

			return new FindDistanceToWallResult((float) Math.Sqrt(radiusSqr), hitPos, hitNormal);
		}

		/// Returns true if the polygon reference is valid and passes the filter restrictions.
		///  @param[in]		ref			The polygon reference to check.
		///  @param[in]		filter		The filter to apply.
		public virtual bool isValidPolyRef(long @ref, QueryFilter filter)
		{
            MeshTile tile = null;
            Poly poly = null;

            if (m_nav.getTileAndPolyByRef(@ref, ref tile, ref poly) == Status.FAILURE)
			{
				return false;
			}
				// If cannot pass filter, assume flags has changed and boundary is invalid.
            if (filter.passFilter(@ref, tile, poly))
			{
				return true;
			}

			return false;
		}

		/// Gets the navigation mesh the query object is using.
		/// @return The navigation mesh the query object is using.
		public virtual NavMesh AttachedNavMesh
		{
			get
			{
				return m_nav;
			}
		}

		/*
		/// @par
		///
		/// The closed list is the list of polygons that were fully evaluated during 
		/// the last navigation graph search. (A* or Dijkstra)
		/// 
		/// Returns true if the polygon reference is in the closed list. 
		///  @param[in]		ref		The reference id of the polygon to check.
		/// @returns True if the polygon is in closed list.
		public boolean isInClosedList(long ref)
		{
			if (m_nodePool == null) return false;
			
			Node nodes[DT_MAX_STATES_PER_NODE];
			int n= m_nodePool->findNodes(ref, nodes, DT_MAX_STATES_PER_NODE);
		
			for (int i=0; i<n; i++)
			{
				if (nodes[i]->flags & DT_NODE_CLOSED)
					return true;
			}		
		
			return false;
		}
		
		
		*/

		private readonly float[] issu = new float[3];
		private readonly float[] issv = new float[3];
		private readonly float[] issw = new float[3];

		internal virtual bool intersectSegSeg2D(float[] ap, float[] aq, float[] bp, float[] bq, ref float s, ref float t)
		{
			DetourCommon.vSub(issu, aq, ap);
            DetourCommon.vSub(issv, bq, bp);
            DetourCommon.vSub(issw, ap, bp);
			float d = DetourCommon.vperpXZ(issu, issv);
			if (Math.Abs(d) < 1e-6f)
			{
				return false;
			}
			s = DetourCommon.vperpXZ(issv, issw) / d;
			t = DetourCommon.vperpXZ(issu, issw) / d;
			//result.first = s;
			//result.second = t;
			return true;
		}


		private readonly float[] ispdir = new float[3];
		private readonly float[] ispedge = new float[3];
		private readonly float[] ispdiff = new float[3];
		internal virtual void intersectSegmentPoly2D(float[] p0, float[] p1, float[] verts, int nverts, DetourCommon.IntersectResult result)
		{
			result.reset();

			float EPS = 0.00000001f;

            DetourCommon.vSub(ispdir, p1, p0);

			for (int i = 0, j = nverts - 1; i < nverts; j = i++)
			{
                DetourCommon.vSub(ispedge, verts, i * 3, verts, j * 3);
                DetourCommon.vSub(ispdiff, p0, 0, verts, j * 3);
				float n = DetourCommon.vPerp2D(ispedge, ispdiff);
				float d = DetourCommon.vPerp2D(ispdir, ispedge);
				if (Math.Abs(d) < EPS)
				{
					// S is nearly parallel to this edge
					if (n < 0)
					{
						return;
					}
					else
					{
						continue;
					}
				}
				float t = n / d;
				if (d < 0)
				{
					// segment S is entering across this edge
					if (t > result.tmin)
					{
						result.tmin = t;
						result.segMin = j;
						// S enters after leaving polygon
						if (result.tmin > result.tmax)
						{
							return;
						}
					}
				}
				else
				{
					// segment S is leaving across this edge
					if (t < result.tmax)
					{
						result.tmax = t;
						result.segMax = j;
						// S leaves before entering polygon
						if (result.tmax < result.tmin)
						{
							return;
						}
					}
				}
			}
			result.intersects = true;

		}


		internal static float eps = 1e-4f;
		private readonly float[] oppaminmax = new float[2];
		private readonly float[] oppbminmax = new float[2];
		private readonly float[] oppn = new float[3];
		internal virtual bool overlapPolyPoly2D(float[] polya, int npolya, float[] polyb, int npolyb)
		{

			int va, vb;
			for (int i = 0, j = npolya - 1; i < npolya; j = i++)
			{
				va = j * 3;
				vb = i * 3;

				oppn[0] = polya[vb + 2] - polya[va + 2];
				oppn[1] = 0;
				oppn[2] = -(polya[vb + 0] - polya[va + 0]);
				//float[] n = new float[] { polya[vb + 2] - polya[va + 2], 0, -(polya[vb + 0] - polya[va + 0]) };

				DetourCommon.projectPoly(oppaminmax, oppn, polya, npolya);
				DetourCommon.projectPoly(oppbminmax, oppn, polyb, npolyb);
				if (!DetourCommon.overlapRange(oppaminmax[0], oppaminmax[1], oppbminmax[0], oppbminmax[1], eps))
				{
					// Found separating axis
					return false;
				}
			}
			for (int i = 0, j = npolyb - 1; i < npolyb; j = i++)
			{
				va = j * 3;
				vb = i * 3;

				oppn[0] = polyb[vb + 2] - polyb[va + 2];
				oppn[1] = 0;
				oppn[2] = -(polyb[vb + 0] - polyb[va + 0]);
				//float[] n = new float[] { polyb[vb + 2] - polyb[va + 2], 0, -(polyb[vb + 0] - polyb[va + 0]) };

				DetourCommon.projectPoly(oppaminmax, oppn, polya, npolya);
				DetourCommon.projectPoly(oppbminmax, oppn, polyb, npolyb);
				if (!DetourCommon.overlapRange(oppaminmax[0], oppaminmax[1], oppbminmax[0], oppbminmax[1], eps))
				{
					// Found separating axis
					return false;
				}
			}
			return true;
		}

	}
}