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
namespace org.recast4j.detour.crowd
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.sqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vCopy;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDist2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDist2DSqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vMad;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSub;


	/// <summary>
	/// Represents a dynamic polygon corridor used to plan agent movement.
	/// 
	/// The corridor is loaded with a path, usually obtained from a
	/// #NavMeshQuery::findPath() query. The corridor is then used to plan local
	/// movement, with the corridor automatically updating as needed to deal with
	/// inaccurate agent locomotion.
	/// 
	/// Example of a common use case:
	/// 
	/// -# Construct the corridor object and call 
	/// -# Obtain a path from a #dtNavMeshQuery object. 
	/// -# Use #reset() to set the agent's current position. (At the beginning of the path.) -# Use
	/// #setCorridor() to load the path and target. -# Use #findCorners() to plan
	/// movement. (This handles dynamic path straightening.) -# Use #movePosition()
	/// to feed agent movement back into the corridor. (The corridor will
	/// automatically adjust as needed.) -# If the target is moving, use
	/// #moveTargetPosition() to update the end of the corridor. (The corridor will
	/// automatically adjust as needed.) -# Repeat the previous 3 steps to continue
	/// to move the agent.
	/// 
	/// The corridor position and target are always constrained to the navigation
	/// mesh.
	/// 
	/// One of the difficulties in maintaining a path is that floating point errors,
	/// locomotion inaccuracies, and/or local steering can result in the agent
	/// crossing the boundary of the path corridor, temporarily invalidating the
	/// path. This class uses local mesh queries to detect and update the corridor as
	/// needed to handle these types of issues.
	/// 
	/// The fact that local mesh queries are used to move the position and target
	/// locations results in two beahviors that need to be considered:
	/// 
	/// Every time a move function is used there is a chance that the path will
	/// become non-optimial. Basically, the further the target is moved from its
	/// original location, and the further the position is moved outside the original
	/// corridor, the more likely the path will become non-optimal. This issue can be
	/// addressed by periodically running the #optimizePathTopology() and
	/// #optimizePathVisibility() methods.
	/// 
	/// All local mesh queries have distance limitations. (Review the #dtNavMeshQuery
	/// methods for details.) So the most accurate use case is to move the position
	/// and target in small increments. If a large increment is used, then the
	/// corridor may not be able to accurately find the new location. Because of this
	/// limiation, if a position is moved in a large increment, then compare the
	/// desired and resulting polygon references. If the two do not match, then path
	/// replanning may be needed. E.g. If you move the target, check #getLastPoly()
	/// to see if it is the expected polygon.
	/// 
	/// </summary>
	public class PathCorridor
	{

		internal readonly float[] m_pos = new float[3];
		internal readonly float[] m_target = new float[3];
		internal List<long> m_path;


		private readonly float[] delta = new float[3];
		private readonly float[] goal = new float[3];
		private readonly float[] temp = new float[3];

		protected internal virtual List<long> mergeCorridorStartMoved(List<long> path, List<long> visited)
		{
			int furthestPath = -1;
			int furthestVisited = -1;

			// Find furthest common polygon.
			for (int i = path.Count - 1; i >= 0; --i)
			{
				bool found = false;
                for (int j = visited.Count - 1; j >= 0; --j)
				{
					if (path[i] == visited[j])
					{
						furthestPath = i;
						furthestVisited = j;
						found = true;
					}
				}
				if (found)
				{
					break;
				}
			}

			// If no intersection found just return current path.
			if (furthestPath == -1 || furthestVisited == -1)
			{
				return path;
			}

			// Concatenate paths.

			// Adjust beginning of the buffer to include the visited.
			List<long> result = new List<long>();
			// Store visited
            for (int i = visited.Count - 1; i > furthestVisited; --i)
			{
				result.Add(visited[i]);
			}
            result.AddRange(path.GetRange(furthestPath, path.Count - furthestPath));
			return result;
		}

		protected internal virtual List<long> mergeCorridorEndMoved(List<long> path, List<long> visited)
		{
			int furthestPath = -1;
			int furthestVisited = -1;

			bool found = false;
			// Find furthest common polygon.
			for (int i = 0; i < path.Count; ++i)
			{
				found = false;
                for (int j = visited.Count - 1; j >= 0; --j)
				{
					if (path[i] == visited[j])
					{
						furthestPath = i;
						furthestVisited = j;
						found = true;
					}
				}
				if (found)
				{
					break;
				}
			}

			// If no intersection found just return current path.
			if (furthestPath == -1 || furthestVisited == -1)
			{
				return path;
			}

			// Concatenate paths.
			List<long> result = path.GetRange(0, furthestPath);
            result.AddRange(visited.GetRange(furthestVisited, visited.Count - furthestVisited));
			return result;
		}

		protected internal virtual List<long> mergeCorridorStartShortcut(List<long> path, List<long> visited)
		{

			int furthestPath = -1;
			int furthestVisited = -1;

			bool found;
			// Find furthest common polygon.
			for (int i = path.Count - 1; i >= 0; --i)
			{
				found = false;
				for (int j = visited.Count - 1; j >= 0; --j)
				{
					if (path[i] == visited[j])
					{
						furthestPath = i;
						furthestVisited = j;
						found = true;
					}
				}
				if (found)
				{
					break;
				}
			}

			// If no intersection found just return current path.
			if (furthestPath == -1 || furthestVisited <= 0)
			{
				return path;
			}

			// Concatenate paths.

			// Adjust beginning of the buffer to include the visited.
			List<long> result = visited.GetRange(0, furthestVisited);
            result.AddRange(path.GetRange(furthestPath, path.Count - furthestPath));
			return result;
		}

		/// <summary>
		/// Allocates the corridor's path buffer.
		/// </summary>
		public PathCorridor()
		{
			m_path = new List<long>();
		}

		/// <summary>
		/// Resets the path corridor to the specified position. </summary>
		/// <param name="ref"> The polygon reference containing the position. </param>
		/// <param name="pos"> The new position in the corridor. [(x, y, z)] </param>
		public virtual void reset(long @ref, float[] pos)
		{
			m_path.Clear();
			m_path.Add(@ref);
			DetourCommon.vCopy(m_pos, pos);
            DetourCommon.vCopy(m_target, pos);
		}

		/// <summary>
		/// Finds the corners in the corridor from the position toward the target.
		/// (The straightened path.)
		/// 
		/// This is the function used to plan local movement within the corridor. One
		/// or more corners can be detected in order to plan movement. It performs
		/// essentially the same function as #dtNavMeshQuery::findStraightPath.
		/// 
		/// Due to internal optimizations, the maximum number of corners returned
		/// will be (@p maxCorners - 1) For example: If the buffers are sized to hold
		/// 10 corners, the function will never return more than 9 corners. So if 10
		/// corners are needed, the buffers should be sized for 11 corners.
		/// 
		/// If the target is within range, it will be the last corner and have a
		/// polygon reference id of zero. </summary>
		/// <param name="filter"> 
		/// 
		/// @param[in] navquery The query object used to build the corridor. </param>
		/// <returns> Corners </returns>
		public virtual IList<StraightPathItem> findCorners(int maxCorners, NavMeshQuery navquery, QueryFilter filter)
		{
//JAVA TO C# CONVERTER WARNING: The original Java variable was marked 'final':
//ORIGINAL LINE: final float MIN_TARGET_DIST = sqr(0.01f);
            float MIN_TARGET_DIST = DetourCommon.sqr(0.01f);

			IList<StraightPathItem> path = navquery.findStraightPath(m_pos, m_target, m_path, maxCorners, 0);
			// Prune points in the beginning of the path which are too close.
			StraightPathItem spi;
            for (int i = 0; i < path.Count; i++ )
            {
                spi = path[i];
                if ((spi.Flags & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0 || DetourCommon.vDist2DSqr(spi.Pos, m_pos) > MIN_TARGET_DIST)
                {
                    break;
                }
                path.RemoveAt(i);
                i -= 1;
            }
			return path;
		}


		/// <summary>
		/// Attempts to optimize the path if the specified point is visible from the
		/// current position.
		/// 
		/// Inaccurate locomotion or dynamic obstacle avoidance can force the agent
		/// position significantly outside the original corridor. Over time this can
		/// result in the formation of a non-optimal corridor. Non-optimal paths can
		/// also form near the corners of tiles.
		/// 
		/// This function uses an efficient local visibility search to try to
		/// optimize the corridor between the current position and @p next.
		/// 
		/// The corridor will change only if @p next is visible from the current
		/// position and moving directly toward the point is better than following
		/// the existing path.
		/// 
		/// The more inaccurate the agent movement, the more beneficial this function
		/// becomes. Simply adjust the frequency of the call to match the needs to
		/// the agent.
		/// 
		/// This function is not suitable for long distance searches.
		/// </summary>
		/// <param name="next">
		///            The point to search toward. [(x, y, z]) </param>
		/// <param name="pathOptimizationRange">
		///            The maximum range to search. [Limit: > 0] </param>
		/// <param name="navquery">
		///            The query object used to build the corridor. </param>
		/// <param name="filter">
		///            The filter to apply to the operation. </param>

		public virtual void optimizePathVisibility(float[] next, float pathOptimizationRange, NavMeshQuery navquery, QueryFilter filter)
		{
			// Clamp the ray to max distance.
            float dist = DetourCommon.vDist2D(m_pos, next);

			// If too close to the goal, do not try to optimize.
			if (dist < 0.01f)
			{
				return;
			}

			// Overshoot a little. This helps to optimize open fields in tiled
			// meshes.
			dist = Math.Min(dist + 0.01f, pathOptimizationRange);

			// Adjust ray length.
			//float[] delta = vSub(next, m_pos);
			//float[] goal = vMad(m_pos, delta, pathOptimizationRange / dist);
            DetourCommon.vSub(delta, next, m_pos);
            DetourCommon.vMad(goal, m_pos, delta, pathOptimizationRange / dist);

			RaycastHit rc = navquery.raycast(m_path[0], m_pos, goal, filter, 0, 0);
			if (rc.path.Count > 1 && rc.t > 0.99f)
			{
				m_path = mergeCorridorStartShortcut(m_path, rc.path);
			}
		}

		/// <summary>
		/// Attempts to optimize the path using a local area search. (Partial
		/// replanning.)
		/// 
		/// Inaccurate locomotion or dynamic obstacle avoidance can force the agent
		/// position significantly outside the original corridor. Over time this can
		/// result in the formation of a non-optimal corridor. This function will use
		/// a local area path search to try to re-optimize the corridor.
		/// 
		/// The more inaccurate the agent movement, the more beneficial this function
		/// becomes. Simply adjust the frequency of the call to match the needs to
		/// the agent.
		/// </summary>
		/// <param name="navquery"> The query object used to build the corridor. </param>
		/// <param name="filter"> The filter to apply to the operation.
		///  </param>
		internal virtual bool optimizePathTopology(NavMeshQuery navquery, QueryFilter filter)
		{
			if (m_path.Count < 3)
			{
				return false;
			}

			const int MAX_ITER = 32;

			navquery.initSlicedFindPath(m_path[0], m_path[m_path.Count - 1], m_pos, m_target, filter, 0);
			navquery.updateSlicedFindPath(MAX_ITER);
			FindPathResult fpr = navquery.finalizeSlicedFindPathPartial(m_path);

			if (fpr.Status.Success && fpr.Refs.Count > 0)
			{
				m_path = mergeCorridorStartShortcut(m_path, fpr.Refs);
				return true;
			}

			return false;
		}

		public virtual bool moveOverOffmeshConnection(long offMeshConRef, long[] refs, float[] start, float[] end, NavMeshQuery navquery)
		{
			// Advance the path up to and over the off-mesh connection.
			long prevRef = 0, polyRef = m_path[0];
			int npos = 0;
			while (npos < m_path.Count && polyRef != offMeshConRef)
			{
				prevRef = polyRef;
				polyRef = m_path[npos];
				npos++;
			}
			if (npos == m_path.Count)
			{
				// Could not find offMeshConRef
				return false;
			}

			// Prune path
            m_path = m_path.GetRange(npos, m_path.Count - npos);
			refs[0] = prevRef;
			refs[1] = polyRef;

			NavMesh nav = navquery.AttachedNavMesh;
            nav.getOffMeshConnectionPolyEndPoints(refs[0], refs[1], ref start, ref end);
            DetourCommon.vCopy(m_pos, end);
			return true;
		}

		/// <summary>
		/// Moves the position from the current location to the desired location,
		/// adjusting the corridor as needed to reflect the change.
		/// 
		/// Behavior:
		/// 
		/// - The movement is constrained to the surface of the navigation mesh. -
		/// The corridor is automatically adjusted (shorted or lengthened) in order
		/// to remain valid. - The new position will be located in the adjusted
		/// corridor's first polygon.
		/// 
		/// The expected use case is that the desired position will be 'near' the
		/// current corridor. What is considered 'near' depends on local polygon
		/// density, query search extents, etc.
		/// 
		/// The resulting position will differ from the desired position if the
		/// desired position is not on the navigation mesh, or it can't be reached
		/// using a local search.
		/// </summary>
		/// <param name="npos">
		///            The desired new position. [(x, y, z)] </param>
		/// <param name="navquery">
		///            The query object used to build the corridor. </param>
		/// <param name="filter">
		///            Thefilter to apply to the operation. </param>
		public virtual void movePosition(float[] npos, NavMeshQuery navquery, QueryFilter filter)
		{
			// Move along navmesh and update new position.
			MoveAlongSurfaceResult masResult = navquery.moveAlongSurface(m_path[0], m_pos, npos, filter);
			m_path = mergeCorridorStartMoved(m_path, masResult.Visited);
			// Adjust the position to stay on top of the navmesh.
            DetourCommon.vCopy(m_pos, masResult.ResultPos);
			m_pos[1] = navquery.getPolyHeight(m_path[0], masResult.ResultPos, 0, m_pos[1]);
		}

		/// <summary>
		/// Moves the target from the curent location to the desired location,
		/// adjusting the corridor as needed to reflect the change. Behavior: - The
		/// movement is constrained to the surface of the navigation mesh. - The
		/// corridor is automatically adjusted (shorted or lengthened) in order to
		/// remain valid. - The new target will be located in the adjusted corridor's
		/// last polygon.
		/// 
		/// The expected use case is that the desired target will be 'near' the
		/// current corridor. What is considered 'near' depends on local polygon
		/// density, query search extents, etc. The resulting target will differ from
		/// the desired target if the desired target is not on the navigation mesh,
		/// or it can't be reached using a local search.
		/// </summary>
		/// <param name="npos">
		///            The desired new target position. [(x, y, z)] </param>
		/// <param name="navquery">
		///            The query object used to build the corridor. </param>
		/// <param name="filter">
		///            The filter to apply to the operation. </param>
		public virtual void moveTargetPosition(float[] npos, NavMeshQuery navquery, QueryFilter filter)
		{
			// Move along navmesh and update new position.
			MoveAlongSurfaceResult masResult = navquery.moveAlongSurface(m_path[m_path.Count - 1], m_target, npos, filter);
			m_path = mergeCorridorEndMoved(m_path, masResult.Visited);
			// TODO: should we do that?
			// Adjust the position to stay on top of the navmesh.
			/*
			 * float h = m_target[1]; navquery->getPolyHeight(m_path[m_npath-1],
			 * result, &h); result[1] = h;
			 */
            DetourCommon.vCopy(m_target, masResult.ResultPos);
		}

		/// <summary>
		/// Loads a new path and target into the corridor. The current corridor
		/// position is expected to be within the first polygon in the path. The
		/// target is expected to be in the last polygon.
		/// 
		/// @warning The size of the path must not exceed the size of corridor's path
		///          buffer set during #init(). </summary>
		/// <param name="target">
		///            The target location within the last polygon of the path. [(x,
		///            y, z)] </param>
		/// <param name="path">
		///            The path corridor. </param>

		public virtual void setCorridor(float[] target, List<long> path)
		{
            DetourCommon.vCopy(m_target, target);
			m_path = new List<long>(path);
		}

		public virtual void fixPathStart(long safeRef, float[] safePos)
		{
            DetourCommon.vCopy(m_pos, safePos);
			if (m_path.Count < 3 && m_path.Count > 0)
			{
				long p = m_path[m_path.Count - 1];
				m_path.Clear();
				m_path.Add(safeRef);
				m_path.Add(0L);
				m_path.Add(p);
			}
			else
			{
				m_path.Clear();
				m_path.Add(safeRef);
				m_path.Add(0L);
			}

		}

		public virtual void trimInvalidPath(long safeRef, float[] safePos, NavMeshQuery navquery, QueryFilter filter)
		{
			// Keep valid path as far as possible.
			int n = 0;
			while (n < m_path.Count && navquery.isValidPolyRef(m_path[n], filter))
			{
				n++;
			}

			if (n == 0)
			{
				// The first polyref is bad, use current safe values.
                DetourCommon.vCopy(m_pos, safePos);
				m_path.Clear();
				m_path.Add(safeRef);
			}
			else if (n < m_path.Count)
			{
				m_path = m_path.GetRange(0, n);
				// The path is partially usable.
			}


			navquery.closestPointOnPolyBoundary(m_path[m_path.Count - 1], m_target, temp);
			// Clamp target pos to last poly
            DetourCommon.vCopy(m_target, temp);
		}

		/// <summary>
		/// Checks the current corridor path to see if its polygon references remain
		/// valid. The path can be invalidated if there are structural changes to the
		/// underlying navigation mesh, or the state of a polygon within the path
		/// changes resulting in it being filtered out. (E.g. An exclusion or
		/// inclusion flag changes.)
		/// </summary>
		/// <param name="maxLookAhead">
		///            The number of polygons from the beginning of the corridor to
		///            search. </param>
		/// <param name="navquery">
		///            The query object used to build the corridor. </param>
		/// <param name="filter">
		///            The filter to apply to the operation.
		/// @return </param>
		internal virtual bool isValid(int maxLookAhead, NavMeshQuery navquery, QueryFilter filter)
		{
			// Check that all polygons still pass query filter.
			int n = Math.Min(m_path.Count, maxLookAhead);
			for (int i = 0; i < n; ++i)
			{
				if (!navquery.isValidPolyRef(m_path[i], filter))
				{
					return false;
				}
			}

			return true;
		}

		/// <summary>
		/// Gets the current position within the corridor. (In the first polygon.) </summary>
		/// <returns> The current position within the corridor. </returns>
		public virtual float[] Pos
		{
			get
			{
				return m_pos;
			}
		}

		/// <summary>
		/// Gets the current target within the corridor. (In the last polygon.) </summary>
		/// <returns> The current target within the corridor. </returns>
		public virtual float[] Target
		{
			get
			{
				return m_target;
			}
		}

		/// <summary>
		/// The polygon reference id of the first polygon in the corridor, the polygon containing the position. </summary>
		/// <returns> The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.) </returns>
		public virtual long FirstPoly
		{
			get
			{
				return m_path.Count <= 0 ? 0 : m_path[0];
			}
		}

		/// <summary>
		/// The polygon reference id of the last polygon in the corridor, the polygon containing the target. </summary>
		/// <returns> The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.) </returns>
		public virtual long LastPoly
		{
			get
			{
                return m_path.Count <= 0 ? 0 : m_path[m_path.Count - 1];
			}
		}

		/// <summary>
		/// The corridor's path. 
		/// </summary>
		public virtual List<long> Path
		{
			get
			{
				return m_path;
			}
		}

		/// <summary>
		/// The number of polygons in the current corridor path. </summary>
		/// <returns> The number of polygons in the current corridor path. </returns>
		public virtual int PathCount
		{
			get
			{
				return m_path.Count;
			}
		}
	}
}