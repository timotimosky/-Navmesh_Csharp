﻿using System;
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
using CrowdAgentState = org.recast4j.detour.crowd.CrowdAgent.CrowdAgentState;
using CrowdNeighbour = org.recast4j.detour.crowd.CrowdAgent.CrowdNeighbour;
using MoveRequestState = org.recast4j.detour.crowd.CrowdAgent.MoveRequestState;
using CrowdAgentDebugInfo = org.recast4j.detour.crowd.debug.CrowdAgentDebugInfo;
using ObstacleAvoidanceDebugData = org.recast4j.detour.crowd.debug.ObstacleAvoidanceDebugData;

namespace org.recast4j.detour.crowd
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.clamp;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.sqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.triArea2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vAdd;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vCopy;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDist2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDist2DSqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vLen;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vLenSqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vLerp;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vMad;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vScale;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSet;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSub;





	/*
	
	struct dtCrowdAgentDebugInfo
	{
		int idx;
		float optStart[3], optEnd[3];
		dtObstacleAvoidanceDebugData* vod;
	};
	
	/// Provides local steering behaviors for a group of agents. 
	/// @ingroup crowd
	class dtCrowd
	{
		int m_maxAgents;
		dtCrowdAgent* m_agents;
		dtCrowdAgent** m_activeAgents;
		dtCrowdAgentAnimation* m_agentAnims;
		
		dtPathQueue m_pathq;
	
		dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
		dtObstacleAvoidanceQuery* m_obstacleQuery;
		
		dtPolyRef* m_pathResult;
		int m_maxPathResult;
		
		float m_ext[3];
	
		dtQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];
	
		float m_maxAgentRadius;
	
		int m_velocitySampleCount;
	
		dtNavMeshQuery* m_navquery;
	
		void updateTopologyOptimization(dtCrowdAgent** agents, int nagents, float dt);
		void updateMoveRequest(float dt);
		void checkPathValidity(dtCrowdAgent** agents, int nagents, float dt);
	
		inline int getAgentIndex(dtCrowdAgent* agent)  { return (int)(agent - m_agents); }
	
		bool requestMoveTargetReplan(int idx, dtPolyRef ref, float* pos);
	
		void purge();
		
	public:
		
		/// Gets the specified agent from the pool.
		///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		dtCrowdAgent* getAgent(int idx);
	
		/// Gets the specified agent from the pool.
		///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		dtCrowdAgent* getEditableAgent(int idx);
	
		/// The maximum number of agents that can be managed by the object.
		/// @return The maximum number of agents.
		int getAgentCount() const;
		
		/// Adds a new agent to the crowd.
		///  @param[in]		pos		The requested position of the agent. [(x, y, z)]
		///  @param[in]		params	The configutation of the agent.
		/// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
		int addAgent(float* pos, dtCrowdAgentParams* params);
	
		/// Updates the specified agent's configuration.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		params	The new agent configuration.
		void updateAgentParameters(int idx, dtCrowdAgentParams* params);
	
		/// Removes the agent from the crowd.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		void removeAgent(int idx);
		
		/// Submits a new move request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		ref		The position's polygon reference.
		///  @param[in]		pos		The position within the polygon. [(x, y, z)]
		/// @return True if the request was successfully submitted.
		bool requestMoveTarget(int idx, dtPolyRef ref, float* pos);
	
		/// Submits a new move request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///  @param[in]		vel		The movement velocity. [(x, y, z)]
		/// @return True if the request was successfully submitted.
		bool requestMoveVelocity(int idx, float* vel);
	
		/// Resets any request for the specified agent.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return True if the request was successfully reseted.
		bool resetMoveTarget(int idx);
	
		/// Gets the active agents int the agent pool.
		///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
		///  @param[in]		maxAgents	The size of the crowd agent array.
		/// @return The number of agents returned in @p agents.
		int getActiveAgents(dtCrowdAgent** agents, int maxAgents);
	
		/// Updates the steering and positions of all agents.
		///  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
		///  @param[out]	debug	A debug object to load with debug information. [Opt]
		void update(float dt, dtCrowdAgentDebugInfo* debug);
		
		/// Gets the filter used by the crowd.
		/// @return The filter used by the crowd.
		inline dtQueryFilter* getFilter(int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }
		
		/// Gets the filter used by the crowd.
		/// @return The filter used by the crowd.
		inline dtQueryFilter* getEditableFilter(int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }
	
		/// Gets the search extents [(x, y, z)] used by the crowd for query operations. 
		/// @return The search extents used by the crowd. [(x, y, z)]
		float* getQueryExtents() { return m_ext; }
		
		/// Gets the velocity sample count.
		/// @return The velocity sample count.
		inline int getVelocitySampleCount() { return m_velocitySampleCount; }
		
		/// Gets the crowd's proximity grid.
		/// @return The crowd's proximity grid.
		dtProximityGrid* getGrid() { return m_grid; }
	
		/// Gets the crowd's path request queue.
		/// @return The crowd's path request queue.
		dtPathQueue* getPathQueue() { return &m_pathq; }
	
		/// Gets the query object used by the crowd.
		dtNavMeshQuery* getNavMeshQuery() { return m_navquery; }
	};
	
	/// Allocates a crowd object using the Detour allocator.
	/// @return A crowd object that is ready for initialization, or null on failure.
	///  @ingroup crowd
	dtCrowd* dtAllocCrowd();
	
	/// Frees the specified crowd object using the Detour allocator.
	///  @param[in]		ptr		A crowd object allocated using #dtAllocCrowd
	///  @ingroup crowd
	void dtFreeCrowd(dtCrowd* ptr);
	
	*/
	/// <summary>
	/// Members in this module implement local steering and dynamic avoidance features.
	/// 
	/// The crowd is the big beast of the navigation features. It not only handles a lot of the path management for you, but
	/// also local steering and dynamic avoidance between members of the crowd. I.e. It can keep your agents from running
	/// into each other.
	/// 
	/// Main class: Crowd
	/// 
	/// The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy to use path planning features. But in
	/// the end they only give you points that your navigation client should be moving toward. When it comes to deciding
	/// things like agent velocity and steering to avoid other agents, that is up to you to implement. Unless, of course, you
	/// decide to use Crowd.
	/// 
	/// Basically, you add an agent to the crowd, providing various configuration settings such as maximum speed and
	/// acceleration. You also provide a local target to move toward. The crowd manager then provides, with every update, the
	/// new agent position and velocity for the frame. The movement will be constrained to the navigation mesh, and steering
	/// will be applied to ensure agents managed by the crowd do not collide with each other.
	/// 
	/// This is very powerful feature set. But it comes with limitations.
	/// 
	/// The biggest limitation is that you must give control of the agent's position completely over to the crowd manager.
	/// You can update things like maximum speed and acceleration. But in order for the crowd manager to do its thing, it
	/// can't allow you to constantly be giving it overrides to position and velocity. So you give up direct control of the
	/// agent's movement. It belongs to the crowd.
	/// 
	/// The second biggest limitation revolves around the fact that the crowd manager deals with local planning. So the
	/// agent's target should never be more than 256 polygons away from its current position. If it is, you risk your agent
	/// failing to reach its target. So you may still need to do long distance planning and provide the crowd manager with
	/// intermediate targets.
	/// 
	/// Other significant limitations:
	/// 
	/// - All agents using the crowd manager will use the same #dtQueryFilter. - Crowd management is relatively expensive.
	/// The maximum agents under crowd management at any one time is between 20 and 30. A good place to start is a maximum of
	/// 25 agents for 0.5ms per frame.
	/// 
	/// @note This is a summary list of members. Use the index or search feature to find minor members.
	/// 
	/// @struct dtCrowdAgentParams </summary>
	/// <seealso cref= CrowdAgent, Crowd::addAgent(), Crowd::updateAgentParameters()
	/// 
	/// @var dtCrowdAgentParams::obstacleAvoidanceType
	/// @par
	/// 
	/// 		#dtCrowd permits agents to use different avoidance configurations. This value is the index of the
	///      #dtObstacleAvoidanceParams within the crowd.
	/// </seealso>
	/// <seealso cref= dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams()
	/// 
	/// @var dtCrowdAgentParams::collisionQueryRange
	/// @par
	/// 
	/// 		Collision elements include other agents and navigation mesh boundaries.
	/// 
	///      This value is often based on the agent radius and/or maximum speed. E.g. radius * 8
	/// 
	/// @var dtCrowdAgentParams::pathOptimizationRange
	/// @par
	/// 
	/// 		Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.
	/// 
	///      This value is often based on the agent radius. E.g. radius * 30
	/// </seealso>
	/// <seealso cref= dtPathCorridor::optimizePathVisibility()
	/// 
	/// @var dtCrowdAgentParams::separationWeight
	/// @par
	/// 
	/// 		A higher value will result in agents trying to stay farther away from each other at the cost of more difficult
	///      steering in tight spaces.
	///  </seealso>
	/// <summary>
	/// This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
	/// of the crowd features.
	/// A common method for setting up the crowd is as follows:
	/// -# Allocate the crowd
	/// -# Set the avoidance configurations using #setObstacleAvoidanceParams().
	/// -# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().
	/// A common process for managing the crowd is as follows:
	/// -# Call #update() to allow the crowd to manage its agents.
	/// -# Retrieve agent information using #getActiveAgents().
	/// -# Make movement requests using #requestMoveTarget() when movement goal changes.
	/// -# Repeat every frame.
	/// Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
	/// agent position.  So it is not possible to update an active agent's position.  If agent position
	/// must be fed back into the crowd, the agent must be removed and re-added.
	/// Notes: 
	/// - Path related information is available for newly added agents only after an #update() has been
	///  performed.
	/// - Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
	///  #dtCrowdAgent::active to determine if the agent is actually in use or not.
	/// - This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.  
	///  So it is not meant to provide automatic pathfinding services over long distances. </summary>
	/// <seealso cref= dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent </seealso>
	public class Crowd
	{


		internal const int MAX_ITERS_PER_UPDATE = 100;

		internal const int MAX_PATHQUEUE_NODES = 4096;
		internal const int MAX_COMMON_NODES = 512;

		/// The maximum number of neighbors that a crowd agent can take into account
		/// for steering decisions.
		/// @ingroup crowd
		internal const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

		/// The maximum number of corners a crowd agent will look ahead in the path.
		/// This value is used for sizing the crowd agent corner buffers.
		/// Due to the behavior of the crowd manager, the actual number of useful
		/// corners will be one less than this number.
		/// @ingroup crowd
		internal const int DT_CROWDAGENT_MAX_CORNERS = 4;

		/// The maximum number of crowd avoidance configurations supported by the
		/// crowd manager.
		/// @ingroup crowd
		/// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
		///		 dtCrowdAgentParams::obstacleAvoidanceType
		internal const int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

		/// The maximum number of query filter types supported by the crowd manager.
		/// @ingroup crowd
		/// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
		///		dtCrowdAgentParams::queryFilterType
		internal const int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

		/// Provides neighbor data for agents managed by the crowd.
		/// @ingroup crowd
		/// @see dtCrowdAgent::neis, dtCrowd


		internal int m_maxAgents;
		internal CrowdAgent[] m_agents;
		internal IList<CrowdAgent> m_activeAgents;
		internal PathQueue m_pathq;

		internal ObstacleAvoidanceParams[] m_obstacleQueryParams = new ObstacleAvoidanceParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
		internal ObstacleAvoidanceQuery m_obstacleQuery;

		internal ProximityGrid m_grid;

		internal float[] m_ext = new float[3];

		internal QueryFilter[] m_filters = new QueryFilter[DT_CROWD_MAX_QUERY_FILTER_TYPE];

		internal float m_maxAgentRadius;

		internal int m_velocitySampleCount;

		internal NavMeshQuery m_navquery;


		/// <summary>
		///**����������Ҫ����ʱ����*** </summary>
		private float[] dvel = new float[3];
		private float[] disp = new float[3];
		private float[] diff = new float[3];
		private float[] temp = new float[3];
		private float[] reqPos = new float[3];
		private float[] agentPos = new float[3];

		private float[] gndiff = new float[3];
		/// <summary>
		///******************** </summary>


		internal virtual float tween(float t, float t0, float t1)
		{
			return DetourCommon.clamp((t - t0) / (t1 - t0), 0.0f, 1.0f);
		}


		internal const int MAX_NEIS = 32;
		private int[] ids = new int[MAX_NEIS];

		internal virtual int getNeighbours(float[] pos, float height, float range, CrowdAgent skip, CrowdNeighbour[] result, int maxResult, IList<CrowdAgent> agents, ProximityGrid grid)
		{
			int n = 0;

			for (int i = 0; i < ids.Length; i++)
			{
				ids[i] = 0;
			}

			int nids = grid.queryItems(pos[0] - range, pos[2] - range, pos[0] + range, pos[2] + range, ids, MAX_NEIS);

			CrowdAgent ag;
			float distSqr;
			for (int i = 0; i < nids; i++)
			{
				ag = agents[ids[i]];

				if (ag == skip)
				{
					continue;
				}
                DetourCommon.vSub(gndiff, pos, ag.npos);
				if (Math.Abs(gndiff[1]) >= (height + ag.@params.height) / 2.0f)
				{
					continue;
				}
				gndiff[1] = 0;

                distSqr = DetourCommon.vLenSqr(gndiff);
                if (distSqr > DetourCommon.sqr(range))
				{
					continue;
				}

				n = addNeighbour(ids[i], distSqr, result, n, maxResult);

				if (n >= CrowdAgent.DT_CROWDAGENT_MAX_NEIGHBOURS)
				{
					break;
				}
			}
			Array.Sort(result, 0, n, neighbourCom);
			return n;
		}


		internal virtual int addNeighbour(int idx, float dist, CrowdNeighbour[] neis, int nneis, int maxNeis)
		{
			CrowdNeighbour neighbour = neis[nneis];

			neighbour.idx = idx;

			neighbour.dist = dist;

			nneis += 1;

			return nneis;
		}

		private NeighbourComparator neighbourCom = new NeighbourComparator();

		private class NeighbourComparator : IComparer<CrowdNeighbour>
		{
			public virtual int Compare(CrowdNeighbour o1, CrowdNeighbour o2)
			{
				return o1.dist.CompareTo(o2.dist);
			}
		}



        public virtual void addToOptQueue(CrowdAgent newag, PriorityQueue<CrowdAgent> agents)
		{
			// Insert neighbour based on greatest time.
			agents.Add(newag);
		}

		// Insert neighbour based on greatest time.
        internal virtual void addToPathQueue(CrowdAgent newag, PriorityQueue<CrowdAgent> agents)
		{
			agents.Add(newag);
		}

		///
		/// Initializes the crowd.  
		/// May be called more than once to purge and re-initialize the crowd.
		///  @param[in]		maxAgents		The maximum number of agents the crowd can manage. [Limit: >= 1]
		///  @param[in]		maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
		///  @param[in]		nav				The navigation mesh to use for planning.
		/// @return True if the initialization succeeded.
		public Crowd(int maxAgents, float maxAgentRadius, NavMesh nav)
		{

			m_maxAgents = maxAgents;
			m_maxAgentRadius = maxAgentRadius;
			DetourCommon.vSet(m_ext, m_maxAgentRadius * 2.0f, m_maxAgentRadius * 1.5f, m_maxAgentRadius * 2.0f);

			m_grid = new ProximityGrid(m_maxAgents * 4, maxAgentRadius * 3);
			m_obstacleQuery = new ObstacleAvoidanceQuery(CrowdAgent.DT_CROWDAGENT_MAX_NEIGHBOURS, 8);

			for (int i = 0; i < DT_CROWD_MAX_QUERY_FILTER_TYPE; i++)
			{
				m_filters[i] = new QueryFilter();
			}
			// Init obstacle query params.
			for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
			{
				m_obstacleQueryParams[i] = new ObstacleAvoidanceParams();
			}

			// Allocate temp buffer for merging paths.
			m_pathq = new PathQueue(MAX_PATHQUEUE_NODES, nav);
			m_agents = new CrowdAgent[m_maxAgents];
            m_activeAgents = new List<CrowdAgent>();
			for (int i = 0; i < m_maxAgents; ++i)
			{
				m_agents[i] = new CrowdAgent(i);
				m_agents[i].active = false;
			}

			// The navquery is mostly used for local searches, no need for large
			// node pool.
			m_navquery = new NavMeshQuery(nav);
		}

		/// Sets the shared avoidance configuration for the specified index.
		/// @param[in] idx The index. [Limits: 0 <= value <
		/// #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
		/// @param[in] params The new configuration.
		public virtual void setObstacleAvoidanceParams(int idx, ObstacleAvoidanceParams @params)
		{
			if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
			{
				m_obstacleQueryParams[idx] = @params;
			}
		}

		/// Gets the shared avoidance configuration for the specified index.
		/// @param[in] idx The index of the configuration to retreive.
		/// [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
		/// @return The requested configuration.
		public virtual ObstacleAvoidanceParams getObstacleAvoidanceParams(int idx)
		{
			if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
			{
				return m_obstacleQueryParams[idx];
			}
			return null;
		}

		/// The maximum number of agents that can be managed by the object.
		/// @return The maximum number of agents.
		public virtual int AgentCount
		{
			get
			{
				return m_maxAgents;
			}
		}

		/// Gets the specified agent from the pool.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
		public virtual CrowdAgent getAgent(int idx)
		{
			return idx < 0 || idx >= m_agents.Length ? null : m_agents[idx];
		}

		/// 
		/// Gets the specified agent from the pool.
		///	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return The requested agent.
		/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
		public virtual CrowdAgent getEditableAgent(int idx)
		{
			return idx < 0 || idx >= m_agents.Length ? null : m_agents[idx];
		}

		/// Updates the specified agent's configuration.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @param[in] params The new agent configuration.
		public virtual void updateAgentParameters(int idx, CrowdAgentParams @params)
		{
			if (idx < 0 || idx >= m_maxAgents)
			{
				return;
			}
			m_agents[idx].@params = @params;
		}

		/// Adds a new agent to the crowd.
		/// @param[in] pos The requested position of the agent. [(x, y, z)]
		/// @param[in] params The configutation of the agent.
		/// @return The index of the agent in the agent pool. Or -1 if the agent
		/// could not be added.
		public virtual int addAgent(float[] pos, CrowdAgentParams @params)
		{
			// Find empty slot.
			int idx = -1;
			for (int i = 0; i < m_maxAgents; ++i)
			{
				if (!m_agents[i].active)
				{
					idx = i;
					break;
				}
			}
			if (idx == -1)
			{
				return -1;
			}

			CrowdAgent ag = m_agents[idx];

			updateAgentParameters(idx, @params);

			// Find nearest position on navmesh and place the agent there.
			FindNearestPolyResult nearest = m_navquery.findNearestPoly(pos, m_ext, m_filters[ag.@params.queryFilterType]);

			if (nearest.NearestRef == 0)
			{
				return -1;
			}

			ag.corridor.reset(nearest.NearestRef, nearest.NearestPos);
			ag.boundary.reset();
			ag.partial = false;

			ag.topologyOptTime = 0;
			ag.targetReplanTime = 0;

            DetourCommon.vSet(ag.dvel, 0, 0, 0);
            DetourCommon.vSet(ag.nvel, 0, 0, 0);
            DetourCommon.vSet(ag.vel, 0, 0, 0);
            DetourCommon.vCopy(ag.npos, nearest.NearestPos);

			ag.desiredSpeed = 0;

			if (nearest.NearestRef != 0)
			{
				ag.state = CrowdAgentState.DT_CROWDAGENT_STATE_WALKING;
			}
			else
			{
				ag.state = CrowdAgentState.DT_CROWDAGENT_STATE_INVALID;
			}

			ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_NONE;

			ag.active = true;

			return idx;
		}


		/// <summary>
		/// ���õ�ǰλ�� </summary>
		/// <param name="index"> </param>
		/// <param name="currentPosition"> </param>
		public virtual void resetCurrentPosition(int index, float[] currentPosition)
		{
			if (index < 0 || index >= m_maxAgents)
			{
				return;
			}

			CrowdAgent ag = m_agents[index];

			if (!ag.Active)
			{
				return;
			}

			// Find nearest position on navmesh and place the agent there.
			FindNearestPolyResult nearest = m_navquery.findNearestPoly(currentPosition, m_ext, m_filters[ag.@params.queryFilterType]);

			if (nearest.NearestRef == 0)
			{
				return;
			}
			ag.corridor.reset(nearest.NearestRef, nearest.NearestPos);
            DetourCommon.vCopy(ag.npos, nearest.NearestPos);
		}


		/// Removes the agent from the crowd.
		///  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
		///
		/// The agent is deactivated and will no longer be processed. Its
		/// #dtCrowdAgent object
		/// is not removed from the pool. It is marked as inactive so that it is
		/// available for reuse.
		/// Removes the agent from the crowd.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		public virtual void removeAgent(int idx)
		{
			if (idx >= 0 && idx < m_maxAgents)
			{
				m_agents[idx].active = false;
			}
		}

		internal virtual bool requestMoveTargetReplan(CrowdAgent ag, long @ref, float[] pos)
		{
			ag.setTarget(@ref, pos);
			ag.targetReplan = true;
			return true;
		}


		/// Submits a new move request for the specified agent.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @param[in] ref The position's polygon reference.
		/// @param[in] pos The position within the polygon. [(x, y, z)]
		/// @return True if the request was successfully submitted.
		/// 
		/// This method is used when a new target is set.
		/// 
		/// The position will be constrained to the surface of the navigation mesh.
		///
		/// The request will be processed during the next #update().
		public virtual bool requestMoveTarget(int idx, long @ref, float[] pos)
		{
			if (idx < 0 || idx >= m_maxAgents)
			{
				return false;
			}
			if (@ref == 0)
			{
				return false;
			}

			CrowdAgent ag = m_agents[idx];

			// Initialize request.
			ag.setTarget(@ref, pos);
			ag.targetReplan = false;

			return true;
		}

		/// Submits a new move request for the specified agent.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @param[in] vel The movement velocity. [(x, y, z)]
		/// @return True if the request was successfully submitted.
		public virtual bool requestMoveVelocity(int idx, float[] vel)
		{
			if (idx < 0 || idx >= m_maxAgents)
			{
				return false;
			}

			CrowdAgent ag = m_agents[idx];

			// Initialize request.
			ag.targetRef = 0;
            DetourCommon.vCopy(ag.targetPos, vel);
			ag.targetPathqRef = PathQueue.DT_PATHQ_INVALID;
			ag.targetReplan = false;
			ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY;

			return true;
		}

		/// Resets any request for the specified agent.
		/// @param[in] idx The agent index. [Limits: 0 <= value < #getAgentCount()]
		/// @return True if the request was successfully reseted.
		public virtual bool resetMoveTarget(int idx)
		{
			if (idx < 0 || idx >= m_maxAgents)
			{
				return false;
			}

			CrowdAgent ag = m_agents[idx];

			// Initialize request.
			ag.targetRef = 0;
            DetourCommon.vSet(ag.targetPos, 0, 0, 0);
            DetourCommon.vSet(ag.dvel, 0, 0, 0);
			ag.targetPathqRef = PathQueue.DT_PATHQ_INVALID;
			ag.targetReplan = false;
			ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_NONE;
			return true;
		}

		/// Gets the active agents int the agent pool.
		///  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
		///  @param[in]		maxAgents	The size of the crowd agent array.
		/// @return The number of agents returned in @p agents.
		public virtual IList<CrowdAgent> ActiveAgents
		{
			get
			{
				IList<CrowdAgent> agents = new List<CrowdAgent>(m_maxAgents);
				for (int i = 0; i < m_maxAgents; ++i)
				{
					if (m_agents[i].active)
					{
						agents.Add(m_agents[i]);
					}
				}
				return agents;
			}
		}

		internal const int MAX_ITER = 20;

        private RequestComparator agentComparator = new RequestComparator();

        private class RequestComparator : IComparer<CrowdAgent>
        {
            public virtual int Compare(CrowdAgent a1, CrowdAgent a2)
            {
                return a2.targetReplanTime.CompareTo(a1.targetReplanTime);
            }
        }


		internal virtual void updateMoveRequest()
		{
//JAVA TO C# CONVERTER TODO TASK: Java lambdas satisfy functional interfaces, while .NET lambdas satisfy delegates - change the appropriate interface to a delegate:
            PriorityQueue<CrowdAgent> queue = new PriorityQueue<CrowdAgent>(agentComparator);

			// Fire off new requests.
			CrowdAgent ag;
			List<long> path;
			List<long> reqPath;
			ClosesPointOnPolyResult cr = new ClosesPointOnPolyResult();
			for (int i = 0; i < m_maxAgents; ++i)
			{
				ag = m_agents[i];
				if (!ag.active)
				{
					continue;
				}
				if (ag.state == CrowdAgentState.DT_CROWDAGENT_STATE_INVALID)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}

				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING)
				{
					path = ag.corridor.Path;
					if (path.Count == 0)
					{
						throw new System.ArgumentException("Empty path");
					}
					// Quick search towards the goal.
					m_navquery.initSlicedFindPath(path[0], ag.targetRef, ag.npos, ag.targetPos, m_filters[ag.@params.queryFilterType], 0);
					m_navquery.updateSlicedFindPath(MAX_ITER);
					FindPathResult pathFound;
					if (ag.targetReplan) // && npath > 10)
					{
						// Try to use existing steady path during replan if
						// possible.
						pathFound = m_navquery.finalizeSlicedFindPathPartial(path);
					}
					else
					{
						// Try to move towards target when goal changes.
						pathFound = m_navquery.finalizeSlicedFindPath();
					}
					reqPath = pathFound.Refs;
                    DetourCommon.vSet(reqPos, 0, 0, 0);
					if (!pathFound.Status.Failed && reqPath.Count > 0)
					{
						// In progress or succeed.
                        if (reqPath[reqPath.Count - 1] != ag.targetRef)
						{
							// Partial path, constrain target position inside the
							// last polygon.
                            m_navquery.closestPointOnPoly(reqPath[reqPath.Count - 1], ag.targetPos, cr);
							//reqPos = cr.getClosest();
                            DetourCommon.vCopy(reqPos, cr.Closest);
						}
						else
						{
                            DetourCommon.vCopy(reqPos, ag.targetPos);
						}
					}
					else
					{
						// Could not find path, start the request from current
						// location.
                        DetourCommon.vCopy(reqPos, ag.npos);
						reqPath = new List<long>();
						reqPath.Add(path[0]);
					}

					ag.corridor.setCorridor(reqPos, reqPath);
					ag.boundary.reset();
					ag.partial = false;

					if (reqPath[reqPath.Count - 1] == ag.targetRef)
					{
						ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_VALID;
						ag.targetReplanTime = 0.0f;
					}
					else
					{
						// The path is longer or potentially unreachable, full plan.
						ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
					}
				}

				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
				{
					addToPathQueue(ag, queue);
				}
			}

			while (queue.Count > 0)
			{
				ag = queue.Poll();
				ag.targetPathqRef = m_pathq.request(ag.corridor.LastPoly, ag.targetRef, ag.corridor.Target, ag.targetPos, m_filters[ag.@params.queryFilterType]);
				if (ag.targetPathqRef != PathQueue.DT_PATHQ_INVALID)
				{
					ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
				}
			}

			// Update requests.
			m_pathq.update(MAX_ITERS_PER_UPDATE);

			// Process path results.
			Status status;
			float[] targetPos;
			bool valid;
			List<long> res;

			for (int i = 0; i < m_maxAgents; ++i)
			{
				ag = m_agents[i];
				if (!ag.active)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}

				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
				{
					// Poll path queue.
					status = m_pathq.getRequestStatus(ag.targetPathqRef);
					if (status != null && status.Failed)
					{
						// Path find failed, retry if the target location is still
						// valid.
						ag.targetPathqRef = PathQueue.DT_PATHQ_INVALID;
						if (ag.targetRef != 0)
						{
							ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING;
						}
						else
						{
							ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_FAILED;
						}
						ag.targetReplanTime = 0.0f;
					}
					else if (status != null && status.Success)
					{
						path = ag.corridor.Path;
						if (path.Count == 0)
						{
							throw new System.ArgumentException("Empty path");
						}

						// Apply results.
						targetPos = ag.targetPos;

						valid = true;
                        FindPathResult pathFound = m_pathq.getPathResult(ag.targetPathqRef);
						res = pathFound.Refs;
						status = pathFound.Status;
						if (status.Failed || res.Count == 0)
						{
							valid = false;
						}

						if (status.Partial)
						{
							ag.partial = true;
						}
						else
						{
							ag.partial = false;
						}

						// Merge result and existing path.
						// The agent might have moved whilst the request is
						// being processed, so the path may have changed.
						// We assume that the end of the path is at the same
						// location
						// where the request was issued.

						// The last ref in the old path should be the same as
						// the location where the request was issued..
						if (valid && path[path.Count - 1] != res[0])
						{
							valid = false;
						}

						if (valid)
						{
							// Put the old path infront of the old path.
							if (path.Count > 1)
							{
								path.RemoveAt(path.Count - 1);
                                path.AddRange(res);
								res = path;
								// Remove trackbacks
								for (int j = 1; j < res.Count - 1; ++j)
								{
                                    if (j - 1 >= 0 && j + 1 < res.Count)
									{
										if (res[j - 1] == res[j + 1])
										{
											res.RemoveAt(j + 1);
											res.RemoveAt(j);
											j -= 2;
										}
									}
								}
							}

							// Check for partial path.
							if (res[res.Count - 1] != ag.targetRef)
							{
								// Partial path, constrain target position inside
								// the last polygon.
								m_navquery.closestPointOnPoly(res[res.Count - 1], targetPos, cr);
								DetourCommon.vCopy(targetPos, cr.Closest);
							}
						}

						if (valid)
						{
							// Set current corridor.
							ag.corridor.setCorridor(targetPos, res);
							// Force to update boundary.
							ag.boundary.reset();
							ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_VALID;
						}
						else
						{
							// Something went wrong.
							ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_FAILED;
						}

						ag.targetReplanTime = 0.0f;
					}
				}
			}
		}

		internal const float OPT_TIME_THR = 0.5f; // seconds

        private OptimizationComparator optimizationComparator = new OptimizationComparator();

        private class OptimizationComparator : IComparer<CrowdAgent>
        {
            public virtual int Compare(CrowdAgent a1, CrowdAgent a2)
            {
                return a2.topologyOptTime.CompareTo(a1.topologyOptTime);
            }
        }

		internal virtual void updateTopologyOptimization(IList<CrowdAgent> agents, float dt)
		{
			if (agents.Count > 0)
			{
				return;
			}

//JAVA TO C# CONVERTER TODO TASK: Java lambdas satisfy functional interfaces, while .NET lambdas satisfy delegates - change the appropriate interface to a delegate:
            PriorityQueue<CrowdAgent> queue = new PriorityQueue<CrowdAgent>(optimizationComparator);

			for (int i = 0; i < agents.Count; ++i)
			{
				CrowdAgent ag = agents[i];
				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}
				if ((ag.@params.updateFlags & CrowdAgent.DT_CROWD_OPTIMIZE_TOPO) == 0)
				{
					continue;
				}
				ag.topologyOptTime += dt;
				if (ag.topologyOptTime >= OPT_TIME_THR)
				{
					addToOptQueue(ag, queue);
				}
			}

			while (queue.Count > 0)
			{
				CrowdAgent ag = queue.Poll();
				ag.corridor.optimizePathTopology(m_navquery, m_filters[ag.@params.queryFilterType]);
				ag.topologyOptTime = 0;
			}

		}

		internal const int CHECK_LOOKAHEAD = 10;
		internal const float TARGET_REPLAN_DELAY = 1.0f; // seconds

		internal virtual void checkPathValidity(IList<CrowdAgent> agents, float dt)
		{

			bool replan;
			for (int i = 0; i < agents.Count; ++i)
			{
				CrowdAgent ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}

				ag.targetReplanTime += dt;

				replan = false;

				// First check that the current location is valid.
				//float[] agentPos = new float[3];

				long agentRef = ag.corridor.FirstPoly;
                DetourCommon.vCopy(agentPos, ag.npos);
				if (!m_navquery.isValidPolyRef(agentRef, m_filters[ag.@params.queryFilterType]))
				{
					// Current location is not valid, try to reposition.
					// TODO: this can snap agents, how to handle that?
					FindNearestPolyResult fnp = m_navquery.findNearestPoly(ag.npos, m_ext, m_filters[ag.@params.queryFilterType]);
					agentRef = fnp.NearestRef;
					if (fnp.NearestPos != null)
					{
                        DetourCommon.vCopy(agentPos, fnp.NearestPos);
					}

					if (agentRef == 0)
					{
						// Could not find location in navmesh, set state to invalid.
						ag.corridor.reset(0, agentPos);
						ag.partial = false;
						ag.boundary.reset();
						ag.state = CrowdAgentState.DT_CROWDAGENT_STATE_INVALID;
						continue;
					}

					// Make sure the first polygon is valid, but leave other valid
					// polygons in the path so that replanner can adjust the path
					// better.
					ag.corridor.fixPathStart(agentRef, agentPos);
					// ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery,
					// &m_filter);
					ag.boundary.reset();
					DetourCommon.vCopy(ag.npos, agentPos);

					replan = true;
				}

				// If the agent does not have move target or is controlled by
				// velocity, no need to recover the target nor replan.
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}

				// Try to recover move request position.
				if (ag.targetState != MoveRequestState.DT_CROWDAGENT_TARGET_NONE && ag.targetState != MoveRequestState.DT_CROWDAGENT_TARGET_FAILED)
				{
					if (!m_navquery.isValidPolyRef(ag.targetRef, m_filters[ag.@params.queryFilterType]))
					{
						// Current target is not valid, try to reposition.
						FindNearestPolyResult fnp = m_navquery.findNearestPoly(ag.targetPos, m_ext, m_filters[ag.@params.queryFilterType]);
						ag.targetRef = fnp.NearestRef;
						if (fnp.NearestPos != null)
						{
                            DetourCommon.vCopy(ag.targetPos, fnp.NearestPos);
						}
						replan = true;
					}
					if (ag.targetRef == 0)
					{
						// Failed to reposition target, fail moverequest.
						ag.corridor.reset(agentRef, agentPos);
						ag.partial = false;
						ag.targetState = MoveRequestState.DT_CROWDAGENT_TARGET_NONE;
					}
				}

				// If nearby corridor is not valid, replan.
				if (!ag.corridor.isValid(CHECK_LOOKAHEAD, m_navquery, m_filters[ag.@params.queryFilterType]))
				{
					// Fix current path.
					// ag.corridor.trimInvalidPath(agentRef, agentPos, m_navquery,
					// &m_filter);
					// ag.boundary.reset();
					replan = true;
				}

				// If the end of the path is near and it is not the requested
				// location, replan.
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VALID)
				{
					if (ag.targetReplanTime > TARGET_REPLAN_DELAY && ag.corridor.PathCount < CHECK_LOOKAHEAD && ag.corridor.LastPoly != ag.targetRef)
					{
						replan = true;
					}
				}

				// Try to replan path to goal.
				if (replan)
				{
					if (ag.targetState != MoveRequestState.DT_CROWDAGENT_TARGET_NONE)
					{
						requestMoveTargetReplan(ag, ag.targetRef, ag.targetPos);
					}
				}
			}
		}

		internal const float COLLISION_RESOLVE_FACTOR = 0.7f;

		public virtual void update(float dt, CrowdAgentDebugInfo debug)
		{
			m_velocitySampleCount = 0;

			int debugIdx = debug != null ? debug.idx : -1;

			IList<CrowdAgent> agents = ActiveAgents;

			// Check that all agents still have valid paths.
			checkPathValidity(agents, dt);

			// Update async move request and path finder.
			updateMoveRequest();

			// Optimize path topology.
			updateTopologyOptimization(agents, dt);

			// Register agents to proximity grid.
			m_grid.clear();
			CrowdAgent agent;
			for (int i = 0; i < agents.Count; ++i)
			{
				agent = agents[i];
				float[] p = agent.npos;
				float r = agent.@params.radius;
				m_grid.addItem(i, p[0] - r, p[2] - r, p[0] + r, p[2] + r);
			}

			CrowdAgent ag;
			float[] target;
			long[] refs = new long[2];

            DetourCommon.vSet(dvel, 0, 0, 0);
            DetourCommon.vSet(disp, 0, 0, 0);
            DetourCommon.vSet(diff, 0, 0, 0);
            DetourCommon.vSet(temp, 0, 0, 0);
			CrowdAgent nei;

			// Get nearby navmesh segments and agents to collide with.
			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}

				// Update the collision boundary after certain distance has been passed or
				// if it has become invalid.
				float updateThr = ag.@params.collisionQueryRange * 0.25f;
                if (DetourCommon.vDist2DSqr(ag.npos, ag.boundary.Center) > DetourCommon.sqr(updateThr) || !ag.boundary.isValid(m_navquery, m_filters[ag.@params.queryFilterType]))
				{
					ag.boundary.update(ag.corridor.FirstPoly, ag.npos, ag.@params.collisionQueryRange, m_navquery, m_filters[ag.@params.queryFilterType]);
				}
				// Query neighbour agents
				ag.neisLength = getNeighbours(ag.npos, ag.@params.height, ag.@params.collisionQueryRange, ag, ag.neis, CrowdAgent.DT_CROWDAGENT_MAX_NEIGHBOURS, agents, m_grid);
			}

			// Find next corner to steer to.
			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}

				// Find corners for steering
				ag.corners = ag.corridor.findCorners(DT_CROWDAGENT_MAX_CORNERS, m_navquery, m_filters[ag.@params.queryFilterType]);

				// Check to see if the corner after the next corner is directly visible,
				// and short cut to there.
				if ((ag.@params.updateFlags & CrowdAgent.DT_CROWD_OPTIMIZE_VIS) != 0 && ag.corners.Count > 0)
				{
					target = ag.corners[Math.Min(1,ag.corners.Count - 1)].Pos;
					ag.corridor.optimizePathVisibility(target, ag.@params.pathOptimizationRange, m_navquery, m_filters[ag.@params.queryFilterType]);

					// Copy data for debug purposes.
					if (debugIdx == i)
					{
                        DetourCommon.vCopy(debug.optStart, ag.corridor.Pos);
                        DetourCommon.vCopy(debug.optEnd, target);
					}
				}
				else
				{
					// Copy data for debug purposes.
					if (debugIdx == i)
					{
                        DetourCommon.vSet(debug.optStart, 0, 0, 0);
                        DetourCommon.vSet(debug.optEnd, 0, 0, 0);
					}
				}
			}

			// Trigger off-mesh connections (depends on corners).
			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					continue;
				}

				// Check 
				float triggerRadius = ag.@params.radius * 2.25f;
				if (ag.overOffmeshConnection(triggerRadius))
				{
					// Prepare to off-mesh connection.
					CrowdAgentAnimation anim = ag.animation;

					// Adjust the path over the off-mesh connection.
					DetourCommon.vResetArray(refs);
					if (ag.corridor.moveOverOffmeshConnection(ag.corners[ag.corners.Count - 1].Ref, refs, anim.startPos, anim.endPos, m_navquery))
					{
                        DetourCommon.vCopy(anim.initPos, ag.npos);
						anim.polyRef = refs[1];
						anim.active = true;
						anim.t = 0.0f;
                        anim.tmax = (DetourCommon.vDist2D(anim.startPos, anim.endPos) / ag.@params.maxSpeed) * 0.5f;

						ag.state = CrowdAgentState.DT_CROWDAGENT_STATE_OFFMESH;
						ag.corners.Clear();
						ag.neisLength = 0;
						continue;
					}
					else
					{
						// Path validity check will ensure that bad/blocked connections will be replanned.
					}
				}
			}

			// Calculate steering.
			float slowDownRadius;
			float speedScale;
			float w;
			float separationDist;
			float invSeparationDist;
			float separationWeight;
			float distSqr;
			float dist;
			float weight;
			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE)
				{
					continue;
				}

				//float[] dvel = new float[3];
				DetourCommon.vSet(dvel, 0, 0, 0);

				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
                    DetourCommon.vCopy(dvel, ag.targetPos);
                    ag.desiredSpeed = DetourCommon.vLen(ag.targetPos);
				}
				else
				{
					// Calculate steering direction.
					if ((ag.@params.updateFlags & CrowdAgent.DT_CROWD_ANTICIPATE_TURNS) != 0)
					{
						//dvel = ag.calcSmoothSteerDirection();
						ag.calcSmoothSteerDirection(dvel);
					}
					else
					{
						//dvel = ag.calcStraightSteerDirection();
						ag.calcStraightSteerDirection(dvel);
					}
					// Calculate speed scale, which tells the agent to slowdown at the end of the path.
					slowDownRadius = ag.@params.radius * 2; // TODO: make less hacky.
					speedScale = ag.getDistanceToGoal(slowDownRadius) / slowDownRadius;

					ag.desiredSpeed = ag.@params.maxSpeed;
					//dvel = vScale(dvel, ag.desiredSpeed * speedScale);
                    DetourCommon.vScale(temp, dvel, ag.desiredSpeed * speedScale);
                    DetourCommon.vCopy(dvel, temp);
				}

				// Separation
				if ((ag.@params.updateFlags & CrowdAgent.DT_CROWD_SEPARATION) != 0)
				{
					separationDist = ag.@params.collisionQueryRange;
					invSeparationDist = 1.0f / separationDist;
					separationWeight = ag.@params.separationWeight;

					w = 0;
					//float[] disp = new float[3];
					DetourCommon.vSet(disp, 0, 0, 0);

					for (int j = 0; j < ag.neisLength; ++j)
					{
						nei = agents[ag.neis[j].idx];

						//float[] diff = vSub(ag.npos, nei.npos);
                        DetourCommon.vSub(diff, ag.npos, nei.npos);

						diff[1] = 0;

                        distSqr = DetourCommon.vLenSqr(diff);
						if (distSqr < 0.00001f)
						{
							continue;
						}
                        if (distSqr > DetourCommon.sqr(separationDist))
						{
							continue;
						}
						dist = (float) Math.Sqrt(distSqr);
                        weight = separationWeight * (1.0f - DetourCommon.sqr(dist * invSeparationDist));

						//disp = vMad(disp, diff, weight/dist);
                        DetourCommon.vMad(disp, disp, diff, weight / dist);
						//vCopy(disp, temp);
						w += 1.0f;
					}

					if (w > 0.0001f)
					{
						// Adjust desired velocity.
						//dvel = vMad(dvel, disp, 1.0f/w);
                        DetourCommon.vMad(dvel, dvel, disp, 1.0f / w);
						//vCopy(dvel, temp);
						// Clamp desired velocity to desired speed.
                        float speedSqr = DetourCommon.vLenSqr(dvel);
                        float desiredSqr = DetourCommon.sqr(ag.desiredSpeed);
						if (speedSqr > desiredSqr)
						{
							//dvel = vScale(dvel, desiredSqr/speedSqr);
                            DetourCommon.vScale(dvel, dvel, desiredSqr / speedSqr);
							//vCopy(dvel, temp);
						}

					}
				}

				// Set the desired velocity.
                DetourCommon.vCopy(ag.dvel, dvel);
			}


			// Velocity planning.	
			float[] s;
			float[] s3 = new float[3];
			ObstacleAvoidanceParams @params;
			bool adaptive;
			int ns = 0;

			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];

				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}

				if ((ag.@params.updateFlags & CrowdAgent.DT_CROWD_OBSTACLE_AVOIDANCE) != 0)
				{
					m_obstacleQuery.reset();

					// Add neighbours as obstacles.
					for (int j = 0; j < ag.neisLength; ++j)
					{
						nei = agents[ag.neis[j].idx];
						m_obstacleQuery.addCircle(nei.npos, nei.@params.radius, nei.vel, nei.dvel);
					}

					// Append neighbour segments as obstacles.
					for (int j = 0; j < ag.boundary.SegmentCount; ++j)
					{
						//float[] s = ag.boundary.getSegment(j);
						//float[] s3 = Arrays.copyOfRange(s, 3, 6);
						s = ag.boundary.getSegment(j);
						Array.Copy(s, 3, s3, 0, 3);
                        if (DetourCommon.triArea2D(ag.npos, s, s3) < 0.0f)
						{
							continue;
						}
						m_obstacleQuery.addSegment(s, s3);
					}

					ObstacleAvoidanceDebugData vod = null;
					if (debugIdx == i)
					{
						vod = debug.vod;
					}

					// Sample new safe velocity.
					adaptive = false;
					ns = 0;

					@params = m_obstacleQueryParams[ag.@params.obstacleAvoidanceType];

					if (adaptive)
					{
						ns = m_obstacleQuery.sampleVelocityAdaptive(ag.npos, ag.@params.radius, ag.desiredSpeed, ag.vel, ag.dvel, ag.nvel, @params, vod);
						//ns = nsnvel.first;
						//ag.nvel = nsnvel.second;
						//vCopy(ag.nvel, nsnvel.second);
					}
					else
					{
						ns = m_obstacleQuery.sampleVelocityGrid(ag.npos, ag.@params.radius, ag.desiredSpeed, ag.vel, ag.dvel, ag.nvel, @params, vod);
						//ns = nsnvel.first;
						//vCopy(ag.nvel, nsnvel.second);
					}
					m_velocitySampleCount += ns;
				}
				else
				{
					// If not using velocity planning, new velocity is directly the desired velocity.
                    DetourCommon.vCopy(ag.nvel, ag.dvel);
				}
			}

			// Integrate.
			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];
				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}
				ag.integrate(dt);
			}

			// Handle collisions.
			for (int iter = 0; iter < 4; ++iter)
			{
				for (int i = 0; i < agents.Count; ++i)
				{
					ag = agents[i];
					int idx0 = ag.AgentIndex;
					if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
					{
						continue;
					}

                    DetourCommon.vSet(ag.disp, 0, 0, 0);

					w = 0;

					for (int j = 0; j < ag.neisLength; ++j)
					{
						nei = agents[ag.neis[j].idx];
						int idx1 = nei.AgentIndex;
						//float[] diff = vSub(ag.npos, nei.npos);
                        DetourCommon.vSub(diff, ag.npos, nei.npos);
						diff[1] = 0;

                        dist = DetourCommon.vLenSqr(diff);
                        if (dist > DetourCommon.sqr(ag.@params.radius + nei.@params.radius))
						{
							continue;
						}
						dist = (float) Math.Sqrt(dist);
						float pen = (ag.@params.radius + nei.@params.radius) - dist;
						if (dist < 0.0001f)
						{
							// Agents on top of each other, try to choose diverging separation directions.
							if (idx0 > idx1)
							{
                                DetourCommon.vSet(diff, -ag.dvel[2], 0, ag.dvel[0]);
							}
							else
							{
                                DetourCommon.vSet(diff, ag.dvel[2], 0, -ag.dvel[0]);
							}
							pen = 0.01f;
						}
						else
						{
							pen = (1.0f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
						}

						//ag.disp  = vMad(ag.disp, diff, pen);	
                        DetourCommon.vMad(temp, ag.disp, diff, pen);
                        DetourCommon.vCopy(ag.disp, temp);

						w += 1.0f;
					}

					if (w > 0.0001f)
					{
						float iw = 1.0f / w;
						//ag.disp = vScale(ag.disp, iw);
                        DetourCommon.vScale(temp, ag.disp, iw);
                        DetourCommon.vCopy(ag.disp, temp);
					}
				}

				for (int i = 0; i < agents.Count; ++i)
				{
					ag = agents[i];
					if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
					{
						continue;
					}

					//ag.npos = vAdd(ag.npos, ag.disp);
                    DetourCommon.vAdd(ag.npos, ag.npos, ag.disp);
					//vCopy(ag.npos, temp);
				}
			}

			for (int i = 0; i < agents.Count; ++i)
			{
				ag = agents[i];
				if (ag.state != CrowdAgentState.DT_CROWDAGENT_STATE_WALKING)
				{
					continue;
				}

				// Move along navmesh.
				ag.corridor.movePosition(ag.npos, m_navquery, m_filters[ag.@params.queryFilterType]);
				// Get valid constrained position back.
                DetourCommon.vCopy(ag.npos, ag.corridor.Pos);

				// If not using path, truncate the corridor to just one poly.
				if (ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_NONE || ag.targetState == MoveRequestState.DT_CROWDAGENT_TARGET_VELOCITY)
				{
					ag.corridor.reset(ag.corridor.FirstPoly, ag.npos);
					ag.partial = false;
				}

			}

			// Update agents using off-mesh connection.
			for (int i = 0; i < m_maxAgents; ++i)
			{
                CrowdAgentAnimation anim = m_agents[i].animation;
				if (!anim.active)
				{
					continue;
				}
				ag = m_agents[i];

				anim.t += dt;
				if (anim.t > anim.tmax)
				{
					// Reset animation
					anim.active = false;
					// Prepare agent for walking.
					ag.state = CrowdAgentState.DT_CROWDAGENT_STATE_WALKING;
					continue;
				}

				// Update position
				float ta = anim.tmax * 0.15f;
				float tb = anim.tmax;
				if (anim.t < ta)
				{
					float u = tween(anim.t, 0.0f, ta);
					//ag.npos = vLerp(anim.initPos, anim.startPos, u);
                    DetourCommon.vLerp(ag.npos, anim.initPos, anim.startPos, u);
				}
				else
				{
					float u = tween(anim.t, ta, tb);
					//ag.npos = vLerp(anim.startPos, anim.endPos, u);
                    DetourCommon.vLerp(ag.npos, anim.startPos, anim.endPos, u);
				}

				// Update velocity.
                DetourCommon.vSet(ag.vel, 0, 0, 0);
                DetourCommon.vSet(ag.dvel, 0, 0, 0);
			}
		}

		public virtual float[] QueryExtents
		{
			get
			{
				return m_ext;
			}
		}

		public virtual QueryFilter getFilter(int i)
		{
			return i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE ? m_filters[i] : null;
		}

	}


}