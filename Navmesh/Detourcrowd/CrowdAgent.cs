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
//	import static org.recast4j.detour.DetourCommon.vMad;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vNormalize;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vScale;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSet;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSub;



	/// Represents an agent managed by a #dtCrowd object.
	/// @ingroup crowd
	public class CrowdAgent
	{

		/// The type of navigation mesh polygon the agent is currently traversing.
		/// @ingroup crowd
		public enum CrowdAgentState
		{
			DT_CROWDAGENT_STATE_INVALID, ///< The agent is not in a valid state.
			DT_CROWDAGENT_STATE_WALKING, ///< The agent is traversing a normal navigation mesh polygon.
			DT_CROWDAGENT_STATE_OFFMESH, ///< The agent is traversing an off-mesh connection.
		}

		internal enum MoveRequestState
		{
			DT_CROWDAGENT_TARGET_NONE,
			DT_CROWDAGENT_TARGET_FAILED,
			DT_CROWDAGENT_TARGET_VALID,
			DT_CROWDAGENT_TARGET_REQUESTING,
			DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
			DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
			DT_CROWDAGENT_TARGET_VELOCITY,
		}

		/// Crowd agent update flags.
		public const int DT_CROWD_ANTICIPATE_TURNS = 1;
		public const int DT_CROWD_OBSTACLE_AVOIDANCE = 2;
		public const int DT_CROWD_SEPARATION = 4;
		public const int DT_CROWD_OPTIMIZE_VIS = 8; ///< Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
		public const int DT_CROWD_OPTIMIZE_TOPO = 16; ///< Use dtPathCorridor::optimizePathTopology() to optimize the agent path.



		public const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

		internal readonly int idx;

		/// True if the agent is active, false if the agent is in an unused slot in the agent pool.
		internal bool active;

		/// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
		internal CrowdAgentState state;

		/// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
		internal bool partial;

		/// The path corridor the agent is using.
		internal PathCorridor corridor;

		/// The local boundary data for the agent.
		internal LocalBoundary boundary;

		/// Time since the agent's path corridor was optimized.
		internal float topologyOptTime;

		/// The known neighbors of the agent.
		//List<CrowdNeighbour> neis = new ArrayList<>();
		internal CrowdNeighbour[] neis;
		internal int neisLength;


		/// The desired speed.
		internal float desiredSpeed;

		internal float[] npos = new float[3]; ///< The current agent position. [(x, y, z)]
		internal float[] disp = new float[3]; ///< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
		internal float[] dvel = new float[3]; ///< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
		internal float[] nvel = new float[3]; ///< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
		internal float[] vel = new float[3]; ///< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

		/// The agent's configuration parameters.
		internal CrowdAgentParams @params;
		/// The local path corridor corners for the agent.
		internal IList<StraightPathItem> corners = new List<StraightPathItem>();

		internal MoveRequestState targetState; ///< State of the movement request.
		internal long targetRef; ///< Target polyref of the movement request.
		internal float[] targetPos = new float[3]; ///< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
		internal long targetPathqRef; ///< Path finder ref.
		internal bool targetReplan; ///< Flag indicating that the current path is being replanned.
		internal float targetReplanTime; /// <Time since the agent's target was replanned.

		internal CrowdAgentAnimation animation;

		/// <summary>
		///***����������ʱ����**** </summary>
		private readonly float[] dv = new float[3];
		private readonly float[] dir0 = new float[3];
		private readonly float[] dir1 = new float[3];
		/// <summary>
		///***************** </summary>

		public CrowdAgent(int idx)
		{
			this.idx = idx;
			corridor = new PathCorridor();
			boundary = new LocalBoundary();
			animation = new CrowdAgentAnimation();
			neis = new CrowdNeighbour[DT_CROWDAGENT_MAX_NEIGHBOURS];
			for (int i = 0; i < neis.Length; i++)
			{
				neis[i] = new CrowdNeighbour(this, 0, 0f);
			}
		}

		internal virtual void integrate(float dt)
		{
			// Fake dynamic constraint.
			float maxDelta = @params.maxAcceleration * dt;
			//float[] dv = vSub(nvel, vel);
			DetourCommon.vSub(dv, nvel, vel);
            float ds = DetourCommon.vLen(dv);
			if (ds > maxDelta)
			{
				//dv = vScale(dv, maxDelta / ds);
                DetourCommon.vScale(dv, dv, maxDelta / ds);
			}
			//vel = vAdd(vel, dv);
            DetourCommon.vAdd(vel, vel, dv);

			// Integrate
            if (DetourCommon.vLen(vel) > 0.0001f)
			{
				//npos = vMad(npos, vel, dt);
                DetourCommon.vMad(npos, npos, vel, dt);
			}
			else
			{
                DetourCommon.vSet(vel, 0, 0, 0);
			}
		}

		internal virtual bool overOffmeshConnection(float radius)
		{
			if (corners.Count == 0)
			{
				return false;
			}

			bool offMeshConnection = ((corners[corners.Count - 1].Flags & NavMeshQuery.DT_STRAIGHTPATH_OFFMESH_CONNECTION) != 0) ? true : false;
			if (offMeshConnection)
			{
                float distSq = DetourCommon.vDist2DSqr(npos, corners[corners.Count - 1].Pos);
				if (distSq < radius * radius)
				{
					return true;
				}
			}

			return false;
		}

		internal virtual float getDistanceToGoal(float range)
		{
			if (corners.Count == 0)
			{
				return range;
			}

			bool endOfPath = ((corners[corners.Count - 1].Flags & NavMeshQuery.DT_STRAIGHTPATH_END) != 0) ? true : false;
			if (endOfPath)
			{
                return Math.Min(DetourCommon.vDist2D(npos, corners[corners.Count - 1].Pos), range);
			}

			return range;
		}



		public virtual void calcSmoothSteerDirection(float[] dir)
		{
			if (corners.Count > 0)
			{
				int ip0 = 0;
				int ip1 = Math.Min(1, corners.Count - 1);
				float[] p0 = corners[ip0].Pos;
				float[] p1 = corners[ip1].Pos;

				//float[] dir0 = vSub(p0, npos);
				//float[] dir1 = vSub(p1, npos);
                DetourCommon.vSub(dir0, p0, npos);
                DetourCommon.vSub(dir1, p1, npos);

				dir0[1] = 0;
				dir1[1] = 0;

                float len0 = DetourCommon.vLen(dir0);
                float len1 = DetourCommon.vLen(dir1);
				if (len1 > 0.001f)
				{
					//dir1 = vScale(dir1, 1.0f / len1);
                    DetourCommon.vScale(dir1, dir1, 1.0f / len1);
				}


				dir[0] = dir0[0] - dir1[0] * len0 * 0.5f;
				dir[1] = 0;
				dir[2] = dir0[2] - dir1[2] * len0 * 0.5f;

                DetourCommon.vNormalize(dir);
			}
		}



		public virtual void calcStraightSteerDirection(float[] dir)
		{
			if (corners.Count > 0)
			{
                DetourCommon.vSub(dir, corners[0].Pos, npos);
				dir[1] = 0;
                DetourCommon.vNormalize(dir);
			}
		}





		internal virtual void setTarget(long @ref, float[] pos)
		{
			targetRef = @ref;
            DetourCommon.vCopy(targetPos, pos);
			targetPathqRef = PathQueue.DT_PATHQ_INVALID;
			if (targetRef != 0)
			{
				targetState = MoveRequestState.DT_CROWDAGENT_TARGET_REQUESTING;
			}
			else
			{
				targetState = MoveRequestState.DT_CROWDAGENT_TARGET_FAILED;
			}
		}

		internal virtual int AgentIndex
		{
			get
			{
				return idx;
			}
		}

		public virtual bool Active
		{
			get
			{
				return active;
			}
		}

		/// <summary>
		/// ��õ�ǰλ��
		/// @return
		/// </summary>
		public virtual float[] CurrentPosition
		{
			get
			{
				return npos;
			}
		}



		/// <summary>
		/// ��ô����״̬
		/// @return
		/// </summary>
		public virtual CrowdAgentState getCrowdAgentState()
		{
			return state;
		}



		internal class CrowdNeighbour
		{
			private readonly CrowdAgent outerInstance;

			internal int idx; ///< The index of the neighbor in the crowd.
			internal float dist; ///< The distance between the current agent and the neighbor.
			public CrowdNeighbour(CrowdAgent outerInstance, int idx, float dist)
			{
				this.outerInstance = outerInstance;
				this.idx = idx;
				this.dist = dist;
			}
		}

	}
}