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

	/// Configuration parameters for a crowd agent.
	/// @ingroup crowd
	public class CrowdAgentParams
	{
		internal float radius; ///< Agent radius. [Limit: >= 0]
		internal float height; ///< Agent height. [Limit: > 0]
		internal float maxAcceleration; ///< Maximum allowed acceleration. [Limit: >= 0]
		internal float maxSpeed; ///< Maximum allowed speed. [Limit: >= 0]

		/// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
		internal float collisionQueryRange;

		internal float pathOptimizationRange; ///< The path visibility optimization range. [Limit: > 0]

		/// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
		internal float separationWeight;

		/// Flags that impact steering behavior. (See: #UpdateFlags)
		internal int updateFlags;

		/// The index of the avoidance configuration to use for the agent. 
		/// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
		internal int obstacleAvoidanceType;

		/// The index of the query filter used by this agent.
		internal int queryFilterType;

		/// User defined data attached to the agent.
		internal object userData;

		public virtual float Radius
		{
			get
			{
				return radius;
			}
			set
			{
				this.radius = value;
			}
		}


		public virtual float Height
		{
			get
			{
				return height;
			}
			set
			{
				this.height = value;
			}
		}


		public virtual float MaxAcceleration
		{
			get
			{
				return maxAcceleration;
			}
			set
			{
				this.maxAcceleration = value;
			}
		}


		public virtual float MaxSpeed
		{
			get
			{
				return maxSpeed;
			}
			set
			{
				this.maxSpeed = value;
			}
		}


		public virtual float CollisionQueryRange
		{
			get
			{
				return collisionQueryRange;
			}
			set
			{
				this.collisionQueryRange = value;
			}
		}


		public virtual float PathOptimizationRange
		{
			get
			{
				return pathOptimizationRange;
			}
			set
			{
				this.pathOptimizationRange = value;
			}
		}


		public virtual float SeparationWeight
		{
			get
			{
				return separationWeight;
			}
			set
			{
				this.separationWeight = value;
			}
		}


		public virtual int UpdateFlags
		{
			get
			{
				return updateFlags;
			}
			set
			{
				this.updateFlags = value;
			}
		}


		public virtual int ObstacleAvoidanceType
		{
			get
			{
				return obstacleAvoidanceType;
			}
			set
			{
				this.obstacleAvoidanceType = value;
			}
		}


		public virtual int QueryFilterType
		{
			get
			{
				return queryFilterType;
			}
			set
			{
				this.queryFilterType = value;
			}
		}


		public virtual object UserData
		{
			get
			{
				return userData;
			}
			set
			{
				this.userData = value;
			}
		}




	}
}