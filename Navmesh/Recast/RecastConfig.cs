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
namespace org.recast4j.recast
{

	using PartitionType = org.recast4j.recast.RecastConstants.PartitionType;

	public class RecastConfig
	{
		public readonly PartitionType partitionType;

		/// <summary>
		/// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx] * </summary>
		public readonly int tileSize;

		/// <summary>
		/// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] * </summary>
		public readonly float cs;

		/// <summary>
		/// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu] * </summary>
		public readonly float ch;

		/// <summary>
		/// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] * </summary>
		public readonly float walkableSlopeAngle;

		/// <summary>
		/// Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3]
		/// [Units: vx]
		/// 
		/// </summary>
		public readonly int walkableHeight;

		/// <summary>
		/// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] * </summary>
		public readonly int walkableClimb;

		/// <summary>
		/// The distance to erode/shrink the walkable area of the heightfield away from obstructions. [Limit: >=0] [Units:
		/// vx]
		/// 
		/// </summary>
		public readonly int walkableRadius;

		/// <summary>
		/// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] * </summary>
		public readonly int maxEdgeLen;

		/// <summary>
		/// The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0]
		/// [Units: vx]
		/// 
		/// </summary>
		public readonly float maxSimplificationError;

		/// <summary>
		/// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] * </summary>
		public readonly int minRegionArea;

		/// <summary>
		/// Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit:
		/// >=0] [Units: vx]
		/// 
		/// </summary>
		public readonly int mergeRegionArea;

		/// <summary>
		/// The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
		/// [Limit: >= 3]
		/// 
		/// </summary>
		public readonly int maxVertsPerPoly;

		/// <summary>
		/// Sets the sampling distance to use when generating the detail mesh. (For height detail only.) [Limits: 0 or >=
		/// 0.9] [Units: wu]
		/// 
		/// </summary>
		public readonly float detailSampleDist;

		/// <summary>
		/// The maximum distance the detail mesh surface should deviate from heightfield data. (For height detail only.)
		/// [Limit: >=0] [Units: wu]
		/// 
		/// </summary>
		public readonly float detailSampleMaxError;

		public RecastConfig(PartitionType partitionType, float cellSize, float cellHeight, float agentHeight, float agentRadius, float agentMaxClimb, float agentMaxSlope, int regionMinSize, int regionMergeSize, float edgeMaxLen, float edgeMaxError, int vertsPerPoly, float detailSampleDist, float detailSampleMaxError, int tileSize)
		{
			this.partitionType = partitionType;
			this.cs = cellSize;
			this.ch = cellHeight;
			this.walkableSlopeAngle = agentMaxSlope;
			this.walkableHeight = (int) Math.Ceiling(agentHeight / ch);
			this.walkableClimb = (int) Math.Floor(agentMaxClimb / ch);
			this.walkableRadius = (int) Math.Ceiling(agentRadius / cs);
			this.maxEdgeLen = (int)(edgeMaxLen / cellSize);
			this.maxSimplificationError = edgeMaxError;
			this.minRegionArea = regionMinSize * regionMinSize; // Note: area = size*size
			this.mergeRegionArea = regionMergeSize * regionMergeSize; // Note: area = size*size
			this.maxVertsPerPoly = vertsPerPoly;
			this.detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
			this.detailSampleMaxError = cellHeight * detailSampleMaxError;
			this.tileSize = tileSize;
		}

	}

}