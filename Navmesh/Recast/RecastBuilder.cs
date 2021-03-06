﻿using System.Collections.Generic;

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

	using ChunkyTriMeshNode = org.recast4j.recast.ChunkyTriMesh.ChunkyTriMeshNode;
	using PartitionType = org.recast4j.recast.RecastConstants.PartitionType;

	public class RecastBuilder
	{

		public class RecastBuilderResult
		{
			private readonly RecastBuilder outerInstance;

			internal readonly PolyMesh pmesh;
			internal readonly PolyMeshDetail dmesh;

			public RecastBuilderResult(RecastBuilder outerInstance, PolyMesh pmesh, PolyMeshDetail dmesh)
			{
				this.outerInstance = outerInstance;
				this.pmesh = pmesh;
				this.dmesh = dmesh;
			}

			public virtual PolyMesh Mesh
			{
				get
				{
					return pmesh;
				}
			}

			public virtual PolyMeshDetail MeshDetail
			{
				get
				{
					return dmesh;
				}
			}
		}

		public virtual RecastBuilderResult build(InputGeom geom, RecastBuilderConfig bcfg)
		{

			RecastConfig cfg = bcfg.cfg;
			Context ctx = new Context();
			CompactHeightfield chf = buildCompactHeightfield(geom, bcfg, ctx);

			// Partition the heightfield so that we can use simple algorithm later
			// to triangulate the walkable areas.
			// There are 3 martitioning methods, each with some pros and cons:
			// 1) Watershed partitioning
			// - the classic Recast partitioning
			// - creates the nicest tessellation
			// - usually slowest
			// - partitions the heightfield into nice regions without holes or
			// overlaps
			// - the are some corner cases where this method creates produces holes
			// and overlaps
			// - holes may appear when a small obstacles is close to large open area
			// (triangulation can handle this)
			// - overlaps may occur if you have narrow spiral corridors (i.e
			// stairs), this make triangulation to fail
			// * generally the best choice if you precompute the nacmesh, use this
			// if you have large open areas
			// 2) Monotone partioning
			// - fastest
			// - partitions the heightfield into regions without holes and overlaps
			// (guaranteed)
			// - creates long thin polygons, which sometimes causes paths with
			// detours
			// * use this if you want fast navmesh generation
			// 3) Layer partitoining
			// - quite fast
			// - partitions the heighfield into non-overlapping regions
			// - relies on the triangulation code to cope with holes (thus slower
			// than monotone partitioning)
			// - produces better triangles than monotone partitioning
			// - does not have the corner cases of watershed partitioning
			// - can be slow and create a bit ugly tessellation (still better than
			// monotone)
			// if you have large open areas with small obstacles (not a problem if
			// you use tiles)
			// * good choice to use for tiled navmesh with medium and small sized
			// tiles

			if (cfg.partitionType == PartitionType.WATERSHED)
			{
				// Prepare for region partitioning, by calculating distance field
				// along the walkable surface.
				RecastRegion.buildDistanceField(ctx, chf);
				// Partition the walkable surface into simple regions without holes.
				RecastRegion.buildRegions(ctx, chf, bcfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea);
			}
			else if (cfg.partitionType == PartitionType.MONOTONE)
			{
				// Partition the walkable surface into simple regions without holes.
				// Monotone partitioning does not need distancefield.
				RecastRegion.buildRegionsMonotone(ctx, chf, bcfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea);
			}
			else
			{
				// Partition the walkable surface into simple regions without holes.
				RecastRegion.buildLayerRegions(ctx, chf, bcfg.borderSize, cfg.minRegionArea);
			}

			//
			// Step 5. Trace and simplify region contours.
			//

			// Create contours.
			ContourSet cset = RecastContour.buildContours(ctx, chf, cfg.maxSimplificationError, cfg.maxEdgeLen, RecastConstants.RC_CONTOUR_TESS_WALL_EDGES);

			//
			// Step 6. Build polygons mesh from contours.
			//

			PolyMesh pmesh = RecastMesh.buildPolyMesh(ctx, cset, cfg.maxVertsPerPoly);

			//
			// Step 7. Create detail mesh which allows to access approximate height
			// on each polygon.
			//

			PolyMeshDetail dmesh = RecastMeshDetail.buildPolyMeshDetail(ctx, pmesh, chf, cfg.detailSampleDist, cfg.detailSampleMaxError);
			return new RecastBuilderResult(this, pmesh, dmesh);
		}

		private CompactHeightfield buildCompactHeightfield(InputGeom geom, RecastBuilderConfig bcfg, Context ctx)
		{
			RecastConfig cfg = bcfg.cfg;
			//
			// Step 2. Rasterize input polygon soup.
			//

			// Allocate voxel heightfield where we rasterize our input data to.
			Heightfield solid = new Heightfield(bcfg.width, bcfg.height, bcfg.bmin, bcfg.bmax, cfg.cs, cfg.ch);

			// Allocate array that can hold triangle area types.
			// If you have multiple meshes you need to process, allocate
			// and array which can hold the max number of triangles you need to
			// process.

			// Find triangles which are walkable based on their slope and rasterize
			// them.
			// If your input data is multiple meshes, you can transform them here,
			// calculate
			// the are type for each of the meshes and rasterize them.
			float[] verts = geom.Verts;
			bool tiled = cfg.tileSize > 0;
			int totaltris = 0;
			if (tiled)
			{
				ChunkyTriMesh chunkyMesh = geom.ChunkyMesh;
				float[] tbmin = new float[2];
				float[] tbmax = new float[2];
				tbmin[0] = bcfg.bmin[0];
				tbmin[1] = bcfg.bmin[2];
				tbmax[0] = bcfg.bmax[0];
				tbmax[1] = bcfg.bmax[2];
				List<ChunkyTriMeshNode> nodes = chunkyMesh.getChunksOverlappingRect(tbmin, tbmax);
				foreach (ChunkyTriMeshNode node in nodes)
				{
					int[] tris = node.tris;
					int ntris = tris.Length / 3;
					totaltris += ntris;
					int[] m_triareas = Recast.markWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, tris, ntris);
					RecastRasterization.rasterizeTriangles(ctx, verts, tris, m_triareas, ntris, solid, cfg.walkableClimb);
				}
			}
			else
			{
				int[] tris = geom.Tris;
				int ntris = tris.Length / 3;
				int[] m_triareas = Recast.markWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, tris, ntris);
				totaltris = ntris;
				RecastRasterization.rasterizeTriangles(ctx, verts, tris, m_triareas, ntris, solid, cfg.walkableClimb);
			}
			//
			// Step 3. Filter walkables surfaces.
			//

			// Once all geometry is rasterized, we do initial pass of filtering to
			// remove unwanted overhangs caused by the conservative rasterization
			// as well as filter spans where the character cannot possibly stand.
			RecastFilter.filterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, solid);
			RecastFilter.filterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, solid);
			RecastFilter.filterWalkableLowHeightSpans(ctx, cfg.walkableHeight, solid);

			//
			// Step 4. Partition walkable surface to simple regions.
			//

			// Compact the heightfield so that it is faster to handle from now on.
			// This will result more cache coherent data as well as the neighbours
			// between walkable cells will be calculated.
			CompactHeightfield chf = Recast.buildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, solid);

			// Erode the walkable area by agent radius.
			RecastArea.erodeWalkableArea(ctx, cfg.walkableRadius, chf);
			// (Optional) Mark areas.
			foreach (ConvexVolume vol in geom.ConvexVolumes)
			{
				RecastArea.markConvexPolyArea(ctx, vol.verts, vol.hmin, vol.hmax, vol.area, chf);
			}
			return chf;
		}

		public virtual HeightfieldLayerSet buildLayers(InputGeom geom, RecastBuilderConfig cfg)
		{
			Context ctx = new Context();
			CompactHeightfield chf = buildCompactHeightfield(geom, cfg, ctx);
			return RecastLayers.buildHeightfieldLayers(ctx, chf, cfg.borderSize, cfg.cfg.walkableHeight);
		}

	}

}