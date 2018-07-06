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


	public class NavMesh
	{

		internal const int DT_VERTS_PER_POLYGON = 6;

		internal static int DT_SALT_BITS = 16;
		internal static int DT_TILE_BITS = 28;
		internal static int DT_POLY_BITS = 20;

		/// A flag that indicates that an entity links to an external entity.
		/// (E.g. A polygon edge is a portal that links to another polygon.)
		internal static int DT_EXT_LINK = 0x8000;

		/// A value that indicates the entity does not link to anything.
		internal static int DT_NULL_LINK = unchecked((int)0xffffffff);

		/// A flag that indicates that an off-mesh connection can be traversed in
		/// both directions. (Is bidirectional.)
		internal static int DT_OFFMESH_CON_BIDIR = 1;

		/// The maximum number of user defined area ids.
		internal static int DT_MAX_AREAS = 64;

		/// Limit raycasting during any angle pahfinding
		/// The limit is given as a multiple of the character radius
		internal static float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

		private readonly NavMeshParams m_params; /// < Current initialization params. TODO: do not store this info twice.
		private float[] m_orig; /// < Origin of the tile (0,0)
		// float m_orig[3]; ///< Origin of the tile (0,0)
		internal float m_tileWidth, m_tileHeight; /// < Dimensions of each tile.
		internal int m_maxTiles; /// < Max number of tiles.
		private readonly int m_tileLutSize; /// < Tile hash lookup size (must be pot).
		private readonly int m_tileLutMask; /// < Tile hash lookup mask.
		private readonly MeshTile[] m_posLookup; /// < Tile hash lookup.
		internal MeshTile m_nextFree; /// < Freelist of tiles.
		private readonly MeshTile[] m_tiles; /// < List of tiles.
		/// <summary>
		/// The maximum number of vertices per navigation polygon. </summary>
		private readonly int m_maxVertPerPoly;
		private int m_tileCount;


		/// <summary>
		/// The maximum number of tiles supported by the navigation mesh.
		/// </summary>
		/// <returns> The maximum number of tiles supported by the navigation mesh. </returns>
		public virtual int MaxTiles
		{
			get
			{
				return m_maxTiles;
			}
		}

		/// <summary>
		/// Returns tile in the tile array.
		/// </summary>
		public virtual MeshTile getTile(int i)
		{
			return m_tiles[i];
		}

		/// <summary>
		/// Gets the polygon reference for the tile's base polygon.
		/// </summary>
		/// <param name="tile">
		///            The tile. </param>
		/// <returns> The polygon reference for the base polygon in the specified tile. </returns>
		public virtual long getPolyRefBase(MeshTile tile)
		{
			if (tile == null)
			{
				return 0;
			}
			int it = tile.index;
			return encodePolyId(tile.salt, it, 0);
		}

		/// <summary>
		/// Derives a standard polygon reference.
		/// 
		/// @note This function is generally meant for internal use only. </summary>
		/// <param name="salt">
		///            The tile's salt value. </param>
		/// <param name="it">
		///            The index of the tile. </param>
		/// <param name="ip">
		///            The index of the polygon within the tile. </param>
		/// <returns> encoded polygon reference </returns>
		public static long encodePolyId(int salt, int it, int ip)
		{
			return (((long) salt) << (DT_POLY_BITS + DT_TILE_BITS)) | ((long) it << DT_POLY_BITS) | ip;
		}

		/// Decodes a standard polygon reference.
		/// @note This function is generally meant for internal use only.
		/// @param[in] ref The polygon reference to decode.
		/// @param[out] salt The tile's salt value.
		/// @param[out] it The index of the tile.
		/// @param[out] ip The index of the polygon within the tile.
		/// @see #encodePolyId
		internal static void decodePolyId(int[] @out, long @ref)
		{
			long saltMask = (1L << DT_SALT_BITS) - 1;
			long tileMask = (1L << DT_TILE_BITS) - 1;
			long polyMask = (1L << DT_POLY_BITS) - 1;
			@out[0] = (int)((@ref >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
			@out[1] = (int)((@ref >> DT_POLY_BITS) & tileMask);
			@out[2] = (int)(@ref & polyMask);
		}

		/// Extracts a tile's salt value from the specified polygon reference.
		/// @note This function is generally meant for internal use only.
		/// @param[in] ref The polygon reference.
		/// @see #encodePolyId
		internal static int decodePolyIdSalt(long @ref)
		{
			long saltMask = (1L << DT_SALT_BITS) - 1;
			return (int)((@ref >> (DT_POLY_BITS + DT_TILE_BITS)) & saltMask);
		}

		/// Extracts the tile's index from the specified polygon reference.
		/// @note This function is generally meant for internal use only.
		/// @param[in] ref The polygon reference.
		/// @see #encodePolyId
		internal static int decodePolyIdTile(long @ref)
		{
			long tileMask = (1L << DT_TILE_BITS) - 1;
			return (int)((@ref >> DT_POLY_BITS) & tileMask);
		}

		/// Extracts the polygon's index (within its tile) from the specified
		/// polygon reference.
		/// @note This function is generally meant for internal use only.
		/// @param[in] ref The polygon reference.
		/// @see #encodePolyId
		internal static int decodePolyIdPoly(long @ref)
		{
			long polyMask = (1L << DT_POLY_BITS) - 1;
			return (int)(@ref & polyMask);
		}

		private int allocLink(MeshTile tile)
		{
			if (tile.linksFreeList == DT_NULL_LINK)
			{
				Link link = new Link();
				link.next = DT_NULL_LINK;
				tile.links.Add(link);
				return tile.links.Count - 1;
			}
			int link1 = tile.linksFreeList;
            tile.linksFreeList = tile.links[link1].next;
            return link1;
		}

		private void freeLink(MeshTile tile, int link)
		{
			tile.links[link].next = tile.linksFreeList;
			tile.linksFreeList = link;
		}

		/// <summary>
		/// Calculates the tile grid location for the specified world position.
		/// </summary>
		/// <param name="pos">
		///            The world position for the query. [(x, y, z)] </param>
		/// <returns> 2-element int array with (tx,ty) tile location </returns>
		public virtual void calcTileLoc(int[] @out, float[] pos)
		{
			int tx = (int) Math.Floor((pos[0] - m_orig[0]) / m_tileWidth);
			int ty = (int) Math.Floor((pos[2] - m_orig[2]) / m_tileHeight);
			@out[0] = tx;
			@out[1] = ty;
			//return new int[] { tx, ty };
		}


		private readonly int[] tpbrsaltitip = new int[3];

		public virtual Status getTileAndPolyByRef(long @ref, ref MeshTile tile, ref Poly poly)
		{
			if (@ref == 0)
			{
				//throw new IllegalArgumentException("ref = 0");
				return Status.FAILURE;
			}
			decodePolyId(tpbrsaltitip, @ref);
			int salt = tpbrsaltitip[0];
			int it = tpbrsaltitip[1];
			int ip = tpbrsaltitip[2];
			if (it >= m_maxTiles)
			{
				//throw new IllegalArgumentException("tile > m_maxTiles");
				return Status.FAILURE;
			}
			if (m_tiles[it].salt != salt || m_tiles[it].data.header == null)
			{
				//throw new IllegalArgumentException("Invalid salt or header");
				return Status.FAILURE;
			}
			if (ip >= m_tiles[it].data.header.polyCount)
			{
				//throw new IllegalArgumentException("poly > polyCount");
				return Status.FAILURE;
			}

            tile = m_tiles[it];
            poly = m_tiles[it].data.polys[ip];

			return Status.SUCCSESS;
		}


		private readonly int[] tpbrusaltitip = new int[3];
		/// @par
			///
			/// @warning Only use this function if it is known that the provided polygon
			/// reference is valid. This function is faster than #getTileAndPolyByRef,
			/// but
			/// it does not validate the reference.
        internal virtual void getTileAndPolyByRefUnsafe(long @ref, ref MeshTile tile, ref Poly poly)
		{
			decodePolyId(tpbrusaltitip, @ref);
			int it = tpbrusaltitip[1];
			int ip = tpbrusaltitip[2];
            tile = m_tiles[it];
            poly = m_tiles[it].data.polys[ip];
		}

		private readonly int[] vprsaltitip = new int[3];
		internal virtual bool isValidPolyRef(long @ref)
		{
			if (@ref == 0)
			{
				return false;
			}
			/*int[] saltitip = decodePolyId(ref);
			int salt = saltitip[0];
			int it = saltitip[1];
			int ip = saltitip[2];*/
			decodePolyId(vprsaltitip, @ref);
			int salt = vprsaltitip[0];
			int it = vprsaltitip[1];
			int ip = vprsaltitip[2];
			if (it >= m_maxTiles)
			{
				return false;
			}
			if (m_tiles[it].salt != salt || m_tiles[it].data.header == null)
			{
				return false;
			}
			if (ip >= m_tiles[it].data.header.polyCount)
			{
				return false;
			}
			return true;
		}

		public virtual NavMeshParams Params
		{
			get
			{
				return m_params;
			}
		}

		public NavMesh(MeshData data, int maxVertsPerPoly, int flags) : this(getNavMeshParams(data), maxVertsPerPoly)
		{
			addTile(data, flags, 0);
		}

		public NavMesh(NavMeshParams @params, int maxVertsPerPoly)
		{
			this.m_params = @params;
			m_orig = @params.orig;
			m_tileWidth = @params.tileWidth;
			m_tileHeight = @params.tileHeight;
			// Init tiles
			m_maxTiles = @params.maxTiles;
			m_maxVertPerPoly = maxVertsPerPoly;
			int lutsize = DetourCommon.nextPow2(@params.maxTiles / 4);
			if (lutsize == 0)
			{
				lutsize = 1;
			}
			m_tileLutSize = lutsize;
			m_tileLutMask = m_tileLutSize - 1;
			m_tiles = new MeshTile[m_maxTiles];
			m_posLookup = new MeshTile[m_tileLutSize];
			m_nextFree = null;
			for (int i = m_maxTiles - 1; i >= 0; --i)
			{
				m_tiles[i] = new MeshTile(i);
				m_tiles[i].salt = 1;
				m_tiles[i].next = m_nextFree;
				m_nextFree = m_tiles[i];
			}

		}

		private static NavMeshParams getNavMeshParams(MeshData data)
		{
			NavMeshParams @params = new NavMeshParams();
            DetourCommon.vCopy(@params.orig, data.header.bmin);
			@params.tileWidth = data.header.bmax[0] - data.header.bmin[0];
			@params.tileHeight = data.header.bmax[2] - data.header.bmin[2];
			@params.maxTiles = 1;
			@params.maxPolys = data.header.polyCount;
			return @params;
		}

		// TODO: These methods are duplicates from dtNavMeshQuery, but are needed
		// for off-mesh connection finding.

		private List<long> queryPolygonsInTile(MeshTile tile, float[] qmin, float[] qmax)
		{
			List<long> polys = new List<long>();
			if (tile.data.bvTree != null)
			{
				int nodeIndex = 0;
				float[] tbmin = tile.data.header.bmin;
				float[] tbmax = tile.data.header.bmax;
				float qfac = tile.data.header.bvQuantFactor;
				// Calculate quantized box
				int[] bmin = new int[3];
				int[] bmax = new int[3];
				// dtClamp query box to world box.
                float minx = DetourCommon.clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
                float miny = DetourCommon.clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
                float minz = DetourCommon.clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
                float maxx = DetourCommon.clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
                float maxy = DetourCommon.clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
                float maxz = DetourCommon.clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
				// Quantize
				bmin[0] = (int)(qfac * minx) & 0xfffe;
				bmin[1] = (int)(qfac * miny) & 0xfffe;
				bmin[2] = (int)(qfac * minz) & 0xfffe;
				bmax[0] = (int)(qfac * maxx + 1) | 1;
				bmax[1] = (int)(qfac * maxy + 1) | 1;
				bmax[2] = (int)(qfac * maxz + 1) | 1;

				// Traverse tree
				long @base = getPolyRefBase(tile);
				int end = tile.data.header.bvNodeCount;
				while (nodeIndex < end)
				{
					BVNode node = tile.data.bvTree[nodeIndex];
                    bool overlap = DetourCommon.overlapQuantBounds(bmin, bmax, node.bmin, node.bmax);
					bool isLeafNode = node.i >= 0;

					if (isLeafNode && overlap)
					{
						polys.Add(@base | node.i);
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
				float[] bmin = new float[3];
				float[] bmax = new float[3];
				long @base = getPolyRefBase(tile);
				for (int i = 0; i < tile.data.header.polyCount; ++i)
				{
					Poly p = tile.data.polys[i];
					// Do not return off-mesh connection polygons.
					if (p.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
					{
						continue;
					}
					// Calc polygon bounds.
					int v = p.verts[0] * 3;
                    DetourCommon.vCopy(bmin, tile.data.verts, v);
                    DetourCommon.vCopy(bmax, tile.data.verts, v);
					for (int j = 1; j < p.vertCount; ++j)
					{
						v = p.verts[j] * 3;
                        DetourCommon.vMin(bmin, tile.data.verts, v);
                        DetourCommon.vMax(bmax, tile.data.verts, v);
					}
                    if (DetourCommon.overlapBounds(qmin, qmax, bmin, bmax))
					{
						polys.Add(@base | i);
					}
				}
				return polys;
			}
		}

		/// Adds a tile to the navigation mesh.
		/// @param[in] data Data for the new tile mesh. (See: #dtCreateNavMeshData)
		/// @param[in] dataSize Data size of the new tile mesh.
		/// @param[in] flags Tile flags. (See: #dtTileFlags)
		/// @param[in] lastRef The desired reference for the tile. (When reloading a
		/// tile.) [opt] [Default: 0]
		/// @param[out] result The tile reference. (If the tile was succesfully
		/// added.) [opt]
		/// @return The status flags for the operation.
		/// @par
		///
		/// The add operation will fail if the data is in the wrong format, the
		/// allocated tile
		/// space is full, or there is a tile already at the specified reference.
		///
		/// The lastRef parameter is used to restore a tile with the same tile
		/// reference it had previously used. In this case the #dtPolyRef's for the
		/// tile will be restored to the same values they were before the tile was
		/// removed.
		///
		/// @see dtCreateNavMeshData, #removeTile
		public virtual long addTile(MeshData data, int flags, long lastRef)
		{
			// Make sure the data is in right format.
			MeshHeader header = data.header;

			// Make sure the location is free.
			if (getTileAt(header.x, header.y, header.layer) != null)
			{
				throw new Exception("Tile already exists");
			}

			// Allocate a tile.
			MeshTile tile = null;
			if (lastRef == 0)
			{
				if (m_nextFree != null)
				{
					tile = m_nextFree;
					m_nextFree = tile.next;
					tile.next = null;
					m_tileCount++;
				}
			}
			else
			{
				// Try to relocate the tile to specific index with same salt.
				int tileIndex = decodePolyIdTile(lastRef);
				if (tileIndex >= m_maxTiles)
				{
					throw new Exception("Tile index too high");
				}
				// Try to find the specific tile id from the free list.
				MeshTile target = m_tiles[tileIndex];
				MeshTile prev = null;
				tile = m_nextFree;
				while (tile != null && tile != target)
				{
					prev = tile;
					tile = tile.next;
				}
				// Could not find the correct location.
				if (tile != target)
				{
					throw new Exception("Could not find tile");
				}
				// Remove from freelist
				if (prev == null)
				{
					m_nextFree = tile.next;
				}
				else
				{
					prev.next = tile.next;
				}

				// Restore salt.
				tile.salt = decodePolyIdSalt(lastRef);
			}

			// Make sure we could allocate a tile.
			if (tile == null)
			{
				throw new Exception("Could not allocate a tile");
			}

			tile.data = data;
			tile.flags = flags;
			tile.links.Clear();

			// Insert tile into the position lut.
			int h = computeTileHash(header.x, header.y, m_tileLutMask);
			tile.next = m_posLookup[h];
			m_posLookup[h] = tile;

	// ---------- the same cpp VS java, Jan 14, 2016 14:15
	/*		try
			{
				FileWriter fw = new FileWriter("test_java.log", true);
				for (int iter = 0; iter < header.vertCount; iter++)
				{
					String result = String.format("vert[%d] = %.1f\n", iter, tile.data.verts[iter]);
					fw.write(result);
				}
				for (int iter = 0; iter < header.detailVertCount; iter++)
				{
					String result = String.format("detailV[%d] = (%.1f, %.1f, %.1f)\n", iter, 
							tile.data.detailVerts[3*iter], tile.data.detailVerts[3*iter+1], tile.data.detailVerts[3*iter+2]);
					fw.write(result);
				}
				for (int iter = 0; iter < header.detailTriCount; iter++)
				{
					String result = String.format("detailT[%d] = (%d, %d, %d)\n", iter, 
							tile.data.detailTris[3*iter], tile.data.detailTris[3*iter+1], tile.data.detailTris[3*iter+2]);
					fw.write(result);
				}
				fw.close();
			}
			catch (Exception e)
			{
				e.printStackTrace();
			}
	*/
			// Patch header pointers.

			// If there are no items in the bvtree, reset the tree pointer.
			if (tile.data.bvTree != null && tile.data.bvTree.Length == 0)
			{
				tile.data.bvTree = null;
			}

			// Init tile.

	/*		try
			{
				FileWriter fw = new FileWriter("test_java.log", true);
				for (int iter = 0; iter < tile.data.header.bvNodeCount; iter++)
				{
					String result = String.format("node.i = %d\n", tile.data.bvTree[iter].i);
					fw.write(result);
				}
				fw.close();
			}
			catch (Exception e)
			{
				e.printStackTrace();
			}
	*/
			connectIntLinks(tile);
			baseOffMeshLinks(tile);

			// Connect with layers in current tile.
			IList<MeshTile> neis = getTilesAt(header.x, header.y);
			for (int j = 0; j < neis.Count; ++j)
			{
				if (neis[j] != tile)
				{
					connectExtLinks(tile, neis[j], -1);
					connectExtLinks(neis[j], tile, -1);
				}
				connectExtOffMeshLinks(tile, neis[j], -1);
				connectExtOffMeshLinks(neis[j], tile, -1);
			}

			// Connect with neighbour tiles.
			for (int i = 0; i < 8; ++i)
			{
				neis = getNeighbourTilesAt(header.x, header.y, i);
				for (int j = 0; j < neis.Count; ++j)
				{
					connectExtLinks(tile, neis[j], i);
                    connectExtLinks(neis[j], tile, DetourCommon.oppositeTile(i));
					connectExtOffMeshLinks(tile, neis[j], i);
                    connectExtOffMeshLinks(neis[j], tile, DetourCommon.oppositeTile(i));
				}
			}

	//		System.out.println("ref = " + getTileRef(tile));
	/*		try
			{
				FileWriter fw = new FileWriter("test_java.log", true);
				for (int iter = 0; iter < tile.data.header.bvNodeCount; iter++)
				{
					String result = String.format("node.i = %d\n", tile.data.bvTree[iter].i);
					fw.write(result);
				}
				fw.close();
			}
			catch (Exception e)
			{
				e.printStackTrace();
			}
	*/			return getTileRef(tile);
		}

		/// Removes the specified tile from the navigation mesh.
		/// @param[in] ref The reference of the tile to remove.
		/// @param[out] data Data associated with deleted tile.
		/// @param[out] dataSize Size of the data associated with deleted tile.
		/// @return The status flags for the operation.
		// dtStatus removeTile(dtTileRef ref, char** data, int* dataSize);
		/// @par
		///
		/// This function returns the data for the tile so that, if desired,
		/// it can be added back to the navigation mesh at a later point.
		///
		/// @see #addTile
		public virtual MeshData removeTile(long @ref)
		{
			if (@ref == 0)
			{
				return null;
			}
			int tileIndex = decodePolyIdTile(@ref);
			int tileSalt = decodePolyIdSalt(@ref);
			if (tileIndex >= m_maxTiles)
			{
				throw new Exception("Invalid tile index");
			}
			MeshTile tile = m_tiles[tileIndex];
			if (tile.salt != tileSalt)
			{
				throw new Exception("Invalid tile salt");
			}

			// Remove tile from hash lookup.
			int h = computeTileHash(tile.data.header.x, tile.data.header.y, m_tileLutMask);
			MeshTile prev = null;
			MeshTile cur = m_posLookup[h];
			while (cur != null)
			{
				if (cur == tile)
				{
					if (prev != null)
					{
						prev.next = cur.next;
					}
					else
					{
						m_posLookup[h] = cur.next;
					}
					break;
				}
				prev = cur;
				cur = cur.next;
			}

			// Remove connections to neighbour tiles.
			// Create connections with neighbour tiles.

			// Disconnect from other layers in current tile.
			IList<MeshTile> nneis = getTilesAt(tile.data.header.x, tile.data.header.y);
			foreach (MeshTile j in nneis)
			{
				if (j == tile)
				{
					continue;
				}
				unconnectLinks(j, tile);
			}

			// Disconnect from neighbour tiles.
			for (int i = 0; i < 8; ++i)
			{
				nneis = getNeighbourTilesAt(tile.data.header.x, tile.data.header.y, i);
				foreach (MeshTile j in nneis)
				{
					unconnectLinks(j, tile);
				}
			}
			MeshData data = tile.data;
			// Reset tile.
			tile.data = null;

			tile.flags = 0;
			tile.links.Clear();

			// Update salt, salt should never be zero.
			tile.salt = (tile.salt + 1) & ((1 << DT_SALT_BITS) - 1);
			if (tile.salt == 0)
			{
				tile.salt++;
			}

			// Add to free list.
			tile.next = m_nextFree;
			m_nextFree = tile;
			m_tileCount--;
			return data;
		}

		/// Builds internal polygons links for a tile.
		internal virtual void connectIntLinks(MeshTile tile)
		{
			if (tile == null)
			{
				return;
			}

			long @base = getPolyRefBase(tile);
			Poly poly;
			Link link;
			for (int i = 0; i < tile.data.header.polyCount; ++i)
			{
				poly = tile.data.polys[i];
				poly.firstLink = DT_NULL_LINK;

				if (poly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
				{
					continue;
				}

				// Build edge links backwards so that the links will be
				// in the linked list from lowest index to highest.
				for (int j = poly.vertCount - 1; j >= 0; --j)
				{
					// Skip hard and non-internal edges.
					if (poly.neis[j] == 0 || (poly.neis[j] & DT_EXT_LINK) != 0)
					{
						continue;
					}

					int idx = allocLink(tile);
					link = tile.links[idx];
					link.@ref = @base | (poly.neis[j] - 1);
					link.edge = j;
					link.side = 0xff;
					link.bmin = link.bmax = 0;
					// Add to linked list.
					link.next = poly.firstLink;
					poly.firstLink = idx;
				}
			}
		}

		internal virtual void unconnectLinks(MeshTile tile, MeshTile target)
		{
			if (tile == null || target == null)
			{
				return;
			}

			int targetNum = decodePolyIdTile(getTileRef(target));

			Poly poly;
			for (int i = 0; i < tile.data.header.polyCount; ++i)
			{
				poly = tile.data.polys[i];
				int j = poly.firstLink;
				int pj = DT_NULL_LINK;
				while (j != DT_NULL_LINK)
				{
					if (decodePolyIdTile(tile.links[j].@ref) == targetNum)
					{
						// Remove link.
						int nj = tile.links[j].next;
						if (pj == DT_NULL_LINK)
						{
							poly.firstLink = nj;
						}
						else
						{
							tile.links[pj].next = nj;
						}
						freeLink(tile, j);
						j = nj;
					}
					else
					{
						// Advance
						pj = j;
						j = tile.links[j].next;
					}
				}
			}
		}

		internal virtual void connectExtLinks(MeshTile tile, MeshTile target, int side)
		{
			if (tile == null)
			{
				return;
			}

			Poly poly;
			int nv;
			int va;
			int vb;
			long[] nei;
			float[] neia;
			int nnei;
			int idx;
			Link link;
			// Connect border links.
			for (int i = 0; i < tile.data.header.polyCount; ++i)
			{
				poly = tile.data.polys[i];

				// Create new links.
				// short m = DT_EXT_LINK | (short)side;

				nv = poly.vertCount;
				for (int j = 0; j < nv; ++j)
				{
					// Skip non-portal edges.
					if ((poly.neis[j] & DT_EXT_LINK) == 0)
					{
						continue;
					}

					int dir = poly.neis[j] & 0xff;
					if (side != -1 && dir != side)
					{
						continue;
					}

					// Create new links
					va = poly.verts[j] * 3;
					vb = poly.verts[(j + 1) % nv] * 3;
                    Tupple3<long[], float[], int> connectedPolys = findConnectingPolys(tile.data.verts, va, vb, target, DetourCommon.oppositeTile(dir), 4);
					nei = connectedPolys.first;
					neia = connectedPolys.second;
					nnei = connectedPolys.third;
					for (int k = 0; k < nnei; ++k)
					{
						idx = allocLink(tile);
						link = tile.links[idx];
						link.@ref = nei[k];
						link.edge = j;
						link.side = dir;

						link.next = poly.firstLink;
						poly.firstLink = idx;

						// Compress portal limits to a byte value.
						if (dir == 0 || dir == 4)
						{
							float tmin = (neia[k * 2 + 0] - tile.data.verts[va + 2]) / (tile.data.verts[vb + 2] - tile.data.verts[va + 2]);
							float tmax = (neia[k * 2 + 1] - tile.data.verts[va + 2]) / (tile.data.verts[vb + 2] - tile.data.verts[va + 2]);
							if (tmin > tmax)
							{
								float temp = tmin;
								tmin = tmax;
								tmax = temp;
							}
							link.bmin = (int)(DetourCommon.clamp(tmin, 0.0f, 1.0f) * 255.0f);
                            link.bmax = (int)(DetourCommon.clamp(tmax, 0.0f, 1.0f) * 255.0f);
						}
						else if (dir == 2 || dir == 6)
						{
							float tmin = (neia[k * 2 + 0] - tile.data.verts[va]) / (tile.data.verts[vb] - tile.data.verts[va]);
							float tmax = (neia[k * 2 + 1] - tile.data.verts[va]) / (tile.data.verts[vb] - tile.data.verts[va]);
							if (tmin > tmax)
							{
								float temp = tmin;
								tmin = tmax;
								tmax = temp;
							}
                            link.bmin = (int)(DetourCommon.clamp(tmin, 0.0f, 1.0f) * 255.0f);
                            link.bmax = (int)(DetourCommon.clamp(tmax, 0.0f, 1.0f) * 255.0f);
						}
					}
				}
			}
		}

		internal virtual void connectExtOffMeshLinks(MeshTile tile, MeshTile target, int side)
		{
			if (tile == null)
			{
				return;
			}

			// Connect off-mesh links.
			// We are interested on links which land from target tile to this tile.
            int oppositeSide = (side == -1) ? 0xff : DetourCommon.oppositeTile(side);
			OffMeshConnection targetCon;
			Poly targetPoly;
			for (int i = 0; i < target.data.header.offMeshConCount; ++i)
			{
				targetCon = target.data.offMeshCons[i];
				if (targetCon.side != oppositeSide)
				{
					continue;
				}

				targetPoly = target.data.polys[targetCon.poly];
				// Skip off-mesh connections which start location could not be
				// connected at all.
				if (targetPoly.firstLink == DT_NULL_LINK)
				{
					continue;
				}

				float[] ext = new float[] {targetCon.rad, target.data.header.walkableClimb, targetCon.rad};

				// Find polygon to connect to.
				float[] p = new float[3];
				p[0] = targetCon.pos[3];
				p[1] = targetCon.pos[4];
				p[2] = targetCon.pos[5];
				FindNearestPolyResult nearest = findNearestPolyInTile(tile, p, ext);
				long @ref = nearest.NearestRef;
				if (@ref == 0)
				{
					continue;
				}
				float[] nearestPt = nearest.NearestPos;
				// findNearestPoly may return too optimistic results, further check
				// to make sure.

                if (DetourCommon.sqr(nearestPt[0] - p[0]) + DetourCommon.sqr(nearestPt[2] - p[2]) > DetourCommon.sqr(targetCon.rad))
				{
					continue;
				}
				// Make sure the location is on current mesh.
				target.data.verts[targetPoly.verts[1] * 3] = nearestPt[0];
				target.data.verts[targetPoly.verts[1] * 3 + 1] = nearestPt[1];
				target.data.verts[targetPoly.verts[1] * 3 + 2] = nearestPt[2];

				// Link off-mesh connection to target poly.
				int idx = allocLink(target);
				Link link = target.links[idx];
				link.@ref = @ref;
				link.edge = 1;
				link.side = oppositeSide;
				link.bmin = link.bmax = 0;
				// Add to linked list.
				link.next = targetPoly.firstLink;
				targetPoly.firstLink = idx;

				// Link target poly to off-mesh connection.
				if ((targetCon.flags & DT_OFFMESH_CON_BIDIR) != 0)
				{
					int tidx = allocLink(tile);
					int landPolyIdx = decodePolyIdPoly(@ref);
					Poly landPoly = tile.data.polys[landPolyIdx];
					link = tile.links[tidx];
					link.@ref = getPolyRefBase(target) | (targetCon.poly);
					link.edge = 0xff;
					link.side = (side == -1 ? 0xff : side);
					link.bmin = link.bmax = 0;
					// Add to linked list.
					link.next = landPoly.firstLink;
					landPoly.firstLink = tidx;
				}
			}
		}

		internal virtual Tupple3<long[], float[], int> findConnectingPolys(float[] verts, int va, int vb, MeshTile tile, int side, int maxcon)
		{
			if (tile == null)
			{
                return new Tupple3<long[], float[], int>(null, null, 0);
			}
			long[] con = new long[maxcon];
			float[] conarea = new float[maxcon * 2];
			float[] amin = new float[2];
			float[] amax = new float[2];
			calcSlabEndPoints(verts, va, vb, amin, amax, side);
			float apos = getSlabCoord(verts, va, side);

			// Remove links pointing to 'side' and compact the links array.
			float[] bmin = new float[2];
			float[] bmax = new float[2];
			int m = DT_EXT_LINK | side;
			int n = 0;
			long @base = getPolyRefBase(tile);

			for (int i = 0; i < tile.data.header.polyCount; ++i)
			{
				Poly poly = tile.data.polys[i];
				int nv = poly.vertCount;
				for (int j = 0; j < nv; ++j)
				{
					// Skip edges which do not point to the right side.
					if (poly.neis[j] != m)
					{
						continue;
					}
					int vc = poly.verts[j] * 3;
					int vd = poly.verts[(j + 1) % nv] * 3;
					float bpos = getSlabCoord(tile.data.verts, vc, side);
					// Segments are not close enough.
					if (Math.Abs(apos - bpos) > 0.01f)
					{
						continue;
					}

					// Check if the segments touch.
					calcSlabEndPoints(tile.data.verts, vc, vd, bmin, bmax, side);

					if (!overlapSlabs(amin, amax, bmin, bmax, 0.01f, tile.data.header.walkableClimb))
					{
						continue;
					}

					// Add return value.
					if (n < maxcon)
					{
						conarea[n * 2 + 0] = Math.Max(amin[0], bmin[0]);
						conarea[n * 2 + 1] = Math.Min(amax[0], bmax[0]);
						con[n] = @base | i;
						n++;
					}
					break;
				}
			}
			return new Tupple3<long[], float[], int>(con, conarea, n);
		}

		internal static float getSlabCoord(float[] verts, int va, int side)
		{
			if (side == 0 || side == 4)
			{
				return verts[va];
			}
			else if (side == 2 || side == 6)
			{
				return verts[va + 2];
			}
			return 0;
		}

		internal static void calcSlabEndPoints(float[] verts, int va, int vb, float[] bmin, float[] bmax, int side)
		{
			if (side == 0 || side == 4)
			{
				if (verts[va + 2] < verts[vb + 2])
				{
					bmin[0] = verts[va + 2];
					bmin[1] = verts[va + 1];
					bmax[0] = verts[vb + 2];
					bmax[1] = verts[vb + 1];
				}
				else
				{
					bmin[0] = verts[vb + 2];
					bmin[1] = verts[vb + 1];
					bmax[0] = verts[va + 2];
					bmax[1] = verts[va + 1];
				}
			}
			else if (side == 2 || side == 6)
			{
				if (verts[va + 0] < verts[vb + 0])
				{
					bmin[0] = verts[va + 0];
					bmin[1] = verts[va + 1];
					bmax[0] = verts[vb + 0];
					bmax[1] = verts[vb + 1];
				}
				else
				{
					bmin[0] = verts[vb + 0];
					bmin[1] = verts[vb + 1];
					bmax[0] = verts[va + 0];
					bmax[1] = verts[va + 1];
				}
			}
		}

		internal virtual bool overlapSlabs(float[] amin, float[] amax, float[] bmin, float[] bmax, float px, float py)
		{
			// Check for horizontal overlap.
			// The segment is shrunken a little so that slabs which touch
			// at end points are not connected.
			float minx = Math.Max(amin[0] + px, bmin[0] + px);
			float maxx = Math.Min(amax[0] - px, bmax[0] - px);
			if (minx > maxx)
			{
				return false;
			}

			// Check vertical overlap.
			float ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
			float ak = amin[1] - ad * amin[0];
			float bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
			float bk = bmin[1] - bd * bmin[0];
			float aminy = ad * minx + ak;
			float amaxy = ad * maxx + ak;
			float bminy = bd * minx + bk;
			float bmaxy = bd * maxx + bk;
			float dmin = bminy - aminy;
			float dmax = bmaxy - amaxy;

			// Crossing segments always overlap.
			if (dmin * dmax < 0)
			{
				return true;
			}

			// Check for overlap at endpoints.
			float thr = (py * 2) * (py * 2);
			if (dmin * dmin <= thr || dmax * dmax <= thr)
			{
				return true;
			}

			return false;
		}

		/// <summary>
		/// Builds internal polygons links for a tile.
		/// </summary>
		/// <param name="tile"> </param>
		internal virtual void baseOffMeshLinks(MeshTile tile)
		{
			if (tile == null)
			{
				return;
			}

			long @base = getPolyRefBase(tile);

			// Base off-mesh connection start points.
			for (int i = 0; i < tile.data.header.offMeshConCount; ++i)
			{
				OffMeshConnection con = tile.data.offMeshCons[i];
				Poly poly = tile.data.polys[con.poly];

				float[] ext = new float[] {con.rad, tile.data.header.walkableClimb, con.rad};

				// Find polygon to connect to.
				FindNearestPolyResult nearestPoly = findNearestPolyInTile(tile, con.pos, ext);
				long @ref = nearestPoly.NearestRef;
				if (@ref == 0)
				{
					continue;
				}
				float[] p = con.pos; // First vertex
				float[] nearestPt = nearestPoly.NearestPos;
				// findNearestPoly may return too optimistic results, further check
				// to make sure.
				float dx = nearestPt[0] - p[0];
				float dz = nearestPt[2] - p[2];
				float dr = con.rad;
				if (dx * dx + dz * dz > dr * dr)
				{
					continue;
				}
				// Make sure the location is on current mesh.
				Array.Copy(nearestPoly.NearestPos, 0, tile.data.verts, poly.verts[0] * 3, 3);

				// Link off-mesh connection to target poly.
				int idx = allocLink(tile);
				Link link = tile.links[idx];
				link.@ref = @ref;
				link.edge = 0;
				link.side = 0xff;
				link.bmin = link.bmax = 0;
				// Add to linked list.
				link.next = poly.firstLink;
				poly.firstLink = idx;

				// Start end-point is always connect back to off-mesh connection.
				int tidx = allocLink(tile);
				int landPolyIdx = decodePolyIdPoly(@ref);
				Poly landPoly = tile.data.polys[landPolyIdx];
				link = tile.links[tidx];
				link.@ref = @base | (con.poly);
				link.edge = 0xff;
				link.side = 0xff;
				link.bmin = link.bmax = 0;
				// Add to linked list.
				link.next = landPoly.firstLink;
				landPoly.firstLink = tidx;
			}
		}

		/// <summary>
		/// Returns closest point on polygon.
		/// </summary>
		/// <param name="ref"> </param>
		/// <param name="pos">
		/// @return </param>
		internal virtual ClosesPointOnPolyResult closestPointOnPoly(long @ref, float[] pos)
		{
            MeshTile tile = null;
            Poly poly = null;
			getTileAndPolyByRefUnsafe(@ref, ref tile, ref poly);
			

			// Off-mesh connections don't have detail polygons.
			if (poly.Type == Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				int v0 = poly.verts[0] * 3;
				int v1 = poly.verts[1] * 3;
				float d0 = DetourCommon.vDist(pos, tile.data.verts, v0);
                float d1 = DetourCommon.vDist(pos, tile.data.verts, v1);
				float u = d0 / (d0 + d1);
				float[] closest = new float[3];
                DetourCommon.vLerp(closest, tile.data.verts, v0, v1, u);
				return new ClosesPointOnPolyResult(false, closest);
			}

			// Clamp point to be inside the polygon.
			float[] verts = new float[m_maxVertPerPoly * 3];
			float[] edged = new float[m_maxVertPerPoly];
			float[] edget = new float[m_maxVertPerPoly];
			int nv = poly.vertCount;
			for (int i = 0; i < nv; ++i)
			{
				Array.Copy(tile.data.verts, poly.verts[i] * 3, verts, i * 3, 3);
			}

			bool posOverPoly = false;
			float[] closest1 = new float[3];
            DetourCommon.vCopy(closest1, pos);
			if (!distancePtPolyEdgesSqr(pos, verts, nv, edged, edget))
			{
				// Point is outside the polygon, dtClamp to nearest edge.
				float dmin = float.MaxValue;
				int imin = -1;
				for (int i = 0; i < nv; ++i)
				{
					if (edged[i] < dmin)
					{
						dmin = edged[i];
						imin = i;
					}
				}
				int va = imin * 3;
				int vb = ((imin + 1) % nv) * 3;
                DetourCommon.vLerp(closest1, verts, va, vb, edget[imin]);
				posOverPoly = false;
			}
			else
			{
				posOverPoly = true;
			}

			// Find height at the location.
			int ip = poly.index;
			if (tile.data.detailMeshes != null && tile.data.detailMeshes.Length > ip)
			{
				PolyDetail pd = tile.data.detailMeshes[ip];
				//VectorPtr posV = new VectorPtr(pos);
				for (int j = 0; j < pd.triCount; ++j)
				{
					int t = (pd.triBase + j) * 4;
					object[] v = new object[3];
					int[] indexs = new int[3];
					for (int k = 0; k < 3; ++k)
					{
						if (tile.data.detailTris[t + k] < poly.vertCount)
						{
							v[k] = tile.data.verts;
							indexs[k] = poly.verts[tile.data.detailTris[t + k]] * 3;
							//v[k] = new VectorPtr(tile.data.verts, poly.verts[tile.data.detailTris[t + k]] * 3);
						}
						else
						{
							//v[k] = new VectorPtr(tile.data.detailVerts,
							//		(pd.vertBase + (tile.data.detailTris[t + k] - poly.vertCount)) * 3);
							v[k] = tile.data.detailVerts;
							indexs[k] = (pd.vertBase + (tile.data.detailTris[t + k] - poly.vertCount)) * 3;
						}

					}
					float h = 0;
					if (closestHeightPointTriangle(pos, 0, (float[])v[0], indexs[0], (float[])v[1], indexs[1], (float[])v[2], indexs[2], ref h))
					{
                        closest1[1] = h;
						break;
					}
				}
			}
            return new ClosesPointOnPolyResult(posOverPoly, closest1);
		}

		internal virtual FindNearestPolyResult findNearestPolyInTile(MeshTile tile, float[] center, float[] extents)
		{
			float[] nearestPt = null;
			float[] bmin = new float[3];
			float[] bmax = new float[3];
			DetourCommon.vSub(bmin, center, extents);
            DetourCommon.vAdd(bmax, center, extents);

			// Get nearby polygons from proximity grid.
			List<long> polys = queryPolygonsInTile(tile, bmin, bmax);

			// Find nearest polygon amongst the nearby polygons.
			long nearest = 0;
			float nearestDistanceSqr = float.MaxValue;
			float[] diff = new float[3];
			for (int i = 0; i < polys.Count; ++i)
			{
				long @ref = polys[i];
				float d;
				ClosesPointOnPolyResult cpp = closestPointOnPoly(@ref, center);
				bool posOverPoly = cpp.PosOverPoly;
				float[] closestPtPoly = cpp.Closest;

				// If a point is directly over a polygon and closer than
				// climb height, favor that instead of straight line nearest point.
                DetourCommon.vSub(diff, center, closestPtPoly);
				if (posOverPoly)
				{
					d = Math.Abs(diff[1]) - tile.data.header.walkableClimb;
					d = d > 0 ? d * d : 0;
				}
				else
				{
                    d = DetourCommon.vLenSqr(diff);
				}
				if (d < nearestDistanceSqr)
				{
					nearestPt = closestPtPoly;
					nearestDistanceSqr = d;
					nearest = @ref;
				}
			}
			return new FindNearestPolyResult(nearest, nearestPt);
		}

		internal virtual MeshTile getTileAt(int x, int y, int layer)
		{
			// Find tile based on hash.
			int h = computeTileHash(x, y, m_tileLutMask);
			MeshTile tile = m_posLookup[h];
			while (tile != null)
			{
				if (tile.data.header != null && tile.data.header.x == x && tile.data.header.y == y && tile.data.header.layer == layer)
				{
					return tile;
				}
				tile = tile.next;
			}
			return null;
		}

		internal virtual IList<MeshTile> getNeighbourTilesAt(int x, int y, int side)
		{
			int nx = x, ny = y;
			switch (side)
			{
			case 0:
				nx++;
				break;
			case 1:
				nx++;
				ny++;
				break;
			case 2:
				ny++;
				break;
			case 3:
				nx--;
				ny++;
				break;
			case 4:
				nx--;
				break;
			case 5:
				nx--;
				ny--;
				break;
			case 6:
				ny--;
				break;
			case 7:
				nx++;
				ny--;
				break;
			}
			return getTilesAt(nx, ny);
		}


		/// <summary>
		/// �ϰ�getTilesAtʵ�� </summary>
		/// <param name="x"> </param>
		/// <param name="y">
		/// @return </param>
		public virtual IList<MeshTile> getTilesAt(int x, int y)
		{
			IList<MeshTile> tiles = new List<MeshTile>();
			// Find tile based on hash.
			int h = computeTileHash(x, y, m_tileLutMask);
			MeshTile tile = m_posLookup[h];
			while (tile != null)
			{
				if (tile.data.header != null && tile.data.header.x == x && tile.data.header.y == y)
				{
					tiles.Add(tile);
				}
				tile = tile.next;
			}
			return tiles;
		}


		/// <summary>
		/// �°�getTilesAtʵ�� </summary>
		/// <param name="x"> </param>
		/// <param name="y"> </param>
		/// <param name="tiles"> </param>
		/// <param name="maxTiles">
		/// @return </param>
//JAVA TO C# CONVERTER WARNING: 'final' parameters are not available in .NET:
//ORIGINAL LINE: public int getTilesAt(int x, int y, MeshTile[] tiles, final int maxTiles)
		public virtual int getTilesAt(int x, int y, MeshTile[] tiles, int maxTiles)
		{
			int n = 0;
			// Find tile based on hash.
			int h = computeTileHash(x, y, m_tileLutMask);
			MeshTile tile = m_posLookup[h];
			while (tile != null)
			{
				if (tile.data.header != null && tile.data.header.x == x && tile.data.header.y == y)
				{
					tiles[n] = tile;
					n += 1;
				}
				tile = tile.next;
			}
			return n;
		}



		public virtual long getTileRefAt(int x, int y, int layer)
		{
			// Find tile based on hash.
			int h = computeTileHash(x, y, m_tileLutMask);
			MeshTile tile = m_posLookup[h];
			while (tile != null)
			{
				if (tile.data.header != null && tile.data.header.x == x && tile.data.header.y == y && tile.data.header.layer == layer)
				{
					return getTileRef(tile);
				}
				tile = tile.next;
			}
			return 0;
		}

		public virtual MeshTile getTileByRef(long @ref)
		{
			if (@ref == 0)
			{
				return null;
			}
			int tileIndex = decodePolyIdTile(@ref);
			int tileSalt = decodePolyIdSalt(@ref);
			if (tileIndex >= m_maxTiles)
			{
				return null;
			}
			MeshTile tile = m_tiles[tileIndex];
			if (tile.salt != tileSalt)
			{
				return null;
			}
			return tile;
		}

		public virtual long getTileRef(MeshTile tile)
		{
			if (tile == null)
			{
				return 0;
			}
			int it = tile.index;
			return encodePolyId(tile.salt, it, 0);
		}

		public static int computeTileHash(int x, int y, int mask)
		{
			int h1 = unchecked((int)0x8da6b343); // Large multiplicative constants;
			int h2 = unchecked((int)0xd8163841); // here arbitrarily chosen primes
			int n = h1 * x + h2 * y;
			return n & mask;
		}



		private readonly int[] omcpedsaltitip = new int[3];

		/// @par
		///
		/// Off-mesh connections are stored in the navigation mesh as special
		/// 2-vertex
		/// polygons with a single edge. At least one of the vertices is expected to
		/// be
		/// inside a normal polygon. So an off-mesh connection is "entered" from a
		/// normal polygon at one of its endpoints. This is the polygon identified
		/// by
		/// the prevRef parameter.
		public virtual bool getOffMeshConnectionPolyEndPoints(long prevRef, long polyRef, ref float[] startPos, ref float[] endPos)
		{
			if (polyRef == 0)
			{
				throw new System.ArgumentException("polyRef = 0");
			}

			// Get current polygon
			/*int[] saltitip = decodePolyId(polyRef);
			int salt = saltitip[0];
			int it = saltitip[1];
			int ip = saltitip[2];*/
			decodePolyId(omcpedsaltitip, polyRef);
			int salt = omcpedsaltitip[0];
			int it = omcpedsaltitip[1];
			int ip = omcpedsaltitip[2];
			if (it >= m_maxTiles)
			{
				throw new System.ArgumentException("Invalid tile ID > max tiles");
			}
			if (m_tiles[it].salt != salt || m_tiles[it].data.header == null)
			{
				throw new System.ArgumentException("Invalid salt or missing tile header");
			}
			MeshTile tile = m_tiles[it];
			if (ip >= tile.data.header.polyCount)
			{
				throw new System.ArgumentException("Invalid poly ID > poly count");
			}
			Poly poly = tile.data.polys[ip];

			// Make sure that the current poly is indeed off-mesh link.
			if (poly.Type != Poly.DT_POLYTYPE_OFFMESH_CONNECTION)
			{
				throw new System.ArgumentException("Invalid poly type");
			}

			// Figure out which way to hand out the vertices.
			int idx0 = 0, idx1 = 1;

			// Find link that points to first vertex.
			for (int i = poly.firstLink; i != DT_NULL_LINK; i = tile.links[i].next)
			{
				if (tile.links[i].edge == 0)
				{
					if (tile.links[i].@ref != prevRef)
					{
						idx0 = 1;
						idx1 = 0;
					}
					break;
				}
			}
            DetourCommon.vCopy(startPos, tile.data.verts, poly.verts[idx0] * 3);
            DetourCommon.vCopy(endPos, tile.data.verts, poly.verts[idx1] * 3);
			return true;

		}

		public virtual int MaxVertsPerPoly
		{
			get
			{
				return m_maxVertPerPoly;
			}
		}

		public virtual int TileCount
		{
			get
			{
				return m_tileCount;
			}
		}



		private readonly float[] chptv0 = new float[3];
		private readonly float[] chptv1 = new float[3];
		private readonly float[] chptv2 = new float[3];

		internal virtual bool closestHeightPointTriangle(float[] p, int pi, float[] a, int ai, float[] b, int bi, float[] c, int ci, ref float h)
		{
			h = 0;
			DetourCommon.vSub(chptv0, c, ci, a, ai);
			DetourCommon.vSub(chptv1, b, bi, a, ai);
			DetourCommon.vSub(chptv2, p, pi, a, ai);

			float dot00 = DetourCommon.vDot2D(chptv0, chptv0);
			float dot01 = DetourCommon.vDot2D(chptv0, chptv1);
			float dot02 = DetourCommon.vDot2D(chptv0, chptv2);
			float dot11 = DetourCommon.vDot2D(chptv1, chptv1);
			float dot12 = DetourCommon.vDot2D(chptv1, chptv2);

			float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
			float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			float v = (dot00 * dot12 - dot01 * dot02) * invDenom;


			if (u >= -DetourCommon.EPS && v >= -DetourCommon.EPS && (u + v) <= 1 + DetourCommon.EPS)
			{
				h = a[ai + 1] + chptv0[1] * u + chptv1[1] * v;
				return true;
			}
			return false;
		}

		internal virtual bool distancePtPolyEdgesSqr(float[] pt, float[] verts, int nverts, float[] ed, float[] et)
		{
			// TODO: Replace pnpoly with triArea2D tests?
			int i, j;
			bool c = false;

			for (i = 0, j = nverts - 1; i < nverts; j = i++)
			{
				int vi = i * 3;
				int vj = j * 3;
				if (((verts[vi + 2] > pt[2]) != (verts[vj + 2] > pt[2])) && (pt[0] < (verts[vj + 0] - verts[vi + 0]) * (pt[2] - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi + 0]))
				{
					c = !c;
				}
                float t = 0;
				ed[j] = DetourCommon.distancePtSegSqr2D(pt, verts, vj, vi, ref t);
				et[j] = t;
			}
			return c;
		}
	}

}