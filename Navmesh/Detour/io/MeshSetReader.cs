using System;

/*
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
namespace org.recast4j.detour.io
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.ilog2;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.nextPow2;



	public class MeshSetReader
	{

		private readonly MeshDataReader meshReader = new MeshDataReader();
		private readonly NavMeshParamReader paramReader = new NavMeshParamReader();

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.NavMesh read(java.io.InputStream is, int maxVertPerPoly) throws java.io.IOException
		public virtual NavMesh read(System.IO.Stream @is, int maxVertPerPoly)
		{
			ByteBuffer bb = IOUtils.toByteBuffer(@is);
			return read(bb, maxVertPerPoly, false);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.NavMesh read(ByteBuffer bb, int maxVertPerPoly) throws java.io.IOException
		public virtual NavMesh read(ByteBuffer bb, int maxVertPerPoly)
		{
			return read(bb, maxVertPerPoly, false);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.NavMesh read32Bit(java.io.InputStream is, int maxVertPerPoly) throws java.io.IOException
		public virtual NavMesh read32Bit(System.IO.Stream @is, int maxVertPerPoly)
		{
			ByteBuffer bb = IOUtils.toByteBuffer(@is);
			return read(bb, maxVertPerPoly, true);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.NavMesh read32Bit(ByteBuffer bb, int maxVertPerPoly) throws java.io.IOException
		public virtual NavMesh read32Bit(ByteBuffer bb, int maxVertPerPoly)
		{
			return read(bb, maxVertPerPoly, true);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: org.recast4j.detour.NavMesh read(ByteBuffer bb, int maxVertPerPoly, boolean is32Bit) throws java.io.IOException
		internal virtual NavMesh read(ByteBuffer bb, int maxVertPerPoly, bool is32Bit)
		{
			NavMeshSetHeader header = new NavMeshSetHeader();
			header.magic = bb.Int;
			if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC)
			{
				header.magic = IOUtils.swapEndianness(header.magic);
				if (header.magic != NavMeshSetHeader.NAVMESHSET_MAGIC)
				{
					throw new IOException("Invalid magic");
				}
				bb.order(bb.order() == ByteOrder.BIG_ENDIAN ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
			}
			header.version = bb.Int;
			if (header.version != NavMeshSetHeader.NAVMESHSET_VERSION)
			{
				if (header.version != NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J)
				{
					throw new IOException("Invalid version");
				}
			}
			bool cCompatibility = header.version == NavMeshSetHeader.NAVMESHSET_VERSION;
			header.numTiles = bb.Int;
			header.@params = paramReader.read(bb);
			NavMesh mesh = new NavMesh(header.@params, maxVertPerPoly);

			// Read tiles.
			for (int i = 0; i < header.numTiles; ++i)
			{
				NavMeshTileHeader tileHeader = new NavMeshTileHeader();
				if (is32Bit)
				{
					tileHeader.tileRef = convert32BitRef(bb.Int, header.@params);
				}
				else
				{
					tileHeader.tileRef = bb.Long;
				}
				tileHeader.dataSize = bb.Int;
				if (tileHeader.tileRef == 0 || tileHeader.dataSize == 0)
				{
					break;
				}
				if (cCompatibility && !is32Bit)
				{
					bb.Int; // C struct padding
				}
				MeshData data = meshReader.read(bb, mesh.MaxVertsPerPoly, is32Bit);
				mesh.addTile(data, i, tileHeader.tileRef);
			}
			return mesh;
		}

		private long convert32BitRef(int @ref, NavMeshParams @params)
		{
			int m_tileBits = ilog2(nextPow2(@params.maxTiles));
			int m_polyBits = ilog2(nextPow2(@params.maxPolys));
			// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
			int m_saltBits = Math.Min(31, 32 - m_tileBits - m_polyBits);
			int saltMask = (1 << m_saltBits) - 1;
			int tileMask = (1 << m_tileBits) - 1;
			int polyMask = (1 << m_polyBits) - 1;
			int salt = ((@ref >> (m_polyBits + m_tileBits)) & saltMask);
			int it = ((@ref >> m_polyBits) & tileMask);
			int ip = @ref & polyMask;
			return NavMesh.encodePolyId(salt, it, ip);
		}
	}

}