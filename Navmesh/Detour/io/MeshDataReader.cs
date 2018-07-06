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



	public class MeshDataReader
	{

		internal const int DT_POLY_DETAIL_SIZE = 10;

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.MeshData read(java.io.InputStream stream, int maxVertPerPoly) throws java.io.IOException
		public virtual MeshData read(System.IO.Stream stream, int maxVertPerPoly)
		{
			ByteBuffer buf = IOUtils.toByteBuffer(stream);
			return read(buf, maxVertPerPoly, false);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.MeshData read(ByteBuffer buf, int maxVertPerPoly) throws java.io.IOException
		public virtual MeshData read(ByteBuffer buf, int maxVertPerPoly)
		{
			return read(buf, maxVertPerPoly, false);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.MeshData read32Bit(java.io.InputStream stream, int maxVertPerPoly) throws java.io.IOException
		public virtual MeshData read32Bit(System.IO.Stream stream, int maxVertPerPoly)
		{
			ByteBuffer buf = IOUtils.toByteBuffer(stream);
			return read(buf, maxVertPerPoly, true);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public org.recast4j.detour.MeshData read32Bit(ByteBuffer buf, int maxVertPerPoly) throws java.io.IOException
		public virtual MeshData read32Bit(ByteBuffer buf, int maxVertPerPoly)
		{
			return read(buf, maxVertPerPoly, true);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: org.recast4j.detour.MeshData read(ByteBuffer buf, int maxVertPerPoly, boolean is32Bit) throws java.io.IOException
		internal virtual MeshData read(ByteBuffer buf, int maxVertPerPoly, bool is32Bit)
		{
			MeshData data = new MeshData();
			MeshHeader header = new MeshHeader();
			data.header = header;
			header.magic = buf.Int;
			if (header.magic != MeshHeader.DT_NAVMESH_MAGIC)
			{
				header.magic = IOUtils.swapEndianness(header.magic);
				if (header.magic != MeshHeader.DT_NAVMESH_MAGIC)
				{
					throw new IOException("Invalid magic");
				}
				buf.order(buf.order() == ByteOrder.BIG_ENDIAN ? ByteOrder.LITTLE_ENDIAN : ByteOrder.BIG_ENDIAN);
			}
			header.version = buf.Int;
			if (header.version != MeshHeader.DT_NAVMESH_VERSION)
			{
				if (header.version != MeshHeader.DT_NAVMESH_VERSION_RECAST4J)
				{
					throw new IOException("Invalid version");
				}
			}
			bool cCompatibility = header.version == MeshHeader.DT_NAVMESH_VERSION;
			header.x = buf.Int;
			header.y = buf.Int;
			header.layer = buf.Int;
			header.userId = buf.Int;
			header.polyCount = buf.Int;
			header.vertCount = buf.Int;
			header.maxLinkCount = buf.Int;
			header.detailMeshCount = buf.Int;
			header.detailVertCount = buf.Int;
			header.detailTriCount = buf.Int;
			header.bvNodeCount = buf.Int;
			header.offMeshConCount = buf.Int;
			header.offMeshBase = buf.Int;
			header.walkableHeight = buf.Float;
			header.walkableRadius = buf.Float;
			header.walkableClimb = buf.Float;
			for (int j = 0; j < 3; j++)
			{
				header.bmin[j] = buf.Float;
			}
			for (int j = 0; j < 3; j++)
			{
				header.bmax[j] = buf.Float;
			}
			header.bvQuantFactor = buf.Float;
			data.verts = readVerts(buf, header.vertCount);
			data.polys = readPolys(buf, header, maxVertPerPoly);
			if (cCompatibility)
			{
				buf.position(buf.position() + header.maxLinkCount * getSizeofLink(is32Bit));
			}
			data.detailMeshes = readPolyDetails(buf, header, cCompatibility);
			data.detailVerts = readVerts(buf, header.detailVertCount);
			data.detailTris = readDTris(buf, header);
			data.bvTree = readBVTree(buf, header);
			data.offMeshCons = readOffMeshCons(buf, header);
			return data;
		}

		public const int LINK_SIZEOF = 16;
		public const int LINK_SIZEOF32BIT = 12;

		internal static int getSizeofLink(bool is32Bit)
		{
			return is32Bit ? LINK_SIZEOF32BIT : LINK_SIZEOF;
		}

		private float[] readVerts(ByteBuffer buf, int count)
		{
			float[] verts = new float[count * 3];
			for (int i = 0; i < verts.Length; i++)
			{
				verts[i] = buf.Float;
			}
			return verts;
		}

		private Poly[] readPolys(ByteBuffer buf, MeshHeader header, int maxVertPerPoly)
		{
			Poly[] polys = new Poly[header.polyCount];
			for (int i = 0; i < polys.Length; i++)
			{
				polys[i] = new Poly(i, maxVertPerPoly);
				polys[i].firstLink = buf.Int;
				for (int j = 0; j < polys[i].verts.Length; j++)
				{
					polys[i].verts[j] = buf.Short & 0xFFFF;
				}
				for (int j = 0; j < polys[i].neis.Length; j++)
				{
					polys[i].neis[j] = buf.Short & 0xFFFF;
				}
				polys[i].flags = buf.Short & 0xFFFF;
				polys[i].vertCount = buf.get() & 0xFF;
				polys[i].areaAndtype = buf.get() & 0xFF;
			}
			return polys;
		}

		private PolyDetail[] readPolyDetails(ByteBuffer buf, MeshHeader header, bool cCompatibility)
		{
			PolyDetail[] polys = new PolyDetail[header.detailMeshCount];
			for (int i = 0; i < polys.Length; i++)
			{
				polys[i] = new PolyDetail();
				polys[i].vertBase = buf.Int;
				polys[i].triBase = buf.Int;
				polys[i].vertCount = buf.get() & 0xFF;
				polys[i].triCount = buf.get() & 0xFF;
				if (cCompatibility)
				{
					buf.Short; // C struct padding
				}
			}
			return polys;
		}

		private int[] readDTris(ByteBuffer buf, MeshHeader header)
		{
			int[] tris = new int[4 * header.detailTriCount];
			for (int i = 0; i < tris.Length; i++)
			{
				tris[i] = buf.get() & 0xFF;
			}
			return tris;
		}

		private BVNode[] readBVTree(ByteBuffer buf, MeshHeader header)
		{
			BVNode[] nodes = new BVNode[header.bvNodeCount];
			for (int i = 0; i < nodes.Length; i++)
			{
				nodes[i] = new BVNode();
				for (int j = 0; j < 3; j++)
				{
					nodes[i].bmin[j] = buf.Short & 0xFFFF;
				}
				for (int j = 0; j < 3; j++)
				{
					nodes[i].bmax[j] = buf.Short & 0xFFFF;
				}
				nodes[i].i = buf.Int;
			}
			return nodes;
		}

		private OffMeshConnection[] readOffMeshCons(ByteBuffer buf, MeshHeader header)
		{
			OffMeshConnection[] cons = new OffMeshConnection[header.offMeshConCount];
			for (int i = 0; i < cons.Length; i++)
			{
				cons[i] = new OffMeshConnection();
				for (int j = 0; j < 6; j++)
				{
					cons[i].pos[j] = buf.Float;
				}
				cons[i].rad = buf.Float;
				cons[i].poly = buf.Short & 0xFFFF;
				cons[i].flags = buf.get() & 0xFF;
				cons[i].side = buf.get() & 0xFF;
				cons[i].userId = buf.Int;
			}
			return cons;
		}

	}

}