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



	public class MeshDataWriter : DetourWriter
	{

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public void write(java.io.OutputStream stream, org.recast4j.detour.MeshData data, java.nio.ByteOrder order, boolean cCompatibility) throws java.io.IOException
		public virtual void write(System.IO.Stream stream, MeshData data, ByteOrder order, bool cCompatibility)
		{
			MeshHeader header = data.header;
			write(stream, header.magic, order);
			write(stream, cCompatibility ? MeshHeader.DT_NAVMESH_VERSION : MeshHeader.DT_NAVMESH_VERSION_RECAST4J, order);
			write(stream, header.x, order);
			write(stream, header.y, order);
			write(stream, header.layer, order);
			write(stream, header.userId, order);
			write(stream, header.polyCount, order);
			write(stream, header.vertCount, order);
			write(stream, header.maxLinkCount, order);
			write(stream, header.detailMeshCount, order);
			write(stream, header.detailVertCount, order);
			write(stream, header.detailTriCount, order);
			write(stream, header.bvNodeCount, order);
			write(stream, header.offMeshConCount, order);
			write(stream, header.offMeshBase, order);
			write(stream, header.walkableHeight, order);
			write(stream, header.walkableRadius, order);
			write(stream, header.walkableClimb, order);
			write(stream, header.bmin[0], order);
			write(stream, header.bmin[1], order);
			write(stream, header.bmin[2], order);
			write(stream, header.bmax[0], order);
			write(stream, header.bmax[1], order);
			write(stream, header.bmax[2], order);
			write(stream, header.bvQuantFactor, order);
			writeVerts(stream, data.verts, header.vertCount, order);
			writePolys(stream, data, order);
			if (cCompatibility)
			{
				sbyte[] linkPlaceholder = new sbyte[header.maxLinkCount * MeshDataReader.getSizeofLink(false)];
				stream.Write(linkPlaceholder, 0, linkPlaceholder.Length);
			}
			writePolyDetails(stream, data, order, cCompatibility);
			writeVerts(stream, data.detailVerts, header.detailVertCount, order);
			writeDTris(stream, data);
			writeBVTree(stream, data, order);
			writeOffMeshCons(stream, data, order);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writeVerts(java.io.OutputStream stream, float[] verts, int count, java.nio.ByteOrder order) throws java.io.IOException
		private void writeVerts(System.IO.Stream stream, float[] verts, int count, ByteOrder order)
		{
			for (int i = 0; i < count * 3; i++)
			{
				write(stream, verts[i], order);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writePolys(java.io.OutputStream stream, org.recast4j.detour.MeshData data, java.nio.ByteOrder order) throws java.io.IOException
		private void writePolys(System.IO.Stream stream, MeshData data, ByteOrder order)
		{
			for (int i = 0; i < data.header.polyCount; i++)
			{
				write(stream, data.polys[i].firstLink, order);
				for (int j = 0; j < data.polys[i].verts.Length; j++)
				{
					write(stream, (short) data.polys[i].verts[j], order);
				}
				for (int j = 0; j < data.polys[i].neis.Length; j++)
				{
					write(stream, (short) data.polys[i].neis[j], order);
				}
				write(stream, (short) data.polys[i].flags, order);
				stream.WriteByte(data.polys[i].vertCount);
				stream.WriteByte(data.polys[i].areaAndtype);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writePolyDetails(java.io.OutputStream stream, org.recast4j.detour.MeshData data, java.nio.ByteOrder order, boolean cCompatibility) throws java.io.IOException
		private void writePolyDetails(System.IO.Stream stream, MeshData data, ByteOrder order, bool cCompatibility)
		{
			for (int i = 0; i < data.header.detailMeshCount; i++)
			{
				write(stream, data.detailMeshes[i].vertBase, order);
				write(stream, data.detailMeshes[i].triBase, order);
				stream.WriteByte(data.detailMeshes[i].vertCount);
				stream.WriteByte(data.detailMeshes[i].triCount);
				if (cCompatibility)
				{
					write(stream, (short) 0, order);
				}
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writeDTris(java.io.OutputStream stream, org.recast4j.detour.MeshData data) throws java.io.IOException
		private void writeDTris(System.IO.Stream stream, MeshData data)
		{
			for (int i = 0; i < data.header.detailTriCount * 4; i++)
			{
				stream.WriteByte(data.detailTris[i]);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writeBVTree(java.io.OutputStream stream, org.recast4j.detour.MeshData data, java.nio.ByteOrder order) throws java.io.IOException
		private void writeBVTree(System.IO.Stream stream, MeshData data, ByteOrder order)
		{
			for (int i = 0; i < data.header.bvNodeCount; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					write(stream, (short) data.bvTree[i].bmin[j], order);
				}
				for (int j = 0; j < 3; j++)
				{
					write(stream, (short) data.bvTree[i].bmax[j], order);
				}
				write(stream, data.bvTree[i].i, order);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: private void writeOffMeshCons(java.io.OutputStream stream, org.recast4j.detour.MeshData data, java.nio.ByteOrder order) throws java.io.IOException
		private void writeOffMeshCons(System.IO.Stream stream, MeshData data, ByteOrder order)
		{
			for (int i = 0; i < data.header.offMeshConCount; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					write(stream, data.offMeshCons[i].pos[j], order);
				}
				write(stream, data.offMeshCons[i].rad, order);
				write(stream, (short) data.offMeshCons[i].poly, order);
				stream.WriteByte(data.offMeshCons[i].flags);
				stream.WriteByte(data.offMeshCons[i].side);
				write(stream, data.offMeshCons[i].userId, order);
			}
		}

	}

}