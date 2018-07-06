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



	public class MeshSetWriter : DetourWriter
	{

		private readonly MeshDataWriter writer = new MeshDataWriter();
		private readonly NavMeshParamWriter paramWriter = new NavMeshParamWriter();

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public void write(java.io.OutputStream stream, org.recast4j.detour.NavMesh mesh, java.nio.ByteOrder order, boolean cCompatibility) throws java.io.IOException
		public virtual void write(System.IO.Stream stream, NavMesh mesh, ByteOrder order, bool cCompatibility)
		{
			// Write header.
			write(stream, NavMeshSetHeader.NAVMESHSET_MAGIC, order);
			write(stream, cCompatibility ? NavMeshSetHeader.NAVMESHSET_VERSION : NavMeshSetHeader.NAVMESHSET_VERSION_RECAST4J, order);
			int numTiles = 0;
			for (int i = 0; i < mesh.MaxTiles; ++i)
			{
				MeshTile tile = mesh.getTile(i);
				if (tile == null || tile.data == null || tile.data.header == null)
				{
					continue;
				}
				numTiles++;
			}
			write(stream, numTiles, order);
			paramWriter.write(stream, mesh.Params, order);

			// Store tiles.
			for (int i = 0; i < mesh.MaxTiles; ++i)
			{
				MeshTile tile = mesh.getTile(i);
				if (tile == null || tile.data == null || tile.data.header == null)
				{
					continue;
				}

				NavMeshTileHeader tileHeader = new NavMeshTileHeader();
				tileHeader.tileRef = mesh.getTileRef(tile);
				ByteArrayOutputStream baos = new ByteArrayOutputStream();
				writer.write(baos, tile.data, order, cCompatibility);
				sbyte[] ba = baos.toByteArray();
				tileHeader.dataSize = ba.Length;
				write(stream, tileHeader.tileRef, order);
				write(stream, tileHeader.dataSize, order);
				if (cCompatibility)
				{
					write(stream, 0, order); // C struct padding
				}
				stream.Write(ba, 0, ba.Length);
			}
		}

	}

}