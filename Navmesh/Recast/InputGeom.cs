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
namespace org.recast4j.recast
{


	public class InputGeom
	{

		internal readonly float[] vertices;
		internal readonly int[] faces;
		internal readonly float[] bmin;
		internal readonly float[] bmax;

		public InputGeom(List<float> vertexPositions, List<int> meshFaces)
		{
			vertices = new float[vertexPositions.Count];
			for (int i = 0; i < vertices.Length; i++)
			{
				vertices[i] = vertexPositions[i];
			}
			faces = new int[meshFaces.Count];
			for (int i = 0; i < faces.Length; i++)
			{
				faces[i] = meshFaces[i];
			}
			bmin = new float[3];
			bmax = new float[3];
			RecastVectors.copy(bmin, vertices, 0);
			RecastVectors.copy(bmax, vertices, 0);
			for (int i = 1; i < vertices.Length / 3; i++)
			{
				RecastVectors.min(bmin, vertices, i * 3);
				RecastVectors.max(bmax, vertices, i * 3);
			}
		}

		public virtual float[] MeshBoundsMin
		{
			get
			{
				return bmin;
			}
		}

		public virtual float[] MeshBoundsMax
		{
			get
			{
				return bmax;
			}
		}

		public virtual float[] Verts
		{
			get
			{
				return vertices;
			}
		}

		public virtual int[] Tris
		{
			get
			{
				return faces;
			}
		}

		public virtual ChunkyTriMesh ChunkyMesh
		{
			get
			{
				return new ChunkyTriMesh(vertices, faces, faces.Length / 3, 256);
			}
		}

		public virtual List<ConvexVolume> ConvexVolumes
		{
			get
			{
                return new List<ConvexVolume>();
			}
		}

	}

}