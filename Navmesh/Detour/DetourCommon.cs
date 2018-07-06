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
namespace org.recast4j.detour
{

	public class DetourCommon
	{

		internal static float EPS = 1e-4f;




		/// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
		/// @param[out] dest The result vector. [(x, y, z)]
		/// @param[in] v1 The base vector. [(x, y, z)]
		/// @param[in] v2 The vector to scale and add to @p v1. [(x, y, z)]
		/// @param[in] s The amount to scale @p v2 by before adding to @p v1.
		public static void vMad(float[] dest, float[] v1, float[] v2, float s)
		{
			dest[0] = v1[0] + v2[0] * s;
			dest[1] = v1[1] + v2[1] * s;
			dest[2] = v1[2] + v2[2] * s;
		}





		/// Performs a linear interpolation between two vectors. (@p v1 toward @p
		/// v2)
		/// @param[out] dest The result vector. [(x, y, x)]
		/// @param[in] v1 The starting vector.
		/// @param[in] v2 The destination vector.
		/// @param[in] t The interpolation factor. [Limits: 0 <= value <= 1.0]
		public static void vLerp(float[] dest, VectorPtr v1, VectorPtr v2, float t)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1.get(0) + (v2.get(0) - v1.get(0)) * t;
			dest[1] = v1.get(1) + (v2.get(1) - v1.get(1)) * t;
			dest[2] = v1.get(2) + (v2.get(2) - v1.get(2)) * t;
		}



		public static void vLerp(float[] dest, float[] v1, int v1i, float[] v2, int v2i, float t)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[v1i + 0] + (v2[v2i + 0] - v1[v1i + 0]) * t;
			dest[1] = v1[v1i + 1] + (v2[v2i + 1] - v1[v1i + 1]) * t;
			dest[2] = v1[v1i + 2] + (v2[v2i + 2] - v1[v1i + 2]) * t;
		}


		/// 
		/// <param name="out"> </param>
		/// <param name="verts"> </param>
		/// <param name="v1"> </param>
		/// <param name="v2"> </param>
		/// <param name="t"> </param>
		public static void vLerp(float[] dest, float[] verts, int v1, int v2, float t)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = verts[v1 + 0] + (verts[v2 + 0] - verts[v1 + 0]) * t;
			dest[1] = verts[v1 + 1] + (verts[v2 + 1] - verts[v1 + 1]) * t;
			dest[2] = verts[v1 + 2] + (verts[v2 + 2] - verts[v1 + 2]) * t;
		}


		/// 
		/// <param name="out"> </param>
		/// <param name="v1"> </param>
		/// <param name="v2"> </param>
		/// <param name="t"> </param>
		public static void vLerp(float[] dest, float[] v1, float[] v2, float t)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[0] + (v2[0] - v1[0]) * t;
			dest[1] = v1[1] + (v2[1] - v1[1]) * t;
			dest[2] = v1[2] + (v2[2] - v1[2]) * t;
		}



		/// 
		/// <param name="out"> </param>
		/// <param name="v1"> </param>
		/// <param name="v2">
		/// @return </param>
		public static void vSub(float[] dest, VectorPtr v1, VectorPtr v2)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1.get(0) - v2.get(0);
			dest[1] = v1.get(1) - v2.get(1);
			dest[2] = v1.get(2) - v2.get(2);
		}



		public static void vSub(float[] dest, float[] v1, int v1i, float[] v2, int v2i)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[v1i + 0] - v2[v2i + 0];
			dest[1] = v1[v1i + 1] - v2[v2i + 1];
			dest[2] = v1[v1i + 2] - v2[v2i + 2];
		}




		/// 
		/// <param name="v1"> </param>
		/// <param name="v2">
		/// @return </param>
		public static void vSub(float[] dest, float[] v1, float[] v2)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[0] - v2[0];
			dest[1] = v1[1] - v2[1];
			dest[2] = v1[2] - v2[2];
		}


		/// 
		/// <param name="v1"> </param>
		/// <param name="v2">
		/// @return </param>
		internal static void vAdd(float[] dest, VectorPtr v1, VectorPtr v2)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1.get(0) + v2.get(0);
			dest[1] = v1.get(1) + v2.get(1);
			dest[2] = v1.get(2) + v2.get(2);
		}



		internal static void vAdd(float[] dest, float[] v1, int v1i, float[] v2, int v2i)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[v1i + 0] + v2[v2i + 0];
			dest[1] = v1[v1i + 1] + v2[v2i + 1];
			dest[2] = v1[v1i + 2] + v2[v2i + 2];
		}



		/// 
		/// <param name="dest"> </param>
		/// <param name="v1"> </param>
		/// <param name="v2"> </param>
		public static void vAdd(float[] dest, float[] v1, float[] v2)
		{
			if (dest == null)
			{
				dest = new float[3];
			}
			dest[0] = v1[0] + v2[0];
			dest[1] = v1[1] + v2[1];
			dest[2] = v1[2] + v2[2];
		}

		public static void vSet(float[] @out, float a, float b, float c)
		{
			@out[0] = a;
			@out[1] = b;
			@out[2] = c;
		}

		public static void vCopy(float[] @out, float[] @in)
		{
			@out[0] = @in[0];
			@out[1] = @in[1];
			@out[2] = @in[2];
		}

		public static void vCopy(float[] @out, float[] @in, int i)
		{
			@out[0] = @in[i];
			@out[1] = @in[i + 1];
			@out[2] = @in[i + 2];
		}

		internal static void vMin(float[] @out, float[] @in, int i)
		{
			@out[0] = Math.Min(@out[0], @in[i]);
			@out[1] = Math.Min(@out[1], @in[i + 1]);
			@out[2] = Math.Min(@out[2], @in[i + 2]);
		}

		internal static void vMax(float[] @out, float[] @in, int i)
		{
			@out[0] = Math.Max(@out[0], @in[i]);
			@out[1] = Math.Max(@out[1], @in[i + 1]);
			@out[2] = Math.Max(@out[2], @in[i + 2]);
		}

		/// Returns the distance between two points.
		/// @param[in] v1 A point. [(x, y, z)]
		/// @param[in] v2 A point. [(x, y, z)]
		/// @return The distance between the two points.
		internal static float vDist(float[] v1, float[] v2)
		{
			float dx = v2[0] - v1[0];
			float dy = v2[1] - v1[1];
			float dz = v2[2] - v1[2];
			return (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
		}

		/// Returns the distance between two points.
		/// @param[in] v1 A point. [(x, y, z)]
		/// @param[in] v2 A point. [(x, y, z)]
		/// @return The distance between the two points.
		internal static float vDistSqr(float[] v1, float[] v2)
		{
			float dx = v2[0] - v1[0];
			float dy = v2[1] - v1[1];
			float dz = v2[2] - v1[2];
			return dx * dx + dy * dy + dz * dz;
		}

		public static float sqr(float a)
		{
			return a * a;
		}

		/// Derives the square of the scalar length of the vector. (len * len)
		/// @param[in] v The vector. [(x, y, z)]
		/// @return The square of the scalar length of the vector.
		public static float vLenSqr(float[] v)
		{
			return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
		}

		public static float vLen(float[] v)
		{
			return (float) Math.Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		}

		internal static float vDist(float[] v1, float[] verts, int i)
		{
			float dx = verts[i] - v1[0];
			float dy = verts[i + 1] - v1[1];
			float dz = verts[i + 2] - v1[2];
			return (float) Math.Sqrt(dx * dx + dy * dy + dz * dz);
		}

		public static float clamp(float v, float min, float max)
		{
			return Math.Max(Math.Min(v, max), min);
		}

		public static int clamp(int v, int min, int max)
		{
			return Math.Max(Math.Min(v, max), min);
		}

		/// Derives the distance between the specified points on the xz-plane.
		/// @param[in] v1 A point. [(x, y, z)]
		/// @param[in] v2 A point. [(x, y, z)]
		/// @return The distance between the point on the xz-plane.
		///
		/// The vectors are projected onto the xz-plane, so the y-values are
		/// ignored.
		public static float vDist2D(VectorPtr v1, VectorPtr v2)
		{
			float dx = v2.get(0) - v1.get(0);
			float dz = v2.get(2) - v1.get(2);
			return (float) Math.Sqrt(dx * dx + dz * dz);
		}


		public static float vDist2D(float[] v1, int v1i, float[] v2, int v2i)
		{
			float dx = v2[v2i + 0] - v1[v1i + 0];
			float dz = v2[v2i + 2] - v1[v1i + 2];
			return (float) Math.Sqrt(dx * dx + dz * dz);
		}


		public static float vDist2D(float[] v1, float[] v2)
		{
			float dx = v2[0] - v1[0];
			float dz = v2[2] - v1[2];
			return (float) Math.Sqrt(dx * dx + dz * dz);
		}

		public static float vDist2DSqr(VectorPtr v1, VectorPtr v2)
		{
			float dx = v2.get(0) - v1.get(0);
			float dz = v2.get(2) - v1.get(2);
			return dx * dx + dz * dz;
		}


		public static float vDist2DSqr(float[] v1, int v1i, float[] v2, int v2i)
		{
			float dx = v2[v2i + 0] - v1[v1i + 0];
			float dz = v2[v2i + 2] - v1[v1i + 2];
			return dx * dx + dz * dz;
		}



		public static float vDist2DSqr(float[] v1, float[] v2)
		{
			float dx = v2[0] - v1[0];
			float dz = v2[2] - v1[2];
			return dx * dx + dz * dz;
		}

		/// Normalizes the vector.
		/// @param[in,out] v The vector to normalize. [(x, y, z)]
		public static void vNormalize(float[] v)
		{
			float d = (float)(1.0f / Math.Sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2])));
			if (d != 0)
			{
				v[0] *= d;
				v[1] *= d;
				v[2] *= d;
			}
		}

		internal static readonly float thr = sqr(1.0f / 16384.0f);

		/// Performs a 'sloppy' colocation check of the specified points.
		/// @param[in] p0 A point. [(x, y, z)]
		/// @param[in] p1 A point. [(x, y, z)]
		/// @return True if the points are considered to be at the same location.
		///
		/// Basically, this function will return true if the specified points are
		/// close enough to eachother to be considered colocated.
		internal static bool vEqual(float[] p0, float[] p1)
		{
			float d = vDistSqr(p0, p1);
			return d < thr;
		}

		/// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
		/// @param[in] u A vector [(x, y, z)]
		/// @param[in] v A vector [(x, y, z)]
		/// @return The dot product on the xz-plane.
		///
		/// The vectors are projected onto the xz-plane, so the y-values are
		/// ignored.
		public static float vDot2D(float[] u, float[] v)
		{
			return u[0] * v[0] + u[2] * v[2];
		}

		internal static float vDot2D(float[] u, float[] v, int vi)
		{
			return u[0] * v[vi] + u[2] * v[vi + 2];
		}

		/// Derives the xz-plane 2D perp product of the two vectors. (uz*vx - ux*vz)
		/// @param[in] u The LHV vector [(x, y, z)]
		/// @param[in] v The RHV vector [(x, y, z)]
		/// @return The dot product on the xz-plane.
		///
		/// The vectors are projected onto the xz-plane, so the y-values are
		/// ignored.
		public static float vPerp2D(float[] u, float[] v)
		{
			return u[2] * v[0] - u[0] * v[2];
		}

		/// @}
		/// @name Computational geometry helper functions.
		/// @{

		/// Derives the signed xz-plane area of the triangle ABC, or the
		/// relationship of line AB to point C.
		/// @param[in] a Vertex A. [(x, y, z)]
		/// @param[in] b Vertex B. [(x, y, z)]
		/// @param[in] c Vertex C. [(x, y, z)]
		/// @return The signed xz-plane area of the triangle.
		public static float triArea2D(float[] verts, int a, int b, int c)
		{
			float abx = verts[b] - verts[a];
			float abz = verts[b + 2] - verts[a + 2];
			float acx = verts[c] - verts[a];
			float acz = verts[c + 2] - verts[a + 2];
			return acx * abz - abx * acz;
		}

		public static float triArea2D(float[] a, float[] b, float[] c)
		{
			float abx = b[0] - a[0];
			float abz = b[2] - a[2];
			float acx = c[0] - a[0];
			float acz = c[2] - a[2];
			return acx * abz - abx * acz;
		}

		/// Determines if two axis-aligned bounding boxes overlap.
		/// @param[in] amin Minimum bounds of box A. [(x, y, z)]
		/// @param[in] amax Maximum bounds of box A. [(x, y, z)]
		/// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
		/// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
		/// @return True if the two AABB's overlap.
		/// @see dtOverlapBounds
		internal static bool overlapQuantBounds(int[] amin, int[] amax, int[] bmin, int[] bmax)
		{
			bool overlap = true;
			overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
			overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
			overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
			return overlap;
		}

		/// Determines if two axis-aligned bounding boxes overlap.
		/// @param[in] amin Minimum bounds of box A. [(x, y, z)]
		/// @param[in] amax Maximum bounds of box A. [(x, y, z)]
		/// @param[in] bmin Minimum bounds of box B. [(x, y, z)]
		/// @param[in] bmax Maximum bounds of box B. [(x, y, z)]
		/// @return True if the two AABB's overlap.
		/// @see dtOverlapQuantBounds
		public static bool overlapBounds(float[] amin, float[] amax, float[] bmin, float[] bmax)
		{
			bool overlap = true;
			overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
			overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
			overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
			return overlap;
		}



		public static float distancePtSegSqr2D(float[] pt, float[] p, float[] q, ref float t)
		{
			float pqx = q[0] - p[0];
			float pqz = q[2] - p[2];
			float dx = pt[0] - p[0];
			float dz = pt[2] - p[2];
			float d = pqx * pqx + pqz * pqz;
			t = pqx * dx + pqz * dz;
			if (d > 0)
			{
				t /= d;
			}
			if (t < 0)
			{
				t = 0;
			}
			else if (t > 1)
			{
				t = 1;
			}
			dx = p[0] + t * pqx - pt[0];
			dz = p[2] + t * pqz - pt[2];
			return dx * dx + dz * dz;
		}


		[Obsolete]
		internal static bool closestHeightPointTriangle(float[] p, int pi, float[] a, int ai, float[] b, int bi, float[] c, int ci, ref float h)
		{
			h = 0;
			float[] v0 = new float[3];
			float[] v1 = new float[3];
			float[] v2 = new float[3];
			vSub(v0, c, ci, a, ai);
			vSub(v1, b, bi, a, ai);
			vSub(v2, p, pi, a, ai);

			float dot00 = vDot2D(v0, v0);
			float dot01 = vDot2D(v0, v1);
			float dot02 = vDot2D(v0, v2);
			float dot11 = vDot2D(v1, v1);
			float dot12 = vDot2D(v1, v2);

			float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
			float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			float v = (dot00 * dot12 - dot01 * dot02) * invDenom;


			if (u >= -EPS && v >= -EPS && (u + v) <= 1 + EPS)
			{
				h = a[ai + 1] + v0[1] * u + v1[1] * v;
				return true;
			}
			return false;
		}



		/// @par
		///
		/// All points are projected onto the xz-plane, so the y-values are ignored.
		internal static bool pointInPolygon(float[] pt, float[] verts, int nverts)
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
			}
			return c;
		}

		internal static bool distancePtPolyEdgesSqr(float[] pt, float[] verts, int nverts, float[] ed, float[] et)
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
                ed[j] = distancePtSegSqr2D(pt, verts, vj, vi, ref t);
				et[j] = t;
			}
			return c;
		}

		internal static void projectPoly(float[] @out, float[] axis, float[] poly, int npoly)
		{
			float rmin, rmax;
			rmin = rmax = vDot2D(axis, poly, 0);
			for (int i = 1; i < npoly; ++i)
			{
				float d = vDot2D(axis, poly, i * 3);
				rmin = Math.Min(rmin, d);
				rmax = Math.Max(rmax, d);
			}
			@out[0] = rmin;
			@out[1] = rmax;
			//return new float[] { rmin, rmax };
		}

		internal static bool overlapRange(float amin, float amax, float bmin, float bmax, float eps)
		{
			return ((amin + eps) > bmax || (amax - eps) < bmin) ? false : true;
		}

		internal static float eps = 1e-4f;

		/// @par
		///
		/// All vertices are projected onto the xz-plane, so the y-values are ignored.
		[Obsolete]
		internal static bool overlapPolyPoly2D(float[] polya, int npolya, float[] polyb, int npolyb)
		{
			/*TODO:��Ҫ�Ż�*/
			float[] aminmax = new float[2];
			float[] bminmax = new float[2];
			float[] n = new float[3];
			int va, vb;
			for (int i = 0, j = npolya - 1; i < npolya; j = i++)
			{
				va = j * 3;
				vb = i * 3;

				n[0] = polya[vb + 2] - polya[va + 2];
				n[1] = 0;
				n[2] = -(polya[vb + 0] - polya[va + 0]);
				//float[] n = new float[] { polya[vb + 2] - polya[va + 2], 0, -(polya[vb + 0] - polya[va + 0]) };

				projectPoly(aminmax, n, polya, npolya);
				projectPoly(bminmax, n, polyb, npolyb);
				if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps))
				{
					// Found separating axis
					return false;
				}
			}
			for (int i = 0, j = npolyb - 1; i < npolyb; j = i++)
			{
				va = j * 3;
				vb = i * 3;

				n[0] = polyb[vb + 2] - polyb[va + 2];
				n[1] = 0;
				n[2] = -(polyb[vb + 0] - polyb[va + 0]);
				//float[] n = new float[] { polyb[vb + 2] - polyb[va + 2], 0, -(polyb[vb + 0] - polyb[va + 0]) };

				projectPoly(aminmax, n, polya, npolya);
				projectPoly(bminmax, n, polyb, npolyb);
				if (!overlapRange(aminmax[0], aminmax[1], bminmax[0], bminmax[1], eps))
				{
					// Found separating axis
					return false;
				}
			}
			return true;
		}

		// Returns a random point in a convex polygon.
		// Adapted from Graphics Gems article.
		internal static float[] randomPointInConvexPoly(float[] pts, int npts, float[] areas, float s, float t)
		{
			// Calc triangle araes
			float areasum = 0.0f;
			for (int i = 2; i < npts; i++)
			{
				areas[i] = triArea2D(pts, 0, (i - 1) * 3, i * 3);
				areasum += Math.Max(0.001f, areas[i]);
			}
			// Find sub triangle weighted by area.
			float thr = s * areasum;
			float acc = 0.0f;
			float u = 0.0f;
			int tri = 0;
			for (int i = 2; i < npts; i++)
			{
				float dacc = areas[i];
				if (thr >= acc && thr < (acc + dacc))
				{
					u = (thr - acc) / dacc;
					tri = i;
					break;
				}
				acc += dacc;
			}

			float v = (float) Math.Sqrt(t);

			float a = 1 - v;
			float b = (1 - u) * v;
			float c = u * v;
			int pa = 0;
			int pb = (tri - 1) * 3;
			int pc = tri * 3;

			return new float[] {a * pts[pa] + b * pts[pb] + c * pts[pc], a * pts[pa + 1] + b * pts[pb + 1] + c * pts[pc + 1], a * pts[pa + 2] + b * pts[pb + 2] + c * pts[pc + 2]};
		}

		public static int nextPow2(int v)
		{
			v--;
			v |= v >> 1;
			v |= v >> 2;
			v |= v >> 4;
			v |= v >> 8;
			v |= v >> 16;
			v++;
			return v;
		}

		public static int ilog2(int v)
		{
			int r;
			int shift;
			r = (v > 0xffff ? 1 : 0) << 4;
			v >>= r;
			shift = (v > 0xff ? 1 : 0) << 3;
			v >>= shift;
			r |= shift;
			shift = (v > 0xf ? 1 : 0) << 2;
			v >>= shift;
			r |= shift;
			shift = (v > 0x3 ? 1 : 0) << 1;
			v >>= shift;
			r |= shift;
			r |= (v >> 1);
			return r;
		}

		public class IntersectResult
		{
			internal bool intersects;
			internal float tmin;
			internal float tmax = 1f;
			internal int segMin = -1;
			internal int segMax = -1;

			public virtual void reset()
			{
				intersects = false;
				tmin = 0;
				tmax = 1f;
				segMin = -1;
				segMax = -1;
			}
		}

		[Obsolete]
		internal static void intersectSegmentPoly2D(float[] p0, float[] p1, float[] verts, int nverts, IntersectResult result)
		{
			/*TODO:��Ҫ�Ż�*/
			result.reset();
			float EPS = 0.00000001f;
			float[] dir = new float[3];
			vSub(dir, p1, p0);

			float[] edge = new float[3];
			float[] diff = new float[3];

			for (int i = 0, j = nverts - 1; i < nverts; j = i++)
			{
				vSub(edge, verts, i * 3, verts, j * 3);
				vSub(diff, p0, 0, verts, j * 3);
				float n = vPerp2D(edge, diff);
				float d = vPerp2D(dir, edge);
				if (Math.Abs(d) < EPS)
				{
					// S is nearly parallel to this edge
					if (n < 0)
					{
						return;
					}
					else
					{
						continue;
					}
				}
				float t = n / d;
				if (d < 0)
				{
					// segment S is entering across this edge
					if (t > result.tmin)
					{
						result.tmin = t;
						result.segMin = j;
						// S enters after leaving polygon
						if (result.tmin > result.tmax)
						{
							return;
						}
					}
				}
				else
				{
					// segment S is leaving across this edge
					if (t < result.tmax)
					{
						result.tmax = t;
						result.segMax = j;
						// S leaves before entering polygon
						if (result.tmax < result.tmin)
						{
							return;
						}
					}
				}
			}
			result.intersects = true;

		}

		/// <summary>
		/// ԭ����intersectSegmentPoly2D </summary>
		/// <param name="p0"> </param>
		/// <param name="p1"> </param>
		/// <param name="verts"> </param>
		/// <param name="nverts">
		/// @return </param>
		[Obsolete]
		internal static IntersectResult oldIntersectSegmentPoly2D(float[] p0, float[] p1, float[] verts, int nverts)
		{
			/*TODO:��Ҫ�Ż�*/
			IntersectResult result = new IntersectResult();
			float EPS = 0.00000001f;
			float[] dir = new float[3];
			vSub(dir, p1, p0);

			float[] edge = new float[3];
			float[] diff = new float[3];

			VectorPtr p0v = new VectorPtr(p0);
			VectorPtr vpj;
			for (int i = 0, j = nverts - 1; i < nverts; j = i++)
			{
				vpj = new VectorPtr(verts, j * 3);
				vSub(edge, new VectorPtr(verts, i * 3), vpj);
				vSub(diff, p0v, vpj);
				float n = vPerp2D(edge, diff);
				float d = vPerp2D(dir, edge);
				if (Math.Abs(d) < EPS)
				{
					// S is nearly parallel to this edge
					if (n < 0)
					{
						return result;
					}
					else
					{
						continue;
					}
				}
				float t = n / d;
				if (d < 0)
				{
					// segment S is entering across this edge
					if (t > result.tmin)
					{
						result.tmin = t;
						result.segMin = j;
						// S enters after leaving polygon
						if (result.tmin > result.tmax)
						{
							return result;
						}
					}
				}
				else
				{
					// segment S is leaving across this edge
					if (t < result.tmax)
					{
						result.tmax = t;
						result.segMax = j;
						// S leaves before entering polygon
						if (result.tmax < result.tmin)
						{
							return result;
						}
					}
				}
			}
			result.intersects = true;
			return result;
		}

		public static float distancePtSegSqr2D(float[] pt, float[] verts, int p, int q, ref float t)
		{
			float pqx = verts[q + 0] - verts[p + 0];
			float pqz = verts[q + 2] - verts[p + 2];
			float dx = pt[0] - verts[p + 0];
			float dz = pt[2] - verts[p + 2];
			float d = pqx * pqx + pqz * pqz;

			t = pqx * dx + pqz * dz;
			if (d > 0)
			{
				t /= d;
			}
			if (t < 0)
			{
				t = 0;
			}
			else if (t > 1)
			{
				t = 1;
			}
			dx = verts[p + 0] + t * pqx - pt[0];
			dz = verts[p + 2] + t * pqz - pt[2];

			return dx * dx + dz * dz;
		}

		internal static int oppositeTile(int side)
		{
			return (side + 4) & 0x7;
		}

		internal static float vperpXZ(float[] a, float[] b)
		{
			return a[0] * b[2] - a[2] * b[0];
		}


		[Obsolete]
		internal static bool intersectSegSeg2D(float[] ap, float[] aq, float[] bp, float[] bq, ref float s, ref float t)
		{
			/// <summary>
			///*TODO:��Ҫ�Ż�** </summary>
			float[] u = new float[3];
			float[] v = new float[3];
			float[] w = new float[3];
			vSub(u, aq, ap);
			vSub(v, bq, bp);
			vSub(w, ap, bp);
			float d = vperpXZ(u, v);
			if (Math.Abs(d) < 1e-6f)
			{
				return false;
			}
			s = vperpXZ(v, w) / d;
			t = vperpXZ(u, w) / d;
			//result.first = s;
			//result.second = t;
			return true;
		}





		public static float[] vScale(float[] @in, float scale)
		{
			float[] @out = new float[3];
			@out[0] = @in[0] * scale;
			@out[1] = @in[1] * scale;
			@out[2] = @in[2] * scale;
			return @out;
		}


		/// 
		/// <param name="out"> </param>
		/// <param name="in"> </param>
		/// <param name="scale"> </param>
		public static void vScale(float[] @out, float[] @in, float scale)
		{
			if (@out == null)
			{
				@out = new float[3];
			}
			@out[0] = @in[0] * scale;
			@out[1] = @in[1] * scale;
			@out[2] = @in[2] * scale;
		}



		/// <summary>
		/// ����ָ�������� </summary>
		/// <param name="reset"> </param>
		public static void vResetArray(float[] reset)
		{
			for (int i = 0; i < reset.Length; i++)
			{
				reset[i] = 0;
			}
		}

		/// <summary>
		/// ����ָ�������� </summary>
		/// <param name="reset"> </param>
		public static void vResetArray(long[] reset)
		{
			for (int i = 0; i < reset.Length; i++)
			{
				reset[i] = 0;
			}
		}


	}

}