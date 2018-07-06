using System;
using org.recast4j.detour.crowd.debug;

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

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.clamp;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.distancePtSegSqr2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.sqr;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.triArea2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vCopy;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDist2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vDot2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vNormalize;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vPerp2D;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vScale;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSet;
//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.vSub;


	public class ObstacleAvoidanceQuery
	{

		private const int DT_MAX_PATTERN_DIVS = 32; ///< Max numver of adaptive divs.
		private const int DT_MAX_PATTERN_RINGS = 4; ///< Max number of adaptive rings.





		private ObstacleAvoidanceParams m_params;
		private float m_invHorizTime;
		private float m_vmax;
		private float m_invVmax;

		private int m_maxCircles;
		private ObstacleCircle[] m_circles;
		private int m_ncircles;

		private int m_maxSegments;
		private ObstacleSegment[] m_segments;
		private int m_nsegments;


		private const float EPS = 0.0001f;
		/// <summary>
		///*******����������ʱ����*********** </summary>
		private readonly float[] pporig = new float[3];
		private readonly float[] ppdv = new float[3];

		private readonly float[] sccs = new float[3];

		private readonly float[] srsv = new float[3];
		private readonly float[] srsw = new float[3];

		private readonly float[] psvab = new float[3];
		private readonly float[] pssdir = new float[3];
		private readonly float[] pssnorm = new float[3];

		private readonly float[] svgvcand = new float[3];

		private float R = 0.01f;
		/// <summary>
		///**************************** </summary>


		public ObstacleAvoidanceQuery(int maxCircles, int maxSegments)
		{
			m_maxCircles = maxCircles;
			m_ncircles = 0;
			m_circles = new ObstacleCircle[m_maxCircles];
			for (int i = 0 ; i < m_maxCircles; i++)
			{
				m_circles[i] = new ObstacleCircle();
			}
			m_maxSegments = maxSegments;
			m_nsegments = 0;
			m_segments = new ObstacleSegment[m_maxSegments];
			for (int i = 0 ; i < m_maxSegments; i++)
			{
				m_segments[i] = new ObstacleSegment();
			}
		}

		public virtual void reset()
		{
			m_ncircles = 0;
			m_nsegments = 0;
		}

		public virtual void addCircle(float[] pos, float rad, float[] vel, float[] dvel)
		{
			if (m_ncircles >= m_maxCircles)
			{
				return;
			}

			ObstacleCircle cir = m_circles[m_ncircles++];
			DetourCommon.vCopy(cir.p, pos);
			cir.rad = rad;
            DetourCommon.vCopy(cir.vel, vel);
            DetourCommon.vCopy(cir.dvel, dvel);
		}

		public virtual void addSegment(float[] p, float[] q)
		{
			if (m_nsegments >= m_maxSegments)
			{
				return;
			}
			ObstacleSegment seg = m_segments[m_nsegments++];
            DetourCommon.vCopy(seg.p, p);
            DetourCommon.vCopy(seg.q, q);
		}



		public virtual int ObstacleCircleCount
		{
			get
			{
				return m_ncircles;
			}
		}

		public virtual ObstacleCircle getObstacleCircle(int i)
		{
			return m_circles[i];
		}

		public virtual int ObstacleSegmentCount
		{
			get
			{
				return m_nsegments;
			}
		}

		public virtual ObstacleSegment getObstacleSegment(int i)
		{
			return m_segments[i];
		}



		private void prepare(float[] pos, float[] dvel, float radius)
		{
			// Prepare obstacles
			ObstacleCircle cir;
			float[] pa;
			float[] pb;
			float a;
			for (int i = 0; i < m_ncircles; ++i)
			{
				cir = m_circles[i];

				// Side
				pa = pos;
				pb = cir.p;

                DetourCommon.vSub(cir.dp, pb, pa);
                DetourCommon.vNormalize(cir.dp);
                DetourCommon.vSub(ppdv, cir.dvel, dvel);

                a = DetourCommon.triArea2D(pporig, cir.dp, ppdv);
				if (a < 0.01f)
				{
					cir.np[0] = -cir.dp[2];
					cir.np[2] = cir.dp[0];
				}
				else
				{
					cir.np[0] = cir.dp[2];
					cir.np[2] = -cir.dp[0];
				}
			}
			ObstacleSegment seg;
            float t;
			for (int i = 0; i < m_nsegments; ++i)
			{
				seg = m_segments[i];
                t = 0;
                seg.touch = DetourCommon.distancePtSegSqr2D(pos, seg.p, seg.q, ref t) < DetourCommon.sqr(R);
			}
		}

		internal virtual bool sweepCircleCircle(float[] c0, float r0, float[] v, float[] c1, float r1, ref float tmin, ref float tmax)
		{
			//float[] s = vSub(c1, c0);
			DetourCommon.vSub(sccs, c1, c0);
			float r = r0 + r1;
            float c = DetourCommon.vDot2D(sccs, sccs) - r * r;
            float a = DetourCommon.vDot2D(v, v);
			if (a < EPS)
			{
				return false; // not moving
			}

			// Overlap, calc time to exit.
            float b = DetourCommon.vDot2D(v, sccs);
			float d = b * b - a * c;
			if (d < 0.0f)
			{
				return false; // no intersection.
			}
			a = 1.0f / a;
			float rd = (float) Math.Sqrt(d);
			tmin = (b - rd) * a;
			tmax = (b + rd) * a;
			return true;
		}


		internal virtual bool isectRaySeg(float[] ap, float[] u, float[] bp, float[] bq, ref float t)
		{
			//float[] v = vSub(bq, bp);
			//float[] w = vSub(ap, bp);
            DetourCommon.vSub(srsv, bq, bp);
            DetourCommon.vSub(srsw, ap, bp);
			//vSub(srsw, bq, bp);
            float d = DetourCommon.vPerp2D(u, srsv);
			if (Math.Abs(d) < 1e-6f)
			{
				return false;
			}
			d = 1.0f / d;
            t = DetourCommon.vPerp2D(srsv, srsw) * d;
			if (t < 0 || t > 1)
			{
				return false;
			}
            float s = DetourCommon.vPerp2D(u, srsw) * d;
			if (s < 0 || s > 1)
			{
				return false;
			}
			return true;
		}


		/// <summary>
		/// Calculate the collision penalty for a given velocity vector
		/// </summary>
		/// <param name="vcand"> sampled velocity </param>
		/// <param name="dvel"> desired velocity </param>
		/// <param name="minPenalty"> threshold penalty for early out </param>
		public virtual float processSample(float[] vcand, float cs, float[] pos, float rad, float[] vel, float[] dvel, float minPenalty, ObstacleAvoidanceDebugData debug)
		{
			// penalty for straying away from the desired and current velocities
            float vpen = m_params.weightDesVel * (DetourCommon.vDist2D(vcand, dvel) * m_invVmax);
            float vcpen = m_params.weightCurVel * (DetourCommon.vDist2D(vcand, vel) * m_invVmax);

			// find the threshold hit time to bail out based on the early out penalty
			// (see how the penalty is calculated below to understnad)
			float minPen = minPenalty - vpen - vcpen;
			float tThresold = (float)(((double) m_params.weightToi / (double) minPen - 0.1) * m_params.horizTime);
			if (tThresold - m_params.horizTime > -float.Epsilon)
			{
				return minPenalty; // already too much
			}

			// Find min time of impact and exit amongst all obstacles.
			float tmin = m_params.horizTime;
			float side = 0;
			int nside = 0;

			ObstacleCircle cir;
			//RefFloat htmin = new RefFloat();
			//RefFloat htmax = new RefFloat();

			for (int i = 0; i < m_ncircles; ++i)
			{
				cir = m_circles[i];

				// RVO
				//float[] vab = vScale(vcand, 2);
				//vab = vSub(vab, vel);
				//vab = vSub(vab, cir.vel);
                DetourCommon.vScale(psvab, vcand, 2);
                DetourCommon.vSub(psvab, psvab, vel);
                DetourCommon.vSub(psvab, psvab, cir.vel);

				// Side
                side += DetourCommon.clamp(Math.Min(DetourCommon.vDot2D(cir.dp, psvab) * 0.5f + 0.5f, DetourCommon.vDot2D(cir.np, psvab) * 2), 0.0f, 1.0f);
				nside++;

				float htmin = 0;
                float htmax = 0;
				if (!sweepCircleCircle(pos, rad, psvab, cir.p, cir.rad, ref htmin, ref htmax))
				{
					continue;
				}

				// Handle overlapping obstacles.
				if (htmin < 0.0f && htmax > 0.0f)
				{
					htmin = -htmin * 0.5f;
				}

				if (htmin >= 0.0f)
				{
					// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
					if (htmin < tmin)
					{
						tmin = htmin;
						if (tmin < tThresold)
						{
							return minPenalty;
						}
					}
				}
			}

			ObstacleSegment seg;
			for (int i = 0; i < m_nsegments; ++i)
			{
				seg = m_segments[i];
				float htmin = 0;

				if (seg.touch)
				{
					// Special case when the agent is very close to the segment.
					//float[] sdir = vSub(seg.q, seg.p);
					//float[] snorm = new float[3];
                    DetourCommon.vSub(pssdir, seg.q, seg.p);
                    DetourCommon.vSet(pssnorm, 0, 0, 0);
					pssnorm[0] = -pssdir[2];
					pssnorm[2] = pssdir[0];
					// If the velocity is pointing towards the segment, no collision.
                    if (DetourCommon.vDot2D(pssnorm, vcand) < 0.0001f)
					{
						continue;
					}
					// Else immediate collision.
					htmin = 0.0f;
				}
				else
				{
					if (!isectRaySeg(pos, vcand, seg.p, seg.q, ref htmin))
					{
						continue;
					}
				}

				// Avoid less when facing walls.
				htmin = htmin * 2.0f;

				// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
				if (htmin < tmin)
				{
					tmin = htmin;
					if (tmin < tThresold)
					{
						return minPenalty;
					}
				}
			}

			// Normalize side bias, to prevent it dominating too much.
			if (nside != 0)
			{
				side /= nside;
			}

			float spen = m_params.weightSide * side;
			float tpen = m_params.weightToi * (1.0f / (0.1f + tmin * m_invHorizTime));

			float penalty = vpen + vcpen + spen + tpen;
			// Store different penalties for debug viewing
			if (debug != null)
			{
				debug.addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);
			}

			return penalty;
		}




		public virtual int sampleVelocityGrid(float[] pos, float rad, float vmax, float[] vel, float[] dvel, float[] nvel, ObstacleAvoidanceParams @params, ObstacleAvoidanceDebugData debug)
		{
			prepare(pos, dvel, rad);
			m_params = @params;
			m_invHorizTime = 1.0f / m_params.horizTime;
			m_vmax = vmax;
			m_invVmax = vmax > 0 ? 1.0f / vmax : float.MaxValue;

			//float[] nvel = new float[3];
            DetourCommon.vSet(nvel, 0f, 0f, 0f);

			if (debug != null)
			{
				debug.reset();
			}

			float cvx = dvel[0] * m_params.velBias;
			float cvz = dvel[2] * m_params.velBias;
			float cs = vmax * 2 * (1 - m_params.velBias) / (m_params.gridSize - 1);
			float half = (m_params.gridSize - 1) * cs * 0.5f;

			float minPenalty = float.MaxValue;
			int ns = 0;

			for (int y = 0; y < m_params.gridSize; ++y)
			{
				for (int x = 0; x < m_params.gridSize; ++x)
				{
					//float[] vcand = new float[3];
					svgvcand[0] = cvx + x * cs - half;
					svgvcand[1] = 0f;
					svgvcand[2] = cvz + y * cs - half;

                    if (DetourCommon.sqr(svgvcand[0]) + DetourCommon.sqr(svgvcand[2]) > DetourCommon.sqr(vmax + cs / 2))
					{
						continue;
					}

					float penalty = processSample(svgvcand, cs, pos, rad, vel, dvel, minPenalty, debug);
					ns++;
					if (penalty < minPenalty)
					{
						minPenalty = penalty;
                        DetourCommon.vCopy(nvel, svgvcand);
					}
				}
			}

			return ns;
		}

		// vector normalization that ignores the y-component.
		internal virtual void dtNormalize2D(float[] v)
		{
			float d = (float) Math.Sqrt(v[0] * v[0] + v[2] * v[2]);
			if (d == 0)
			{
				return;
			}
			d = 1.0f / d;
			v[0] *= d;
			v[2] *= d;
		}

		// vector normalization that ignores the y-component.
		internal virtual void dtRotate2D(float[] @out, float[] v, float ang)
		{
			//float[] dest = new float[3];
			float c = (float) Math.Cos(ang);
			float s = (float) Math.Sin(ang);
			@out[0] = v[0] * c - v[2] * s;
			@out[2] = v[0] * s + v[2] * c;
			@out[1] = v[1];
			//return dest;
		}

		internal const float DT_PI = 3.14159265f;


		private readonly float[] svapat = new float[(DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2];
		private readonly float[] svaddir = new float[6];
		private readonly float[] svarotated = new float[3];
		private readonly float[] svares = new float[3];
		private readonly float[] svabvel = new float[3];
		private readonly float[] svavcand = new float[3];


		public virtual int sampleVelocityAdaptive(float[] pos, float rad, float vmax, float[] vel, float[] dvel, float[] nvel, ObstacleAvoidanceParams @params, ObstacleAvoidanceDebugData debug)
		{
			prepare(pos, dvel, rad);
			m_params = @params;
			m_invHorizTime = 1.0f / m_params.horizTime;
			m_vmax = vmax;
			m_invVmax = vmax > 0 ? 1.0f / vmax : float.MaxValue;

			//float[] nvel = new float[3];
            DetourCommon.vSet(nvel, 0f, 0f, 0f);

			if (debug != null)
			{
				debug.reset();
			}

			// Build sampling pattern aligned to desired velocity.
			//float[] pat = new float[(DT_MAX_PATTERN_DIVS * DT_MAX_PATTERN_RINGS + 1) * 2];
			DetourCommon.vResetArray(svapat);
			int npat = 0;

			int ndivs = m_params.adaptiveDivs;
			int nrings = m_params.adaptiveRings;
			int depth = m_params.adaptiveDepth;

            int nd = DetourCommon.clamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
            int nr = DetourCommon.clamp(nrings, 1, DT_MAX_PATTERN_RINGS);
			//int nd2 = nd / 2;
			float da = (1.0f / nd) * DT_PI * 2;
			float ca = (float) Math.Cos(da);
			float sa = (float) Math.Sin(da);

			// desired direction
			//float[] ddir = new float[6];
			DetourCommon.vResetArray(svaddir);
            DetourCommon.vCopy(svaddir, dvel);
			dtNormalize2D(svaddir);
			//float[] rotated = dtRotate2D(ddir, da * 0.5f); // rotated by da/2
			dtRotate2D(svarotated, svaddir, da * 0.5f);
			svaddir[3] = svarotated[0];
			svaddir[4] = svarotated[1];
			svaddir[5] = svarotated[2];

			// Always add sample at zero
			svapat[npat * 2 + 0] = 0;
			svapat[npat * 2 + 1] = 0;
			npat++;

			float r;
			int last1, last2;
			for (int j = 0; j < nr; ++j)
			{
				r = (float)(nr - j) / (float) nr;
				svapat[npat * 2 + 0] = svaddir[(j % 2) * 3] * r;
				svapat[npat * 2 + 1] = svaddir[(j % 2) * 3 + 2] * r;
				last1 = npat * 2;
				last2 = last1;
				npat++;

				for (int i = 1; i < nd - 1; i += 2)
				{
					// get next point on the "right" (rotate CW)
					svapat[npat * 2 + 0] = svapat[last1] * ca + svapat[last1 + 1] * sa;
					svapat[npat * 2 + 1] = -svapat[last1] * sa + svapat[last1 + 1] * ca;
					// get next point on the "left" (rotate CCW)
					svapat[npat * 2 + 2] = svapat[last2] * ca - svapat[last2 + 1] * sa;
					svapat[npat * 2 + 3] = svapat[last2] * sa + svapat[last2 + 1] * ca;

					last1 = npat * 2;
					last2 = last1 + 2;
					npat += 2;
				}

				if ((nd & 1) == 0)
				{
					svapat[npat * 2 + 2] = svapat[last2] * ca - svapat[last2 + 1] * sa;
					svapat[npat * 2 + 3] = svapat[last2] * sa + svapat[last2 + 1] * ca;
					npat++;
				}
			}

			// Start sampling.
			float cr = vmax * (1.0f - m_params.velBias);
			//float[] res = new float[3];
            DetourCommon.vSet(svares, dvel[0] * m_params.velBias, 0, dvel[2] * m_params.velBias);
			int ns = 0;
			float minPenalty;
			float penalty;
			for (int k = 0; k < depth; ++k)
			{
				minPenalty = float.MaxValue;
				//float[] bvel = new float[3];
                DetourCommon.vSet(svabvel, 0, 0, 0);

				for (int i = 0; i < npat; ++i)
				{
					//float[] vcand = new float[3];
					svavcand[0] = svares[0] + svapat[i * 2 + 0] * cr;
					svavcand[1] = 0f;
					svavcand[2] = svares[2] + svapat[i * 2 + 1] * cr;

                    if (DetourCommon.sqr(svavcand[0]) + DetourCommon.sqr(svavcand[2]) > DetourCommon.sqr(vmax + 0.001f))
					{
						continue;
					}

					penalty = processSample(svavcand, cr / 10, pos, rad, vel, dvel, minPenalty, debug);
					ns++;
					if (penalty < minPenalty)
					{
						minPenalty = penalty;
                        DetourCommon.vCopy(svabvel, svavcand);
					}
				}

                DetourCommon.vCopy(svares, svabvel);

				cr *= 0.5f;
			}
            DetourCommon.vCopy(nvel, svares);

			return ns;
		}

	}


    public class ObstacleCircle
    {
        /// <summary>
        /// Position of the obstacle </summary>
        internal readonly float[] p = new float[3];
        /// <summary>
        /// Velocity of the obstacle </summary>
        internal readonly float[] vel = new float[3];
        /// <summary>
        /// Velocity of the obstacle </summary>
        internal readonly float[] dvel = new float[3];
        /// <summary>
        /// Radius of the obstacle </summary>
        internal float rad;
        /// <summary>
        /// Use for side selection during sampling. </summary>
        internal readonly float[] dp = new float[3];
        /// <summary>
        /// Use for side selection during sampling. </summary>
        internal readonly float[] np = new float[3];
    }

    public class ObstacleSegment
    {
        /// <summary>
        /// End points of the obstacle segment </summary>
        internal readonly float[] p = new float[3];
        /// <summary>
        /// End points of the obstacle segment </summary>
        internal readonly float[] q = new float[3];
        internal bool touch;
    }

    public class ObstacleAvoidanceParams
    {
        public float velBias;
        public float weightDesVel;
        public float weightCurVel;
        public float weightSide;
        public float weightToi;
        public float horizTime;
        public int gridSize; ///< grid
        public int adaptiveDivs; ///< adaptive
        public int adaptiveRings; ///< adaptive
        public int adaptiveDepth; ///< adaptive

        public ObstacleAvoidanceParams()
        {
            velBias = 0.4f;
            weightDesVel = 2.0f;
            weightCurVel = 0.75f;
            weightSide = 0.75f;
            weightToi = 2.5f;
            horizTime = 2.5f;
            gridSize = 33;
            adaptiveDivs = 7;
            adaptiveRings = 2;
            adaptiveDepth = 5;
        }
    }

}