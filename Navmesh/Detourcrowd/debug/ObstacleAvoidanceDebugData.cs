using System;

namespace org.recast4j.detour.crowd.debug
{

//JAVA TO C# CONVERTER TODO TASK: This Java 'import static' statement cannot be converted to C#:
//	import static org.recast4j.detour.DetourCommon.clamp;

	public class ObstacleAvoidanceDebugData
	{
		internal int m_nsamples;
		internal int m_maxSamples;
		internal float[] m_vel;
		internal float[] m_ssize;
		internal float[] m_pen;
		internal float[] m_vpen;
		internal float[] m_vcpen;
		internal float[] m_spen;
		internal float[] m_tpen;


		public ObstacleAvoidanceDebugData(int maxSamples)
		{
			m_maxSamples = maxSamples;
			m_vel = new float[3 * m_maxSamples];
			m_pen = new float[m_maxSamples];
			m_ssize = new float[m_maxSamples];
			m_vpen = new float[m_maxSamples];
			m_vcpen = new float[m_maxSamples];
			m_spen = new float[m_maxSamples];
			m_tpen = new float[m_maxSamples];
		}

		public virtual void reset()
		{
			m_nsamples = 0;
		}

		internal virtual void normalizeArray(float[] arr, int n)
		{
			// Normalize penaly range.
			float minPen = float.MaxValue;
			float maxPen = -float.MaxValue;
			for (int i = 0; i < n; ++i)
			{
				minPen = Math.Min(minPen, arr[i]);
				maxPen = Math.Max(maxPen, arr[i]);
			}
			float penRange = maxPen - minPen;
			float s = penRange > 0.001f ? (1.0f / penRange) : 1;
			for (int i = 0; i < n; ++i)
			{
				arr[i] = DetourCommon.clamp((arr[i] - minPen) * s, 0.0f, 1.0f);
			}
		}

		internal virtual void normalizeSamples()
		{
			normalizeArray(m_pen, m_nsamples);
			normalizeArray(m_vpen, m_nsamples);
			normalizeArray(m_vcpen, m_nsamples);
			normalizeArray(m_spen, m_nsamples);
			normalizeArray(m_tpen, m_nsamples);
		}

		public virtual void addSample(float[] vel, float ssize, float pen, float vpen, float vcpen, float spen, float tpen)
		{
			if (m_nsamples >= m_maxSamples)
			{
				return;
			}
			m_vel[m_nsamples * 3] = vel[0];
			m_vel[m_nsamples * 3 + 1] = vel[1];
			m_vel[m_nsamples * 3 + 2] = vel[2];
			m_ssize[m_nsamples] = ssize;
			m_pen[m_nsamples] = pen;
			m_vpen[m_nsamples] = vpen;
			m_vcpen[m_nsamples] = vcpen;
			m_spen[m_nsamples] = spen;
			m_tpen[m_nsamples] = tpen;
			m_nsamples++;
		}

		public virtual int SampleCount
		{
			get
			{
				return m_nsamples;
			}
		}

		public virtual float[] getSampleVelocity(int i)
		{
			float[] vel = new float[3];
			vel[0] = m_vel[i * 3];
			vel[1] = m_vel[i * 3 + 1];
			vel[2] = m_vel[i * 3 + 2];
			return vel;
		}

		public virtual float getSampleSize(int i)
		{
			return m_ssize[i];
		}

		public virtual float getSamplePenalty(int i)
		{
			return m_pen[i];
		}

		public virtual float getSampleDesiredVelocityPenalty(int i)
		{
			return m_vpen[i];
		}

		public virtual float getSampleCurrentVelocityPenalty(int i)
		{
			return m_vcpen[i];
		}

		public virtual float getSamplePreferredSidePenalty(int i)
		{
			return m_spen[i];
		}

		public virtual float getSampleCollisionTimePenalty(int i)
		{
			return m_tpen[i];
		}
	}
}