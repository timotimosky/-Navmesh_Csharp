namespace org.recast4j.detour.crowd
{

	public class SweepCircleCircleResult
	{

		/// <summary>
		/// ��
		/// </summary>
		public static readonly SweepCircleCircleResult EMPTY_RESULT = new SweepCircleCircleResult(false, 0f, 0f);



		internal readonly bool intersection;
		internal readonly float htmin;
		internal readonly float htmax;

		public SweepCircleCircleResult(bool intersection, float htmin, float htmax)
		{
			this.intersection = intersection;
			this.htmin = htmin;
			this.htmax = htmax;
		}

	}

}