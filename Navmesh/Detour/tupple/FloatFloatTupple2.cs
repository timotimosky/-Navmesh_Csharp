namespace org.recast4j.detour
{

	/// <summary>
	/// Ԫ��
	/// @author ����
	/// @create 2016��3��15��
	/// </summary>
	public class FloatFloatTupple2
	{
		public float first;
		public float second;


		public FloatFloatTupple2()
		{
			this.first = 0;
			this.second = 0;
		}

		public FloatFloatTupple2(float first, float second)
		{
			this.first = first;
			this.second = second;
		}

		public virtual float First
		{
			get
			{
				return first;
			}
		}

		public virtual float Second
		{
			get
			{
				return second;
			}
		}


		public virtual void reset()
		{
			this.first = 0;
		}


	}

}