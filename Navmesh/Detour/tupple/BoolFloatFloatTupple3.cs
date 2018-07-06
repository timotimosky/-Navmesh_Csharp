namespace org.recast4j.detour
{

	/// <summary>
	/// �洢3������Ԫ��
	/// @author ����
	/// @create 2016��3��15��
	/// </summary>
	public class BoolFloatFloatTupple3
	{
		public bool first;
		public float second;
		public float third;


		public BoolFloatFloatTupple3()
		{
			this.first = false;
			this.second = 0;
			this.third = 0;
		}


		public BoolFloatFloatTupple3(bool first, float second, float third)
		{
			this.first = first;
			this.second = second;
			this.third = third;
		}


		public virtual bool First
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



		public virtual float Third
		{
			get
			{
				return third;
			}
		}


		public virtual void reset()
		{
			this.first = false;
			this.second = 0;
			this.third = 0;
		}

	}

}