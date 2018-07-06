namespace org.recast4j.detour
{

	/// <summary>
	/// Ԫ��
	/// @author ����
	/// @create 2016��3��16�� </summary>
	/// @param <T> </param>
	public class IntObjectTupple2<T>
	{
		public readonly int first;
		public readonly T second;

		public IntObjectTupple2(int first, T second)
		{
			this.first = first;
			this.second = second;
		}

		public virtual int First
		{
			get
			{
				return first;
			}
		}

		public virtual T Second
		{
			get
			{
				return second;
			}
		}


	}

}