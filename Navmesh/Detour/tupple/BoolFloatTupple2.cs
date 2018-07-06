namespace org.recast4j.detour
{

	/// <summary>
	/// Ԫ��
	/// @author ����
	/// @create 2016��3��15��
	/// </summary>
	public class BoolFloatTupple2
	{

		public readonly bool first;
		public readonly float second;

		public BoolFloatTupple2(bool first, float second)
		{
			this.first = first;
			this.second = second;
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
	}

}