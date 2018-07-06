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
namespace org.recast4j.detour
{

	public sealed class Status
	{

		public static readonly Status FAILURE = new Status("FAILURE", InnerEnum.FAILURE, 1 << 31);
		public static readonly Status SUCCSESS = new Status("SUCCSESS", InnerEnum.SUCCSESS, 1 << 30);
		public static readonly Status IN_PROGRESS = new Status("IN_PROGRESS", InnerEnum.IN_PROGRESS, 1 << 29);
		public static readonly Status STATUS_DETAIL_MASK = new Status("STATUS_DETAIL_MASK", InnerEnum.STATUS_DETAIL_MASK, 0x0ffffff);
		public static readonly Status WRONG_MAGIC = new Status("WRONG_MAGIC", InnerEnum.WRONG_MAGIC, 1 << 0);
		public static readonly Status WRONG_VERSION = new Status("WRONG_VERSION", InnerEnum.WRONG_VERSION, 1 << 1);
		public static readonly Status OUT_OF_MEMORY = new Status("OUT_OF_MEMORY", InnerEnum.OUT_OF_MEMORY, 1 << 2);
		public static readonly Status INVALID_PARAM = new Status("INVALID_PARAM", InnerEnum.INVALID_PARAM, 1 << 3);
		public static readonly Status BUFFER_TOO_SMALL = new Status("BUFFER_TOO_SMALL", InnerEnum.BUFFER_TOO_SMALL, 1 << 4);
		public static readonly Status OUT_OF_NODES = new Status("OUT_OF_NODES", InnerEnum.OUT_OF_NODES, 1 << 5);
		public static readonly Status PARTIAL_RESULT = new Status("PARTIAL_RESULT", InnerEnum.PARTIAL_RESULT, 1 << 6);

		private static readonly IList<Status> valueList = new List<Status>();

		static Status()
		{
			valueList.Add(FAILURE);
			valueList.Add(SUCCSESS);
			valueList.Add(IN_PROGRESS);
			valueList.Add(STATUS_DETAIL_MASK);
			valueList.Add(WRONG_MAGIC);
			valueList.Add(WRONG_VERSION);
			valueList.Add(OUT_OF_MEMORY);
			valueList.Add(INVALID_PARAM);
			valueList.Add(BUFFER_TOO_SMALL);
			valueList.Add(OUT_OF_NODES);
			valueList.Add(PARTIAL_RESULT);
		}

		public enum InnerEnum
		{
			FAILURE,
			SUCCSESS,
			IN_PROGRESS,
			STATUS_DETAIL_MASK,
			WRONG_MAGIC,
			WRONG_VERSION,
			OUT_OF_MEMORY,
			INVALID_PARAM,
			BUFFER_TOO_SMALL,
			OUT_OF_NODES,
			PARTIAL_RESULT
		}

		private readonly string nameValue;
		private readonly int ordinalValue;
		private readonly InnerEnum innerEnumValue;
		private static int nextOrdinal = 0;



		private int mask;

		private Status(string name, InnerEnum innerEnum, int mask)
		{
			this.mask = mask;

			nameValue = name;
			ordinalValue = nextOrdinal++;
			innerEnumValue = innerEnum;
		}



		public int Mask
		{
			get
			{
				return mask;
			}
		}




		public bool Failed
		{
			get
			{
				return this == FAILURE;
			}
		}

		public bool InProgress
		{
			get
			{
				return this == IN_PROGRESS;
			}
		}

		public bool Success
		{
			get
			{
				return this == Status.SUCCSESS || this == Status.PARTIAL_RESULT;
			}
		}

		public bool Partial
		{
			get
			{
				return this == Status.PARTIAL_RESULT;
			}
		}

		public static IList<Status> values()
		{
			return valueList;
		}

		public InnerEnum InnerEnumValue()
		{
			return innerEnumValue;
		}

		public int ordinal()
		{
			return ordinalValue;
		}

		public override string ToString()
		{
			return nameValue;
		}

		public static Status valueOf(string name)
		{
			foreach (Status enumInstance in Status.values())
			{
				if (enumInstance.nameValue == name)
				{
					return enumInstance;
				}
			}
			throw new System.ArgumentException(name);
		}
	}


}