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
namespace org.recast4j.detour.crowd
{







	public class ProximityGrid
	{


		private readonly float m_cellSize;
		private readonly float m_invCellSize;


		private readonly Item[] m_pool;
		private int m_poolHead;
		private readonly int m_poolSize;


		private readonly int[] m_buckets;
		private int m_bucketsSize;


		internal int[] m_bounds = new int[4];

		public ProximityGrid(int poolSize, float cellsize)
		{
			this.m_cellSize = cellsize;
            this.m_invCellSize = 1.0f / m_cellSize;
			//this.m_invCellSize = cellsize;

			m_bucketsSize = DetourCommon.nextPow2(poolSize);
			m_buckets = new int[m_bucketsSize];

			m_poolSize = poolSize;
			m_poolHead = 0;
			m_pool = new Item[poolSize];
			for (int i = 0; i < poolSize; i++)
			{
				m_pool[i] = new Item(this);
			}


			clear();
		}

		internal virtual void clear()
		{
			for (int i = 0; i < m_bucketsSize; i++)
			{
				m_buckets[i] = -1;
			}
			m_poolHead = 0;
			m_bounds[0] = 0xffff;
			m_bounds[1] = 0xffff;
			m_bounds[2] = -0xffff;
			m_bounds[3] = -0xffff;
		}

		internal virtual void addItem(int id, float minx, float miny, float maxx, float maxy)
		{
			int iminx = (int) Math.Floor(minx * m_invCellSize);
			int iminy = (int) Math.Floor(miny * m_invCellSize);
			int imaxx = (int) Math.Floor(maxx * m_invCellSize);
			int imaxy = (int) Math.Floor(maxy * m_invCellSize);

			m_bounds[0] = Math.Min(m_bounds[0], iminx);
			m_bounds[1] = Math.Min(m_bounds[1], iminy);
			m_bounds[2] = Math.Max(m_bounds[2], imaxx);
			m_bounds[3] = Math.Max(m_bounds[3], imaxy);

			int h;
			int idx;
			for (int y = iminy; y <= imaxy; ++y)
			{
				for (int x = iminx; x <= imaxx; ++x)
				{
					if (m_poolHead < m_poolSize)
					{
						h = getHashPos2(x, y);
						idx = m_poolHead;
						m_poolHead++;
						Item item = m_pool[idx];
						item.x = x;
						item.y = y;
						item.id = id;
						item.next = m_buckets[h];
						m_buckets[h] = idx;
					}
				}
			}
		}

		internal virtual int queryItems(float minx, float miny, float maxx, float maxy, int[] result, int maxId)
		{
			int iminx = (int) Math.Floor(minx * m_invCellSize);
			int iminy = (int) Math.Floor(miny * m_invCellSize);
			int imaxx = (int) Math.Floor(maxx * m_invCellSize);
			int imaxy = (int) Math.Floor(maxy * m_invCellSize);

			int n = 0;

			for (int y = iminy; y <= imaxy; ++y)
			{
				for (int x = iminx; x <= imaxx; ++x)
				{
					int h = getHashPos2(x, y);
					int idx = m_buckets[h];
					while (idx != -1)
					{
						Item item = m_pool[idx];
						if (item.x == x && item.y == y)
						{
							int i = 0;
							while (i != n && result[i] != item.id)
							{
								++i;
							}

							if (i == n)
							{
								result[n] = item.id;
								n++;
								if (n >= maxId)
								{
									return n;
								}
							}
						}
						idx = item.next;
					}
				}
			}
			return n;
		}


		internal virtual int getItemCountAt(int x, int y)
		{
			int n = 0;
			int h = getHashPos2(x, y);
			int idx = m_buckets[h];
			while (idx != 0xffff)
			{
				Item item = m_pool[idx];
				if (item.x == x && item.y == y)
				{
					n++;
				}
				idx = item.next;
			}
			return n;
		}


		/// <summary>
		/// ��ö�Ӧ��XY��hashֵ </summary>
		/// <param name="x"> </param>
		/// <param name="y">
		/// @return </param>
		private int getHashPos2(int x, int y)
		{
			return ((x * 73856093) ^ (y * 19349663)) & (m_bucketsSize-1);
		}



		/// <summary>
		/// �ڵ�
		/// @author ����
		/// @create 2016��3��28��
		/// </summary>
		private class Item
		{
			private readonly ProximityGrid outerInstance;

			public Item(ProximityGrid outerInstance)
			{
				this.outerInstance = outerInstance;
			}

			internal int id = -1;
			internal int x, y;
			internal int next = -1;
		}
	}

}