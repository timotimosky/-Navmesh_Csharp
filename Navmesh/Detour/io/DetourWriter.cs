/*
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
namespace org.recast4j.detour.io
{


	public abstract class DetourWriter
	{

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: protected void write(java.io.OutputStream stream, float value, java.nio.ByteOrder order) throws java.io.IOException
		protected internal virtual void write(System.IO.Stream stream, float value, ByteOrder order)
		{
			write(stream, float.floatToIntBits(value), order);
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: protected void write(java.io.OutputStream stream, short value, java.nio.ByteOrder order) throws java.io.IOException
		protected internal virtual void write(System.IO.Stream stream, short value, ByteOrder order)
		{
			if (order == ByteOrder.BIG_ENDIAN)
			{
				stream.WriteByte((value >> 8) & 0xFF);
				stream.WriteByte(value & 0xFF);
			}
			else
			{
				stream.WriteByte(value & 0xFF);
				stream.WriteByte((value >> 8) & 0xFF);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: protected void write(java.io.OutputStream stream, long value, java.nio.ByteOrder order) throws java.io.IOException
		protected internal virtual void write(System.IO.Stream stream, long value, ByteOrder order)
		{
			if (order == ByteOrder.BIG_ENDIAN)
			{
				write(stream, (int)((long)((ulong)value >> 32)), order);
				write(stream, unchecked((int)(value & 0xFFFFFFFF)), order);
			}
			else
			{
				write(stream, unchecked((int)(value & 0xFFFFFFFF)), order);
				write(stream, (int)((long)((ulong)value >> 32)), order);
			}
		}

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: protected void write(java.io.OutputStream stream, int value, java.nio.ByteOrder order) throws java.io.IOException
		protected internal virtual void write(System.IO.Stream stream, int value, ByteOrder order)
		{
			if (order == ByteOrder.BIG_ENDIAN)
			{
				stream.WriteByte((value >> 24) & 0xFF);
				stream.WriteByte((value >> 16) & 0xFF);
				stream.WriteByte((value >> 8) & 0xFF);
				stream.WriteByte(value & 0xFF);
			}
			else
			{
				stream.WriteByte(value & 0xFF);
				stream.WriteByte((value >> 8) & 0xFF);
				stream.WriteByte((value >> 16) & 0xFF);
				stream.WriteByte((value >> 24) & 0xFF);
			}
		}

	}

}