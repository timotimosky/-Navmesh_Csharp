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


	public class IOUtils
	{

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public static ByteBuffer toByteBuffer(java.io.InputStream inputStream) throws java.io.IOException
		public static ByteBuffer toByteBuffer(System.IO.Stream inputStream)
		{
			ByteArrayOutputStream baos = new ByteArrayOutputStream();
			sbyte[] buffer = new sbyte[4096];
			int l;
			while ((l = inputStream.Read(buffer, 0, buffer.Length)) != -1)
			{
				baos.write(buffer, 0, l);
			}
			return ByteBuffer.wrap(baos.toByteArray());
		}

		public static int swapEndianness(int i)
		{
			return (((int)((uint)i >> 24)) & 0xFF) | ((i >> 8) & 0xFF00) | ((i << 8) & 0xFF0000) | ((i << 24) & unchecked((int)0xFF000000));
		}
	}

}