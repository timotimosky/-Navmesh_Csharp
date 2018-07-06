namespace org.recast4j.detour.io
{


	public class NavMeshParamWriter : DetourWriter
	{

//JAVA TO C# CONVERTER WARNING: Method 'throws' clauses are not available in .NET:
//ORIGINAL LINE: public void write(java.io.OutputStream stream, org.recast4j.detour.NavMeshParams params, java.nio.ByteOrder order) throws java.io.IOException
		public virtual void write(System.IO.Stream stream, NavMeshParams @params, ByteOrder order)
		{
			write(stream, @params.orig[0], order);
			write(stream, @params.orig[1], order);
			write(stream, @params.orig[2], order);
			write(stream, @params.tileWidth, order);
			write(stream, @params.tileHeight, order);
			write(stream, @params.maxTiles, order);
			write(stream, @params.maxPolys, order);
		}

	}

}