namespace org.recast4j.detour.io
{


	public class NavMeshParamReader
	{

		public virtual NavMeshParams read(ByteBuffer bb)
		{
			NavMeshParams @params = new NavMeshParams();
			@params.orig[0] = bb.Float;
			@params.orig[1] = bb.Float;
			@params.orig[2] = bb.Float;
			@params.tileWidth = bb.Float;
			@params.tileHeight = bb.Float;
			@params.maxTiles = bb.Int;
			@params.maxPolys = bb.Int;
			return @params;
		}

	}

}