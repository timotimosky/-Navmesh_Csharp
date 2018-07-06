namespace org.recast4j.detour.io
{


	public class NavMeshSetHeader
	{

		internal static readonly int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
		internal const int NAVMESHSET_VERSION = 1;
		internal const int NAVMESHSET_VERSION_RECAST4J = 0x8801;

		internal int magic;
		internal int version;
		internal int numTiles;
		internal NavMeshParams @params = new NavMeshParams();

	}


}