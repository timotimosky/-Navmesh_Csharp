﻿/*
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
using System;
using System.Collections.Generic;


namespace org.recast4j.detour
{


	//TODO: (PP) Add comments
	public class FindLocalNeighbourhoodResult
	{
		private readonly List<long> refs;
		private readonly List<long> parentRefs;

		public FindLocalNeighbourhoodResult(List<long> refs, List<long> parentRefs)
		{
			this.refs = refs;
			this.parentRefs = parentRefs;
		}

		public virtual List<long> Refs
		{
			get
			{
				return refs;
			}
		}

		public virtual List<long> ParentRefs
		{
			get
			{
				return parentRefs;
			}
		}


	}
}