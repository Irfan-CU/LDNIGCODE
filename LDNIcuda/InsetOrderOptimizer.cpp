//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "InsetOrderOptimizer.h"

	bool InsetOrderOptimizer::optimizingInsetsIsWorthwhile(const SliceLayerPart& part)
	{
		bool optimize_wall_printing_order = true;
		if (!optimize_wall_printing_order)
		{
			// optimization disabled
			return false;
		}
		if (part.insets.size() == 0)
		{
			// no outlines at all, definitely not worth optimizing
			return false;
		}
		if (part.insets.size() < 2 && part.insets[0].size() < 2)
		{
			// only a single outline and no holes, definitely not worth optimizing
			return false;
		}
		// optimize all other combinations of walls and holes
		return true;
	}

//namespace cura
