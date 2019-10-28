//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffGcodeWriter.h"
#include "InsetOrderOptimizer.h"
#include "WallOverlap.h"
#include "LayerPlan.h"



bool InsetOrderOptimizer::processInsetsWithOptimizedOrdering()
{
	added_something = false;
	const unsigned int num_insets = part.insets.size();

	// if overlap compensation is enabled, gather all the level 0 and/or level X walls together
	// and initialise the respective overlap computers
	// NOTE: this code assumes that the overlap computers do not alter the order or number of the polys!
	Polygons wall_0_polys;
	bool travel_compensate_overlapping_walls_0_enabled = true;
	if (travel_compensate_overlapping_walls_0_enabled)
	{
		wall_0_polys = part.insets[0];
		wall_overlapper_0 = new WallOverlapComputation(wall_0_polys, mesh_config.inset0_config.getLineWidth());
	}

	Polygons wall_x_polys;
	bool travel_compensate_overlapping_walls_x_enabled = true;
	if (travel_compensate_overlapping_walls_x_enabled)
	{
		for (unsigned int inset_level = 1; inset_level < num_insets; ++inset_level)
		{
			wall_x_polys.add(part.insets[inset_level]);
		}
		// use a slightly reduced line width so that compensation only occurs between insets at the same level (and not between insets in adjacent levels)
		wall_overlapper_x = new WallOverlapComputation(wall_x_polys, mesh_config.insetX_config.getLineWidth() - 1);
	}

	// create a vector of vectors containing all the inset polys
	inset_polys.clear();

	// if overlap compensation is enabled, use the polys that have been tweaked by the
	// overlap computers, otherwise use the original, un-compensated, polys

	inset_polys.emplace_back();
	for (unsigned int poly_idx = 0; poly_idx < part.insets[0].size(); ++poly_idx)
	{
		if (wall_overlapper_0)
		{
			inset_polys[0].push_back(wall_0_polys[poly_idx]);
		}
		else
		{
			inset_polys[0].push_back(part.insets[0][poly_idx]);
		}
	}

	unsigned int wall_x_polys_index = 0;
	for (unsigned int inset_level = 1; inset_level < num_insets; ++inset_level)
	{
		inset_polys.emplace_back();
		for (unsigned int poly_idx = 0; poly_idx < part.insets[inset_level].size(); ++poly_idx)
		{
			if (wall_overlapper_x)
			{
				inset_polys[inset_level].push_back(wall_x_polys[wall_x_polys_index++]);
			}
			else
			{
				inset_polys[inset_level].push_back(part.insets[inset_level][poly_idx]);
			}
		}
	}

	// if the print has thin walls due to the distance from a hole to the outer wall being smaller than a line width, it will produce a nicer finish on
	// the outer wall if it is printed before the holes because the outer wall does not get flow reduced but the hole walls will get flow reduced where
	// they are close to the outer wall. However, we only want to do this if the level 0 insets are being printed before the higher level insets.
	

	if (layer_nr == 0)
	{
		// first process the outer wall only
		processOuterWallInsets(true, false);

		// then process all the holes and their enclosing insets
		processHoleInsets();

		// finally, process the insets enclosed by the part's outer wall
		processOuterWallInsets(false, true);
	}
	else
	{
		// first process all the holes and their enclosing insets
		processHoleInsets();

		// then process the part's outer wall and its enclosed insets
		processOuterWallInsets(true, true);
	}

	// finally, mop up all the remaining insets that can occur in the gaps between holes
	if (extruder_nr ==0)
	{
		Polygons remaining;
		for (unsigned int inset_level = 1; inset_level < inset_polys.size(); ++inset_level)
		{
			const unsigned int num_polys = inset_polys[inset_level].size();
			if (inset_level == 1 && num_polys > 0)
			{
				printf("##Error Layer %d, %lu level 1 insets remaining to be output (should be 0!)\n", layer_nr, num_polys);
			}
			for (unsigned int poly_idx = 0; poly_idx < num_polys; ++poly_idx)
			{
				remaining.add(*inset_polys[inset_level][poly_idx]);
			}
		}
		if (remaining.size() > 0)
		{
			//gcode_writer.setExtruder_addPrime(storage, gcode_layer, extruder_nr);
			gcode_layer.setIsInside(true); // going to print stuff inside print object
			gcode_layer.addWalls(remaining, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlapper_x);
			added_something = true;
		}
	}
	if (wall_overlapper_0)
	{
		delete wall_overlapper_0;
		wall_overlapper_0 = nullptr;
	}
	if (wall_overlapper_x)
	{
		delete wall_overlapper_x;
		wall_overlapper_x = nullptr;
	}
	return added_something;
}


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
