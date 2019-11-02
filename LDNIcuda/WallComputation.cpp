//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include "sliceDataStorage.h"
#include "WallComputation.h"
#include "Ratio.h"


	WallsComputation::WallsComputation(const int layer_nr)
		: layer_nr(layer_nr)
	{
	}

	
	void WallsComputation::generateInsets(SliceLayerPart* part)
	{
		size_t inset_count = 3;// settings.get<size_t>("wall_line_count");
		
		const bool spiralize = false;// settings.get<bool>("magic_spiralize");
		
		
	//	printf("the size of the indie the insets functions is parts outline is %d \n", part->outline.pointCount());
		const coord_tIrfan wall_0_inset = MM2INT(0.0);
		coord_tIrfan line_width_0 = MM2INT(0.35);
		coord_tIrfan line_width_x = MM2INT(0.30);

		if (layer_nr == 0)
		{
			//const ExtruderTrain& train_wall_0 = 0;// settings.get<ExtruderTrain&>("wall_0_extruder_nr");
			line_width_0 *= Ratio(120/100);// train_wall_0.settings.get<Ratio>("initial_layer_line_width_factor");
			//const ExtruderTrain& train_wall_x = 0;// settings.get<ExtruderTrain&>("wall_x_extruder_nr");
			line_width_x *= Ratio(120 / 100);// train_wall_x.settings.get<Ratio>("initial_layer_line_width_factor");
		}
		bool support_enable = false;
		bool support_tree_enable = true;
		bool fill_outline_gaps = false;

		const bool recompute_outline_based_on_outer_wall = (support_enable || support_tree_enable) && !fill_outline_gaps;
	//	printf("the recompute_outline_based_on_outer_wall is %d \n", recompute_outline_based_on_outer_wall);
		//const bool recompute_outline_based_on_outer_wall = true;
		for (size_t i = 0; i < inset_count; i++)
		{
			//printf("the i value is %d and inset count is %d \n", i,inset_count);
			part->insets.push_back(Polygons());
			if (i == 0)
			{
				//printf("Starting the inset of the i %d \n", i);
				part->insets[0] = part->outline.offset(-line_width_0 / 2 - wall_0_inset);
				//printf("the distance to offset is %d \n", -line_width_0_int / 2);
				//printf("Succesffully did the inset of the i %d \n", part->insets[0].size());
			}
			else if (i == 1)
			{
				//printf("Starting the inset of the i %d \n", i);
				part->insets[1] = part->insets[0].offset(-line_width_0 / 2 + wall_0_inset - line_width_x / 2);
				//printf("Succesffully did the inset of the i %d  \n", part->insets[1].size());
			}
			else
			{
			//printf("Starting the inset of the i %d \n", i);
				part->insets[i] = part->insets[i - 1].offset(-line_width_x);
				//printf("Succesffully did the inset of the i %d \n", part->insets[i].size());
			}
			//printf("working 2 \n");
			//printf("the insets %d point count is %d \n",i, part->outline.pointCount());
			const size_t inset_part_count = part->insets[i].size();
			constexpr size_t minimum_part_saving = 3; //Only try if the part has more pieces than the previous inset and saves at least this many parts.
			constexpr coord_tIrfan try_smaller = 10; //How many micrometres to inset with the try with a smaller inset.
			//printf("inset_part_count is i %d \n", inset_part_count);
			if (inset_part_count > minimum_part_saving + 1 && (i == 0 || (i > 0 && inset_part_count > part->insets[i - 1].size() + minimum_part_saving)))
			{
				//Try a different line thickness and see if this fits better, based on these criteria:
				// - There are fewer parts to the polygon (fits better in slim areas).
				// - The polygon area is largely unaffected.
				Polygons alternative_inset;
				if (i == 0)
				{
					alternative_inset = part->outline.offset(-(line_width_0 - try_smaller) / 2 - wall_0_inset);
				}
				else if (i == 1)
				{
					alternative_inset = part->insets[0].offset(-(line_width_0 - try_smaller) / 2 + wall_0_inset - line_width_x / 2);
				}
				else
				{
					alternative_inset = part->insets[i - 1].offset(-(line_width_x - try_smaller));

				}
				if (alternative_inset.size() < inset_part_count - minimum_part_saving) //Significantly fewer parts (saves more than 3 parts).
				{
					part->insets[i] = alternative_inset;
				}
			}
			//printf("working 3 \n");
			//printf("Before simplification the size of the indie the insets functions is parts outline is %d and insets size is %d \n", part->insets[i].pointCount(),part->insets[i].size());
			//printf("Starting simplifications simplified the inset of i %d \n", i);
			//Finally optimize all the polygons. Every point removed saves time in the long run.
			
			//part->insets[i].simplify();
			//printf("After Simplification the size of the indie the insets functions is parts outline is %d and the inset is %d \n", part->insets[i].pointCount(),i);
			//printf("succesfully simplified the inset of i %d \n", i);
		     part->insets[i].removeDegenerateVerts();
			//printf("After removing vertices the size of the indie the insets functions is parts outline is %d \n", part->insets[i].pointCount());
			//printf("succesfully removeDegenerateVerts the inset of i %d \n", i);
			if (i == 0)
			{
				if (recompute_outline_based_on_outer_wall)
				{
					part->print_outline = part->insets[0].offset(line_width_0 / 2, ClipperLib::jtSquare);
					//printf("In print outline point insets functions is parts outline is %d \n", part->print_outline.pointCount());
				}
				else
				{
					part->print_outline = part->outline;
					//printf("In print outline point insets functions is parts outline is %d \n", part->print_outline.pointCount());
				}
			}
			if (part->insets[i].size() < 1)
			{
				part->insets.pop_back();
				break;
			}
			//printf("working 4 \n");
			//printf("the size of the indie the insets functions is parts outline is %d and the part number is %d \n", part->insets[i].pointCount());
		}
		

	}

	/*
	 * This function is executed in a parallel region based on layer_nr.
	 * When modifying make sure any changes does not introduce data races.
	 *
	 * generateInsets only reads and writes data for the current layer
	 */
	void WallsComputation::generateInsets(SliceLayer* layer)
	{
		for (unsigned int partNr = 0; partNr < layer->parts.size(); partNr++)
		{
			//printf("working 1 \n");
			generateInsets(&layer->parts[partNr]);
			//printf("working 5 \n");
		//	printf("the inset is genrated for layer %d are %d \n",layer_nr, layer->parts[partNr].insets.size());
		}
	//	printf("the code is at the line 117 of wallcomputation.cpp \n");
		const bool remove_parts_with_no_insets = true;// !settings.get<bool>("fill_outline_gaps");
		//Remove the parts which did not generate an inset. As these parts are too small to print,
		// and later code can now assume that there is always minimal 1 inset line.
		for (unsigned int part_idx = 0; part_idx < layer->parts.size(); part_idx++)
		{
			if (layer->parts[part_idx].insets.size() == 0 && remove_parts_with_no_insets)
			{
				if (part_idx != layer->parts.size() - 1)
				{ // move existing part into part to be deleted
					layer->parts[part_idx] = std::move(layer->parts.back());
				}
				layer->parts.pop_back(); // always remove last element from array (is more efficient)
				part_idx -= 1; // check the part we just moved here
			}
		}
	}

//namespace cura
