//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include "sliceDataStorage.h"
#include "WallComputation.h"
#include "Ratio.h"


	WallsComputation::WallsComputation(const int layer_nr)
		: layer_nr(layer_nr)
	{
	}

	
	void WallsComputation::generateInsets(SliceLayerPart* part, SliceLayerPart* part_zigzag)
	{
		size_t inset_count = 4;
		
		const bool spiralize = false;
		const coord_tIrfan wall_0_inset = MM2INT(0.1);
		coord_tIrfan line_width_0 = MM2INT(0.35);
		coord_tIrfan line_width_x = MM2INT(0.30);
		coord_tIrfan line_width_zigzag = MM2INT(0.10);



		if (layer_nr == 0)
		{
			
			line_width_0 *= 100/100;
			line_width_x *= 100/100;
		}
		bool support_enable = false;
		bool support_tree_enable = true;
		bool fill_outline_gaps = false;

		const bool recompute_outline_based_on_outer_wall = (support_enable || support_tree_enable) && !fill_outline_gaps;
	
		for (size_t i = 0; i < inset_count; i++)
		{
			
			part->insets.push_back(Polygons());
			if (i == 0)
			{
				
				part->insets[0] = part->outline.offset(wall_0_inset);
				
				
				
			}
			else if (i == 1)
			{
				
				part->insets[1] = part->insets[0].offset(-line_width_0 / 2 + wall_0_inset - line_width_x / 2);
				
			}
			else if (i == 2)
			{
			
				part->insets[i] = part->insets[i - 1].offset(-line_width_x);
				
			
			}

			else if (i == 3)
			{
				part->insets[i] = part_zigzag->outline_zigzag.offset(wall_0_inset);
			}
			
			const size_t inset_part_count = part->insets[i].size();
			constexpr size_t minimum_part_saving = 3; 
			constexpr coord_tIrfan try_smaller = 10; 
			
			if (inset_part_count > minimum_part_saving + 1 && (i == 0 || (i > 0 && inset_part_count > part->insets[i - 1].size() + minimum_part_saving)))
			{
				
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
				if (alternative_inset.size() < inset_part_count - minimum_part_saving) 
				{
					part->insets[i] = alternative_inset;
				}
			}
			
		     part->insets[i].removeDegenerateVerts();
			
			if (i == 0)
			{
				if (recompute_outline_based_on_outer_wall)
				{
					part->print_outline = part->insets[0].offset(line_width_0 / 2, ClipperLib::jtSquare);
					
				}
				else
				{
					part->print_outline = part->outline;
					
				}
			}
			if (part->insets[i].size() < 1)
			{
				part->insets.pop_back();
				break;
			}
			
		}
		printf("the parts inset size is %d \n", part->insets.size());
		
	}									   						   

	void WallsComputation::generateInsets(SliceLayerPart* part)
	{
		size_t inset_count = 3;
		if (part->getpartMat() == 5)
		{
			size_t inset_count = 1;
		}

		const bool spiralize = false;
		const coord_tIrfan wall_0_inset = MM2INT(0.1);
		coord_tIrfan line_width_0 = MM2INT(0.0);
		coord_tIrfan line_width_x = MM2INT(0.0);

		if (layer_nr == 0)
		{

			line_width_0 *= 100 / 100;
			line_width_x *= 100 / 100;
		}
		bool support_enable = false;
		bool support_tree_enable = true;
		bool fill_outline_gaps = false;

		const bool recompute_outline_based_on_outer_wall = (support_enable || support_tree_enable) && !fill_outline_gaps;

		for (size_t i = 0; i < inset_count; i++)
		{

			part->insets.push_back(Polygons());
			if (i == 0)
			{

				part->insets[0] = part->outline.offset(wall_0_inset);
				//for Intersecting Circles
				/*if (part->getpartMat() == 5)
				{
					part->insets[0] = part->outline.offset(wall_0_inset * 4);
				}*/
				//for Intersecting Circles
				
			}
			else if (i == 1)
			{

				part->insets[1] = part->insets[0].offset(-line_width_0 / 2 + wall_0_inset - line_width_x / 2);
				
			}
			else
			{

				part->insets[i] = part->insets[i - 1].offset(-line_width_x);
				

			}
			
			
			const size_t inset_part_count = part->insets[i].size();
			
			constexpr size_t minimum_part_saving = 3;
			constexpr coord_tIrfan try_smaller = 10;

			if (inset_part_count > minimum_part_saving + 1 && (i == 0 || (i > 0 && inset_part_count > part->insets[i - 1].size() + minimum_part_saving)))
			{

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
				if (alternative_inset.size() < inset_part_count - minimum_part_saving)
				{
					part->insets[i] = alternative_inset;
				}
			}

			part->insets[i].removeDegenerateVerts();

			if (i == 0)
			{
				if (recompute_outline_based_on_outer_wall)
				{
					part->print_outline = part->insets[0].offset(line_width_0 / 2, ClipperLib::jtRound); //jtsquare for the rectangles

				}
				else
				{
					part->print_outline = part->outline;

				}
			}
			if (part->insets[i].size() < 1)
			{
				part->insets.pop_back();
				break;
			}

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
			
			/*if (layer->parts_zigzag[partNr].outline_zigzag.size() != 0)
			{
				generateInsets(&layer->parts[partNr], &layer->parts_zigzag[partNr]);
			}
			else*/			
			{
					
					generateInsets(&layer->parts[partNr]);
					
				
			}
			
			
		}
		
		const bool remove_parts_with_no_insets = true;// !settings.get<bool>("fill_outline_gaps");
		
		for (unsigned int part_idx = 0; part_idx < layer->parts.size(); part_idx++)
		{
			if (layer->parts[part_idx].insets.size() == 0 && remove_parts_with_no_insets)
			{
				if (part_idx != layer->parts.size() - 1)
				{
					layer->parts[part_idx] = std::move(layer->parts.back());
				}
				layer->parts.pop_back();
				part_idx -= 1; 
			}
		}
	}


