//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include "LayerPlan.h"
#include "MergeInfillLines.h"
#include "PrintFeature.h"
#include "LinearAlgebra2D.h"


MergeInfillLines::MergeInfillLines(ExtruderPlan& plan)
	: extruder_plan(plan)
	, nozzle_size(MM2INT(0.4))// Application::getInstance().current_slice->scene.extruders[extruder_plan.extruder_nr].settings.get<coord_tIrfan>("machine_nozzle_size"))
	, maximum_deviation(MM2INT(0.04))//Application::getInstance().current_slice->scene.extruders[extruder_plan.extruder_nr].settings.get<coord_tIrfan>("meshfix_maximum_deviation"))
	{
		//Just copy the parameters to their fields.
	}

	coord_tIrfan MergeInfillLines::calcPathLength(const curaIrfan::PointIrfan path_start, GCodePath& path) const
	{
		curaIrfan::PointIrfan previous_point = path_start;
		coord_tIrfan result = 0;
		for (const curaIrfan::PointIrfan point : path.points)
		{
			result += curaIrfan::vSize(curaIrfan::operator-(point, previous_point));
			previous_point = point;
		}
		return result;
	}

	/*
	 * first_is_already_merged == false
	 *
	 *       o     o
	 *      /     /
	 *     /     /
	 *    /  +  /      --->   -(t)-o-----o
	 *   /     /
	 *  /     /
	 * o     o
	 *
	 * travel (t) to first location is done through first_path_start_changed and new_first_path_start.
	 * this gets rid of the tiny "blips". Depending on the merged line distance a small gap may appear, but this is
	 * accounted for in the volume.
	 *
	 * first_is_already_merged == true
	 *
	 *                   o
	 *                  /
	 *                 /
	 *    o-----o  +  /     --->    o-----------o   or with slight   o-----o-----o
	 *   /           /             /                   bend         /
	 *  /           /             /                                /
	 * o           o             o                                o
	 *
	 */
	bool MergeInfillLines::mergeLinesSideBySide(const bool first_is_already_merged, GCodePath& first_path, const curaIrfan::PointIrfan first_path_start, GCodePath& second_path, const curaIrfan::PointIrfan second_path_start, curaIrfan::PointIrfan& new_first_path_start, coord_tIrfan& error_area) const
	{
		curaIrfan::PointIrfan average_first_path;

		coord_tIrfan first_path_length = calcPathLength(first_path_start, first_path);
		coord_tIrfan first_path_length_flow = first_path_length * first_path.flow; //To get the volume we don't need to include the line width since it's the same for both lines.
		const coord_tIrfan line_width = first_path.config->getLineWidth();

		if (first_is_already_merged)
		{
			// take second point of path, first_path.points[0]
			average_first_path = first_path.points[0];
		}
		else
		{
			curaIrfan::operator+=(average_first_path, first_path_start);
		//	average_first_path += first_path_start;
			for (const curaIrfan::PointIrfan& point : first_path.points)
			{
				curaIrfan::operator+=(average_first_path, point);
				//average_first_path += point;
			}
			average_first_path = curaIrfan::operator/(average_first_path,static_cast<coord_tIrfan>(first_path.points.size() + 1));
		}

		coord_tIrfan second_path_length = calcPathLength(second_path_start, second_path);
		curaIrfan::PointIrfan average_second_path = second_path_start;
		for (const curaIrfan::PointIrfan& point : second_path.points)
		{
			curaIrfan::operator+=(average_second_path, point);
			//average_second_path += point;
		}
		second_path_length *= second_path.flow;
		coord_tIrfan second_path_length_flow = second_path_length;
		average_second_path = curaIrfan::operator/(average_second_path, static_cast<coord_tIrfan>(second_path.points.size() + 1));
		//average_second_path = average_second_path / static_cast<coord_tIrfan>(second_path.points.size() + 1);

		// predict new length and flow and if the new flow is to big, don't merge. conditions in this part must exactly match the actual merging
		coord_tIrfan new_path_length = first_path_length;
		coord_tIrfan dist2_from_line = 0;
		coord_tIrfan new_error_area = 0;
		coord_tIrfan merged_part_length = 0;
		if (first_is_already_merged)
		{
			// check if the new point is a good extension of last part of existing polyline
			// because of potential accumulation of errors introduced each time a line is merged, we do not allow any error.
			if (first_path.points.size() > 1) {
				dist2_from_line = LinearAlg2D::getDist2FromLine(average_second_path, first_path.points[first_path.points.size() - 2], first_path.points[first_path.points.size() - 1]);
				merged_part_length = curaIrfan::vSize(curaIrfan::operator-(first_path.points[first_path.points.size() - 2], average_second_path));
				new_error_area = sqrt(dist2_from_line) * merged_part_length / 2;
			}
			// The max error margin uses the meshfix_maximum_deviation setting.
			if (first_path.points.size() > 1 && error_area + new_error_area < merged_part_length * maximum_deviation)
			{
				
				new_path_length -= curaIrfan::vSize(curaIrfan::operator-(first_path.points[first_path.points.size() - 2], first_path.points[first_path.points.size() - 1]));
				
				new_path_length += curaIrfan::vSize(curaIrfan::operator-(first_path.points[first_path.points.size() - 2], average_second_path));
			}
			else
			{
				new_path_length += curaIrfan::vSize(curaIrfan::operator-(first_path.points[first_path.points.size() - 1], average_second_path));
				//new_path_length += curaIrfan::vSize(first_path.points[first_path.points.size() - 1] - average_second_path);
			}
		}
		else
		{
			
			new_path_length -= curaIrfan::vSize(curaIrfan::operator-(first_path.points.back(), first_path_start));
			
			new_path_length += curaIrfan::vSize(curaIrfan::operator-(average_second_path, average_first_path));
		}
		double new_flow = ((first_path_length_flow + second_path_length_flow) / static_cast<double>(new_path_length));
		if (new_flow > 3.0 * nozzle_size / line_width)  // line width becomes too wide.
		{
			return false;
		}

		// do the actual merging
		if (first_is_already_merged)
		{
			// check if the new point is a good extension of last part of existing polyline
			// because of potential accumulation of errors introduced each time a line is merged, we do not allow any error.
			if (first_path.points.size() > 1 && error_area + new_error_area < line_width * line_width)
			{
				first_path.points[first_path.points.size() - 1] = average_second_path;
				error_area += new_error_area;
			}
			else
			{
				first_path.points.push_back(average_second_path);
				error_area = 0;
			}
		}
		else
		{
			new_first_path_start = average_first_path;
			first_path.points.clear();
			first_path.points.push_back(average_second_path);
			error_area = 0;
		}

		first_path.flow = new_flow;

		return true;
	}


	bool MergeInfillLines::tryMerge(const bool first_is_already_merged, GCodePath& first_path, const curaIrfan::PointIrfan first_path_start, GCodePath& second_path, const curaIrfan::PointIrfan second_path_start, curaIrfan::PointIrfan& new_first_path_start, coord_tIrfan& error_area) const
	{
		const curaIrfan::PointIrfan first_path_end = first_path.points.back();
		const curaIrfan::PointIrfan second_path_end = second_path.points.back();
		const coord_tIrfan line_width = first_path.config->getLineWidth();

		// This check prevents [CURA-5690] fat skin lines:
		const coord_tIrfan line_width_squared = line_width * line_width;
		if (curaIrfan::vSize2(curaIrfan::operator-(first_path_end, second_path_start)) < line_width_squared || curaIrfan::vSize2(curaIrfan::operator-(first_path_start, second_path_end)) < line_width_squared)
		{
			// Define max_dot_product_squared as 20*20, where 20 micron is the allowed inaccuracy in the dot product, allowing a slight curve:
			constexpr coord_tIrfan max_dot_product_squared = 400;
			
			const curaIrfan::PointIrfan first_direction = curaIrfan::operator-(first_path_end, first_path_start);
			
			const curaIrfan::PointIrfan second_direction = curaIrfan::operator-(second_path_end, second_path_start);

			// Only continue to try-merge at this point if the lines line up straight:
			if (curaIrfan::dot(first_direction, second_direction) + max_dot_product_squared > curaIrfan::vSize(first_direction) * curaIrfan::vSize(second_direction))
			{
				return false;
			}
		}

		//Lines may be adjacent side-by-side then.
		curaIrfan::PointIrfan first_path_leave_point;
		coord_tIrfan merged_size2;
		if (first_is_already_merged)
		{
			first_path_leave_point = first_path.points.back();  // this is the point that's going to merge
		}
		else
		{
			
			first_path_leave_point = curaIrfan::operator/((curaIrfan::operator-(first_path_start, first_path_end)), 2);
		}
		
		const curaIrfan::PointIrfan second_path_destination_point = curaIrfan::operator/((curaIrfan::operator-(second_path_start, second_path_end)), 2);
		
		const curaIrfan::PointIrfan merged_direction = curaIrfan::operator-(second_path_destination_point, first_path_leave_point);
		if (first_is_already_merged)
		{
			merged_size2 = curaIrfan::vSize2(curaIrfan::operator-(second_path_destination_point, first_path.points.back()));  // check distance with last point in merged line that is to be replaced
		}
		else
		{
			merged_size2 = curaIrfan::vSize2(merged_direction);
		}
		if (merged_size2 > 25 * line_width * line_width)
		{
			return false; //Lines are too far away from each other.
		}
		if (merged_direction.X == 0 && merged_direction.Y == 0)
		{
			new_first_path_start = first_path_start;
			return false;  // returning true will not work for the gradual infill
		}

		// Max 1.5 line widths to the side of the merged_direction
		if (LinearAlg2D::getDist2FromLine(first_path_end, second_path_destination_point, curaIrfan::operator+(second_path_destination_point, merged_direction)) > 2.25 * line_width * line_width
			|| LinearAlg2D::getDist2FromLine(second_path_start, first_path_leave_point, curaIrfan::operator+(first_path_leave_point, merged_direction)) > 2.25 * line_width * line_width
			|| LinearAlg2D::getDist2FromLine(second_path_end, first_path_leave_point, curaIrfan::operator+(first_path_leave_point, merged_direction)) > 2.25 * line_width * line_width)
		{
			return false; //One of the lines is too far from the merged line. Lines would be too wide or too far off.
		}
		if (first_is_already_merged && first_path.points.size() > 1 && first_path.points[first_path.points.size() - 2] == second_path_destination_point)  // yes this can actually happen
		{
			return false;
		}

		return mergeLinesSideBySide(first_is_already_merged, first_path, first_path_start, second_path, second_path_start, new_first_path_start, error_area);
	}

	bool MergeInfillLines::mergeInfillLines(std::vector<GCodePath>& paths, const curaIrfan::PointIrfan& starting_position) const
	{
		/* Algorithm overview:
			1. Loop over all lines to see if they can be merged.
			1a. Check if two adjacent lines can be merged (skipping travel moves in
				between).
			1b. If they can, merge both lines into the first line.
			1c. If they are merged, check next that the first line can be merged
				with the line after the second line.
			2. Do a second iteration over all paths to remove the tombstones. */
		std::vector<size_t> remove_path_indices;
		std::set<size_t> is_merged;
		std::set<size_t> removed;  // keep track of what we already removed, so don't remove it again

		//For each two adjacent lines, see if they can be merged.
		size_t first_path_index = 0;
		curaIrfan::PointIrfan first_path_start = curaIrfan::PointIrfan(starting_position.X, starting_position.Y);  // this one is not going to be overwritten
		size_t second_path_index = 1;
		bool has_first_path = true;// paths.empty() ? false : !paths[0].config->isTravelPath();  // in case the first path is not an extrusion path.
		coord_tIrfan error_area = 0;
		//printf("*****the path size inside optimize paths is %d \n", paths.size());
		for (; second_path_index < paths.size(); second_path_index++)
		{
			//printf("****inside the for loop inside optimize paths is %d \n", paths.size());
			GCodePath& first_path = paths[first_path_index];
			GCodePath& second_path = paths[second_path_index];
			//printf("*****taking the positions first and second \n");
			curaIrfan::PointIrfan second_path_start = paths[second_path_index - 1].points.back();
			//printf("*****@265 \n");
			if (second_path.isTravelPath())
			{
				has_first_path = false;
				continue; //Skip travel paths, we're looking for the first non-travel path.
			}
			//printf("*****@271 \n");
			// FIXME: This is difficult to fix, need to put extra effort into it.
			// CURA-5776:  This works in some cases but it is not exactly correct, because what this will avoid merging
			// lines are kind like in parallel but with a travel move in between, which is a case mergeLinesSideBySide()
			// tries to handle. We found that something can go wrong when it tries to merge some pattern like those
			// parallel lines, and we think that the current merging method is not suitable for that. In short, we
			// probably need treat different patterns with different methods.
			//
			// Use the first non-travel path as the first path that can be used for merging. After we encounter
			// a travel path, we need to find another first non-travel path for merging.
			// if (
			//	(
			//		first_path.config->type == PrintFeatureType::Infill ||
			//		first_path.config->type == PrintFeatureType::SupportInfill ||
			//		first_path.skip_agressive_merge_hint
			//		) && !has_first_path)
			{
				first_path_index = second_path_index;
				first_path_start = second_path_start;
				has_first_path = true;
				continue;
			}
			//printf("*****@293 \n");
			bool allow_try_merge = true;
			// see if we meet criteria to merge. should be: travel - path1 not travel - (...) - travel - path2 not travel - travel
			// we're checking the travels here
			if (first_path_index <= 1 || !paths[first_path_index - 1].isTravelPath())  // "<= 1" because we don't want the first travel being changed. That may introduce a hole somewhere
			{
				allow_try_merge = false;
			}
			if (second_path_index + 1 >= paths.size() || !paths[second_path_index + 1].isTravelPath())
			{
				allow_try_merge = false;
			}
			if (first_path.isTravelPath()) //Don't merge travel moves.
			{
				allow_try_merge = false;
			}
			if (first_path.config->type != second_path.config->type) //Only merge lines that have the same type.
			{
				allow_try_merge = false;
			}
			if (first_path.config->type != PrintFeatureType::Infill && first_path.config->type != PrintFeatureType::Skin) //Only merge skin and infill lines.
			{
				allow_try_merge = false;
			}
			const bool first_is_already_merged = is_merged.find(first_path_index) != is_merged.end();
			if ((!first_is_already_merged && first_path.points.size() > 1) || second_path.points.size() > 1)
			{
				// For now we only merge simple lines, not polylines, to keep it simple.
				// If the first line is already a merged line, then allow it.
				allow_try_merge = false;
			}

			curaIrfan::PointIrfan new_first_path_start;
			if (allow_try_merge && tryMerge(first_is_already_merged, first_path, first_path_start, second_path, second_path_start, new_first_path_start, error_area))
			{
				if (!first_is_already_merged)
				{
					paths[first_path_index - 1].points.back().X = new_first_path_start.X;
					paths[first_path_index - 1].points.back().Y = new_first_path_start.Y;
				}
				/* If we combine two lines, the next path may also be merged into the fist line, so we do NOT update
				first_path_index. */
				for (size_t to_delete_index = first_path_index + 1; to_delete_index <= second_path_index; to_delete_index++)
				{
					if (removed.find(to_delete_index) == removed.end())  // if there are line(s) between first and second, then those lines are already marked as to be deleted, only add the new line(s)
					{
						remove_path_indices.push_back(to_delete_index);
						removed.insert(to_delete_index);
					}
				}
				is_merged.insert(first_path_index);
			}
			else
			{
				/* If we do not combine, the next iteration we must simply merge the
				second path with the line after it. */
				first_path_index = second_path_index;
				first_path_start = second_path_start;
			}
		}

		//Delete all removed lines in one pass so that we need to move lines less often.
		if (!remove_path_indices.empty())
		{
			size_t path_index = remove_path_indices[0];
			/*
			for (size_t removed_position = 1; removed_position < remove_path_indices.size(); removed_position++)
			{
				for (; path_index < remove_path_indices[removed_position] - removed_position; path_index++)
				{
					paths[path_index] = paths[path_index + removed_position]; //Shift all paths.
				}
			}
			for (; path_index < paths.size() - remove_path_indices.size(); path_index++) //Remaining shifts at the end.
			{
				paths[path_index] = paths[path_index + remove_path_indices.size()];
			}
			
			paths.erase(paths.begin() + path_index, paths.end());
			*/
			return true;
			
		}
		else
		{
			return false;
		}
	
}//namespace cura
