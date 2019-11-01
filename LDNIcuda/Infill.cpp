
#include <algorithm>
#include <functional>
#include <unordered_set>

#include "Infill.h"
#include "SliceDataStorage.h"
#include "UnionFind.h"


static inline int computeScanSegmentIdx(int x, int line_width)
{
	if (x < 0)
	{
		return (x + 1) / line_width - 1;
		// - 1 because -1 belongs to scansegment -1
		// + 1 because -line_width belongs to scansegment -1
	}
	return x / line_width;
}

void Infill::generate(Polygons& result_polygons, Polygons& result_lines)
{
	
	coord_tIrfan outline_offset_raw = outline_offset;
	outline_offset -= wall_line_count * infill_line_width; // account for extra walls
	Polygons generated_result_polygons;
	Polygons generated_result_lines;

	//Polygons polygon = part.infill_area;
	if (in_outline.empty())
	{
		
		//printf("@@the part infill area is empty************************************* \n");
		return;
	}

	//_generate(generated_result_lines, polygon);

	_generate(generated_result_polygons, generated_result_lines);
	result_polygons.add(generated_result_polygons);
	result_lines.add(generated_result_lines);

}

void Infill::_generate(Polygons& result_polygons, Polygons& result_lines)
{
	if (in_outline.empty())
	{
			//printf("Inside generate in_outline is empty \n");
			return; 
	}
		
	if (line_distance == 0)
	{
		//printf("@@Error line_distance == 0 inside generate \n");
		return;
	}

	outline_offset -= infill_line_width / 2; // the infill line zig zag connections must lie next to the border, not on it

	generatetriangleinfill(result_lines); 
	
	printf("@@before connecting infill linzes size inside the infill is %d \n", result_lines.size());

	result_lines.clear();
	connectLines(result_lines);
	printf("@@the connected infill linzes size inside the infill is %d \n", result_lines.size());
	crossings_on_line.clear();
	
	//connectLines(result_lines, polygon);
	
	
}

void Infill::generatetriangleinfill(Polygons& result)
{

	generateLineInfill(result, line_distance, fill_angle, 0);
	generateLineInfill(result, line_distance, fill_angle + 60, 0);
	generateLineInfill(result, line_distance, fill_angle + 120, 0);

}

coord_tIrfan Infill::getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation)
{
	if (infill_origin.X != 0 || infill_origin.Y != 0)
	{
		const double rotation_rads = infill_rotation * M_PI / 180;
		return infill_origin.X * std::cos(rotation_rads) - infill_origin.Y * std::sin(rotation_rads);
	}
	return 0;
}


void Infill::generateLineInfill(Polygons& result, int line_distance, int infill_rotation, coord_tIrfan shift)
{
	
	shift += getShiftOffsetFromInfillOriginAndRotation(infill_rotation);
	curaIrfan::PointMatrix rotation_matrix(infill_rotation);
	NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
	bool connected_zigzags = false;
	generateLinearBasedInfill(outline_offset, result, line_distance, rotation_matrix, lines_processor, connected_zigzags, shift);
}

void Infill::generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const curaIrfan::PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags, coord_tIrfan extra_shift)
{
	if (line_distance == 0)
	{
		return;
	}
	if (in_outline.size() == 0)
	{
		return;
	}

	coord_tIrfan shift = extra_shift + this->shift;

	Polygons outline = in_outline.offset(outline_offset + infill_overlap);

	if (outline.size() == 0)
	{
		return;
	}
	//TODO: Currently we find the outline every time for each rotation.
	//We should compute it only once and rotate that accordingly.
	//We'll also have the guarantee that they have the same size every time.
	//Currently we assume that the above operations are all rotation-invariant,
	//which they aren't if vertices fall on the same coordinate due to rounding.
	crossings_on_line.resize(outline.size()); //One for each polygon.

	outline.applyMatrix(rotation_matrix);

	if (shift < 0)
	{
		shift = line_distance - (-shift) % line_distance;
	}
	else
	{
		shift = shift % line_distance;
	}

	AABB boundary(outline);
	for (int i = 0; i < outline.size(); i++)
	{
		ConstPolygonRef polyon = outline[i];
		for (int j = 0; j < polyon.size(); j++)
		{
			curaIrfan::PointIrfan check = polyon[j];
			printf("the check is %f  %f \n ", INT2MM(check.X), INT2MM(check.Y));
		}
	}
	//printf(" the boundary max in %d and max is %d and line distance is %d \n", boundary.min.X, boundary.max.Y, line_distance);
	int scanline_min_idx = computeScanSegmentIdx(boundary.min.X - shift, line_distance);
	printf("the boundary dimesions are %f %f  %f \n ",INT2MM(boundary.min.X),INT2MM(boundary.max.X),INT2MM(line_distance));
	int line_count = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1 - scanline_min_idx;
	printf("the line count is %d zn", line_count);

	std::vector<std::vector<coord_tIrfan>> cut_list; // mapping from scanline to all intersections with polygon segments

	for (int scanline_idx = 0; scanline_idx < line_count; scanline_idx++)
	{
		cut_list.push_back(std::vector<coord_tIrfan>());
	}

	//When we find crossings, keep track of which crossing belongs to which scanline and to which polygon line segment.
	//Then we can later join two crossings together to form lines and still know what polygon line segments that infill line connected to.
	struct Crossing
	{
		Crossing(curaIrfan::PointIrfan coordinate, size_t polygon_index, size_t vertex_index) : coordinate(coordinate), polygon_index(polygon_index), vertex_index(vertex_index) {};
		curaIrfan::PointIrfan coordinate;
		size_t polygon_index;
		size_t vertex_index;
		bool operator <(const Crossing& other) const //Crossings will be ordered by their Y coordinate so that they get ordered along the scanline.
		{
			return coordinate.Y < other.coordinate.Y;
		}
	};
	std::vector<std::vector<Crossing>> crossings_per_scanline; //For each scanline, a list of crossings.
	const int min_scanline_index = computeScanSegmentIdx(boundary.min.X - shift, line_distance) + 1;
	const int max_scanline_index = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1;

	//printf("the min scaline idx is %d nad max is %d \n", min_scanline_index, max_scanline_index);
	
	crossings_per_scanline.resize(max_scanline_index - min_scanline_index);
	
	for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
	{
		
		PolygonRef poly = outline[poly_idx];
		crossings_on_line[poly_idx].resize(poly.size()); //One for each line in this polygon.
		
		curaIrfan::PointIrfan p0 = poly.back();
		zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type

		for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
		{
			curaIrfan::PointIrfan p1 = poly[point_idx];
			if (p1.X == p0.X)
			{
				zigzag_connector_processor.registerVertex(p1);
				// TODO: how to make sure it always adds the shortest line? (in order to prevent overlap with the zigzag connectors)
				// note: this is already a problem for normal infill, but hasn't really bothered anyone so far.
				p0 = p1;
				continue;
			}

			int scanline_idx0;
			int scanline_idx1;
			// this way of handling the indices takes care of the case where a boundary line segment ends exactly on a scanline:
			// in case the next segment moves back from that scanline either 2 or 0 scanline-boundary intersections are created
			// otherwise only 1 will be created, counting as an actual intersection
			int direction = 1;
			if (p0.X < p1.X)
			{
				scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
				scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance); // -1 cause the vertex point is handled in the next segment (or not in the case which looks like >)
			}
			else
			{
				direction = -1;
				scanline_idx0 = computeScanSegmentIdx(p0.X - shift, line_distance); // -1 cause the vertex point is handled in the previous segment (or not in the case which looks like >)
				scanline_idx1 = computeScanSegmentIdx(p1.X - shift, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
			}

			for (int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1 + direction; scanline_idx += direction)
			{
				int x = scanline_idx * line_distance + shift;
				int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
				assert(scanline_idx - scanline_min_idx >= 0 && scanline_idx - scanline_min_idx < int(cut_list.size()) && "reading infill cutlist index out of bounds!");
				cut_list[scanline_idx - scanline_min_idx].push_back(y);
				curaIrfan::PointIrfan scanline_linesegment_intersection(x, y);
				zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx);
				crossings_per_scanline[scanline_idx - min_scanline_index].emplace_back(scanline_linesegment_intersection, poly_idx, point_idx);
			}
			zigzag_connector_processor.registerVertex(p1);
			p0 = p1;
		}
		zigzag_connector_processor.registerPolyFinished();
	}

	//Gather all crossings per scanline and find out which crossings belong together, then store them in crossings_on_line.
	for (int scanline_index = min_scanline_index; scanline_index < max_scanline_index; scanline_index++)
	{
		std::sort(crossings_per_scanline[scanline_index - min_scanline_index].begin(), crossings_per_scanline[scanline_index - min_scanline_index].end()); //Sorts them by Y coordinate.
		for (long crossing_index = 0; crossing_index < static_cast<long>(crossings_per_scanline[scanline_index - min_scanline_index].size()) - 1; crossing_index += 2) //Combine each 2 subsequent crossings together.
		{
			const Crossing& first = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index];
			const Crossing& second = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index + 1];
			//Avoid creating zero length crossing lines
			const curaIrfan::PointIrfan unrotated_first = rotation_matrix.unapply(first.coordinate);
			const curaIrfan::PointIrfan unrotated_second = rotation_matrix.unapply(second.coordinate);
			if (unrotated_first == unrotated_second)
			{
				continue;
			}
			InfillLineSegment* new_segment = new InfillLineSegment(unrotated_first, first.vertex_index, first.polygon_index, unrotated_second, second.vertex_index, second.polygon_index);
			//Put the same line segment in the data structure twice: Once for each of the polygon line segment that it crosses.
			crossings_on_line[first.polygon_index][first.vertex_index].push_back(new_segment);
			crossings_on_line[second.polygon_index][second.vertex_index].push_back(new_segment);
		}
	}

	if (cut_list.size() == 0)
	{
		return;
	}
	if (connected_zigzags && cut_list.size() == 1 && cut_list[0].size() <= 2)
	{
		return;  // don't add connection if boundary already contains whole outline!
	}
	printf("the cutlist size is %d \n", cut_list.size());

	addLineInfill(result, rotation_matrix, scanline_min_idx, line_distance, boundary, cut_list, shift);
}

/*
void Infill::generateLineInfill(Polygons& result, int line_distance, double infill_rotation, Polygons polygons)
{
	outline_offset = 0.0;
	printf("infill_rotation is %f \n ", infill_rotation);
	//shift += getShiftOffsetFromInfillOriginAndRotation(infill_rotation);
	curaIrfan::PointMatrix rotation_matrix(infill_rotation);
	NoZigZagConnectorProcessor lines_processor(rotation_matrix, result);
	bool connected_zigzags = false;

	generateLinearBasedInfill(outline_offset, result, line_distance, rotation_matrix, lines_processor, connected_zigzags, polygons);
}


void Infill::generateLinearBasedInfill( int outline_offset, Polygons& result, int line_distance, curaIrfan::PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, bool connected_zigzags, Polygons polygons)
{
	printf("The line_distance is %d \n", line_distance);
	printf("The polygon1.size() is %d \n", polygons.size());
	printf("The outline_offset is %d \n", outline_offset);

	coord_tIrfan shift = 2.00000;

	if (line_distance == 0)
	{
		return;
	}
	if (polygons.size() == 0)
	{

		return;
	}




	Polygons outline = polygons.offset(outline_offset);
	printf("The outline is done %d \n", outline.size());

	//TODO: Currently we find the outline every time for each rotation.
	//We should compute it only once and rotate that accordingly.
	//We'll also have the guarantee that they have the same size every time.
	//Currently we assume that the above operations are all rotation-invariant,
	//which they aren't if vertices fall on the same coordinate due to rounding.
	crossings_on_line.resize(outline.size()); //One for each polygon.
	printf("The crossing is done \n");
	outline.applyMatrix(rotation_matrix);


	if (shift < 0)
	{
		shift = line_distance - (-shift) % line_distance;
	}
	else
	{
		shift = shift % line_distance;
	}


	AABB boundary(outline);
	printf("The boundary is done \n");
	int scanline_min_idx = computeScanSegmentIdx(boundary.min.X - shift, line_distance);

	int line_count = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1 - scanline_min_idx;

	std::vector<std::vector<coord_tIrfan>> cut_list; // mapping from scanline to all intersections with polygon segments

	for (int scanline_idx = 0; scanline_idx < line_count; scanline_idx++)
	{
		cut_list.push_back(std::vector<coord_tIrfan>());

	}

	//When we find crossings, keep track of which crossing belongs to which scanline and to which polygon line segment.
	//Then we can later join two crossings together to form lines and still know what polygon line segments that infill line connected to.
	struct Crossing
	{
		Crossing(curaIrfan::PointIrfan coordinate, size_t polygon_index, size_t vertex_index) : coordinate(coordinate), polygon_index(polygon_index), vertex_index(vertex_index) {};
		curaIrfan::PointIrfan coordinate;
		size_t polygon_index;
		size_t vertex_index;
		bool operator <(const Crossing& other) const //Crossings will be ordered by their Y coordinate so that they get ordered along the scanline.
		{
			return coordinate.Y < other.coordinate.Y;
		}
	};
	std::vector<std::vector<Crossing>> crossings_per_scanline; //For each scanline, a list of crossings.
	const int min_scanline_index = computeScanSegmentIdx(boundary.min.X - shift, line_distance) + 1;
	const int max_scanline_index = computeScanSegmentIdx(boundary.max.X - shift, line_distance) + 1;
	printf("min_scanline_index is %d \n", min_scanline_index);
	crossings_per_scanline.resize(max_scanline_index - min_scanline_index);

	printf("min_scanline_index is %d \n", min_scanline_index);
	//printf(" max_scanline_index is %d \n", max_scanline_index);
	printf("line count is %d \n", line_count);

	for (size_t poly_idx = 0; poly_idx < outline.size(); poly_idx++)
	{
		PolygonRef poly = outline[poly_idx];
		//printf("poly.size() is %d \n", poly.size());
		//crossings_on_line[poly_idx].resize(poly.size()); //One for each line in this polygon. poly_idx= id of the polygon

		curaIrfan::PointIrfan  p0 = poly.back();
		//zigzag_connector_processor.registerVertex(p0); // always adds the first point to ZigzagConnectorProcessorEndPieces::first_zigzag_connector when using a zigzag infill type
		//printf("point is () is %d \n", poly.size());
		for (size_t point_idx = 0; point_idx < poly.size(); point_idx++)
		{
			curaIrfan::PointIrfan  p1 = poly[point_idx];
			printf("point is %d and %d \n", p1.X, p0.X);
			if (p1.X == p0.X)
			{
				//zigzag_connector_processor.registerVertex(p1);
				// TODO: how to make sure it always adds the shortest line? (in order to prevent overlap with the zigzag connectors)
				// note: this is already a problem for normal infill, but hasn't really bothered anyone so far.
				p0 = p1;
				continue;
			}

			int scanline_idx0;
			int scanline_idx1;
			// this way of handling the indices takes care of the case where a boundary line segment ends exactly on a scanline:
			// in case the next segment moves back from that scanline either 2 or 0 scanline-boundary intersections are created
			// otherwise only 1 will be created, counting as an actual intersection
			int direction = 1;
			if (p0.X < p1.X)
			{
				scanline_idx0 = computeScanSegmentIdx(p0.X, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
				scanline_idx1 = computeScanSegmentIdx(p1.X, line_distance); // -1 cause the vertex point is handled in the next segment (or not in the case which looks like >)
			}
			else
			{
				direction = -1;
				scanline_idx0 = computeScanSegmentIdx(p0.X, line_distance); // -1 cause the vertex point is handled in the previous segment (or not in the case which looks like >)
				scanline_idx1 = computeScanSegmentIdx(p1.X, line_distance) + 1; // + 1 cause we don't cross the scanline of the first scan segment
			}

			for (int scanline_idx = scanline_idx0; scanline_idx != scanline_idx1 + direction; scanline_idx += direction)
			{
				int x = scanline_idx * line_distance;
				int y = p1.Y + (p0.Y - p1.Y) * (x - p1.X) / (p0.X - p1.X);
				assert(scanline_idx - scanline_min_idx >= 0 && scanline_idx - scanline_min_idx < int(cut_list.size()) && "reading infill cutlist index out of bounds!");

				cut_list[scanline_idx - scanline_min_idx].push_back(y);
				curaIrfan::PointIrfan scanline_linesegment_intersection(x, y);
				zigzag_connector_processor.registerScanlineSegmentIntersection(scanline_linesegment_intersection, scanline_idx);
				crossings_per_scanline[scanline_idx - min_scanline_index].emplace_back(scanline_linesegment_intersection, poly_idx, point_idx);
			}
			zigzag_connector_processor.registerVertex(p1);
			p0 = p1;
		}
		zigzag_connector_processor.registerPolyFinished();
	}
	printf("The cutlist size is %d \n", cut_list.size());
	//Gather all crossings per scanline and find out which crossings belong together, then store them in crossings_on_line.
	for (int scanline_index = min_scanline_index; scanline_index < max_scanline_index; scanline_index++)
	{
		std::sort(crossings_per_scanline[scanline_index - min_scanline_index].begin(), crossings_per_scanline[scanline_index - min_scanline_index].end()); //Sorts them by Y coordinate.
		for (long crossing_index = 0; crossing_index < static_cast<long>(crossings_per_scanline[scanline_index - min_scanline_index].size()) - 1; crossing_index += 2) //Combine each 2 subsequent crossings together.
		{
			const Crossing& first = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index];
			const Crossing& second = crossings_per_scanline[scanline_index - min_scanline_index][crossing_index + 1];
			//Avoid creating zero length crossing lines
			const curaIrfan::PointIrfan  unrotated_first = rotation_matrix.unapply(first.coordinate);
			const curaIrfan::PointIrfan  unrotated_second = rotation_matrix.unapply(second.coordinate);
			if (unrotated_first == unrotated_second)
			{
				continue;
			}
			InfillLineSegment* new_segment = new InfillLineSegment(unrotated_first, first.vertex_index, first.polygon_index, unrotated_second, second.vertex_index, second.polygon_index);
			//Put the same line segment in the data structure twice: Once for each of the polygon line segment that it crosses.
			crossings_on_line[first.polygon_index][first.vertex_index].push_back(new_segment);
			crossings_on_line[second.polygon_index][second.vertex_index].push_back(new_segment);
		}
	}
	printf("The crossings on line is done \n");

	if (cut_list.size() == 0)
	{
		return;
	}

	if (connected_zigzags && cut_list.size() == 1 && cut_list[0].size() <= 2)
	{
		return;  // don't add connection if boundary already contains whole outline!
	}
	addLineInfill(result, rotation_matrix, scanline_min_idx, line_distance, boundary, cut_list, shift);

}
*/
void Infill::addLineInfill(Polygons& result, const curaIrfan::PointMatrix& rotation_matrix, int scanline_min_idx, int line_distance, AABB boundary, std::vector<std::vector<coord_tIrfan>>& cut_list, coord_tIrfan shift)
{
	
	auto compare_coord_t = [](const void* a, const void* b)
	{
		coord_tIrfan n = (*(coord_tIrfan*)a) - (*(coord_tIrfan*)b);
		if (n < 0)
		{
			return -1;
		}
		if (n > 0)
		{
			return 1;
		}
		return 0;
	};

	unsigned int scanline_idx = 0;
	for (coord_tIrfan x = scanline_min_idx * line_distance; x < boundary.max.X; x += line_distance)
	{
		if (scanline_idx >= cut_list.size())
		{
			break;
		}
		std::vector<coord_tIrfan>& crossings = cut_list[scanline_idx];
		qsort(crossings.data(), crossings.size(), sizeof(coord_tIrfan), compare_coord_t);
		for (unsigned int crossing_idx = 0; crossing_idx + 1 < crossings.size(); crossing_idx += 2)
		{
			if (crossings[crossing_idx + 1] - crossings[crossing_idx] < infill_line_width / 5)
			{ // segment is too short to create infill
				continue;
			}
			//We have to create our own lines when they are not created by the method connectLines.
			
			if (!zig_zaggify || pattern == EFillMethod::ZIG_ZAG || pattern == EFillMethod::LINES)
			{
				result.addLine(rotation_matrix.unapply(curaIrfan::PointIrfan(x, crossings[crossing_idx])), rotation_matrix.unapply(curaIrfan::PointIrfan(x, crossings[crossing_idx + 1])));
			}
			
		}
		scanline_idx += 1;
	}
	printf("the result add line in add lines is %d \n", result.size());
}

bool Infill::InfillLineSegment::operator ==(const InfillLineSegment& other) const
{
	return start == other.start && end == other.end;
}

void Infill::connectLines(Polygons& result_lines)
{
	//TODO: We're reconstructing the outline here. We should store it and compute it only once.
	Polygons outline = in_outline.offset(outline_offset);//infill_overlap deleted for simplification

    UnionFind<InfillLineSegment*> connected_lines; //Keeps track of which lines are connected to which.
	for (std::vector<std::vector<InfillLineSegment*>>& crossings_on_polygon : crossings_on_line)
	{
		for (std::vector<InfillLineSegment*>& crossings_on_polygon_segment : crossings_on_polygon)
		{
			for (InfillLineSegment* infill_line : crossings_on_polygon_segment)
			{
				if (connected_lines.find(infill_line) == (size_t)-1)
				{
					connected_lines.add(infill_line); //Put every line in there as a separate set.
				}
			}
		}
	}

	for (size_t polygon_index = 0; polygon_index < outline.size(); polygon_index++)
	{
		if (outline[polygon_index].empty())
		{
			continue;
		}

		InfillLineSegment* previous_crossing = nullptr; //The crossing that we should connect to. If nullptr, we have been skipping until we find the next crossing.
		InfillLineSegment* previous_segment = nullptr; //The last segment we were connecting while drawing a line along the border.
		curaIrfan::PointIrfan vertex_before = outline[polygon_index].back();
		for (size_t vertex_index = 0; vertex_index < outline[polygon_index].size(); vertex_index++)
		{
			curaIrfan::PointIrfan vertex_after = outline[polygon_index][vertex_index];

			//Sort crossings on every line by how far they are from their initial point.
			struct CompareByDistance
			{
				CompareByDistance(curaIrfan::PointIrfan to_point, size_t polygon_index, size_t vertex_index) : to_point(to_point), polygon_index(polygon_index), vertex_index(vertex_index) {};
				curaIrfan::PointIrfan to_point; //The distance to this point is compared.
				size_t polygon_index; //The polygon which the vertex_index belongs to.
				size_t vertex_index; //The vertex indicating a line segment. This determines which endpoint of each line should be used.
				inline bool operator ()(InfillLineSegment*& left_hand_side, InfillLineSegment*& right_hand_side) const
				{
					//Find the two endpoints that are relevant.
					const curaIrfan::PointIrfan left_hand_point = (left_hand_side->start_segment == vertex_index && left_hand_side->start_polygon == polygon_index) ? left_hand_side->start : left_hand_side->end;
					const curaIrfan::PointIrfan right_hand_point = (right_hand_side->start_segment == vertex_index && right_hand_side->start_polygon == polygon_index) ? right_hand_side->start : right_hand_side->end;
					curaIrfan::PointIrfan checkpoint = curaIrfan::operator-(left_hand_point, to_point);
					curaIrfan::PointIrfan checkpoint1 = curaIrfan::operator-(right_hand_point, to_point);
					return curaIrfan::vSize(checkpoint) < curaIrfan::vSize(checkpoint1);
				}
			};
			std::sort(crossings_on_line[polygon_index][vertex_index].begin(), crossings_on_line[polygon_index][vertex_index].end(), CompareByDistance(vertex_before, polygon_index, vertex_index));

			for (InfillLineSegment* crossing : crossings_on_line[polygon_index][vertex_index])
			{
				if (!previous_crossing) //If we're not yet drawing, then we have been trying to find the next vertex. We found it! Let's start drawing.
				{
					previous_crossing = crossing;
					previous_segment = crossing;
				}
				else
				{
					const size_t crossing_handle = connected_lines.find(crossing);
					assert(crossing_handle != (size_t)-1);
					const size_t previous_crossing_handle = connected_lines.find(previous_crossing);
					assert(previous_crossing_handle != (size_t)-1);
					if (crossing_handle == previous_crossing_handle) //These two infill lines are already connected. Don't create a loop now. Continue connecting with the next crossing.
					{
						continue;
					}

					//Join two infill lines together with a connecting line.
					//Here the InfillLineSegments function as a linked list, so that they can easily be joined.
					const curaIrfan::PointIrfan previous_point = (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index) ? previous_segment->start : previous_segment->end;
					const curaIrfan::PointIrfan next_point = (crossing->start_segment == vertex_index && crossing->start_polygon == polygon_index) ? crossing->start : crossing->end;
					InfillLineSegment* new_segment;
					// If the segment is zero length, we avoid creating it but still want to connect the crossing with the previous segment
					if (previous_point == next_point)
					{
						if (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index)
						{
							previous_segment->previous = crossing;
						}
						else
						{
							previous_segment->next = crossing;
						}
						new_segment = previous_segment;
					}
					else
					{
						new_segment = new InfillLineSegment(previous_point, vertex_index, polygon_index, next_point, vertex_index, polygon_index); //A connecting line between them.
						new_segment->previous = previous_segment;
						if (previous_segment->start_segment == vertex_index && previous_segment->start_polygon == polygon_index)
						{
							previous_segment->previous = new_segment;
						}
						else
						{
							previous_segment->next = new_segment;
						}
						new_segment->next = crossing;
					}

					if (crossing->start_segment == vertex_index && crossing->start_polygon == polygon_index)
					{
						crossing->previous = new_segment;
					}
					else
					{
						crossing->next = new_segment;
					}
					connected_lines.unite(crossing_handle, previous_crossing_handle);
					previous_crossing = nullptr;
					previous_segment = nullptr;
				}
			}

			//Upon going to the next vertex, if we're drawing, put an extra vertex in our infill lines.
			if (previous_crossing)
			{
				InfillLineSegment* new_segment;
				if (vertex_index == previous_segment->start_segment && polygon_index == previous_segment->start_polygon)
				{
					if (previous_segment->start == vertex_after)
					{
						//Edge case when an infill line ends directly on top of vertex_after: We skip the extra connecting line segment, as that would be 0-length.
						previous_segment = nullptr;
						previous_crossing = nullptr;
					}
					else
					{
						new_segment = new InfillLineSegment(previous_segment->start, vertex_index, polygon_index, vertex_after, (vertex_index + 1) % outline[polygon_index].size(), polygon_index);
						previous_segment->previous = new_segment;
						new_segment->previous = previous_segment;
						previous_segment = new_segment;
					}
				}
				else
				{
					if (previous_segment->end == vertex_after)
					{
						//Edge case when an infill line ends directly on top of vertex_after: We skip the extra connecting line segment, as that would be 0-length.
						previous_segment = nullptr;
						previous_crossing = nullptr;
					}
					else
					{
						new_segment = new InfillLineSegment(previous_segment->end, vertex_index, polygon_index, vertex_after, (vertex_index + 1) % outline[polygon_index].size(), polygon_index);
						previous_segment->next = new_segment;
						new_segment->previous = previous_segment;
						previous_segment = new_segment;
					}
				}
			}

			vertex_before = vertex_after;
		}
	}

	//Save all lines, now connected, to the output.
	
	std::unordered_set<size_t> completed_groups;
	for (InfillLineSegment* infill_line : connected_lines)
	{
		const size_t group = connected_lines.find(infill_line);
		if (completed_groups.find(group) != completed_groups.end()) //We already completed this group.
		{
			continue;
		}

		//Find where the polyline ends by searching through previous and next lines.
		//Note that the "previous" and "next" lines don't necessarily match up though, because the direction while connecting infill lines was not yet known.
		curaIrfan::PointIrfan previous_vertex = infill_line->start; //Take one side arbitrarily to start from. This variable indicates the vertex that connects to the previous line.
		InfillLineSegment* current_infill_line = infill_line;
		while (current_infill_line->next && current_infill_line->previous) //Until we reached an endpoint.
		{
			const curaIrfan::PointIrfan next_vertex = (previous_vertex == current_infill_line->start) ? current_infill_line->end : current_infill_line->start;
			current_infill_line = (previous_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
			previous_vertex = next_vertex;
		}

		//Now go along the linked list of infill lines and output the infill lines to the actual result.
		InfillLineSegment* old_line = current_infill_line;
		const curaIrfan::PointIrfan first_vertex = (!current_infill_line->previous) ? current_infill_line->start : current_infill_line->end;
		previous_vertex = (!current_infill_line->previous) ? current_infill_line->end : current_infill_line->start;
		current_infill_line = (first_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
		result_lines.addLine(first_vertex, previous_vertex);
		delete old_line;
		while (current_infill_line)
		{
			old_line = current_infill_line; //We'll delete this after we've traversed to the next line.
			const curaIrfan::PointIrfan next_vertex = (previous_vertex == current_infill_line->start) ? current_infill_line->end : current_infill_line->start; //Opposite side of the line.
			current_infill_line = (previous_vertex == current_infill_line->start) ? current_infill_line->next : current_infill_line->previous;
			result_lines.addLine(previous_vertex, next_vertex);
			previous_vertex = next_vertex;
			delete old_line;
		}

		completed_groups.insert(group);
	}
	printf("the connected groups inside infill size is %d \n", completed_groups.size());
}