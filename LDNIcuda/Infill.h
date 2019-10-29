#pragma once

#ifndef INFILL_H
#define INFILL_H


#include "ZigZagConnector.h"
#include "ENUMSettings.h"
#include "IntpointIrfan.h"

class AABB;
class SliceMeshStorage;
class SliceLayerPart;

class Infill
{

	static constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors
	EFillMethod pattern; //!< the space filling pattern of the infill to generate
	bool zig_zaggify; 
	bool connect_polygons; //!< Whether to connect as much polygons together into a single path
	const Polygons& in_outline;  //!< the space filling pattern of the infill to generate
	coord_tIrfan outline_offset;
	coord_tIrfan infill_line_width; //!< The line width of the infill lines to generate
	coord_tIrfan line_distance; //!< The distance between two infill lines / polygons
	coord_tIrfan infill_overlap; //!< the distance by which to overlap with the actual area within which to generate infill
	size_t infill_multiplier; //!< the number of infill lines next to each other
	double fill_angle; //!< for linear infill types: the angle of the infill lines (or the angle of the grid)
	coord_tIrfan z; //!< height of the layer for which we generate infill
	coord_tIrfan shift; //!< shift of the scanlines in the direction perpendicular to the fill_angle
	size_t wall_line_count; //!< Number of walls to generate at the boundary of the infill region, spaced \ref infill_line_width apart
	const curaIrfan::PointIrfan infill_origin; //!< origin of the infill pattern
	Polygons* perimeter_gaps; //!< (optional output) The areas in between consecutive insets when Concentric infill is used.
	bool connected_zigzags; //!< (ZigZag) Whether endpieces of zigzag infill should be connected to the nearest infill line on both sides of the zigzag connector
	bool use_endpieces; //!< (ZigZag) Whether to include endpieces: zigzag connector segments from one infill line to itself
	bool skip_some_zags;  //!< (ZigZag) Whether to skip some zags
	size_t zag_skip_count;  //!< (ZigZag) To skip one zag in every N if skip some zags is enabled
	coord_tIrfan pocket_size; //!< The size of the pockets at the intersections of the fractal in the cross 3d pattern

	static constexpr double one_over_sqrt_2 = 0.7071067811865475244008443621048490392848359376884740; //!< 1.0 / sqrt(2.0)
public:
	/*!
	 * \warning If \p perimeter_gaps is given, then the difference between the \p in_outline
	 * and the polygons which result from offsetting it by the \p outline_offset
	 * and then expanding it again by half the \p infill_line_width
	 * is added to the \p perimeter_gaps
	 *
	 * \param[out] perimeter_gaps (optional output) The areas in between consecutive insets when Concentric infill is used.
	 */
	Infill(EFillMethod pattern
		, bool zig_zaggify
		, bool connect_polygons
		, const Polygons& in_outline
    	, coord_tIrfan outline_offset
		, coord_tIrfan infill_line_width
		, coord_tIrfan line_distance
		, coord_tIrfan infill_overlap
		, size_t infill_multiplier
		, double fill_angle
		, coord_tIrfan z
		, coord_tIrfan shift
		, size_t wall_line_count = 0
		, const curaIrfan::PointIrfan& infill_origin = curaIrfan::PointIrfan()
		, Polygons* perimeter_gaps = nullptr
		, bool connected_zigzags = false
		, bool use_endpieces = false
		, bool skip_some_zags = false
		, size_t zag_skip_count = 0
		, coord_tIrfan pocket_size = 0
	)
		: pattern(pattern)
		, zig_zaggify(zig_zaggify)
		, connect_polygons(connect_polygons)
		, in_outline(in_outline)
		, outline_offset(outline_offset)
		, infill_line_width(infill_line_width)
		, line_distance(line_distance)
		, infill_overlap(infill_overlap)
		, infill_multiplier(infill_multiplier)
		, fill_angle(fill_angle)
		, z(z)
		, shift(shift)
		, wall_line_count(wall_line_count)
		, infill_origin(infill_origin)
		, perimeter_gaps(perimeter_gaps)
		, connected_zigzags(connected_zigzags)
		, use_endpieces(use_endpieces)
		, skip_some_zags(skip_some_zags)
		, zag_skip_count(zag_skip_count)
		, pocket_size(pocket_size)
	{
	}

public:

	void generate(Polygons& result_lines, const SliceLayerPart& part);
private:
	//void _generate(Polygons& result_lines, Polygons polygon);
	void _generate(Polygons& result_lines);
	void generatetriangleinfill(Polygons& result);

	void generateLineInfill(Polygons& result, int line_distance, int infill_rotation, coord_tIrfan shift);

	void generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const curaIrfan::PointMatrix& rotation_matrix, ZigzagConnectorProcessor& zigzag_connector_processor, const bool connected_zigzags, coord_tIrfan extra_shift);


	//void generateLinearBasedInfill(const int outline_offset, Polygons& result, const int line_distance, const curaIrfan::PointMatrix& rotation_matrix,  const bool connected_zigzags, Polygons polygons);

	//void addLineInfill(Polygons& result, const curaIrfan::PointMatrix& rotation_matrix, const int scanline_min_idx, int line_distance, const AABB boundary, std::vector<std::vector<coord_tIrfan>>& cut_list);
	void addLineInfill(Polygons& result, const curaIrfan::PointMatrix& rotation_matrix, int scanline_min_idx, int line_distance, AABB boundary, std::vector<std::vector<coord_tIrfan>>& cut_list, coord_tIrfan total_shift);
	struct InfillLineSegment
	{
		/*!
		 * Creates a new infill line segment.
		 *
		 * The previous and next line segments will not yet be connected. You
		 * have to set those separately.
		 * \param start Where the line segment starts.
		 * \param end Where the line segment ends.
		 */
		InfillLineSegment(const curaIrfan::PointIrfan start, const size_t start_segment, const size_t start_polygon, const curaIrfan::PointIrfan end, const size_t end_segment, const size_t end_polygon)
			: start(start)
			, start_segment(start_segment)
			, start_polygon(start_polygon)
			, end(end)
			, end_segment(end_segment)
			, end_polygon(end_polygon)
			, previous(nullptr)
			, next(nullptr)
		{
		};

		/*!
		 * Where the line segment starts.
		 */
		curaIrfan::PointIrfan start;

		/*!
		 * Which polygon line segment the start of this infill line belongs to.
		 *
		 * This is an index of a vertex in the PolygonRef that this infill line
		 * is inside. It is used to disambiguate between the start and end of
		 * the line segment.
		 */
		size_t start_segment;

		/*!
		 * Which polygon the start of this infill line belongs to.
		 *
		 * This is an index of a PolygonRef that this infill line
		 * is inside. It is used to know which polygon the start segment belongs to.
		 */
		size_t start_polygon;

		/*!
		 * Where the line segment ends.
		 */
		curaIrfan::PointIrfan end;

		/*!
		 * Which polygon line segment the end of this infill line belongs to.
		 *
		 * This is an index of a vertex in the PolygonRef that this infill line
		 * is inside. It is used to disambiguate between the start and end of
		 * the line segment.
		 */
		size_t end_segment;

		/*!
		 * Which polygon the end of this infill line belongs to.
		 *
		 * This is an index of a PolygonRef that this infill line
		 * is inside. It is used to know which polygon the end segment belongs to.
		 */
		size_t end_polygon;

		/*!
		 * The previous line segment that this line segment is connected to, if
		 * any.
		 */
		InfillLineSegment* previous;

		/*!
		 * The next line segment that this line segment is connected to, if any.
		 */
		InfillLineSegment* next;

		/*!
		 * Compares two infill line segments for equality.
		 *
		 * This is necessary for putting line segments in a hash set.
		 * \param other The line segment to compare this line segment with.
		 */
		bool operator ==(const InfillLineSegment& other) const;
	};

	std::vector<std::vector<std::vector<InfillLineSegment*>>> crossings_on_line;
	//void multiplyInfill(Polygons& result_lines);
	void connectLines(Polygons& result_lines);
	coord_tIrfan getShiftOffsetFromInfillOriginAndRotation(const double& infill_rotation);




};
#endif