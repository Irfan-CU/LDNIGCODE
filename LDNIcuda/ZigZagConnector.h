#pragma once


#ifndef INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H
#define INFILL_ZIGZAG_CONNECTOR_PROCESSOR_H


#include "Polygon.h"



class ZigzagConnectorProcessor
{
public:
	/*!
	 * Constructor.
	 *
	 * \param rotation_matrix The rotation matrix used to enforce the infill angle
	 * \param result The resulting line segments (Each line segment is a Polygon with 2 points)
	 * \param use_endpieces Whether to include end pieces or not
	 * \param connected_endpieces Whether the end pieces should be connected with the rest part of the infill
	 * \param skip_some_zags Whether to skip some zags
	 * \param zag_skip_count Skip 1 zag in every N zags
	 */
	ZigzagConnectorProcessor(const curaIrfan::PointMatrix& rotation_matrix, Polygons& result,
		bool use_endpieces, bool connected_endpieces,
		bool skip_some_zags, int zag_skip_count)
		: rotation_matrix(rotation_matrix)
		, result(result)
		, use_endpieces(use_endpieces)
		, connected_endpieces(connected_endpieces)
		, skip_some_zags(skip_some_zags)
		, zag_skip_count(zag_skip_count)
		, is_first_connector(true)
		, first_connector_end_scanline_index(0)
		, last_connector_index(0)
	{}

	virtual ~ZigzagConnectorProcessor()
	{}

	/*!
	 * Handle the next vertex on the outer boundary.
	 * \param vertex The vertex
	 */
	virtual void registerVertex(const curaIrfan::PointIrfan & vertex);

	/*!
	 * Handle the next intersection between a scanline and the outer boundary.
	 *
	 * \param intersection The intersection
	 * \param scanline_index Index of the current scanline
	 */
	virtual void registerScanlineSegmentIntersection(const curaIrfan::PointIrfan & intersection, int scanline_index);

	/*!
	 * Handle the end of a polygon and prepare for the next.
	 * This function should reset all member variables.
	 */
	virtual void registerPolyFinished();

protected:
	/*!
	 * Reset the state so it can be used for processing another polygon.
	 */
	void reset();

	/*!
	 * Add a line to the result but not applying the rotation matrix.
	 *
	 * \param from The one end of the line segment
	 * \param to The other end of the line segment
	 */
	void addLine(curaIrfan::PointIrfan from, curaIrfan::PointIrfan  to);

	/*!
	 * Checks whether the current connector should be added or not.
	 *
	 * \param start_scanline_idx the start scanline index of this scanline segment
	 * \param end_scanline_idx The the end scanline index of this scanline segment
	 */
	bool shouldAddCurrentConnector(int start_scanline_idx, int end_scanline_idx) const;

	/*!
	 * Checks whether two points are separated at least by "threshold" microns.
	 * If they are far away from each other enough, the line represented by the two points
	 * will be added; In case they are close, the second point will be set to be the same
	 * as the first and this line won't be added.
	 *
	 * \param first_point The first of the points
	 * \param second_point The second of the points
	 */
	void checkAndAddZagConnectorLine(curaIrfan::PointIrfan* first_point, curaIrfan::PointIrfan* second_point);

	/*!
	 * Adds a Zag connector represented by the given points. The last line of the connector will not be
	 * added if the given connector is an end piece and "connected_endpieces" is not enabled.
	 *
	 * \param points All the points on this connector
	 * \param is_endpiece Whether this connector is an end piece
	 */
	void addZagConnector(std::vector<curaIrfan::PointIrfan>& points, bool is_endpiece);

protected:
	const curaIrfan::PointMatrix& rotation_matrix; //!< The rotation matrix used to enforce the infill angle
	Polygons& result; //!< The result of the computation

	bool use_endpieces; //!< Whether to include end pieces or not
	bool connected_endpieces; //!< Whether the end pieces should be connected with the rest part of the infill
	int skip_some_zags; //!< Whether to skip some zags
	int zag_skip_count; //!< Skip 1 zag in every N zags

	bool is_first_connector; //!< indicating whether we are still looking for the first connector or not
	int first_connector_end_scanline_index; //!< scanline segment index of the first connector
	int last_connector_index; //!< scanline segment index of the last connector

	/*!
	 * The line segments belonging the zigzag connector to which the very first vertex belongs.
	 * This will be combined with the last handled zigzag_connector, which combine to a whole zigzag connector.
	 *
	 * Because the boundary polygon may start in in the middle of a zigzag connector,
	 */
	std::vector<curaIrfan::PointIrfan> first_connector;
	/*!
	 * The currently built up zigzag connector (not the first/last) or end piece or discarded boundary segment
	 */
	std::vector<curaIrfan::PointIrfan> current_connector;
};

inline void ZigzagConnectorProcessor::reset()
{
	is_first_connector = true;
	first_connector_end_scanline_index = 0;
	last_connector_index = 0;
	first_connector.clear();
	current_connector.clear();
}

inline void ZigzagConnectorProcessor::addLine(curaIrfan::PointIrfan from, curaIrfan::PointIrfan  to)
{
	result.addLine(rotation_matrix.unapply(from), rotation_matrix.unapply(to));
}

class NoZigZagConnectorProcessor : public ZigzagConnectorProcessor
{
public:
	NoZigZagConnectorProcessor(const curaIrfan::PointMatrix& rotation_matrix, Polygons& result)
		: ZigzagConnectorProcessor(rotation_matrix, result,
			false, false, // settings for zig-zag end pieces, no use here
			false, 0) // settings for skipping some zags, no use here
	{
	}

	void registerVertex(const curaIrfan::PointIrfan& vertex);
	void registerScanlineSegmentIntersection(const curaIrfan::PointIrfan& intersection, int scanline_index);
	void registerPolyFinished();
};
#endif