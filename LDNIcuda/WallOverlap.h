#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WALL_OVERLAP_H
#define WALL_OVERLAP_H

#include <functional> // hash function object
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

 //For flow ratios.
#include "Ratio.h"
#include "LinearAlgebra2D.h"
#include "IntPointIrfan.h"
#include "Polygon.h"
#include "PolygonProximityLinker.h"
#include "ProximityPointLink.h"
#include "SymmetricPair.h"

	/*!
	 * Class for computing and compensating for overlapping (outer) wall lines.
	 * The overlapping area is approximated with connected trapzoids.
	 * All places where the wall is closer than the nozzle width to another piece of wall are recorded.
	 * The area of a trapezoid is then the length between two such locations multiplied by the average overlap at the two locations.
	 *
	 * The amount of overlap between two locations is recorded in a link, so that we can look up the overlap at a given curaIrfan::PointIrfan in the polygon.
	 * A link always occurs between a curaIrfan::PointIrfan already on a polygon and either another curaIrfan::PointIrfan of a polygon or a curaIrfan::PointIrfan on a line segment of a polygon.
	 * In the latter case we insert the curaIrfan::PointIrfan into the polygon so that we can later look up by how much to reduce the extrusion at the corresponding line segment.
	 *
	 * At the end of a sequence of trapezoids the overlap area generally ends with a residual triangle.
	 * Therefore curaIrfan::PointIrfans are introduced on the line segments involved and a link is created with overlap zero.
	 *
	 * \see PolygonProximityLinker
	 *
	 * Each curaIrfan::PointIrfan on the polygons then maps to a link, so that we can easily look up which links corresponds
	 * to the current line segment being produced when producing gcode.
	 *
	 * When producing gcode, the first line crossing the overlap area is laid down normally and the second line is reduced by the overlap amount.
	 * For this reason the function WallOverlapComputation::getFlow changes the internal state of the PolygonProximityLinker.
	 *
	 * The main functionality of this class is performed by the constructor, by calling the constructor of PolygonProximityLinker.
	 * The adjustment during gcode generation is made with the help of WallOverlapComputation::getFlow
	 */
	class WallOverlapComputation
	{
		PolygonProximityLinker overlap_linker;
		coord_tIrfan line_width;

		std::unordered_set<SymmetricPair<ProximityPointLink>> passed_links;
	public:
		/*!
		 * Compute the flow for a given line segment in the wall.
		 *
		 * \warning the first time this function is called it returns a different thing than the second, because the second time it thinks it already passed this segment once.
		 *
		 * \param from The beginning of the line segment
		 * \param to The ending of the line segment
		 * \return a value between zero and one representing the reduced flow of the line segment
		 */
		Ratio getFlow(const curaIrfan::PointIrfan& from, const curaIrfan::PointIrfan& to);

		/*!
		 * Computes the neccesary priliminaries in order to efficiently compute the flow when generatign gcode paths.
		 * \param polygons The wall polygons for which to compute the overlaps
		 */
		WallOverlapComputation(Polygons& polygons, const coord_tIrfan lineWidth);

	private:
		/*!
		 * Check whether \p from_it and \p from_other_it are connected and if so,
		 * return the overlap area between those and the link \p to_link
		 *
		 * This presupposes that \p to_link and the link from \p from_it to \p from_other_it forms a single overlap quadrilateral
		 *
		 * from_other         to_other
		 *          o<--------o
		 *          ?         :
		 *          ?         :
		 *          ?         :
		 *          o-------->o
		 *       from         to
		 *
		 * \param from_it The first curaIrfan::PointIrfan possibly invovled in the second link
		 * \param to_it The first curaIrfan::PointIrfan of \p to_link connected to \p from_it
		 * \param to_link The first link involved in the overlap: from \p from_it to \p to_it
		 * \param from_other_it The second curaIrfan::PointIrfan possibly involved in the second link
		 * \param to_other_it The second curaIrfan::PointIrfan of \p to_link connected to \p from_other_it
		 * \return The overlap area between the two links, or zero if there was no such link
		 */
		coord_tIrfan handlePotentialOverlap(const ListPolyIt from_it, const ListPolyIt to_it, const ProximityPointLink& to_link, const ListPolyIt from_other_it, const ListPolyIt to_other_it);

		/*!
		 * Compute the approximate overlap area between two line segments
		 * or between a line segment and a curaIrfan::PointIrfan when one of the line segments has the same start as end curaIrfan::PointIrfan.
		 *
		 *   other_to         other_from
		 *          o<--------o
		 *          :         :
		 *          :,,,,,,,,,:
		 *          ://///////: \
		 *          ://///////:  } overlap area
		 *          ://///////: /
		 *          :''''''''':
		 *          :         :
		 *          o-------->o
		 *       from         to
		 *
		 * \param from The starting curaIrfan::PointIrfan of the one line segment
		 * \param to the end curaIrfan::PointIrfan of the one line segment
		 * \param to_dist The distance between \p to and \p to_other
		 * \param other_from The starting curaIrfan::PointIrfan of the other line segment (across the overlap of \p to)
		 * \param other_to The end curaIrfan::PointIrfan of the other line segment (across the overlap of \p from)
		 * \param from_dist The distance between \p from and \p from_other
		 */
		coord_tIrfan getApproxOverlapArea(const curaIrfan::PointIrfan from, const curaIrfan::PointIrfan to, const coord_tIrfan to_dist, const curaIrfan::PointIrfan other_from, const curaIrfan::PointIrfan other_to, const coord_tIrfan from_dist);

		/*!
		 * Check whether an overlap segment between two consecutive links is already passed
		 *
		 * \note \p link_a and \p link_b are assumed to be consecutive
		 *
		 * \param link_a the one link of the overlap area
		 * \param link_b the other link of the overlap area
		 * \return whether the link has already been passed once
		 */
		bool getIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b);

		/*!
		 * Mark an overlap area between two consecutive links as being passed once already.
		 *
		 * \note \p link_a and \p link_b are assumed to be consecutive
		 *
		 * \param link_a the one link of the overlap area
		 * \param link_b the other link of the overlap area
		 */
		void setIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b);
	};


//namespace cura



#endif//WALL_OVERLAP_H

