/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include "wallOverlap.h"

#include <cmath> // isfinite
#include <sstream>

#include "AABB.h"
#include  "SVG.h"

    WallOverlapComputation::WallOverlapComputation(Polygons& polygons, const coord_tIrfan line_width)
		: overlap_linker(polygons, line_width)
		, line_width(line_width)
	{
		
	}

	Ratio WallOverlapComputation::getFlow(const curaIrfan::PointIrfan& from, const curaIrfan::PointIrfan& to)
	{
		using Point2LinkIt = PolygonProximityLinker::Point2Link::iterator;

		if (!overlap_linker.isLinked(from))
		{ // [from] is not linked
			return 1;
		}
		const std::pair<Point2LinkIt, Point2LinkIt> to_links = overlap_linker.getLinks(to);
		if (to_links.first == to_links.second)
		{ // [to] is not linked
			return 1;
		}

		coord_tIrfan overlap_area = 0;
		// note that we don't need to loop over all from_links, because they are handled in the previous getFlow(.) call (or in the very last)
		for (Point2LinkIt to_link_it = to_links.first; to_link_it != to_links.second; ++to_link_it)
		{
			const ProximityPointLink& to_link = to_link_it->second;
			ListPolyIt to_it = to_link.a;
			ListPolyIt to_other_it = to_link.b;
			if (to_link.a.p() != to)
			{
				assert(to_link.b.p() == to && "Either part of the link should be the point in the link!");
				std::swap(to_it, to_other_it);
			}
			ListPolyIt from_it = to_it.prev();

			ListPolyIt to_other_next_it = to_other_it.next(); // move towards [from]; the lines on the other side move in the other direction
			//           to  from
			//   o<--o<--T<--F
			//   |       :   :
			//   v       :   :
			//   o-->o-->o-->o
			//           ,   ,
			//           ;   to_other_next
			//           to other

			bool are_in_same_general_direction = curaIrfan::dot(curaIrfan::operator-(from, to), curaIrfan::operator-(to_other_it.p(), to_other_next_it.p())) > 0;
			// handle multiple points  linked to [to]
			//   o<<<T<<<F
			//     / |
			//    /  |
			//   o>>>o>>>o
			//   ,   ,
			//   ;   to other next
			//   to other
			if (!are_in_same_general_direction)
			{
				overlap_area = std::max(overlap_area, handlePotentialOverlap(to_it, to_it, to_link, to_other_next_it, to_other_it));
			}

			// handle multiple points  linked to [to_other]
			//   o<<<T<<<F
			//       |  /
			//       | /
			//   o>>>o>>>o
			bool all_are_in_same_general_direction = are_in_same_general_direction && curaIrfan::dot(curaIrfan::operator-(from, to), curaIrfan::operator-(to_other_it.prev().p(), to_other_it.p())) > 0;
			if (!all_are_in_same_general_direction)
			{
				overlap_area = std::max(overlap_area, handlePotentialOverlap(from_it, to_it, to_link, to_other_it, to_other_it));
			}

			// handle normal case where the segment from-to overlaps with another segment
			//   o<<<T<<<F
			//       |   |
			//       |   |
			//   o>>>o>>>o
			//       ,   ,
			//       ;   to other next
			//       to other
			if (!are_in_same_general_direction)
			{
				overlap_area = std::max(overlap_area, handlePotentialOverlap(from_it, to_it, to_link, to_other_next_it, to_other_it));
			}
		}

		coord_tIrfan normal_area = curaIrfan::vSize(curaIrfan::operator-(from, to)) * line_width;
		Ratio ratio = Ratio(normal_area - overlap_area) / normal_area;
		// clamp the ratio because overlap compensation might be faulty because
		// WallOverlapComputation::getApproxOverlapArea only gives roughly accurate results
		return std::min(1.0_r, std::max(0.0_r, ratio));
	}

	coord_tIrfan WallOverlapComputation::handlePotentialOverlap(const ListPolyIt from_it, const ListPolyIt to_it, const ProximityPointLink& to_link, const ListPolyIt from_other_it, const ListPolyIt to_other_it)
	{
		if (from_it == to_other_it && from_it == from_other_it)
		{ // don't compute overlap with a line and itself
			return 0;
		}
		const ProximityPointLink* from_link = overlap_linker.getLink(from_it, from_other_it);
		if (!from_link)
		{
			return 0;
		}
		if (!getIsPassed(to_link, *from_link))
		{ // check whether the segment is already passed
			setIsPassed(to_link, *from_link);
			return 0;
		}
		return getApproxOverlapArea(from_it.p(), to_it.p(), to_link.dist, to_other_it.p(), from_other_it.p(), from_link->dist);
	}

	coord_tIrfan WallOverlapComputation::getApproxOverlapArea(const curaIrfan::PointIrfan from, const curaIrfan::PointIrfan to, const coord_tIrfan to_dist, const curaIrfan::PointIrfan other_from, const curaIrfan::PointIrfan other_to, const coord_tIrfan from_dist)
	{
		const coord_tIrfan overlap_width_2 = line_width * 2 - from_dist - to_dist; //Twice the width of the overlap area, perpendicular to the lines.

		// check whether the line segment overlaps with the point if one of the line segments is just a point
		if (from == to)
		{
			if (LinearAlg2D::pointIsProjectedBeyondLine(from, other_from, other_to) != 0)
			{
				return 0;
			}
			const coord_tIrfan overlap_length_2 = curaIrfan::vSize(curaIrfan::operator-(other_to,other_from)); //Twice the length of the overlap area, alongside the lines.
			return overlap_length_2 * overlap_width_2 / 4; //Area = width * height.
		}
		if (other_from == other_to)
		{
			if (LinearAlg2D::pointIsProjectedBeyondLine(other_from, from, to) != 0)
			{
				return 0;
			}
			const coord_tIrfan overlap_length_2 = curaIrfan::vSize(curaIrfan::operator-(from, to)); //Twice the length of the overlap area, alongside the lines.
			return overlap_length_2 * overlap_width_2 / 4; //Area = width * height.
		}

		short from_rel = LinearAlg2D::pointIsProjectedBeyondLine(from, other_from, other_to);
		short to_rel = LinearAlg2D::pointIsProjectedBeyondLine(to, other_from, other_to);
		short other_from_rel = LinearAlg2D::pointIsProjectedBeyondLine(other_from, from, to);
		short other_to_rel = LinearAlg2D::pointIsProjectedBeyondLine(other_to, from, to);
		if (from_rel != 0 && to_rel == from_rel && other_from_rel != 0 && other_to_rel == other_from_rel)
		{
			// both segments project fully beyond or before each other
			// for example:             or:
			// O<------O   .            O------>O
			//         :   :                     \_
			//         '   O------->O             O------>O
			return 0;
		}

		if (from_rel != 0 && from_rel == other_from_rel && to_rel == 0 && other_to_rel == 0)
		{
			// only ends of line segments overlap
			//
			//       to_proj
			//         ^^^^^
			//         O<--+----O
			//         :   :
			//   O-----+-->O
			//         ,,,,,
			//         other_to_proj
			const curaIrfan::PointIrfan other_vec = curaIrfan::operator-(other_from , other_to);
			const coord_tIrfan to_proj = curaIrfan::dot(curaIrfan::operator-(to , other_to), other_vec) / curaIrfan::vSize(other_vec);

			const curaIrfan::PointIrfan vec = curaIrfan::operator-(from, to);
			const coord_tIrfan other_to_proj = curaIrfan::dot(curaIrfan::operator-(other_to, to), vec) / curaIrfan::vSize(vec);

			const coord_tIrfan overlap_length_2 = to_proj + other_to_proj; //Twice the length of the overlap area, alongside the lines.
			return overlap_length_2 * overlap_width_2 / 4; //Area = width * height.
		}
		if (to_rel != 0 && to_rel == other_to_rel && from_rel == 0 && other_from_rel == 0)
		{
			// only beginnings of line segments overlap
			//
			//           from_proj
			//           ^^^^^
			//      O<---+---O
			//           :   :
			//           O---+---->O
			//           ,,,,,
			// other_from_proj
			const curaIrfan::PointIrfan other_vec = curaIrfan::operator-(other_to , other_from);
			const coord_tIrfan from_proj = curaIrfan::dot(curaIrfan::operator-(from , other_from), other_vec) / curaIrfan::vSize(other_vec);

			const curaIrfan::PointIrfan vec = curaIrfan::operator-(to , from);
			const coord_tIrfan other_from_proj = curaIrfan::dot(curaIrfan::operator-(other_from , from), vec) / curaIrfan::vSize(vec);

			const coord_tIrfan overlap_length_2 = from_proj + other_from_proj; //Twice the length of the overlap area, alongside the lines.
			return overlap_length_2 * overlap_width_2 / 4; //Area = width * height.
		}

		//More complex case.
		const curaIrfan::PointIrfan from_middle = curaIrfan::operator+(other_to , from); // don't divide by two just yet
		const curaIrfan::PointIrfan to_middle = curaIrfan::operator+(other_from , to); // don't divide by two just yet

		const coord_tIrfan overlap_length_2 = curaIrfan::vSize(curaIrfan::operator-(from_middle , to_middle)); //(An approximation of) twice the length of the overlap area, alongside the lines.
		return overlap_length_2 * overlap_width_2 / 4; //Area = width * height.
	}

	bool WallOverlapComputation::getIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b)
	{
		return passed_links.find(SymmetricPair<ProximityPointLink>(link_a, link_b)) != passed_links.end();
	}

	void WallOverlapComputation::setIsPassed(const ProximityPointLink& link_a, const ProximityPointLink& link_b)
	{
		passed_links.emplace(link_a, link_b);
	}


//namespace cura 
