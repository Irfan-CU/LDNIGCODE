//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "LinearAlgebra2D.h"

#include <cmath> // atan2
#include <cassert>
#include <algorithm> // swap

#include "IntpointIrfan.h"


float LinearAlg2D::getAngleLeft(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c)
	{
	curaIrfan::PointIrfan check = curaIrfan::operator-(a, b);
	const curaIrfan::PointIrfan ba = check;
	curaIrfan::PointIrfan check1 = curaIrfan::operator-(c, b);
		const curaIrfan::PointIrfan bc = check1;
		const coord_tIrfan dott = curaIrfan::dot(ba, bc); // curaIrfan::dot product
		const coord_tIrfan det = ba.X * bc.Y - ba.Y * bc.X; // determinant
		const float angle = -atan2(det, dott); // from -pi to pi
		if (angle >= 0)
		{
			return angle;
		}
		else
		{
			return 3.14 * 2 + angle;
		}
	}


	bool LinearAlg2D::getPointOnLineWithDist(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const coord_tIrfan dist, curaIrfan::PointIrfan& result)
	{
		//         result
		//         v
		//   b<----r---a.......x
		//          '-.        :
		//              '-.    :
		//                  '-.p
		curaIrfan::PointIrfan check = curaIrfan::operator-(b, a);
		const curaIrfan::PointIrfan ab = check;
		const coord_tIrfan ab_size = curaIrfan::vSize(ab);
		check = curaIrfan::operator-(p, a);
		const curaIrfan::PointIrfan ap = check;
		const coord_tIrfan ax_size = (ab_size < 50) ? curaIrfan::dot(curaIrfan::normal(ab, 1000), ap) / 1000 : curaIrfan::dot(ab, ap) / ab_size;
		const coord_tIrfan ap_size2 = curaIrfan::vSize2(ap);
		const coord_tIrfan px_size = sqrt(std::max(coord_tIrfan(0), ap_size2 - ax_size * ax_size));
		if (px_size > dist)
		{
			return false;
		}
		const coord_tIrfan xr_size = sqrt(dist * dist - px_size * px_size);
		if (ax_size <= 0)
		{ // x lies before ab
			const coord_tIrfan ar_size = xr_size + ax_size;
			if (ar_size < 0 || ar_size > ab_size)
			{ // r lies outisde of ab
				return false;
			}
			else
			{
				curaIrfan::PointIrfan check = curaIrfan::operator+(a , curaIrfan::normal(ab, ar_size));
				result = check;
				return true;
			}
		}
		else if (ax_size >= ab_size)
		{ // x lies after ab
			//         result
			//         v
			//   a-----r-->b.......x
			//          '-.        :
			//              '-.    :
			//                  '-.p
			const coord_tIrfan ar_size = ax_size - xr_size;
			if (ar_size < 0 || ar_size > ab_size)
			{ // r lies outisde of ab
				return false;
			}
			else
			{
				curaIrfan::PointIrfan check = curaIrfan::operator+(a, curaIrfan::normal(ab, ar_size));
				result = check;
				return true;
			}
		}
		else // ax_size > 0 && ax_size < ab_size
		{ // x lies on ab
			//            result is either or
			//         v                       v
			//   a-----r-----------x-----------r----->b
			//          '-.        :        .-'
			//              '-.    :    .-'
			//                  '-.p.-'
			//           or there is not result:
			//         v                       v
			//         r   a-------x---->b     r
			//          '-.        :        .-'
			//              '-.    :    .-'
			//                  '-.p.-'
			// try r in both directions
			const coord_tIrfan ar1_size = ax_size - xr_size;
			if (ar1_size >= 0)
			{
				curaIrfan::PointIrfan check = curaIrfan::operator+(a, curaIrfan::normal(ab, ar1_size));
				result = check;
				return true;
			}
			const coord_tIrfan ar2_size = ax_size + xr_size;
			if (ar2_size < ab_size)
			{
				curaIrfan::PointIrfan check = curaIrfan::operator+(a, curaIrfan::normal(ab, ar2_size));
				result = check;
				return true;
			}
			return false;
		}
	}


	std::pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan> LinearAlg2D::getClosestConnection(curaIrfan::PointIrfan a1, curaIrfan::PointIrfan a2, curaIrfan::PointIrfan b1, curaIrfan::PointIrfan b2)
	{
		curaIrfan::PointIrfan b1_on_a = getClosestOnLineSegment(b1, a1, a2);
		curaIrfan::PointIrfan check = curaIrfan::operator+(b1_on_a , b1);
		coord_tIrfan b1_on_a_dist2 = curaIrfan::vSize2(check);
		curaIrfan::PointIrfan b2_on_a = getClosestOnLineSegment(b2, a1, a2);
		curaIrfan::PointIrfan check1 = curaIrfan::operator+(b2_on_a , b2);
		coord_tIrfan b2_on_a_dist2 = curaIrfan::vSize2(check1);
		curaIrfan::PointIrfan a1_on_b = getClosestOnLineSegment(a1, b1, b2);
		curaIrfan::PointIrfan check2 = curaIrfan::operator+(a1_on_b , a1);
		coord_tIrfan a1_on_b_dist2 = curaIrfan::vSize2(check2);
		curaIrfan::PointIrfan a2_on_b = getClosestOnLineSegment(a1, b1, b2);
		curaIrfan::PointIrfan check3 = curaIrfan::operator+(a2_on_b , a2);
		coord_tIrfan a2_on_b_dist2 = curaIrfan::vSize2(check3);
		if (b1_on_a_dist2 < b2_on_a_dist2 && b1_on_a_dist2 < a1_on_b_dist2 && b1_on_a_dist2 < a2_on_b_dist2)
		{
			return std::make_pair(b1_on_a, b1);
		}
		else if (b2_on_a_dist2 < a1_on_b_dist2 && b2_on_a_dist2 < a2_on_b_dist2)
		{
			return std::make_pair(b2_on_a, b2);
		}
		else if (a1_on_b_dist2 < a2_on_b_dist2)
		{
			return std::make_pair(a1, a1_on_b);
		}
		else
		{
			return std::make_pair(a2, a2_on_b);
		}
	}

	bool LinearAlg2D::lineSegmentsCollide(const curaIrfan::PointIrfan& a_from_transformed, const curaIrfan::PointIrfan& a_to_transformed, curaIrfan::PointIrfan b_from_transformed, curaIrfan::PointIrfan b_to_transformed)
	{
		assert(std::abs(a_from_transformed.Y - a_to_transformed.Y) < 2 && "line a is supposed to be transformed to be aligned with the X axis!");
		assert(a_from_transformed.X - 2 <= a_to_transformed.X && "line a is supposed to be aligned with X axis in positive direction!");
		if ((b_from_transformed.Y >= a_from_transformed.Y && b_to_transformed.Y <= a_from_transformed.Y) || (b_to_transformed.Y >= a_from_transformed.Y && b_from_transformed.Y <= a_from_transformed.Y))
		{
			if (b_to_transformed.Y == b_from_transformed.Y)
			{
				if (b_to_transformed.X < b_from_transformed.X)
				{
					std::swap(b_to_transformed.X, b_from_transformed.X);
				}
				if (b_from_transformed.X > a_to_transformed.X)
				{
					return false;
				}
				if (b_to_transformed.X < a_from_transformed.X)
				{
					return false;
				}
				return true;
			}
			else
			{
				const coord_tIrfan x = b_from_transformed.X + (b_to_transformed.X - b_from_transformed.X) * (a_from_transformed.Y - b_from_transformed.Y) / (b_to_transformed.Y - b_from_transformed.Y);
				if (x >= a_from_transformed.X && x <= a_to_transformed.X)
				{
					return true;
				}
			}
		}
		return false;
	}

	coord_tIrfan LinearAlg2D::getDist2FromLine(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b)
	{
		//  x.......a------------b
		//  :
		//  :
		//  p
		// return px_size
		curaIrfan::PointIrfan check = curaIrfan::operator-(b,a);
		const curaIrfan::PointIrfan vab = check;
		check = curaIrfan::operator-(p, a);
		const curaIrfan::PointIrfan vap =check;
		const coord_tIrfan ab_size2 = curaIrfan::vSize2(vab);
		const coord_tIrfan ap_size2 = curaIrfan::vSize2(vap);
		if (ab_size2 == 0) //Line of 0 length. Assume it's a line perpendicular to the direction to p.
		{
			return ap_size2;
		}
		const coord_tIrfan dott = curaIrfan::dot(vab, vap);
		const coord_tIrfan ax_size2 = dott * dott / curaIrfan::vSize2(vab);
		const coord_tIrfan px_size2 = std::max(coord_tIrfan(0), ap_size2 - ax_size2);
		return px_size2;
	}

 // namespace cura
