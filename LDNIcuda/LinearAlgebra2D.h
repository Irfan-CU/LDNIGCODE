#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_LINEAR_ALG_2D_H
#define UTILS_LINEAR_ALG_2D_H

#include "IntPointIrfan.h"

	class LinearAlg2D
	{
	public:
		static short pointLiesOnTheRightOfLine(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& p0, const curaIrfan::PointIrfan& p1)
		{
			// no tests unless the segment p0-p1 is at least partly at, or to right of, p.X
			if (std::max(p0.X, p1.X) >= p.X)
			{
				const coord_tIrfan pd_y = p1.Y - p0.Y;
				if (pd_y < 0) // p0->p1 is 'falling'
				{
					if (p1.Y <= p.Y && p0.Y > p.Y) // candidate
					{
						// dx > 0 if intersection is to right of p.X
						const coord_tIrfan dx = (p1.X - p0.X) * (p1.Y - p.Y) - (p1.X - p.X) * pd_y;
						if (dx == 0) // includes p == p1
						{
							return 0;
						}
						if (dx > 0)
						{
							return 1;
						}
					}
				}
				else if (p.Y >= p0.Y)
				{
					if (p.Y < p1.Y) // candidate for p0->p1 'rising' and includes p.Y
					{
						// dx > 0 if intersection is to right of p.X
						const coord_tIrfan dx = (p1.X - p0.X) * (p.Y - p0.Y) - (p.X - p0.X) * pd_y;
						if (dx == 0) // includes p == p0
						{
							return 0;
						}
						if (dx > 0)
						{
							return 1;
						}
					}
					else if (p.Y == p1.Y)
					{
						// some special cases here, points on border:
						// - p1 exactly matches p (might otherwise be missed)
						// - p0->p1 exactly horizontal, and includes p.
						// (we already tested std::max(p0.X,p1.X) >= p.X )
						if (p.X == p1.X ||
							(pd_y == 0 && std::min(p0.X, p1.X) <= p.X))
						{
							return 0;
						}
					}
				}
			}
			return -1;

		}

		/*!
		 * Find whether a curaIrfan::PointIrfan projected on a line segment would be projected to
		 * - properly on the line : zero returned
		 * - closer to \p a : -1 returned
		 * - closer to \p b : 1 returned
		 *
		 * \param from The curaIrfan::PointIrfan to check in relation to the line segment
		 * \param a The start curaIrfan::PointIrfan of the line segment
		 * \param b The end curaIrfan::PointIrfan of the line segment
		 * \return the sign of the projection wrt the line segment
		 */
		inline static short pointIsProjectedBeyondLine(const curaIrfan::PointIrfan& from, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b)
		{
			curaIrfan::PointIrfan check1 = curaIrfan::operator-(b, a);
			const curaIrfan::PointIrfan vec = check1;
			curaIrfan::PointIrfan check2 = curaIrfan::operator-(from,a);
			const curaIrfan::PointIrfan point_vec = check2;
			coord_tIrfan dot_prod = curaIrfan::dot(point_vec, vec);
			
			if (dot_prod < 0)
			{ // curaIrfan::PointIrfan is projected to before ab
				return -1;
			}
			if (dot_prod > curaIrfan::vSize2(vec))
			{ // curaIrfan::PointIrfan is projected to after ab
				return 1;
			}
			return 0;
		}

		/*!
		* Find the curaIrfan::PointIrfan closest to \p from on the line from \p p0 to \p p1
		*/
		static curaIrfan::PointIrfan getClosestOnLineSegment(const curaIrfan::PointIrfan& from, const curaIrfan::PointIrfan& p0, const curaIrfan::PointIrfan& p1)
		{
			curaIrfan::PointIrfan check3 = curaIrfan::operator-(p1,p0);
			const curaIrfan::PointIrfan direction = check3;
			curaIrfan::PointIrfan check4 = curaIrfan::operator-(from, p0);
			const curaIrfan::PointIrfan to_from = check4;
			const coord_tIrfan projected_x = curaIrfan::dot(to_from, direction);

			const coord_tIrfan x_p0 = 0;
			const coord_tIrfan x_p1 = curaIrfan::vSize2(direction);

			if (x_p1 == 0)
			{
				return p0;
			}
			if (projected_x <= x_p0)
			{
				return p0;
			}
			if (projected_x >= x_p1)
			{
				return p1;
			}
			else
			{
				curaIrfan::PointIrfan check1 = curaIrfan::operator+(p0 , projected_x);
				curaIrfan::PointIrfan check2 = curaIrfan::operator*(curaIrfan::vSize(direction), direction);
				curaIrfan::PointIrfan check3 = curaIrfan::operator/(check1,check2);
				curaIrfan::PointIrfan check4 = curaIrfan::operator/(check3, curaIrfan::vSize(direction));
				curaIrfan::PointIrfan ret = check4;
				return ret;
			}
		}

		/*!
		 * Find the two points on two line segments closest to each other.
		 *
		 * Find the smallest line segment connecting the two line segments a and b.
		 *
		 * \param a1 first curaIrfan::PointIrfan on line a
		 * \param a2 second curaIrfan::PointIrfan on line a
		 * \param b1 first curaIrfan::PointIrfan on line b
		 * \param b2 second curaIrfan::PointIrfan on line b
		 * \return A pair: the first curaIrfan::PointIrfan on line a and the second pouint on line b
		 */
		static std::pair<curaIrfan::PointIrfan, curaIrfan::PointIrfan> getClosestConnection(curaIrfan::PointIrfan a1, curaIrfan::PointIrfan a2, curaIrfan::PointIrfan b1, curaIrfan::PointIrfan b2);

		/*!
		* Get the squared distance from curaIrfan::PointIrfan \p b to a line *segment* from \p a to \p c.
		*
		* In case \p b is on \p a or \p c, \p b_is_beyond_ac should become 0.
		*
		* \param a the first curaIrfan::PointIrfan of the line segment
		* \param b the curaIrfan::PointIrfan to measure the distance from
		* \param c the second curaIrfan::PointIrfan on the line segment
		* \param b_is_beyond_ac optional output parameter: whether \p b is closest to the line segment (0), to \p a (-1) or \p b (1)
		*/
		static coord_tIrfan getDist2FromLineSegment(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c, int16_t* b_is_beyond_ac = nullptr)
		{
			/*
			*     a,
			*     /|
			*    / |
			* b,/__|, x
			*   \  |
			*    \ |
			*     \|
			*      'c
			*
			* x = b projected on ac
			* ax = ab curaIrfan::dot ac / curaIrfan::vSize(ac)
			* xb = ab - ax
			* error = curaIrfan::vSize(xb)
			*/
			const curaIrfan::PointIrfan check5 = curaIrfan::operator-(c, a);
			const curaIrfan::PointIrfan ac = check5;
			const coord_tIrfan ac_size = curaIrfan::vSize(ac);
			const curaIrfan::PointIrfan check6 = curaIrfan::operator-(b, a);
			const curaIrfan::PointIrfan ab = check6;
			if (ac_size == 0)
			{
				const coord_tIrfan ab_dist2 = curaIrfan::vSize2(ab);
				if (ab_dist2 == 0 && b_is_beyond_ac)
				{
					*b_is_beyond_ac = 0; // a is on b is on c
				}
				// otherwise variable b_is_beyond_ac remains its value; it doesn't make sense to choose between -1 and 1
				return ab_dist2;
			}
			const coord_tIrfan projected_x = curaIrfan::dot(ab, ac);
			const coord_tIrfan ax_size = projected_x / ac_size;

			if (ax_size < 0)
			{// b is 'before' segment ac 
				if (b_is_beyond_ac)
				{
					*b_is_beyond_ac = -1;
				}
				return curaIrfan::vSize2(ab);
			}
			if (ax_size > ac_size)
			{// b is 'after' segment ac
				if (b_is_beyond_ac)
				{
					*b_is_beyond_ac = 1;
				}
				curaIrfan::PointIrfan check7 = curaIrfan::operator-(b,c);
				return curaIrfan::vSize2(check7);
			}

			if (b_is_beyond_ac)
			{
				*b_is_beyond_ac = 0;
			}
			curaIrfan::PointIrfan check7 = curaIrfan::operator*(ac, ax_size);
			curaIrfan::PointIrfan check8 = curaIrfan::operator/(check7, ac_size);
			const curaIrfan::PointIrfan ax = check8;
			curaIrfan::PointIrfan check9 = curaIrfan::operator-(ab, ax);
			const curaIrfan::PointIrfan bx = check9;
			return curaIrfan::vSize2(bx);
			//return curaIrfan::vSize2(ab) - ax_size*ax_size; // less accurate
		}

		/*!
		 * Checks whether the minimal distance between two line segments is at most \p max_dist
		 * The first line semgent is given by end points \p a and \p b, the second by \p c and \p d.
		 *
		 * \param a One end curaIrfan::PointIrfan of the first line segment
		 * \param b Another end curaIrfan::PointIrfan of the first line segment
		 * \param c One end curaIrfan::PointIrfan of the second line segment
		 * \param d Another end curaIrfan::PointIrfan of the second line segment
		 * \param max_dist The maximal distance between the two line segments for which this function will return true.
		 */
		static bool lineSegmentsAreCloserThan(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c, const curaIrfan::PointIrfan& d, const coord_tIrfan max_dist)
		{
			const coord_tIrfan max_dist2 = max_dist * max_dist;

			return getDist2FromLineSegment(a, c, b) <= max_dist2
				|| getDist2FromLineSegment(a, d, b) <= max_dist2
				|| getDist2FromLineSegment(c, a, d) <= max_dist2
				|| getDist2FromLineSegment(c, b, d) <= max_dist2;
		}

		/*!
		 * Get the minimal distance between two line segments
		 * The first line semgent is given by end points \p a and \p b, the second by \p c and \p d.
		 *
		 * \param a One end curaIrfan::PointIrfan of the first line segment
		 * \param b Another end curaIrfan::PointIrfan of the first line segment
		 * \param c One end curaIrfan::PointIrfan of the second line segment
		 * \param d Another end curaIrfan::PointIrfan of the second line segment
		 */
		static coord_tIrfan getDist2BetweenLineSegments(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c, const curaIrfan::PointIrfan& d)
		{
			return
				std::min(getDist2FromLineSegment(a, c, b),
					std::min(getDist2FromLineSegment(a, d, b),
						std::min(getDist2FromLineSegment(c, a, d),
							getDist2FromLineSegment(c, b, d))));
		}

		/*!
		 * Check whether two line segments collide.
		 *
		 * \warning Edge cases (end points of line segments fall on other line segment) register as a collision.
		 *
		 * \note All points are assumed to be transformed by the transformation matrix of the vector from \p a_from to \p a_to.
		 * I.e. a is a vertical line; the Y of \p a_from_transformed is the same as the Y of \p a_to_transformed.
		 *
		 * \param a_from_transformed The transformed from location of line a
		 * \param a_from_transformed The transformed to location of line a
		 * \param b_from_transformed The transformed from location of line b
		 * \param b_from_transformed The transformed to location of line b
		 * \return Whether the two line segments collide
		 */
		static bool lineSegmentsCollide(const curaIrfan::PointIrfan& a_from_transformed, const curaIrfan::PointIrfan& a_to_transformed, curaIrfan::PointIrfan b_from_transformed, curaIrfan::PointIrfan b_to_transformed);

		/*!
		 * Compute the angle between two consecutive line segments.
		 *
		 * The angle is computed from the left side of b when looking from a.
		 *
		 *   c
		 *    \                     .
		 *     \ b
		 * angle|
		 *      |
		 *      a
		 *
		 * \param a start of first line segment
		 * \param b end of first segment and start of second line segment
		 * \param c end of second line segment
		 * \return the angle in radians between 0 and 2 * pi of the corner in \p b
		 */
		static float getAngleLeft(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c);

		/*!
		 * Returns the determinant of the 2D matrix defined by the the vectors ab and ap as rows.
		 *
		 * The returned value is zero for \p p lying (approximately) on the line going through \p a and \p b
		 * The value is positive for values lying to the left and negative for values lying to the right when looking from \p a to \p b.
		 *
		 * \param p the curaIrfan::PointIrfan to check
		 * \param a the from curaIrfan::PointIrfan of the line
		 * \param b the to curaIrfan::PointIrfan of the line
		 * \return a positive value when \p p lies to the left of the line from \p a to \p b
		 */
		static inline coord_tIrfan pointIsLeftOfLine(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b)
		{
			return (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);
		}

		/*!
		 * Get a curaIrfan::PointIrfan on the line segment (\p a - \p b)with a given distance to curaIrfan::PointIrfan \p p
		 *
		 * In case there are two possible curaIrfan::PointIrfan that meet the criteria, choose the one closest to a.
		 *
		 * \param p The reference curaIrfan::PointIrfan
		 * \param a Start of the line segment
		 * \param b End of the line segment
		 * \param dist The required distance of \p result to \p p
		 * \param[out] result The result (if any was found)
		 * \return Whether any such curaIrfan::PointIrfan has been found
		 */
		static bool getPointOnLineWithDist(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const coord_tIrfan dist, curaIrfan::PointIrfan& result);

		/*!
		 * Get the squared distance from a curaIrfan::PointIrfan \p p to the line on which \p a and
		 * \p b lie
		 * \param p The curaIrfan::PointIrfan to measure the distance from.
		 * \param a One of the points through which the line goes.
		 * \param b One of the points through which the line goes.
		 * \return The distance between the curaIrfan::PointIrfan and the line, squared.
		 */
		static coord_tIrfan getDist2FromLine(const curaIrfan::PointIrfan& p, const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b);

		/*!
		 * Check whether a corner is acute or obtuse.
		 *
		 * This function is irrespective of the order between \p a and \p c;
		 * the lowest angle among bot hsides of the corner is always chosen.
		 *
		 * isAcuteCorner(a, b, c) === isAcuteCorner(c, b, a)
		 *
		 * \param a start of first line segment
		 * \param b end of first segment and start of second line segment
		 * \param c end of second line segment
		 * \return positive if acute, negative if obtuse, zero if 90 degree corner
		 */
		static inline int isAcuteCorner(const curaIrfan::PointIrfan& a, const curaIrfan::PointIrfan& b, const curaIrfan::PointIrfan& c)
		{
			curaIrfan::PointIrfan check = curaIrfan::operator-(a, b);
			const curaIrfan::PointIrfan ba =check;
			check = curaIrfan::operator-(c, b);
			const curaIrfan::PointIrfan bc = check;
			return curaIrfan::dot(ba, bc);
		}

		/*!
		 * Get the rotation matrix for rotating around a specific curaIrfan::PointIrfan in place.
		 */
		static curaIrfan::Point3Matrix rotateAround(const curaIrfan::PointIrfan& middle, double rotation)
		{
			curaIrfan::Point3Matrix rotation_matrix(rotation);
			curaIrfan::Point3Matrix rotation_matrix_homogeneous(rotation_matrix);
			curaIrfan::PointIrfan check = curaIrfan::operator-(middle);
			return curaIrfan::Point3Matrix::translate(middle).compose(rotation_matrix_homogeneous).compose(curaIrfan::Point3Matrix::translate(check));
		}
	};



//namespace cura
#endif//UTILS_LINEAR_ALG_2D_H



