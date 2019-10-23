#pragma once


#ifndef AABB_H
#define AABB_H

#include "IntpointIrfan.h"


class ConstPolygonRef;
class Polygon;
class Polygons;


class AABB

{
public:
	curaIrfan::PointIrfan min, max;

	AABB(); //!< initializes with invalid min and max
	AABB(const curaIrfan::PointIrfan& min, const curaIrfan::PointIrfan& max); //!< initializes with given min and max
	AABB(const Polygons& polys); //!< Computes the boundary box for the given polygons
	AABB(ConstPolygonRef poly); //!< Computes the boundary box for the given polygons

	void calculate(const Polygons& polys); //!< Calculates the aabb for the given polygons (throws away old min and max data of this aabb)
	void calculate(ConstPolygonRef poly); //!< Calculates the aabb for the given polygon (throws away old min and max data of this aabb)

	/*!
	 * Whether the bounding box contains the specified point.
	 * \param point The point to check whether it is inside the bounding box.
	 * \return ``true`` if the bounding box contains the specified point, or
	 * ``false`` otherwise.
	 */
	bool contains(const curaIrfan::PointIrfan& point) const;

	/*!
	 * Get the middle of the bounding box
	 */
	curaIrfan::PointIrfan getMiddle() const;

	/*!
	 * Check whether this aabb overlaps with another.
	 *
	 * In the boundary case false is returned.
	 *
	 * \param other the aabb to check for overlaps with
	 * \return Whether the two aabbs overlap
	 */
	bool hit(const AABB& other) const;

	/*!
	 * \brief Includes the specified point in the bounding box.
	 *
	 * The bounding box is expanded if the point is not within the bounding box.
	 *
	 * \param point The point to include in the bounding box.
	 */
	void include(curaIrfan::PointIrfan point);

	/*!
	 * \brief Includes the specified bounding box in the bounding box.
	 *
	 * The bounding box is expanded to include the other bounding box.
	 *
	 * This performs a union on two bounding boxes.
	 *
	 * \param other The bounding box to include in this one.
	 */
	void include(const AABB other);

	/*!
	 * Expand the borders of the bounding box in each direction with the given amount
	 *
	 * \param dist The distance by which to expand the borders of the bounding box
	 */
	void expand(int dist);

	/*!
	 * Generate a square polygon which coincides with this aabb
	 * \return the polygon of this aabb
	 */
	 //	void toPolygon(Polygon& polygon) const;

	Polygon toPolygon() const;
};

/*!
An Axis Aligned Bounding Box. Has a min and max vector, representing minimal and maximal coordinates in the three axes.
*/

#endif // !AABB_H
