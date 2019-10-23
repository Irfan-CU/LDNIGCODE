//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>
#include "AABB.h"
#include "Polygon.h"

	AABB::AABB()
		: min(POINTIrfan_MAX, POINTIrfan_MAX), max(POINTIrfan_MIN, POINTIrfan_MIN)
	{
	}

	AABB::AABB(const curaIrfan::PointIrfan& min, const curaIrfan::PointIrfan& max)
		: min(min), max(max)
	{
	}

	AABB::AABB(const Polygons& polys)
		: min(POINTIrfan_MAX, POINTIrfan_MAX), max(POINTIrfan_MIN, POINTIrfan_MIN)
	{
		calculate(polys);
	}

	AABB::AABB(ConstPolygonRef poly)
		: min(POINTIrfan_MAX, POINTIrfan_MAX), max(POINTIrfan_MIN, POINTIrfan_MIN)
	{
		calculate(poly);
	}

	curaIrfan::PointIrfan AABB::getMiddle() const
	{
		curaIrfan::PointIrfan check= curaIrfan::operator+(min,max);
		curaIrfan::PointIrfan check1 = curaIrfan::operator/(check, 2);
		return check1;
	}

	void AABB::calculate(const Polygons& polys)
	{
		min = curaIrfan::PointIrfan(POINTIrfan_MAX, POINTIrfan_MAX);
		max = curaIrfan::PointIrfan(POINTIrfan_MIN, POINTIrfan_MIN);
		for (unsigned int i = 0; i < polys.size(); i++)
		{
			for (unsigned int j = 0; j < polys[i].size(); j++)
			{
				include(polys[i][j]);
			}
		}
	}

	void AABB::calculate(ConstPolygonRef poly)
	{
		min = curaIrfan::PointIrfan(POINTIrfan_MAX, POINTIrfan_MAX);
		max = curaIrfan::PointIrfan(POINTIrfan_MIN, POINTIrfan_MIN);
		for (const curaIrfan::PointIrfan& p : poly)
		{
			include(p);
		}
	}

	bool AABB::contains(const curaIrfan::PointIrfan& point) const
	{
		return point.X >= min.X && point.X <= max.X && point.Y >= min.Y && point.Y <= max.Y;
	}

	bool AABB::hit(const AABB& other) const
	{
		if (max.X < other.min.X) return false;
		if (min.X > other.max.X) return false;
		if (max.Y < other.min.Y) return false;
		if (min.Y > other.max.Y) return false;
		return true;
	}

	void AABB::include(curaIrfan::PointIrfan point)
	{
		min.X = std::min(min.X, point.X);
		min.Y = std::min(min.Y, point.Y);
		max.X = std::max(max.X, point.X);
		max.Y = std::max(max.Y, point.Y);
	}

	void AABB::include(const AABB other)
	{
		include(other.min);
		include(other.max);
	}

	void AABB::expand(int dist)
	{
		if (min == curaIrfan::PointIrfan(POINTIrfan_MAX, POINTIrfan_MAX) || max == curaIrfan::PointIrfan(POINTIrfan_MIN, POINTIrfan_MIN))
		{
			return;
		}
		min.X -= dist;
		min.Y -= dist;
		max.X += dist;
		max.Y += dist;
	}

	Polygon AABB::toPolygon() const
	{
		Polygon ret;
		ret.add(min);
		ret.add(curaIrfan::PointIrfan(max.X, min.Y));
		ret.add(max);
		ret.add(curaIrfan::PointIrfan(min.X, max.Y));
		return ret;
	}

