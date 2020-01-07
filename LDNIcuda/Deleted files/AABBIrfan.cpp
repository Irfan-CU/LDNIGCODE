//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <limits>
#include "AABBIrfan.h"
#include "polygonsIrfan.h" //To create the AABB of a polygon.

namespace curaIrfan
{


	AABB::AABB()
		: min(PointIrfan_MAX, PointIrfan_MAX), max(PointIrfan_MIN, PointIrfan_MIN)
	{
	}

	AABB::AABB(const PointIrfan& min, const PointIrfan& max)
		: min(min), max(max)
	{
	}

	AABB::AABB(const Polygons& polys)
		: min(PointIrfan_MAX, PointIrfan_MAX), max(PointIrfan_MIN, PointIrfan_MIN)
	{
		calculate(polys);
	}

	AABB::AABB(ConstPolygonRef poly)
		: min(PointIrfan_MAX, PointIrfan_MAX), max(PointIrfan_MIN, PointIrfan_MIN)
	{
		calculate(poly);
	}

	PointIrfan AABB::getMiddle() const
	{
		return (min + max) / 2;
	}

	void AABB::calculate(const Polygons& polys)
	{
		min = PointIrfan(PointIrfan_MAX, PointIrfan_MAX);
		max = PointIrfan(PointIrfan_MIN, PointIrfan_MIN);
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
		min = PointIrfan(PointIrfan_MAX, PointIrfan_MAX);
		max = PointIrfan(PointIrfan_MIN, PointIrfan_MIN);
		for (const PointIrfan& p : poly)
		{
			include(p);
		}
	}

	bool AABB::contains(const PointIrfan& point) const
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

	void AABB::include(PointIrfan point)
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
		if (min == PointIrfan(PointIrfan_MAX, PointIrfan_MAX) || max == PointIrfan(PointIrfan_MIN, PointIrfan_MIN))
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
		ret.add(PointIrfan(max.X, min.Y));
		ret.add(max);
		ret.add(PointIrfan(min.X, max.Y));
		return ret;
	}

}//namespace cura

