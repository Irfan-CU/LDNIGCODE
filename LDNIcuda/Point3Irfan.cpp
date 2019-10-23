//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "Point3Irfan.h" //The headers we're implementing.

namespace curaIrfan
{

	Point3Irfan Point3Irfan::operator +(const Point3Irfan& p) const
	{
		return Point3Irfan(x + p.x, y + p.y, z + p.z);
	}

	Point3Irfan Point3Irfan::operator -(const Point3Irfan& p) const
	{
		return Point3Irfan(x - p.x, y - p.y, z - p.z);
	}

	Point3Irfan Point3Irfan::operator *(const Point3Irfan& p) const
	{
		return Point3Irfan(x * p.x, y * p.y, z * p.z);
	}

	Point3Irfan Point3Irfan::operator /(const Point3Irfan& p) const
	{
		return Point3Irfan(x / p.x, y / p.y, z / p.z);
	}

	Point3Irfan& Point3Irfan::operator +=(const Point3Irfan& p)
	{
		x += p.x;
		y += p.y;
		z += p.z;
		return *this;
	}

	Point3Irfan& Point3Irfan::operator -=(const Point3Irfan& p)
	{
		x -= p.x;
		y -= p.y;
		z -= p.z;
		return *this;
	}

	Point3Irfan& Point3Irfan::operator *=(const Point3Irfan& p)
	{
		x *= p.x;
		y *= p.y;
		z *= p.z;
		return *this;
	}

	Point3Irfan& Point3Irfan::operator /=(const Point3Irfan& p)
	{
		x /= p.x;
		y /= p.y;
		z /= p.z;
		return *this;
	}

	bool Point3Irfan::operator ==(const Point3Irfan& p) const
	{
		return x == p.x && y == p.y && z == p.z;
	}

	bool Point3Irfan::operator !=(const Point3Irfan& p) const
	{
		return x != p.x || y != p.y || z != p.z;
	}

}
