#ifndef UTILS_POINT3_H
#define UTILS_POINT3_H


#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <stdint.h> //For int32_t and int64_t.
#include <type_traits> // for operations on any arithmetic number type

#include "Coord_tIrfan.h"

class Point3
{
public:
	coord_tIrfan x, y, z;
	
	Point3() {}
	Point3(const coord_tIrfan _x, const coord_tIrfan _y, const coord_tIrfan _z) : x(_x), y(_y), z(_z) {}

	Point3 operator +(const Point3& p) const;
	Point3 operator -(const Point3& p) const;
	Point3 operator *(const Point3& p) const; //!< Element-wise multiplication. For dot product, use .dot()!
	Point3 operator /(const Point3& p) const;
	template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
	Point3 operator *(const num_t i) const
	{
		return Point3(x * i, y * i, z * i);
	}
	template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
	Point3 operator /(const num_t i) const
	{
		return Point3(x / i, y / i, z / i);
	}

	Point3& operator +=(const Point3& p);
	Point3& operator -=(const Point3& p);
	Point3& operator *=(const Point3& p);
	Point3& operator /=(const Point3& p);
	template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
	Point3& operator *=(const num_t i)
	{
		x *= i;
		y *= i;
		z *= i;
		return *this;
	}
	template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
	Point3& operator /=(const num_t i)
	{
		x /= i;
		y /= i;
		z /= i;
		return *this;
	}

	bool operator==(const Point3& p) const;
	bool operator!=(const Point3& p) const;


	template<class CharT, class TraitsT>
	friend
		std::basic_ostream<CharT, TraitsT>&
		operator <<(std::basic_ostream<CharT, TraitsT>& os, const Point3& p)
	{
		return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
	}


	coord_tIrfan max() const
	{
		if (x > y && x > z) return x;
		if (y > z) return y;
		return z;
	}

	bool testLength(coord_tIrfan len) const
	{
		if (x > len || x < -len)
			return false;
		if (y > len || y < -len)
			return false;
		if (z > len || z < -len)
			return false;
		return vSize2() <= len * len;
	}

	coord_tIrfan vSize2() const
	{
		return x * x + y * y + z * z;
	}

	coord_tIrfan vSize() const
	{
		return sqrt(vSize2());
	}

	double vSizeMM() const
	{
		double fx = INT2MM(x);
		double fy = INT2MM(y);
		double fz = INT2MM(z);
		return sqrt(fx*fx + fy*fy + fz*fz);
	}

	coord_tIrfan dot(const Point3& p) const
	{
		return x * p.x + y * p.y + z * p.z;
	}

	

};

#endif