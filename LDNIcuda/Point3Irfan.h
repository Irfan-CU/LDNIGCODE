#pragma once

#ifndef UTILS_POINT3_H
#define UTILS_POINT3_H

#include <cmath> //For sqrt.
#include <iostream> //Auto-serialization.
#include <limits> //For numeric_limits::min and max.
#include <stdint.h> //For int32_t and int64_t.
#include <type_traits> // for operations on any arithmetic number type
#include "coord_tIrfan.h"



namespace curaIrfan
{
	class Point3Irfan
	{
	public:
		coord_tIrfan x, y, z;
		Point3Irfan() {}
		Point3Irfan(const coord_tIrfan _x, const coord_tIrfan _y, const coord_tIrfan _z) : x(_x), y(_y), z(_z) {}

		Point3Irfan operator +(const Point3Irfan& p) const;
		Point3Irfan operator -(const Point3Irfan& p) const;
		Point3Irfan operator *(const Point3Irfan& p) const; //!< Element-wise multiplication. For dot product, use .dot()!
		Point3Irfan operator /(const Point3Irfan& p) const;
		template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
		Point3Irfan operator *(const num_t i) const
		{
			return Point3(x * i, y * i, z * i);
		}
		template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
		Point3Irfan operator /(const num_t i) const
		{
			return Point3Irfan(x / i, y / i, z / i);
		}

		Point3Irfan& operator +=(const Point3Irfan& p);
		Point3Irfan& operator -=(const Point3Irfan& p);
		Point3Irfan& operator *=(const Point3Irfan& p);
		Point3Irfan& operator /=(const Point3Irfan& p);
		template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
		Point3Irfan& operator *=(const num_t i)
		{
			x *= i;
			y *= i;
			z *= i;
			return *this;
		}
		template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
		Point3Irfan& operator /=(const num_t i)
		{
			x /= i;
			y /= i;
			z /= i;
			return *this;
		}

		bool operator==(const Point3Irfan& p) const;
		bool operator!=(const Point3Irfan& p) const;


		template<class CharT, class TraitsT>
		friend
			std::basic_ostream<CharT, TraitsT>&
			operator <<(std::basic_ostream<CharT, TraitsT>& os, const Point3Irfan& p)
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
			return sqrt(fx*fx + fy * fy + fz * fz);
		}

		coord_tIrfan dot(const Point3Irfan& p) const
		{
			return x * p.x + y * p.y + z * p.z;
		}

	};

	/*!
	 * \brief Placeholder coordinate point (3D).
	 *
	 * Its value is something that is rarely used.
	 */
	static Point3Irfan no_Point3Irfan(std::numeric_limits<coord_tIrfan>::min(), std::numeric_limits<coord_tIrfan>::min(), std::numeric_limits<coord_tIrfan>::min());

	template<typename num_t, typename = typename std::enable_if<std::is_arithmetic<num_t>::value, num_t>::type>
	inline Point3Irfan operator*(const num_t i, const Point3Irfan& rhs)
	{
		return rhs * i;
	}

}

#endif //UTILS_POINT3_H
