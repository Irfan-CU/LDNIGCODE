//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef UTILS_INT_POINTIrfan_H
#define UTILS_INT_POINTIrfan_H

/**
The integer point classes are used as soon as possible and represent microns in 2D or 3D space.
Integer points are used to avoid floating point rounding errors, and because ClipperLib uses them.
*/
#define INLINE static inline

//Include Clipper to get the ClipperLib::IntPoint definition, which we reuse as Point definition.
#include <clipper.hpp>
#include <cmath>
#include <functional> // for hash function object
#include <iostream> // auto-serialization / auto-toString()
#include <limits>
#include <stdint.h>

#include "Point3.h"
#include "math.h"

//#include "Point3.h" //For applying Point3Matrices.


#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif


namespace curaIrfan
{

/* 64bit Points are used mostly throughout the code, these are the 2D Points from ClipperLib */
	typedef ClipperLib::IntPoint PointIrfan;

	class IntPoint 
	{
	public:
		int X, Y;
		PointIrfan p() { return PointIrfan(X, Y); }
#define POINTIrfan_MIN std::numeric_limits<ClipperLib::cInt>::min()
#define POINTIrfan_MAX std::numeric_limits<ClipperLib::cInt>::max()
	};


static PointIrfan no_point(std::numeric_limits<ClipperLib::cInt>::min(), std::numeric_limits<ClipperLib::cInt>::min());
	
	/* Extra operators to make it easier to do math with the 64bit PointIrfan objects */
	INLINE PointIrfan operator-(const PointIrfan& p0) { return PointIrfan(-p0.X, -p0.Y); }
	INLINE PointIrfan operator+(const PointIrfan& p0, const PointIrfan& p1) { return PointIrfan(p0.X + p1.X, p0.Y + p1.Y); }
	INLINE PointIrfan operator-(const PointIrfan& p0, const PointIrfan& p1) { return PointIrfan(p0.X - p1.X, p0.Y - p1.Y); }
	INLINE PointIrfan operator*(const PointIrfan& p0, const coord_tIrfan i) { return PointIrfan(p0.X * i, p0.Y * i); }
	template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> //Use only for numeric types.
	INLINE PointIrfan operator*(const PointIrfan& p0, const T i) { return PointIrfan(p0.X * i, p0.Y * i); }
	template<typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, T>::type> //Use only for numeric types.
	INLINE PointIrfan operator*(const T i, const PointIrfan& p0) { return p0 * i; }
	INLINE PointIrfan operator/(const PointIrfan& p0, const coord_tIrfan i) { return PointIrfan(p0.X / i, p0.Y / i); }
	INLINE PointIrfan operator/(const PointIrfan& p0, const PointIrfan& p1) { return PointIrfan(p0.X / p1.X, p0.Y / p1.Y); }

	INLINE PointIrfan& operator += (PointIrfan& p0, const PointIrfan& p1) { p0.X += p1.X; p0.Y += p1.Y; return p0; }
	INLINE PointIrfan& operator -= (PointIrfan& p0, const PointIrfan& p1) { p0.X -= p1.X; p0.Y -= p1.Y; return p0; }

	/* ***** NOTE *****
	   TL;DR: DO NOT implement operators *= and /= because of the default values in ClipperLib::IntPoint's constructor.

	   We DO NOT implement operators *= and /= because the class PointIrfan is essentially a ClipperLib::IntPoint and it has a
	   constructor IntPoint(int x = 0, int y = 0), and this causes problems. If you implement *= as *=(int) and when you
	   do "PointIrfan a = a * 5", you probably intend to do "a.x *= 5" and "a.y *= 5", but with that constructor, it will create
	   an IntPoint(5, y = 0) and you end up with wrong results.
	 */

	 //INLINE bool operator==(const PointIrfan& p0, const PointIrfan& p1) { return p0.X==p1.X&&p0.Y==p1.Y; }
	 //INLINE bool operator!=(const PointIrfan& p0, const PointIrfan& p1) { return p0.X!=p1.X||p0.Y!=p1.Y; }

	INLINE coord_tIrfan vSize2(const PointIrfan& p0)
	{
		return p0.X*p0.X + p0.Y*p0.Y;
	}
	INLINE float vSize2f(const PointIrfan& p0)
	{
		return float(p0.X)*float(p0.X) + float(p0.Y)*float(p0.Y);
	}

	INLINE bool shorterThen(const PointIrfan& p0, int32_t len)
	{
		if (p0.X > len || p0.X < -len)
			return false;
		if (p0.Y > len || p0.Y < -len)
			return false;
		return vSize2(p0) <= len * len;
	}

	INLINE coord_tIrfan vSize(const PointIrfan& p0)
	{
		return sqrt(vSize2(p0));
	}

	INLINE double vSizeMM(const PointIrfan& p0)
	{
		double fx = INT2MM(p0.X);
		double fy = INT2MM(p0.Y);
		return sqrt(fx*fx + fy * fy);
	}

	INLINE PointIrfan normal(const PointIrfan& p0, coord_tIrfan len)
	{
		coord_tIrfan _len = vSize(p0);
		if (_len < 1)
			return PointIrfan(len, 0);
		return p0 * len / _len;
	}

	INLINE PointIrfan turn90CCW(const PointIrfan& p0)
	{
		return PointIrfan(-p0.Y, p0.X);
	}

	INLINE PointIrfan rotate(const PointIrfan& p0, double angle)
	{
		const double cos_component = std::cos(angle);
		const double sin_component = std::sin(angle);
		return PointIrfan(cos_component * p0.X - sin_component * p0.Y, sin_component * p0.X + cos_component * p0.Y);
	}

	INLINE coord_tIrfan dot(const PointIrfan& p0, const PointIrfan& p1)
	{
		return p0.X * p1.X + p0.Y * p1.Y;
	}

	INLINE int angle(const PointIrfan& p)
	{
		double angle = std::atan2(p.X, p.Y) / 3.14 * 180.0;
		if (angle < 0.0) angle += 360.0;
		return angle;
	}
};


namespace std 
{
	template <>
	struct hash<curaIrfan::PointIrfan> {
		size_t operator()(const curaIrfan::PointIrfan & pp) const
		{
			static int prime = 31;
			int result = 89;
			result = result * prime + pp.X;
			result = result * prime + pp.Y;
			return result;
		}
	};
};


namespace curaIrfan
{
	class PointMatrix
	{
	public:
		double matrix[4];

		PointMatrix()
		{
			matrix[0] = 1;
			matrix[1] = 0;
			matrix[2] = 0;
			matrix[3] = 1;
		}

		PointMatrix(double rotation)
		{

			rotation = rotation / 180 * 3.14;
			matrix[0] = cos(rotation);
			matrix[1] = -sin(rotation);
			matrix[2] = -matrix[1];
			matrix[3] = matrix[0];
		}

		PointMatrix(const PointIrfan p)
		{
			matrix[0] = p.X;
			matrix[1] = p.Y;
			double f = sqrt((matrix[0] * matrix[0]) + (matrix[1] * matrix[1]));
			matrix[0] /= f;
			matrix[1] /= f;
			matrix[2] = -matrix[1];
			matrix[3] = matrix[0];
		}

		PointIrfan apply(const PointIrfan p) const
		{
			return PointIrfan(p.X * matrix[0] + p.Y * matrix[1], p.X * matrix[2] + p.Y * matrix[3]);
		}

		PointIrfan unapply(const PointIrfan p) const
		{
			return PointIrfan(p.X * matrix[0] + p.Y * matrix[2], p.X * matrix[1] + p.Y * matrix[3]);
		}
	};

	class Point3Matrix
	{
	public:
		double matrix[9];

		Point3Matrix()
		{
			matrix[0] = 1;
			matrix[1] = 0;
			matrix[2] = 0;
			matrix[3] = 0;
			matrix[4] = 1;
			matrix[5] = 0;
			matrix[6] = 0;
			matrix[7] = 0;
			matrix[8] = 1;
		}

		/*!
		 * Initializes the top left corner with the values of \p b
		 * and the rest as if it's a unit matrix
		 */
		Point3Matrix(const PointMatrix& b)
		{
			matrix[0] = b.matrix[0];
			matrix[1] = b.matrix[1];
			matrix[2] = 0;
			matrix[3] = b.matrix[2];
			matrix[4] = b.matrix[3];
			matrix[5] = 0;
			matrix[6] = 0;
			matrix[7] = 0;
			matrix[8] = 1;
		}

		Point3 apply(const Point3 p) const
		{
			return Point3(p.x * matrix[0] + p.y * matrix[1] + p.z * matrix[2]
				, p.x * matrix[3] + p.y * matrix[4] + p.z * matrix[5]
				, p.x * matrix[6] + p.y * matrix[7] + p.z * matrix[8]);
		}

		/*!
		 * Apply matrix to vector as homogeneous coordinates.
		 */
		curaIrfan::PointIrfan apply(const curaIrfan::PointIrfan p) const
		{
			Point3 result = apply(Point3(p.X, p.Y, 1));
			return curaIrfan::PointIrfan(result.x / result.z, result.y / result.z);
		}

		static Point3Matrix translate(const curaIrfan::PointIrfan p)
		{
			Point3Matrix ret; // uniform matrix
			ret.matrix[2] = p.X;
			ret.matrix[5] = p.Y;
			return ret;
		}

		Point3Matrix compose(const Point3Matrix& b)
		{
			Point3Matrix ret;
			for (int outx = 0; outx < 3; outx++)
			{
				for (int outy = 0; outy < 3; outy++)
				{
					ret.matrix[outy * 3 + outx] = 0;
					for (int in = 0; in < 3; in++)
					{
						ret.matrix[outy * 3 + outx] += matrix[outy * 3 + in] * b.matrix[in * 3 + outx];
					}
				}
			}
			return ret;
		}
	};


	inline Point3 operator+(const Point3& p3, const curaIrfan::PointIrfan& p2) {
		return Point3(p3.x + p2.X, p3.y + p2.Y, p3.z);
	}
	inline Point3& operator+=(Point3& p3, const curaIrfan::PointIrfan& p2) {
		p3.x += p2.X;
		p3.y += p2.Y;
		return p3;
	}

	inline curaIrfan::PointIrfan operator+(const curaIrfan::PointIrfan& p2, const Point3& p3) {
		return curaIrfan::PointIrfan(p3.x + p2.X, p3.y + p2.Y);
	}


	inline Point3 operator-(const Point3& p3, const curaIrfan::PointIrfan& p2) {
		return Point3(p3.x - p2.X, p3.y - p2.Y, p3.z);
	}
	inline Point3& operator-=(Point3& p3, const curaIrfan::PointIrfan& p2) {
		p3.x -= p2.X;
		p3.y -= p2.Y;
		return p3;
	}

	inline curaIrfan::PointIrfan operator-(const curaIrfan::PointIrfan& p2, const Point3& p3) {
		return curaIrfan::PointIrfan(p2.X - p3.x, p2.Y - p3.y);
	}
}

#endif


