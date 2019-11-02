#pragma once
#ifndef UTILS_POLYGON_H
#define UTILS_POLYGON_H


#include <vector>
#include <assert.h>
#include <float.h>
#include <clipper.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <list>

#include <initializer_list>
#include "IntpointIrfan.h"

#define CHECK_POLY_ACCESS
#ifdef CHECK_POLY_ACCESS
#define POLY_ASSERT(e) assert(e)
#else
#define POLY_ASSERT(e) do {} while(0)
#endif


class Polygons;
class Polygon;
class PolygonRef;
class ListPolyIt;

typedef std::list<curaIrfan::PointIrfan> ListPolygon; //!< A polygon represented by a linked list instead of a vector
typedef std::vector<ListPolygon> ListPolygons; //!< Polygons represented by a vector of linked lists instead of a vector of vectors

const static int clipper_init = (0);
#define NO_INDEX (std::numeric_limits<unsigned int>::max())


class ConstPolygonPointer;

class ConstPolygonRef
{
	friend class Polygon;
	friend class Polygons;
	friend class PolygonRef;
	friend class ConstPolygonPointer;

	ClipperLib::Path* path;
public:

	ConstPolygonRef(const ClipperLib::Path& polygong)
		: path(const_cast<ClipperLib::Path*>(&polygong))
	{}

	virtual ~ConstPolygonRef()
	{
	}
	size_t size() const;
	double area() const
	{
		return ClipperLib::Area(*path);
	}
	bool empty() const;
	const curaIrfan::PointIrfan& operator[] (unsigned int index) const
	{
		//POLY_ASSERT(index < size());
		return (*path)[index];
	}

	Polygons intersection(const ConstPolygonRef& other) const;

	bool _inside(curaIrfan::PointIrfan p, bool border_result = false) const;

	curaIrfan::PointIrfan centerOfMass() const
	{
		double x = 0, y = 0;
		curaIrfan::PointIrfan p0 = (*path)[path->size() - 1];
		for (unsigned int n = 0; n < path->size(); n++)
		{
			curaIrfan::PointIrfan p1 = (*path)[n];
			double second_factor = (p0.X * p1.Y) - (p1.X * p0.Y);

			x += double(p0.X + p1.X) * second_factor;
			y += double(p0.Y + p1.Y) * second_factor;
			p0 = p1;
		}

		double area = Area(*path);

		x = x / 6 / area;
		y = y / 6 / area;

		return curaIrfan::PointIrfan(x, y);
	}


	void smooth(int remove_length, PolygonRef result) const;
	//void smooth_outward(const double angle, int shortcut_length, PolygonRef result) const;
	void smooth2(int remove_length, PolygonRef result) const;
	Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;
	/*!
	 * Clipper function.
	 * Returns false if outside, true if inside; if the curaIrfan::PointIrfan lies exactly on the border, will return 'border_result'.
	 *
	 * http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
	 */
	bool inside(curaIrfan::PointIrfan p, bool border_result = false) const
	{
		int res = ClipperLib::PointInPolygon(p, *path);
		if (res == -1)
		{
			return border_result;
		}
		return res == 1;
	}

	curaIrfan::PointIrfan closestPointTo(curaIrfan::PointIrfan  p) const
	{
		curaIrfan::PointIrfan  ret = p;
		float bestDist = FLT_MAX;
		for (unsigned int n = 0; n < path->size(); n++)
		{
			float dist = curaIrfan::vSize2f(curaIrfan::operator-(p,(*path)[n]));
			if (dist < bestDist)
			{
				ret = (*path)[n];
				bestDist = dist;
			}
		}
		return ret;
	}

	const ClipperLib::Path& operator*() const
	{
		return *path;
	}
	ClipperLib::Path::const_iterator begin() const
	{
		return path->begin();
	}
	ClipperLib::Path::const_iterator end() const
	{
		return path->end();
	}

	ClipperLib::Path::const_reference back() const
	{
		return path->back();
	}

	bool orientation() const
	{
		return ClipperLib::Orientation(*path);
	}
};

class PolygonPointer;

class PolygonRef : public ConstPolygonRef
{
public:
	PolygonRef(ClipperLib::Path& polygong)
		: ConstPolygonRef(polygong)
	{}

	PolygonRef(const PolygonRef& other)
		: ConstPolygonRef(*other.path)
	{}

	virtual ~PolygonRef()
	{
	}
	template <typename... Args>
	void emplace_back(Args&&... args)
	{
		path->emplace_back(args...);
	}
	

	PolygonRef& operator=(const ConstPolygonRef& other) = delete; // polygon assignment is expensive and probably not what you want when you use the assignment operator

	PolygonRef& operator=(ConstPolygonRef& other) = delete; // polygon assignment is expensive and probably not what you want when you use the assignment operator
//     { path = other.path; return *this; }

	PolygonRef& operator=(PolygonRef&& other)
	{
		*path = std::move(*other.path);
		return *this;
	}

	curaIrfan::PointIrfan& operator[] (unsigned int index)
	{
		//POLY_ASSERT(index < size());
		return (*path)[index];
	}

	void add(const curaIrfan::PointIrfan p)
	{
		return path->push_back(p);
	}
	unsigned int size() const
	{
		return path->size();
	}
	ClipperLib::Path::iterator begin()
	{
		return path->begin();
	}

	ClipperLib::Path::iterator end()
	{
		return path->end();
	}

	ClipperLib::Path::reference back()
	{
		return path->back();
	}

	ClipperLib::Path& operator*()
	{
		return *path;
	}

	void reverse()
	{
		ClipperLib::ReversePath(*path);
	}

	void simplify(const coord_tIrfan smallest_line_segment_squared = 100, const coord_tIrfan allowed_error_distance_squared = 25);
	
	void clear()
	{
		path->clear();
	}
	
	

	void pop_back()
	{
		path->pop_back();
	}
};

class ConstPolygonPointer
{
protected:
	const ClipperLib::Path* path;
public:
	ConstPolygonPointer()
		: path(nullptr)
	{}
	ConstPolygonPointer(const ConstPolygonRef* ref)
		: path(ref->path)
	{}
	ConstPolygonPointer(const ConstPolygonRef& ref)
		: path(ref.path)
	{}

	ConstPolygonRef operator*() const
	{
		//assert(path);
		return ConstPolygonRef(*path);
	}
	const ClipperLib::Path* operator->() const
	{
		//assert(path);
		return path;
	}

	operator bool() const
	{
		return path;
	}

	bool operator==(const ConstPolygonPointer& rhs)
	{
		return path == rhs.path;
	}
};

class Polygon : public PolygonRef

{
	ClipperLib::Path poly;
public:
	Polygon()
		: PolygonRef(poly)
	{
	}

	Polygon(const ConstPolygonRef& other)
		: PolygonRef(poly)
		, poly(*other.path)
	{
	}

	Polygon(const Polygon& other)
		: PolygonRef(poly)
		, poly(*other.path)
	{
	}

	Polygon(Polygon&& moved)
		: PolygonRef(poly)
		, poly(std::move(moved.poly))
	{
	}

	virtual ~Polygon()
	{
	}

	Polygon& operator=(const ConstPolygonRef& other) = delete; // copying a single polygon is generally not what you want
//     {
//         path = other.path;
//         poly = *other.path;
//         return *this;
//     }

	Polygon& operator=(Polygon&& other) //!< move assignment
	{
		poly = std::move(other.poly);
		return *this;
	}

};

class PolygonsPart;

class Polygons
{
	friend class Polygon;
	friend class PolygonRef;
	friend class ConstPolygonRef;
protected:
	ClipperLib::Paths paths;
public:
	unsigned int size() const
	{
		return paths.size();
	}

	/*!
	 * Convenience function to check if the polygon has no points.
	 *
	 * \return `true` if the polygon has no points, or `false` if it does.
	 */
	bool empty() const;

	unsigned int pointCount() const; //!< Return the amount of points in all polygons

	PolygonRef operator[] (unsigned int index)
	{
		//POLY_ASSERT(index < size() && index <= std::numeric_limits<int>::max());
		return paths[index];
	}
	ConstPolygonRef operator[] (unsigned int index) const
	{
		//POLY_ASSERT(index < size() && index <= std::numeric_limits<int>::max());
		return paths[index];
	}
	ClipperLib::Paths::iterator begin()
	{
		return paths.begin();
	}
	ClipperLib::Paths::const_iterator begin() const
	{
		return paths.begin();
	}
	ClipperLib::Paths::iterator end()
	{
		return paths.end();
	}
	ClipperLib::Paths::const_iterator end() const
	{
		return paths.end();
	}
	/*!
	 * Remove a polygon from the list and move the last polygon to its place
	 *
	 * \warning changes the order of the polygons!
	 */
	void remove(unsigned int index)
	{
		POLY_ASSERT(index < size() && index <= std::numeric_limits<int>::max());
		if (index < paths.size() - 1)
		{
			paths[index] = std::move(paths.back());
		}
		paths.resize(paths.size() - 1);
	}
	/*!
	 * Remove a range of polygons
	 */
	void erase(ClipperLib::Paths::iterator start, ClipperLib::Paths::iterator end)
	{
		paths.erase(start, end);
	}
	void clear()
	{
		paths.clear();
	}
	void add(ConstPolygonRef& poly)
	{
		paths.push_back(*poly.path);
	}
	void add(const ConstPolygonRef& poly)
	{
		paths.push_back(*poly.path);
	}
	
	void add(Polygon&& other_poly)
	{
		paths.emplace_back(std::move(*other_poly));
	}
	void add(const Polygons& other)
	{
		std::copy(other.paths.begin(), other.paths.end(), std::back_inserter(paths));
	}
	/*!
	 * Add a 'polygon' consisting of two points
	 */
	void addLine(const curaIrfan::PointIrfan from, const curaIrfan::PointIrfan to)
	{
		paths.emplace_back(ClipperLib::Path{ from, to });
	}

	


	template<typename... Args>
	void emplace_back(Args... args)
	{
		paths.emplace_back(args...);
	}

	PolygonRef newPoly()
	{
		paths.emplace_back();
		return PolygonRef(paths.back());
	}
	PolygonRef back()
	{
		return PolygonRef(paths.back());
	}
	ConstPolygonRef back() const
	{
		return ConstPolygonRef(paths.back());
	}

	Polygons() {}

	Polygons(const Polygons& other) { paths = other.paths; }
	Polygons(Polygons&& other) { paths = std::move(other.paths); }
	Polygons& operator=(const Polygons& other) { paths = other.paths; return *this; }
	Polygons& operator=(Polygons&& other) { paths = std::move(other.paths); return *this; }

	bool operator==(const Polygons& other) const = delete;

	/*!
	 * Convert ClipperLib::PolyTree to a Polygons object,
	 * which uses ClipperLib::Paths instead of ClipperLib::PolyTree
	 */
	static Polygons toPolygons(ClipperLib::PolyTree& poly_tree);

	Polygons difference(const Polygons& other) const
	{
		Polygons ret;
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptSubject, true);
		clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctDifference, ret.paths);
		return ret;
	}
	Polygons unionPolygons(const Polygons& other) const
	{
		Polygons ret;
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptSubject, true);
		clipper.AddPaths(other.paths, ClipperLib::ptSubject, true);
		clipper.Execute(ClipperLib::ctUnion, ret.paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
		return ret;
	}
	/*!
	 * Union all polygons with each other (When polygons.add(polygon) has been called for overlapping polygons)
	 */
	Polygons unionPolygons() const
	{
		return unionPolygons(Polygons());
	}
	Polygons intersection(const Polygons& other) const
	{
		Polygons ret;
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptSubject, true);
		clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctIntersection, ret.paths);
		return ret;
	}

	/*!
	 * Intersect polylines with this area Polygons object.
	 */
	Polygons intersectionPolyLines(const Polygons& polylines) const;

	/*!
	 * Clips input line segments by this Polygons.
	 * \param other Input line segments to be cropped
	 * \param segment_tree the resulting interior line segments
	 */
	void lineSegmentIntersection(const Polygons& other, ClipperLib::PolyTree& segment_tree) const
	{
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptClip, true);
		clipper.AddPaths(other.paths, ClipperLib::ptSubject, false);
		clipper.Execute(ClipperLib::ctIntersection, segment_tree);
	}
	Polygons xorPolygons(const Polygons& other) const
	{
		Polygons ret;
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptSubject, true);
		clipper.AddPaths(other.paths, ClipperLib::ptClip, true);
		clipper.Execute(ClipperLib::ctXor, ret.paths);
		return ret;
	}

	Polygons offset(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter, double miter_limit = 1.2) const;

	Polygons offsetPolyLine(int distance, ClipperLib::JoinType joinType = ClipperLib::jtMiter) const
	{
		Polygons ret;
		double miterLimit = 1.2;
		ClipperLib::ClipperOffset clipper(miterLimit, 10.0);
		clipper.AddPaths(paths, joinType, ClipperLib::etOpenSquare);
		clipper.MiterLimit = miterLimit;
		clipper.Execute(ret.paths, distance);
		return ret;
	}

	/*!
	 * Check if we are inside the polygon.
	 *
	 * We do this by counting the number of polygons inside which this curaIrfan::PointIrfan lies.
	 * An odd number is inside, while an even number is outside.
	 *
	 * Returns false if outside, true if inside; if the curaIrfan::PointIrfan lies exactly on the border, will return \p border_result.
	 *
	 * \param p The curaIrfan::PointIrfan for which to check if it is inside this polygon
	 * \param border_result What to return when the curaIrfan::PointIrfan is exactly on the border
	 * \return Whether the curaIrfan::PointIrfan \p p is inside this polygon (or \p border_result when it is on the border)
	 */
	bool inside(curaIrfan::PointIrfan p, bool border_result = false) const;

	/*!
	 * Check if we are inside the polygon. We do this by tracing from the curaIrfan::PointIrfan towards the positive X direction,
	 * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
	 * Care needs to be taken, if p.Y exactly matches a vertex to the right of p, then we need to count 1 intersect if the
	 * outline passes vertically past; and 0 (or 2) intersections if that curaIrfan::PointIrfan on the outline is a 'top' or 'bottom' vertex.
	 * The easiest way to do this is to break out two cases for increasing and decreasing Y ( from p0 to p1 ).
	 * A segment is tested if pa.Y <= p.Y < pb.Y, where pa and pb are the points (from p0,p1) with smallest & largest Y.
	 * When both have the same Y, no intersections are counted but there is a special test to see if the curaIrfan::PointIrfan falls
	 * exactly on the line.
	 *
	 * Returns false if outside, true if inside; if the curaIrfan::PointIrfan lies exactly on the border, will return \p border_result.
	 *
	 * \deprecated This function is old and no longer used. instead use \ref Polygons::inside
	 *
	 * \param p The curaIrfan::PointIrfan for which to check if it is inside this polygon
	 * \param border_result What to return when the curaIrfan::PointIrfan is exactly on the border
	 * \return Whether the curaIrfan::PointIrfan \p p is inside this polygon (or \p border_result when it is on the border)
	 */
	bool insideOld(curaIrfan::PointIrfan p, bool border_result = false) const;

	/*!
	 * Find the polygon inside which curaIrfan::PointIrfan \p p resides.
	 *
	 * We do this by tracing from the curaIrfan::PointIrfan towards the positive X direction,
	 * every line we cross increments the crossings counter. If we have an even number of crossings then we are not inside the polygon.
	 * We then find the polygon with an uneven number of crossings which is closest to \p p.
	 *
	 * If \p border_result, we return the first polygon which is exactly on \p p.
	 *
	 * \param p The curaIrfan::PointIrfan for which to check in which polygon it is.
	 * \param border_result Whether a curaIrfan::PointIrfan exactly on a polygon counts as inside
	 * \return The index of the polygon inside which the curaIrfan::PointIrfan \p p resides
	 */
	unsigned int findInside(curaIrfan::PointIrfan p, bool border_result = false);

	/*!
	 * Approximates the convex hull of the polygons.
	 * \p extra_outset Extra offset outward
	 * \return the convex hull (approximately)
	 *
	 */
	Polygons approxConvexHull(int extra_outset = 0);

	/*!
	 * Compute the area enclosed within the polygons (minus holes)
	 *
	 * \return The area in square micron
	 */
	double area() const;

	/*!
	 * Smooth out small perpendicular segments
	 * Smoothing is performed by removing the inner most vertex of a line segment smaller than \p remove_length
	 * which has an angle with the next and previous line segment smaller than roughly 150*
	 *
	 * Note that in its current implementation this function doesn't remove line segments with an angle smaller than 30*
	 * Such would be the case for an N shape.
	 *
	 * \param remove_length The length of the largest segment removed
	 * \return The smoothed polygon
	 */
	Polygons smooth(int remove_length) const;

	/*!
	 * Smooth out sharp inner corners, by taking a shortcut which bypasses the corner
	 *
	 * \param angle The maximum angle of inner corners to be smoothed out
	 * \param shortcut_length The desired length of the shortcut line segment introduced (shorter shortcuts may be unavoidable)
	 * \return The resulting polygons
	 */
	//Polygons smooth_outward(const double angle, int shortcut_length);

	Polygons smooth2(int remove_length, int min_area) const; //!< removes points connected to small lines

	/*!
	 * removes points connected to similarly oriented lines
	 *
	 * \param smallest_line_segment maximal length of removed line segments
	 * \param allowed_error_distance The distance of the middle curaIrfan::PointIrfan to the line segment of the consecutive and previous curaIrfan::PointIrfan for which the middle curaIrfan::PointIrfan is removed
	 */
	void simplify(const coord_tIrfan smallest_line_segment = 10, const coord_tIrfan allowed_error_distance = 5)
	{
		const coord_tIrfan allowed_error_distance_squared = allowed_error_distance * allowed_error_distance;
		const coord_tIrfan smallest_line_segment_squared = smallest_line_segment * smallest_line_segment;
		Polygons& thiss = *this;
		for (size_t p = 0; p < size(); p++)
		{
			//printf("Before simplify in simplification the pointcount is %d \n", thiss.pointCount());
			thiss[p].simplify(smallest_line_segment_squared, allowed_error_distance_squared);
			//printf("After clear in simplification the pointcount is %d \n",thiss.pointCount());
			if (thiss[p].size() < 3)
			{
				remove(p);
				p--;
			}
		}
	}

	/*!
	 * Remove all but the polygons on the very outside.
	 * Exclude holes and parts within holes.
	 * \return the resulting polygons.
	 */
	Polygons getOutsidePolygons() const;

	/*!
	 * Exclude holes which have no parts inside of them.
	 * \return the resulting polygons.
	 */
	Polygons removeEmptyHoles() const;

	/*!
	 * Return hole polygons which have no parts inside of them.
	 * \return the resulting polygons.
	 */
	Polygons getEmptyHoles() const;

	/*!
	 * Split up the polygons into groups according to the even-odd rule.
	 * Each PolygonsPart in the result has an outline as first polygon, whereas the rest are holes.
	 */
	std::vector<PolygonsPart> splitIntoParts(bool unionAll = false) const;
private:
	/*!
	 * recursive part of \ref Polygons::removeEmptyHoles and \ref Polygons::getEmptyHoles
	 * \param node The node of the polygons part to process
	 * \param remove_holes Whether to remove empty holes or everything but the empty holes
	 * \param ret Where to store polygons which are not empty holes
	 */
	void removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Polygons& ret) const;
	void splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const;

	/*!
	 * Convert a node from a ClipperLib::PolyTree and add it to a Polygons object,
	 * which uses ClipperLib::Paths instead of ClipperLib::PolyTree
	 */
	void addPolyTreeNodeRecursive(const ClipperLib::PolyNode& node);
public:
	/*!
	 * Split up the polygons into groups according to the even-odd rule.
	 * Each vector in the result has the index to an outline as first index, whereas the rest are indices to holes.
	 *
	 * \warning Note that this function reorders the polygons!
	 */
	//PartsView splitIntoPartsView(bool unionAll = false);
private:
	//void splitIntoPartsView_processPolyTreeNode(PartsView& partsView, Polygons& reordered, ClipperLib::PolyNode* node) const;
public:
	/*!
	 * Removes polygons with area smaller than \p minAreaSize (note that minAreaSize is in mm^2, not in micron^2).
	 */
	void removeSmallAreas(double minAreaSize)
	{
		Polygons& thiss = *this;
		for (unsigned int i = 0; i < size(); i++)
		{
			double area = INT2MM(INT2MM(fabs(thiss[i].area())));
			if (area < minAreaSize) // Only create an up/down skin if the area is large enough. So you do not create tiny blobs of "trying to fill"
			{
				remove(i);
				i -= 1;
			}
		}
	}
	/*!
	 * Removes overlapping consecutive line segments which don't delimit a positive area.
	 */
	void removeDegenerateVerts()
	{
		Polygons& thiss = *this;
		for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
		{
			PolygonRef poly = thiss[poly_idx];
			Polygon result;

			auto isDegenerate = [](curaIrfan::PointIrfan& last, curaIrfan::PointIrfan& now, curaIrfan::PointIrfan& next)
			{
				curaIrfan::PointIrfan check = curaIrfan::operator-(now, last);
				curaIrfan::PointIrfan last_line = check;
				curaIrfan::PointIrfan check1 = curaIrfan::operator-(next, now);
				curaIrfan::PointIrfan next_line = check1;
				return curaIrfan::dot(last_line, next_line) == -1 * curaIrfan::vSize(last_line) * curaIrfan::vSize(next_line);
			};
			bool isChanged = false;
			for (unsigned int idx = 0; idx < poly.size(); idx++)
			{
				curaIrfan::PointIrfan& last = (result.size() == 0) ? poly.back() : result.back();
				if (idx + 1 == poly.size() && result.size() == 0) { break; }
				curaIrfan::PointIrfan& next = (idx + 1 == poly.size()) ? result[0] : poly[idx + 1];
				if (isDegenerate(last, poly[idx], next))
				{ // lines are in the opposite direction
					// don't add vert to the result
					isChanged = true;
					while (result.size() > 1 && isDegenerate(result[result.size() - 2], result.back(), next))
					{
						result.pop_back();
					}
				}
				else
				{
					result.add(poly[idx]);
				}
			}

			if (isChanged)
			{
				if (result.size() > 2)
				{


					*poly = *result;
				}
				else
				{
					thiss.remove(poly_idx);
					poly_idx--; // effectively the next iteration has the same poly_idx (referring to a new poly which is not yet processed)
				}
			}
		}
	}
	/*!
	 * Removes the same polygons from this set (and also empty polygons).
	 * Polygons are considered the same if all points lie within [same_distance] of their counterparts.
	 */
	Polygons remove(const Polygons& to_be_removed, int same_distance = 0) const
	{
		Polygons result;
		for (unsigned int poly_keep_idx = 0; poly_keep_idx < size(); poly_keep_idx++)
		{
			ConstPolygonRef poly_keep = (*this)[poly_keep_idx];
			bool should_be_removed = false;
			if (poly_keep.size() > 0)
				//             for (int hole_poly_idx = 0; hole_poly_idx < to_be_removed.size(); hole_poly_idx++)
				for (ConstPolygonRef poly_rem : to_be_removed)
				{
					//                 PolygonRef poly_rem = to_be_removed[hole_poly_idx];
					if (poly_rem.size() != poly_keep.size() || poly_rem.size() == 0) continue;

					// find closest curaIrfan::PointIrfan, supposing this curaIrfan::PointIrfan aligns the two shapes in the best way
					int closest_point_idx = 0;
					int smallestDist2 = -1;
					for (unsigned int point_rem_idx = 0; point_rem_idx < poly_rem.size(); point_rem_idx++)
					{
						curaIrfan::PointIrfan check = curaIrfan::operator-(poly_rem[point_rem_idx], poly_keep[0]);
						int dist2 = curaIrfan::vSize2(check);
						if (dist2 < smallestDist2 || smallestDist2 < 0)
						{
							smallestDist2 = dist2;
							closest_point_idx = point_rem_idx;
						}
					}
					bool poly_rem_is_poly_keep = true;
					// compare the two polygons on all points
					if (smallestDist2 > same_distance * same_distance)
						continue;
					for (unsigned int point_idx = 0; point_idx < poly_rem.size(); point_idx++)
					{
						curaIrfan::PointIrfan check = curaIrfan::operator-(poly_rem[(closest_point_idx + point_idx) % poly_rem.size()], poly_keep[point_idx]);
						int dist2 = curaIrfan::vSize2(check);
						if (dist2 > same_distance * same_distance)
						{
							poly_rem_is_poly_keep = false;
							break;
						}
					}
					if (poly_rem_is_poly_keep)
					{
						should_be_removed = true;
						break;
					}
				}
			if (!should_be_removed)
				result.add(poly_keep);

		}
		return result;
	}

	Polygons processEvenOdd() const
	{
		Polygons ret;
		ClipperLib::Clipper clipper(clipper_init);
		clipper.AddPaths(paths, ClipperLib::ptSubject, true);
		clipper.Execute(ClipperLib::ctUnion, ret.paths);
		return ret;
	}

	coord_tIrfan polygonLength() const
	{
		coord_tIrfan length = 0;
		for (unsigned int i = 0; i < paths.size(); i++)
		{
			curaIrfan::PointIrfan p0 = paths[i][paths[i].size() - 1];
			for (unsigned int n = 0; n < paths[i].size(); n++)
			{
				curaIrfan::PointIrfan p1 = paths[i][n];
				curaIrfan::PointIrfan check = curaIrfan::operator-(p0,p1);
				length += curaIrfan::vSize(check);
				p0 = p1;
			}
		}
		return length;
	}

	coord_tIrfan polyLineLength() const;

	void applyMatrix(const curaIrfan::PointMatrix& matrix)
	{
		for (unsigned int i = 0; i < paths.size(); i++)
		{
			for (unsigned int j = 0; j < paths[i].size(); j++)
			{
				paths[i][j] = matrix.apply(paths[i][j]);
			}
		}
	}
};

class PolygonsPart : public Polygons
{
public:
	PolygonRef outerPolygon()
	{
		return paths[0];
	}
	ConstPolygonRef outerPolygon() const
	{
		return paths[0];
	}

	/*!
	 * Tests whether the given curaIrfan::PointIrfan is inside this polygon part.
	 * \param p The curaIrfan::PointIrfan to test whether it is inside.
	 * \param border_result If the curaIrfan::PointIrfan is exactly on the border, this will be
	 * returned instead.
	 */
	bool inside(curaIrfan::PointIrfan p, bool border_result = false) const;
};

#endif // !UTILS_POLYGON_h
