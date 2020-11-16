
#include "Polygon.h"
#include "LinearAlgebra2D.h"
#include "Slicer.h"

bool PolygonsPart::inside(curaIrfan::PointIrfan p, bool border_result) const
{
	if (size() < 1)
	{
		return false;
	}
	if (!(*this)[0].inside(p, border_result))
	{
		return false;
	}
	for (unsigned int n = 1; n < paths.size(); n++)
	{
		if ((*this)[n].inside(p, border_result))
		{
			return false;
		}
	}
	return true;
}

Polygons Polygons::offset(int distance, ClipperLib::JoinType join_type, double miter_limit) const
{
	//printf("the distance for infill offsets is %d \n", distance);
	if (distance == 0)
	{
		return *this;
	}
	Polygons ret;
	ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
	clipper.AddPaths(unionPolygons().paths, join_type, ClipperLib::etClosedPolygon);
	clipper.MiterLimit = miter_limit;
	
	clipper.Execute(ret.paths, distance);
	return ret;
}

double Polygons::area() const
{
	double area = 0.0;
	for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
	{
		area += operator[](poly_idx).area();
		// note: holes already have negative area
	}
	return area;
}

bool Polygons::empty() const
{
	return paths.empty();
}

size_t ConstPolygonRef::size() const
{
	return path->size();
}

bool ConstPolygonRef::empty() const
{
	return path->empty();
}

Polygons ConstPolygonRef::offset(int distance, ClipperLib::JoinType join_type, double miter_limit) const
{
	if (distance == 0)
	{
		Polygons ret;
		ret.add(*this);
		return ret;
	}
	Polygons ret;
	ClipperLib::ClipperOffset clipper(miter_limit, 10.0);
	clipper.AddPath(*path, join_type, ClipperLib::etClosedPolygon);
	clipper.MiterLimit = miter_limit;
	clipper.Execute(ret.paths, distance);
	return ret;
}

Polygons ConstPolygonRef::intersection(const ConstPolygonRef& other) const
{
	Polygons ret;
	ClipperLib::Clipper clipper(clipper_init);
	clipper.AddPath(*path, ClipperLib::ptSubject, true);
	clipper.AddPath(*other.path, ClipperLib::ptClip, true);
	clipper.Execute(ClipperLib::ctIntersection, ret.paths);
	return ret;
}

void PolygonRef::simplify(const coord_tIrfan smallest_line_segment_squared, const coord_tIrfan allowed_error_distance_squared)
{
	//printf("inside PolygonRef simplication and is at line 67 of Polygon.cpp the size() is %d \n.", size());
	if (size() < 3)
	{
		clear();
		return;
	}
	if (size() == 3)
	{
		return;
	}

	ClipperLib::Path new_path;
	curaIrfan::PointIrfan previous = path->at(0);
	curaIrfan::PointIrfan current = path->at(1);
	/* When removing a vertex, we'll check if the delta area of the polygon
	 * remains below allowed_error_distance_squared. However when removing
	 * multiple consecutive vertices, each individual vertex may result in a
	 * delta area below the threshold, while the total effect of removing all of
	 * those vertices results in too much area being removed. So we accumulate
	 * the area that is going to be removed by a streak of consecutive vertices
	 * and don't allow that to exceed allowed_error_distance_squared. */
	coord_tIrfan accumulated_area_removed = previous.X * current.Y - previous.Y * current.X; //Shoelace formula for area of polygon per line segment.
	//printf("inside PolygonRef simplication and is at line 89 of Polygon.cpp the area is %d \n.", accumulated_area_removed);
	for (size_t point_idx = 1; point_idx <= size(); point_idx++)
	{
		current = path->at(point_idx % size());
		//printf("inside for loop @ 93 of Polygon.cpp the area is %d \n.", accumulated_area_removed);
		curaIrfan::PointIrfan check = curaIrfan::operator-(current, previous);
		const coord_tIrfan length2 = curaIrfan::vSize2(check);

		//Check if the accumulated area doesn't exceed the maximum.
		curaIrfan::PointIrfan next;
		if (point_idx + 1 < size())
		{
			next = path->at(point_idx + 1);
		}
		else if (!new_path.empty())
		{
			next = new_path[0]; //Spill over to new polygon for checking removed area.
		}
		else
		{
			break; //New polygon also doesn't have any vertices yet, meaning we've completed the loop without adding any vertices. The entire polygon is too small to be significant.
		}
		accumulated_area_removed += current.X * next.Y - current.Y * next.X; //Shoelace formula for area of polygon per line segment.
		const coord_tIrfan area_removed_so_far = accumulated_area_removed + next.X * previous.Y - next.Y * previous.X; 
		check = curaIrfan::operator-(next, previous);//Close the polygon.
		const coord_tIrfan base_length_2 = curaIrfan::vSize2(check);
		if (base_length_2 == 0) //Two line segments form a line back and forth with no area.
		{
			continue; //Remove the vertex.
		}
		//printf("inside PolygonRef simplication and is at line 120 \n.");
		//We want to check if the height of the triangle formed by previous, current and next vertices is less than allowed_error_distance_squared.
		//A = 1/2 * b * h     [triangle area formula]
		//2A = b * h          [multiply by 2]
		//h = 2A / b          [divide by b]
		//h^2 = (2A / b)^2    [square it]
		//h^2 = (2A)^2 / b^2  [factor the divisor]
		//h^2 = 4A^2 / b^2    [remove brackets of (2A)^2]
		
		const coord_tIrfan height_2 = (4 * area_removed_so_far * area_removed_so_far) / base_length_2;
		//printf("inside PolygonRef simplication and is at line 146 \n.");
		if (length2 < smallest_line_segment_squared && height_2 <= allowed_error_distance_squared) //Line is small and removing it doesn't introduce too much error.
		{
			continue; //Remove the vertex.
		}
		else if (length2 >= smallest_line_segment_squared && new_path.size() > 2 ) //Almost exactly straight (barring rounding errors).
		{
			new_path.pop_back(); //Remove the previous vertex but still add the new one.
		}
		//Don't remove the vertex.
		
		accumulated_area_removed = current.X * next.Y - current.Y * next.X;
		previous = current; //Note that "previous" is only updated if we don't remove the vertex.
		new_path.push_back(current);
	}
	//printf("inside PolygonRef simplication and is at line 144 \n.");
	//For the last/first vertex, we didn't check the connection that closes the polygon yet. Add the first vertex back if this connection is too long, or remove it if it's too short.
	curaIrfan::PointIrfan check = curaIrfan::operator-(new_path.back(), new_path[0]);
	curaIrfan::PointIrfan check1 = curaIrfan::operator-(new_path.back(), path->at(0));
	curaIrfan::PointIrfan check2= curaIrfan::operator-(new_path[0], path->at(0));


	if (!new_path.empty() && curaIrfan::vSize2(check) > smallest_line_segment_squared
		&& curaIrfan::vSize2(check1) >= smallest_line_segment_squared
		&& curaIrfan::vSize2(check2) >= smallest_line_segment_squared)
	{
		new_path.push_back(path->at(0));
	}
	curaIrfan::PointIrfan check4 = curaIrfan::operator-(new_path.back(),new_path[new_path.size() - 2]);
	if (new_path.size() > 2 && (curaIrfan::vSize2(check) < smallest_line_segment_squared || curaIrfan::vSize2(check4) < smallest_line_segment_squared))
	{
		if (LinearAlg2D::getDist2FromLine(new_path.back(), new_path[new_path.size() - 2], new_path[0]) < allowed_error_distance_squared)
		{
			new_path.pop_back();
		}
	}
	if (new_path.size() > 2 && LinearAlg2D::getDist2FromLine(new_path[0], new_path.back(), new_path[1]) <= 25)
	{
		new_path.erase(new_path.begin());
	}

	*path = new_path;
}

std::vector<PolygonsPart> Polygons::splitIntoParts(bool unionAll) const
{
	std::vector<PolygonsPart> ret;
	ClipperLib::Clipper clipper(clipper_init);
	ClipperLib::PolyTree resultPolyTree;
	clipper.AddPaths(paths, ClipperLib::ptSubject, true);
	if (unionAll)
		clipper.Execute(ClipperLib::ctUnion, resultPolyTree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
	else
		clipper.Execute(ClipperLib::ctUnion, resultPolyTree);

	splitIntoParts_processPolyTreeNode(&resultPolyTree, ret);
	return ret;
}

void Polygons::splitIntoParts_processPolyTreeNode(ClipperLib::PolyNode* node, std::vector<PolygonsPart>& ret) const
{
	for (int n = 0; n < node->ChildCount(); n++)
	{
		
		ClipperLib::PolyNode* child = node->Childs[n];
		
		PolygonsPart part;
		part.add(child->Contour);
		
		for (int i = 0; i < child->ChildCount(); i++)
		{
			part.add(child->Childs[i]->Contour);
			splitIntoParts_processPolyTreeNode(child->Childs[i], ret);
		}
		ret.push_back(part);
	}
}

unsigned int Polygons::pointCount() const
{
	unsigned int count = 0;
	for (const ClipperLib::Path& path : paths)
	{
		count += path.size();
	}
	return count;
}

Polygons Polygons::toPolygons(ClipperLib::PolyTree& poly_tree)
{
	Polygons ret;
	ret.addPolyTreeNodeRecursive(poly_tree);
	return ret;
}

Polygons Polygons::intersectionPolyLines(const Polygons& polylines) const
{
	ClipperLib::PolyTree result;
	ClipperLib::Clipper clipper(clipper_init);
	clipper.AddPaths(polylines.paths, ClipperLib::ptSubject, false);
	clipper.AddPaths(paths, ClipperLib::ptClip, true);
	clipper.Execute(ClipperLib::ctIntersection, result);
	Polygons ret;
	ret.addPolyTreeNodeRecursive(result);
	return ret;
}

bool Polygons::inside(curaIrfan::PointIrfan p, bool border_result) const
{
	int poly_count_inside = 0;
	for (const ClipperLib::Path& poly : *this)
	{
		const int is_inside_this_poly = ClipperLib::PointInPolygon(p, poly);
		if (is_inside_this_poly == -1)
		{
			return border_result;
		}
		poly_count_inside += is_inside_this_poly;
	}
	return (poly_count_inside % 2) == 1;
}

bool Polygons::insideOld(curaIrfan::PointIrfan p, bool border_result) const
{
	const Polygons& thiss = *this;
	if (size() < 1)
	{
		return false;
	}

	int crossings = 0;
	for (const ClipperLib::Path& poly : thiss)
	{
		curaIrfan::PointIrfan p0 = poly.back();
		for (const curaIrfan::PointIrfan& p1 : poly)
		{
			short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
			if (comp == 1)
			{
				crossings++;
			}
			else if (comp == 0)
			{
				return border_result;
			}
			p0 = p1;
		}
	}
	return (crossings % 2) == 1;
}

unsigned int Polygons::findInside(curaIrfan::PointIrfan p, bool border_result)
{
	Polygons& thiss = *this;
	if (size() < 1)
	{
		return false;
	}

	// NOTE: Keep these vectors fixed-size, they replace an (non-standard, sized at runtime) arrays.
	std::vector<int64_t> min_x(size(), std::numeric_limits<int64_t>::max());
	std::vector<int64_t> crossings(size());

	for (unsigned int poly_idx = 0; poly_idx < size(); poly_idx++)
	{
		PolygonRef poly = thiss[poly_idx];
		curaIrfan::PointIrfan p0 = poly.back();
		for (curaIrfan::PointIrfan& p1 : poly)
		{
			short comp = LinearAlg2D::pointLiesOnTheRightOfLine(p, p0, p1);
			if (comp == 1)
			{
				crossings[poly_idx]++;
				int64_t x;
				if (p1.Y == p0.Y)
				{
					x = p0.X;
				}
				else
				{
					x = p0.X + (p1.X - p0.X) * (p.Y - p0.Y) / (p1.Y - p0.Y);
				}
				if (x < min_x[poly_idx])
				{
					min_x[poly_idx] = x;
				}
			}
			else if (border_result && comp == 0)
			{
				return poly_idx;
			}
			p0 = p1;
		}
	}

	int64_t min_x_uneven = std::numeric_limits<int64_t>::max();
	unsigned int ret = NO_INDEX;
	unsigned int n_unevens = 0;
	for (unsigned int array_idx = 0; array_idx < size(); array_idx++)
	{
		if (crossings[array_idx] % 2 == 1)
		{
			n_unevens++;
			if (min_x[array_idx] < min_x_uneven)
			{
				min_x_uneven = min_x[array_idx];
				ret = array_idx;
			}
		}
	}
	if (n_unevens % 2 == 0) { ret = NO_INDEX; }
	return ret;
}

void ConstPolygonRef::smooth(int remove_length, PolygonRef result) const
{
	// a typical zigzag with the middle part to be removed by removing (1) :
	//
	//               3
	//               ^
	//               |
	//               |
	// inside        |     outside
	//          1--->2
	//          ^
	//          |
	//          |
	//          |
	//          0
	const ConstPolygonRef& thiss = *path;
	ClipperLib::Path* poly = result.path;
	if (size() > 0)
	{
		poly->push_back(thiss[0]);
	}
	auto is_zigzag = [remove_length](const int64_t v02_size, const int64_t v12_size, const int64_t v13_size, const int64_t dot1, const int64_t dot2)
	{
		if (v12_size > remove_length)
		{ // v12 or v13 is too long
			return false;
		}
		const bool p1_is_left_of_v02 = dot1 < 0;
		if (!p1_is_left_of_v02)
		{ // removing p1 wouldn't smooth outward
			return false;
		}
		const bool p2_is_left_of_v13 = dot2 > 0;
		if (p2_is_left_of_v13)
		{ // l0123 doesn't constitute a zigzag ''|,,
			return false;
		}
		if (-dot1 <= v02_size * v12_size / 2)
		{ // angle at p1 isn't sharp enough
			return false;
		}
		if (-dot2 <= v13_size * v12_size / 2)
		{ // angle at p2 isn't sharp enough
			return false;
		}
		return true;
	};
	curaIrfan::PointIrfan v02 = curaIrfan::operator-(thiss[2], thiss[0]);
	curaIrfan::PointIrfan v02T = curaIrfan::turn90CCW(v02);
	int64_t v02_size = curaIrfan::vSize(v02);
	bool force_push = false;
	for (unsigned int poly_idx = 1; poly_idx < size(); poly_idx++)
	{
		const curaIrfan::PointIrfan& p1 = thiss[poly_idx];
		const curaIrfan::PointIrfan& p2 = thiss[(poly_idx + 1) % size()];
		const curaIrfan::PointIrfan& p3 = thiss[(poly_idx + 2) % size()];
		// v02 computed in last iteration
		// v02_size as well
		const curaIrfan::PointIrfan v12 = curaIrfan::operator-(p2,p1);
		const int64_t v12_size = curaIrfan::vSize(v12);
		const curaIrfan::PointIrfan v13 = curaIrfan::operator-(p3, p1);
		const int64_t v13_size = curaIrfan::vSize(v13);

		// v02T computed in last iteration
		const int64_t dot1 = curaIrfan::dot(v02T, v12);
		const curaIrfan::PointIrfan v13T = curaIrfan::turn90CCW(v13);
		const int64_t dot2 = curaIrfan::dot(v13T, v12);
		bool push_point = force_push || !is_zigzag(v02_size, v12_size, v13_size, dot1, dot2);
		force_push = false;
		if (push_point)
		{
			poly->push_back(p1);
		}
		else
		{
			// do not add the current one to the result
			force_push = true; // ensure the next point is added; it cannot also be a zigzag
		}
		v02T = v13T;
		v02 = v13;
		v02_size = v13_size;
	}
}

Polygons Polygons::smooth(int remove_length) const
{
	Polygons ret;
	for (unsigned int p = 0; p < size(); p++)
	{
		ConstPolygonRef poly(paths[p]);
		if (poly.size() < 3)
		{
			continue;
		}
		if (poly.size() == 3)
		{
			ret.add(poly);
			continue;
		}
		poly.smooth(remove_length, ret.newPoly());
		PolygonRef back = ret.back();
		if (back.size() < 3)
		{
			back.path->resize(back.path->size() - 1);
		}
	}
	return ret;
}

void ConstPolygonRef::smooth2(int remove_length, PolygonRef result) const
{
	const ConstPolygonRef& thiss = *this;
	ClipperLib::Path* poly = result.path;
	if (thiss.size() > 0)
	{
		poly->push_back(thiss[0]);
	}
	for (unsigned int poly_idx = 1; poly_idx < thiss.size(); poly_idx++)
	{
		const curaIrfan::PointIrfan& last = thiss[poly_idx - 1];
		const curaIrfan::PointIrfan& now = thiss[poly_idx];
		const curaIrfan::PointIrfan& next = thiss[(poly_idx + 1) % thiss.size()];
		
		if (curaIrfan::shorterThen(curaIrfan::operator-(last, now), remove_length) && curaIrfan::shorterThen(curaIrfan::operator-(now, next), remove_length))
		{
			poly_idx++; // skip the next line piece (dont escalate the removal of edges)
			if (poly_idx < thiss.size())
			{
				poly->push_back(thiss[poly_idx]);
			}
		}
		else
		{
			poly->push_back(thiss[poly_idx]);
		}
	}
}

Polygons Polygons::smooth2(int remove_length, int min_area) const
{
	Polygons ret;
	for (unsigned int p = 0; p < size(); p++)
	{
		ConstPolygonRef poly(paths[p]);
		if (poly.size() == 0)
		{
			continue;
		}
		if (poly.area() < min_area || poly.size() <= 5) // when optimally removing, a poly with 5 pieces results in a triangle. Smaller polys dont have area!
		{
			ret.add(poly);
			continue;
		}
		if (poly.size() < 4)
		{
			ret.add(poly);
		}
		else
		{
			poly.smooth2(remove_length, ret.newPoly());
		}
	}
	return ret;
}

/*
void ConstPolygonRef::smooth_outward(const double min_angle, int shortcut_length, PolygonRef result) const
{
	// example of smoothed out corner:
	//
	//               6
	//               ^
	//               |
	// inside        |     outside
	//         2>3>4>5
	//         ^    /                   .
	//         |   /                    .
	//         1  /                     .
	//         ^ /                      .
	//         |/                       .
	//         |
	//         |
	//         0

	int shortcut_length2 = shortcut_length * shortcut_length;
	float cos_min_angle = cos(min_angle / 180 * M_PI);

	//ListPolygon poly;
	//ListPolyIt::convertPolygonToList(*this, poly);

	{ // remove duplicate vertices
		//ListPolyIt p1_it(poly, poly.begin());
		do
		{
			//ListPolyIt next = p1_it.next();
			if (curaIrfan::vSize2(curaIrfan::operator-(p1_it.p(), next.p()) < 10 * 10)
			{
				p1_it.remove();
			}
			p1_it = next;
		} while (p1_it != ListPolyIt(poly, poly.begin()));
	}

	ListPolyIt p1_it(poly, poly.begin());
	do
	{
		const Point p1 = p1_it.p();
		ListPolyIt p0_it = p1_it.prev();
		ListPolyIt p2_it = p1_it.next();
		const Point p0 = p0_it.p();
		const Point p2 = p2_it.p();

		const Point v10 = p0 - p1;
		const Point v12 = p2 - p1;
		float cos_angle = INT2MM(INT2MM(dot(v10, v12))) / vSizeMM(v10) / vSizeMM(v12);
		bool is_left_angle = LinearAlg2D::pointIsLeftOfLine(p1, p0, p2) > 0;
		if (cos_angle > cos_min_angle && is_left_angle)
		{
			// angle is so sharp that it can be removed
			Point v02 = p2_it.p() - p0_it.p();
			if (vSize2(v02) >= shortcut_length2)
			{
				smooth_corner_simple(p0, p1, p2, p0_it, p1_it, p2_it, v10, v12, v02, shortcut_length, cos_angle);
			}
			else
			{
				bool remove_poly = smooth_corner_complex(p1, p0_it, p2_it, shortcut_length); // edits p0_it and p2_it!
				if (remove_poly)
				{
					// don't convert ListPolygon into result
					return;
				}
			}
			// update:
			p1_it = p2_it; // next point to consider for whether it's an internal corner
		}
		else
		{
			++p1_it;
		}
	} while (p1_it != ListPolyIt(poly, poly.begin()));

	ListPolyIt::convertListPolygonToPolygon(poly, result);
}

Polygons Polygons::smooth_outward(const double max_angle, int shortcut_length)
{
	Polygons ret;
	for (unsigned int p = 0; p < size(); p++)
	{
		PolygonRef poly(paths[p]);
		if (poly.size() < 3)
		{
			continue;
		}
		if (poly.size() == 3)
		{
			ret.add(poly);
			continue;
		}
		poly.smooth_outward(max_angle, shortcut_length, ret.newPoly());
		if (ret.back().size() < 3)
		{
			ret.paths.resize(ret.paths.size() - 1);
		}
	}
	return ret;
}	   */

Polygons Polygons::approxConvexHull(int extra_outset)
{
	constexpr int overshoot = 100000; //10cm (hard-coded value).

	Polygons convex_hull;
	//Perform the offset for each polygon one at a time.
	//This is necessary because the polygons may overlap, in which case the offset could end up in an infinite loop.
	//See http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
	for (const ClipperLib::Path path : paths)
	{
		Polygons offset_result;
		ClipperLib::ClipperOffset offsetter(1.2, 10.0);
		offsetter.AddPath(path, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
		offsetter.Execute(offset_result.paths, overshoot);
		convex_hull.add(offset_result);
	}
	return convex_hull.unionPolygons().offset(-overshoot + extra_outset, ClipperLib::jtRound);
}

Polygons Polygons::getOutsidePolygons() const
{
	Polygons ret;
	ClipperLib::Clipper clipper(clipper_init);
	ClipperLib::PolyTree poly_tree;
	constexpr bool paths_are_closed_polys = true;
	clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
	clipper.Execute(ClipperLib::ctUnion, poly_tree);

	for (int outer_poly_idx = 0; outer_poly_idx < poly_tree.ChildCount(); outer_poly_idx++)
	{
		ClipperLib::PolyNode* child = poly_tree.Childs[outer_poly_idx];
		ret.emplace_back(child->Contour);
	}
	return ret;
}

Polygons Polygons::removeEmptyHoles() const
{
	Polygons ret;
	ClipperLib::Clipper clipper(clipper_init);
	ClipperLib::PolyTree poly_tree;
	constexpr bool paths_are_closed_polys = true;
	clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
	clipper.Execute(ClipperLib::ctUnion, poly_tree);

	bool remove_holes = true;
	removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
	return ret;
}

Polygons Polygons::getEmptyHoles() const
{
	Polygons ret;
	ClipperLib::Clipper clipper(clipper_init);
	ClipperLib::PolyTree poly_tree;
	constexpr bool paths_are_closed_polys = true;
	clipper.AddPaths(paths, ClipperLib::ptSubject, paths_are_closed_polys);
	clipper.Execute(ClipperLib::ctUnion, poly_tree);

	bool remove_holes = false;
	removeEmptyHoles_processPolyTreeNode(poly_tree, remove_holes, ret);
	return ret;
}

void Polygons::removeEmptyHoles_processPolyTreeNode(const ClipperLib::PolyNode& node, const bool remove_holes, Polygons& ret) const
{
	for (int outer_poly_idx = 0; outer_poly_idx < node.ChildCount(); outer_poly_idx++)
	{
		ClipperLib::PolyNode* child = node.Childs[outer_poly_idx];
		if (remove_holes)
		{
			ret.emplace_back(child->Contour);
		}
		for (int hole_node_idx = 0; hole_node_idx < child->ChildCount(); hole_node_idx++)
		{
			ClipperLib::PolyNode& hole_node = *child->Childs[hole_node_idx];
			if ((hole_node.ChildCount() > 0) == remove_holes)
			{
				ret.emplace_back(hole_node.Contour);
				removeEmptyHoles_processPolyTreeNode(hole_node, remove_holes, ret);
			}
		}
	}
}

coord_tIrfan Polygons::polyLineLength() const
{
	coord_tIrfan length = 0; curaIrfan::PointIrfan check;
	for (unsigned int poly_idx = 0; poly_idx < paths.size(); poly_idx++)
	{
		curaIrfan::PointIrfan p0 = paths[poly_idx][0];
		for (unsigned int point_idx = 1; point_idx < paths[poly_idx].size(); point_idx++)
		{
			curaIrfan::PointIrfan p1 = paths[poly_idx][point_idx];
			check = curaIrfan::operator-(p0, p1);
			length += curaIrfan::vSize(check);
			p0 = p1;
		}
	}
	return length;
}

void Polygons::addPolyTreeNodeRecursive(const ClipperLib::PolyNode& node)
{
	for (int outer_poly_idx = 0; outer_poly_idx < node.ChildCount(); outer_poly_idx++)
	{
		ClipperLib::PolyNode* child = node.Childs[outer_poly_idx];
		paths.push_back(child->Contour);
		addPolyTreeNodeRecursive(*child);
	}
}


Polygon Polygon::T_joint(Polygon poly_circle, std::vector<int>poly_circle_material, int shift)
{
	return poly_circle;
	std::vector<int> t_jointPointsX, t_jointPointsY, t_jointPointsmat;
	int mat5Counter = 0;
	bool opposite_Tjoint = true;
	int edge_length = 200;
	SlicerSegment segment;
	Polygon T_joint;

	for (int st_edge = 0; st_edge < poly_circle.size(); st_edge++)
	{
		t_jointPointsX.push_back((poly_circle[st_edge].X));
		t_jointPointsY.push_back((poly_circle[st_edge].Y));
		t_jointPointsmat.push_back(poly_circle_material[st_edge]);
		if (poly_circle_material[st_edge] == 5)
		{
			mat5Counter++;
		}
	}
	auto it = find(poly_circle_material.begin(), poly_circle_material.end(), 5);
	int count_interface_mat = count(poly_circle_material.begin(), poly_circle_material.end(), 5);	//interface mat id=5
	int id;
	if (it != poly_circle_material.end())
	{
		id = distance(poly_circle_material.begin(), it);
	}
	else {
		std::cout << "No Interface of materials occurs in the gemoetry";
		return poly_circle;
	}
	if (shift != 0)
		opposite_Tjoint = false;

	for (int st_edge_tmp = id + (24 + shift); st_edge_tmp < (id + (count_interface_mat - 1)); st_edge_tmp += 80)
	{

		float nx, ny, length;

		nx = (t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
		ny = (t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);
		if (opposite_Tjoint)
		{

			nx = -(t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
			ny = -(t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);
			length = edge_length * 6;

		}
		else if (!opposite_Tjoint)
		{

			nx = (t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
			ny = (t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);
			length = edge_length*0.75;
		}
		float nMod = std::sqrt(nx*nx + ny * ny);
		float nx1 = nx / nMod;
		float ny1 = ny / nMod;

		t_jointPointsX.insert(t_jointPointsX.begin() + (st_edge_tmp + 1), (t_jointPointsX[st_edge_tmp] + (nx1 * length)));
		t_jointPointsY.insert(t_jointPointsY.begin() + (st_edge_tmp + 1), (t_jointPointsY[st_edge_tmp] + (ny1 * length)));

		for (int T_st = 0; T_st < 5; T_st++) //7 is the last nnumber of T joint Point
		{
			nx = -(t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
			ny = -(t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);

			if (opposite_Tjoint)
			{
				nx = (t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
				ny = (t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);


			}
			else
			{
				nx = -(t_jointPointsY[st_edge_tmp] - t_jointPointsY[st_edge_tmp + 1]);
				ny = -(t_jointPointsX[st_edge_tmp + 1] - t_jointPointsX[st_edge_tmp]);


			}

			nMod = std::sqrt(nx*nx + ny * ny);
			if (T_st == 0)
			{
				nx = -(nx);
				ny = -(ny);
			}
			nx1 = nx / nMod;
			ny1 = ny / nMod;
			length = edge_length;
			if ((T_st == 1) || (T_st == 3))
			{
				if (opposite_Tjoint)
				{
					length = edge_length * 2;
				}
				else if (!opposite_Tjoint)
				{
					length = edge_length * 7;
				}
			}

			else if ((T_st == 0) || (T_st == 4))
			{
				if (opposite_Tjoint)
				{
					length = (edge_length * 2);
				}
				else
				{
					length = (edge_length * 3);
				}

			}

			else if (T_st == 2)
			{
				if (opposite_Tjoint)
				{
					length = (edge_length * 5);
				}
				else
				{
					length = (edge_length * 10);
				}

				//length = 3000;
			}


			if (T_st <= 4)
			{
				t_jointPointsX.insert(t_jointPointsX.begin() + (st_edge_tmp + 2), ((t_jointPointsX[st_edge_tmp + 1]) + (nx1 * length)));
				t_jointPointsY.insert(t_jointPointsY.begin() + (st_edge_tmp + 2), ((t_jointPointsY[st_edge_tmp + 1]) + (ny1 * length)));
			}

			st_edge_tmp++;


		}
		opposite_Tjoint = !opposite_Tjoint;


	}

	for (int T_chk = 0; T_chk < t_jointPointsX.size() - 1; T_chk++)
	{

		if (T_chk == 0)
		{
			segment.start.X = t_jointPointsX[T_chk];
			segment.start.Y = t_jointPointsY[T_chk];
			T_joint.add(segment.start);
			segment.end.X = t_jointPointsX[T_chk + 1];
			segment.end.Y = t_jointPointsY[T_chk + 1];
			T_joint.add(segment.end);
		}
		else
		{
			segment.end.X = t_jointPointsX[T_chk + 1];
			segment.end.Y = t_jointPointsY[T_chk + 1];
			T_joint.add(segment.end);
		}

	}

	return T_joint;

}