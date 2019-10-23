//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef SLICER_H
#define SLICER_H

#include <queue>
#include <unordered_map>
#include "Polygon.h"



    class Mesh;
	class VSAMesh;
	class GLKObList;
	class ContourMesh;
	class SliceDataStorage;
	
	

	class SlicerSegment
	{
	public:
		curaIrfan::PointIrfan start, end;
		int faceIndex = -1;
		// The index of the other face connected via the edge that created end
		int endOtherFaceIdx = -1;
		// If end corresponds to a vertex of the mesh, then this is populated
		// with the vertex that it ended on.
		
		bool addedToPolygon = false;
		int segmentidx;//segment id;
		
	};

	class ClosePolygonResult
	{   //The result of trying to find a point on a closed polygon line. This gives back the point index, the polygon index, and the point of the connection.
		//The line on which the point lays is between pointIdx-1 and pointIdx
	public:
		int polygonIdx = -1;
		size_t pointIdx = -1;
	};
	class GapCloserResult
	{
	public:
		coord_tIrfan len = -1;
		int polygonIdx = -1;
		size_t pointIdxA = -1;
		size_t pointIdxB = -1;
		bool AtoB = false;
	};

	class SlicerLayer
	{
	public:
		std::vector<SlicerSegment> segments;
		std::unordered_map<int, int> face_idx_to_segment_idx; // topology
		std::vector<Polygons> infillpolygons;
		int z = -1;
		Polygons polygons;
		Polygons openPolylines;
			
	};

	class Slicer
	{
		friend class SlicerLayer;
	public:
		std::vector<SlicerLayer> layers;
		const Mesh* mesh = nullptr;
		Slicer(SliceDataStorage& storage, GLKObList& mesh_list,ContourMesh& c_mesh, const coord_tIrfan thickness, int slice_layer_count, bool use_variable_layer_heights, std::vector<int>& meshin_layer);
		
		

		/*!
		 * \brief Linear interpolation between coordinates of a line.
		 *
		 * Get the Y of a point with X \p x in the line through (\p x0, \p y0) and
		 * (\p x1, \p y1).
		 * \param x The X coordinate of the point to find.
		 * \param x0 The X coordinate of the first end point of the line segment.
		 * \param x1 The X coordinate of the second end point of the line segment.
		 * \param y0 The Y coordinate of the first end point of the line segment.
		 * \param y1 The Y coordinate of the second end point of the line segment.
		 * \return The Y coordinate of the point to find.
		 */
	
	};

//namespace cura

#endif//SLICER_H
