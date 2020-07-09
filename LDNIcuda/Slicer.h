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
		std::vector<SlicerSegment> segments_zigzag; //duplicate 
		std::unordered_map<int, int> face_idx_to_segment_idx; // topology
		std::vector<Polygons> infillpolygons;
		int z = -1;
		Polygons polygons;
		Polygons polygons_Zigzag;//these polygos are for 
		Polygons openPolylines;
		bool inteference_zigzag;//bool for the zig-zag boundary at the interference
			
	};

	class Slicer
	{
		friend class SlicerLayer;
	public:
		std::vector<SlicerLayer> layers;
		const Mesh* mesh = nullptr;
		Slicer(SliceDataStorage& storage, GLKObList& mesh_list,ContourMesh& c_mesh, const coord_tIrfan thickness, int slice_layer_count, bool use_variable_layer_heights, std::vector<int>& meshin_layer);
		
	};

//namespace cura

#endif//SLICER_H
