//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include <stdio.h>
//#include <windows.h>
#include <algorithm>
#include "PMBody.h"
#include "Slicer.h"
#include "EnumSettings.h"
#include "Polygon.h"
#include "SliceDataStorage.h"


	int largest_neglected_gap_first_phase = MM2INT(0.01); //!< distance between two line segments regarded as connected
	int largest_neglected_gap_second_phase = MM2INT(0.02); //!< distance between two line segments regarded as connected
	int max_stitch1 = MM2INT(10.0); //!< maximal distance stitched between open polylines to form polygons


	/*
	void SlicerLayer::makeBasicPolygonLoops(Polygons& open_polylines)
	{
		for (unsigned int start_segment_idx = 0; start_segment_idx < segments.size(); start_segment_idx++)
		{
			if (!segments[start_segment_idx].addedToPolygon)
			{
				Polygong poly;
				poly.add(segments[start_segment_idx].start);

				for (int segment_idx = start_segment_idx; segment_idx != -1; )
				{
					SlicerSegment& segment = segments[segment_idx];
					poly.add(segment.end);
					segment.addedToPolygon = true;
					segment_idx = getNextSegmentIdx(segment, start_segment_idx);
					if (segment_idx == static_cast<int>(start_segment_idx))
					{ // polyon is closed
						polygons.add(poly);
						return;
					}
				}
				// polygon couldn't be closed
				open_polylines.add(poly);
			}
		}
		//Clear the segmentList to save memory, it is no longer needed after this point.
		segments.clear();
	}

	void SlicerLayer::makeBasicPolygonLoop(Polygons& open_polylines, unsigned int start_segment_idx)
	{
	    
		Polygong poly;
		poly.add(segments[start_segment_idx].start);

		for (int segment_idx = start_segment_idx; segment_idx != -1; )
		{
			SlicerSegment& segment = segments[segment_idx];
			poly.add(segment.end);
			segment.addedToPolygon = true;
			segment_idx = getNextSegmentIdx(segment, start_segment_idx);
			if (segment_idx == static_cast<int>(start_segment_idx))
			{ // polyon is closed
				polygons.add(poly);
				return;
			}
		}
		// polygon couldn't be closed
		open_polylines.add(poly);
	}

	int SlicerLayer::tryFaceNextSegmentIdx(const SlicerSegment& segment, int face_idx, unsigned int start_segment_idx) const
	{
		decltype(face_idx_to_segment_idx.begin()) it;
		auto it_end = face_idx_to_segment_idx.end();
		it = face_idx_to_segment_idx.find(face_idx);
		if (it != it_end)
		{
			int segment_idx = (*it).second;
			curaIrfan::PointIrfan p1 = segments[segment_idx].start;
			curaIrfan::PointIrfan diff = segment.end;// -p1;
			if (curaIrfan::shorterThen(diff, largest_neglected_gap_first_phase))
			{
				if (segment_idx == static_cast<int>(start_segment_idx))
				{
					return start_segment_idx;
				}
				if (segments[segment_idx].addedToPolygon)
				{
					return -1;
				}
				return segment_idx;
			}
		}

		return -1;
	}

	int SlicerLayer::getNextSegmentIdx(const SlicerSegment& segment, unsigned int start_segment_idx)
	{
		
		int next_segment_idx = -1;

		bool segment_ended_at_edge = segment.endVertex == nullptr;
		if (segment_ended_at_edge)
		{
			int face_to_try = segment.endOtherFaceIdx;
			if (face_to_try == -1)
			{
				return -1;
			}
			return tryFaceNextSegmentIdx(segment, face_to_try, start_segment_idx);
		}
		else
		{
			// segment ended at vertex

			const std::vector<uint32_t> &faces_to_try = segment.endVertex->connected_faces;
			for (int face_to_try : faces_to_try)
			{
				int result_segment_idx =
					tryFaceNextSegmentIdx(segment, face_to_try, start_segment_idx);
				if (result_segment_idx == static_cast<int>(start_segment_idx))
				{
					return start_segment_idx;
				}
				else if (result_segment_idx != -1)
				{
					// not immediately returned since we might still encounter the start_segment_idx
					next_segment_idx = result_segment_idx;
				}
			}
		}

		return next_segment_idx;
	}

	
	void SlicerLayer::makePolygons()
	{
		Polygons open_polylines;
		
		makeBasicPolygonLoops(open_polylines);

		/*
		// TODO: (?) for mesh surface mode: connect open polygons. Maybe the above algorithm can create two open polygons which are actually connected when the starting segment is in the middle between the two open polygons.

		//Remove all the tiny polygons, or polygons that are not closed. As they do not contribute to the actual print.
		const coord_tIrfan snap_distance = mesh->settings.get<coord_t>("minimum_polygon_circumference");
		auto it = std::remove_if(polygons.begin(), polygons.end(), [snap_distance](PolygonRef poly) { return poly.shorterThan(snap_distance); });
		polygons.erase(it, polygons.end());

		//Finally optimize all the polygons. Every point removed saves time in the long run.
		const coord_tIrfan line_segment_resolution = mesh->settings.get<coord_t>("meshfix_maximum_resolution");
		const coord_tIrfan line_segment_deviation = mesh->settings.get<coord_t>("meshfix_maximum_deviation");
		polygons.simplify(line_segment_resolution, line_segment_deviation);

		polygons.removeDegenerateVerts(); // remove verts connected to overlapping line segments
		
	}
	*/

	Slicer::Slicer(SliceDataStorage& storage, GLKObList& mesh_list, ContourMesh& c_mesh, const coord_tIrfan thickness, int slice_layer_count, bool use_variable_layer_heights, std::vector<int>& meshin_layer)
	:mesh(mesh)
	{
		
		SlicingTolerance slicing_tolerance = SlicingTolerance::MIDDLE; 
		GLKPOSITION Pos_1=NULL;
		GLKPOSITION Pos_2 = NULL;
		VSAMesh* mesh1;
		VSAEdge* edge1;
		ContourMesh cmesh;
		double xx, aa, bb, cc, yy, zz;
		assert(slice_layer_count > 0);
	   	layers.resize(slice_layer_count);

		// set (and initialize compensation for) initial layer, depending on slicing mode
		const coord_tIrfan initial_layer_thickness = storage.getlayer_thickness();// MM2INT(0.1);// Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height_0");
		layers[0].z = std::max(0LL, initial_layer_thickness - thickness);
		coord_tIrfan adjusted_layer_offset = initial_layer_thickness;
		
		if (slicing_tolerance == SlicingTolerance::MIDDLE)
		{
			layers[0].z = initial_layer_thickness / 2;
			adjusted_layer_offset = initial_layer_thickness + (thickness / 2);
		}
		for (unsigned int layer_nr = 1; layer_nr < slice_layer_count; layer_nr++)
		{
		layers[layer_nr].z = adjusted_layer_offset + (thickness * (layer_nr - 1));
		}
		//printf("@ 190 of slicer.cpp\n");
		//loop over all mesh faces
		unsigned int layer_nr = 0; 
		
		SlicerSegment segment; SlicerLayer	slicerlayer; int mesh_count = 0;

		for (Pos_1 = mesh_list.GetHeadPosition(); Pos_1 != NULL; )
		{
		//	printf("layernr is %d \n", layer_nr);
			//assert(layer_nr < slice_layer_count);
			
				int edge = 1;  int i = 1;
				mesh1 = (VSAMesh *)(mesh_list.GetNext(Pos_1));
				Polygon poly;

				for (Pos_2 = mesh1->GetVSAEdgeList().GetHeadPosition(); Pos_2 != NULL; )
				{
					//printf("Yeah I am here on line 1933 of the code %d \n");
					edge1 = (VSAEdge *)(mesh1->GetVSAEdgeList().GetNext(Pos_2));
					//printf("Yeah I am here on line 1935 of the code %d \n", mesh_idx);
					edge1->GetStartPoint()->GetCoord3D(xx, yy, zz);
					edge1->GetEndPoint()->GetCoord3D(aa, bb, cc);
					int edge_id = edge1->GetIndexNo();// .GetIndexNo();
					segment.segmentidx = edge_id;
					

					const coord_tIrfan xx_int = MM2INT(xx * 10000);
					const coord_tIrfan aa_int = MM2INT(aa * 10000);
					const coord_tIrfan zz_int = MM2INT(zz * 10000);
					const coord_tIrfan cc_int = MM2INT(cc * 10000);

					curaIrfan::PointIrfan st(xx_int, zz_int);
					curaIrfan::PointIrfan ed(aa_int, cc_int);
					
					segment.start.X = xx_int;
					segment.start.Y = zz_int;
					segment.end.X = aa_int;
					segment.end.Y = cc_int;
					//printf("Yeah I am here on line 228 of the code %d \n");
					layers[layer_nr].segments.push_back(segment);
					//printf("Yeah I am here on line 230 of the code %d \n");
					segment.addedToPolygon = false;
					if (edge == 1)
					{
						poly.add(segment.end);
						poly.add(segment.start);   
					}
					else
					{
						poly.add(segment.start);
						poly.size();
						
					}
					edge++;
					
				}
				layers[layer_nr].polygons.add(poly);
				//printf("the polygons point count is %d \n", layers[layer_nr].polygons.pointCount());
				mesh_count++;
				if (meshin_layer[layer_nr] == mesh_count)
				{
					layer_nr++;
					//printf("layer no is %d \n and contours in this layer are %d \n", layer_nr, mesh_count);
					mesh_count = 0;	
				}
		}
			//printf("the polygons size inside is is %d \n", layers[layer_nr].polygons.size());
		
		std::vector<SlicerLayer>& layers_ref = layers; // force layers not to be copied into the threads

	   	int  layer_apply_initial_xy_offset = 0;
		if (layers.size() > 0 && layers[0].polygons.size() == 0)
		{
			layer_apply_initial_xy_offset = 1;
		}


		// Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
		for (int layer_nr = 0; layer_nr < static_cast<int>(layers_ref.size()); layer_nr++)
		{
			const coord_tIrfan xy_offset = 0;

			if (xy_offset != 0)
			{
				layers_ref[layer_nr].polygons = layers_ref[layer_nr].polygons.offset(xy_offset);
			}
		}

		
	}
