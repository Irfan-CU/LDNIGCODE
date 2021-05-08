//Copyright (c) 2019 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include <stdio.h>
//#include <windows.h>
#include <vector>
#include <algorithm>
#include "PMBody.h"
#include "Slicer.h"
#include "EnumSettings.h"
#include "Polygon.h"
#include "SliceDataStorage.h"

using namespace std;

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

	Slicer::Slicer(SliceDataStorage& storage, GLKObList &mesh_list, ContourMesh& c_mesh, const coord_tIrfan thickness, int slice_layer_count, bool use_variable_layer_heights)
	:mesh(mesh)
	{
		
		SlicingTolerance slicing_tolerance = SlicingTolerance::MIDDLE; 
		GLKPOSITION Pos_1 = NULL;
		GLKPOSITION Pos_2 = NULL;
		VSAMesh* mesh1;
		VSAEdge* edge1;
		ContourMesh cmesh;

		double xx, aa, bb, cc, yy, zz;
		assert(slice_layer_count > 0);
	   	layers.resize(slice_layer_count);
		
		const coord_tIrfan initial_layer_thickness = storage.getlayer_thickness();
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
		
		unsigned int layer_nr = 0; 
		
		SlicerSegment segment; SlicerLayer	slicerlayer; int mesh_count = 0;
		
		

		int mesh_matCopy = 1;

			for (Pos_1 = mesh_list.GetHeadPosition(); Pos_1 != NULL; )
			{
				int edge = 1;  int i = 1;	int n = 0;
				int edg_mat_copy;
				int edg_mat;
				bool mat_change = true;

				mesh1 = (VSAMesh *)(mesh_list.GetNext(Pos_1));
				char * ext_material = mesh1->getRegionMaterial();
				int mesh_mat = mesh1->getmeshMaterial();
				int layer_nr = mesh1->getmeshLayer();
			   	int meshID = mesh1->getMeshID();
				
				
				Polygon T_Joint;
				// These Polygons are just used for intersection circles case
				double xx1, zz1, aa1, cc1;
				bool inter_circle = false;  //boolean used to add the intersection to form polygon
				


				for (Pos_2 = mesh1->GetVSAEdgeList().GetHeadPosition(); Pos_2 != NULL; )
				{


					edge1 = (VSAEdge *)(mesh1->GetVSAEdgeList().GetNext(Pos_2));
					edge1->GetStartPoint()->GetCoord3D(xx, yy, zz);
					edge1->GetEndPoint()->GetCoord3D(aa, bb, cc);
					edg_mat = edge1->GetEdgeMaterial();

					int scale = storage.get_scale();
					const coord_tIrfan xx_int = MM2INT(xx * scale);
					const coord_tIrfan aa_int = MM2INT(aa * scale);
					const coord_tIrfan zz_int = MM2INT(zz * scale);
					const coord_tIrfan cc_int = MM2INT(cc * scale);
					//printf("the yy is %f and edg mat is %d \n", yy,edg_mat);


					segment.start.X = xx_int;
					segment.start.Y = zz_int;
					segment.end.X = aa_int;
					segment.end.Y = cc_int;
					layers[layer_nr].segments.push_back(segment);
					layers[layer_nr].segments_zigzag.push_back(segment);
					segment.addedToPolygon = false;

					// Polygon A is made up for the material 1 and material 5
					
					

					if (edge == 1)
					{
						LDMIpolyCircle.add(segment.end);
						LDMIpolyCircle.add(segment.start);
					}
					else
					{
						LDMIpolyCircle.add(segment.start);
					}

					edge++;

				}
				//Hard coded Memoery allocation here to LDMI Polygons here take care
				// Memeory for setPolygonExtruders as well
				layers[layer_nr].LDMIpolygons[mesh1->getMeshID()].add(LDMIpolyCircle);
				layers[layer_nr].LDMIpolygons[mesh1->getMeshID()].setPolygonExtruders(ext_material);
				char* newMaterials = layers[layer_nr].LDMIpolygons[mesh1->getMeshID()].getPolygonExtruders();
				layers[layer_nr].LDMIpolygons[mesh1->getMeshID()].setPolygonMatID(mesh_matCopy);
				LDMIpolyCircle.clear();
				mesh_matCopy = mesh_mat;

				

			}
		
						  
		//int edge_length = 200;

			//------------------Program for tthe T-Joint--------------//
			//Polygon T_Joint_B;
			//int user_choice;

			//int shift = 22;
			//Polygon T_Joint_A,poly;
			//if (!poly_circle_A.empty())
			//{
			//	shift = 0;
//				T_Joint_A = poly.T_joint(poly_circle_A, poly_circle_A_material,shift);
			//   	//layers[layer_nr].polygons_Circle_interA.add(poly_circle_A);
			//	//T_Joint_A.clear();
			//	//layers[layer_nr].polygons_Circle_interA.add(poly_circle_A);
			//	//layers[layer_nr].polygons_Circle_interA.add(T_Joint);
			//	//layers[layer_nr].polygons_Circle_interA.setId(mesh_count);//mesh count or polygon id actually doent matter its here for the future use and scaling the code
			//	//layers[layer_nr].polygons_Circle_interA.polygons_matid.push_back(3);
			//}

			//Polygon T_Joint_inter;
			

			//if (!poly_circle_A.empty())
			//mesh_count++;
			//if (edg_mat == 10)
		//	{
				//double st_x, st_y, ed_x, ed_y;
				//for (int seg_st = 0; seg_st < poly_circle_intersection.size()-1; seg_st++)
				//{
				//	segment.end = poly_circle_intersection[seg_st];
				//	segment.start = poly_circle_intersection[seg_st+1];
				//	ed_x = INT2MM(segment.end.X / 100);
				//	ed_y = INT2MM(segment.end.Y / 100);
				//	st_x = INT2MM(segment.start.X / 100);
				//	st_y = INT2MM(segment.start.Y / 100);
				//	
				//	if ((abs((st_x) - (ed_x)) >0.007) || (abs((st_y) - (ed_y)) > 0.007)) //0.005*100 = 0.5 so 0.7
				//	{
				//		printf("the segments are %f %f %f %f and the layer is %d \n", st_x, ed_x, st_y, ed_y,layer_nr);
				//	}

				//	poly_circle_intersection.add(segment.end);
				//	poly_circle_intersection.add(segment.start);
				//}

				//std::vector<int>t_jointPointsXinter;
				//std::vector<int>t_jointPointsYinter;

				//if ((layer_nr <= 5))
				/*
				{

					for (int st_edge = 0; st_edge < poly_circle_intersection.size(); st_edge+=2)
					{

						t_jointPointsXinter.push_back((poly_circle_intersection[st_edge].X));
						t_jointPointsYinter.push_back((poly_circle_intersection[st_edge].Y));
					}
					int T_jointSegSize = 0;

					for (int st_edge = 0,st_edge_tmp = 0; st_edge < t_jointPointsXinter.size(); st_edge ++, st_edge_tmp++)
					{

							//Calculating the normal of the stick for making a T interlock

							if ((st_edge_tmp == 4) || (st_edge_tmp == 12) ||  (st_edge_tmp == 20) || (st_edge_tmp == 30) || (st_edge_tmp == 38) ||  (st_edge_tmp == 46))   //6,13+6,26+6,39+6 6 is offset whereas 0
							{

								//printf("the inter points are %f %f \n", INT2MM(t_jointPointsXinter[st_edge_tmp] / 100), INT2MM(t_jointPointsYinter[st_edge_tmp] / 100));
								float nx, ny;
								if ((st_edge_tmp == 12) || (st_edge_tmp == 38))
								{
									nx = -(t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
									ny = -(t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);

								}
								else if ((st_edge_tmp == 4) || (st_edge_tmp == 20) || (st_edge_tmp == 30) || (st_edge_tmp == 46))
								{
									nx = (t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
									ny = (t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);
								}
								float nMod = std::sqrt(nx*nx + ny * ny);
								float nx1 = nx / nMod;
								float ny1 = ny / nMod;
								float length = edge_length * 2;;
								//t_jointPointsXinter.push_back((poly_circle_intersection[st_edge].X));
								//t_jointPointsYinter.push_back((poly_circle_intersection[st_edge].Y));

								//printf("the nmod is %f and nxq or ny1 is %F %f and length is %f \n", nMod, nx1, ny1, length);
								t_jointPointsXinter.insert(t_jointPointsXinter.begin() + (st_edge + 1),(t_jointPointsXinter[st_edge] + (nx1 * length)));
								t_jointPointsYinter.insert(t_jointPointsYinter.begin() + (st_edge + 1), (t_jointPointsYinter[st_edge] + (ny1 * length)));
								for (int T_st = 0; T_st < 5; T_st++) //7 is the last nnumber of T joint Point
								{

									if ((st_edge_tmp == 12) || (st_edge_tmp == 38))
									{
										nx = (t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
										ny = (t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);

									}
									else if ((st_edge_tmp == 4) || (st_edge_tmp == 20) || (st_edge_tmp == 30) || (st_edge_tmp == 46))
									{
										nx = -(t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
										ny = -(t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);
									}
									//nx = (t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
									//ny = (t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);
									nMod = std::sqrt(nx*nx + ny * ny);
									if (T_st == 0)
									{
										if ((st_edge_tmp == 12) || (st_edge_tmp == 38))
										{
											nx = -(t_jointPointsYinter[st_edge] - t_jointPointsYinter[st_edge + 1]);
											ny = -(t_jointPointsXinter[st_edge + 1] - t_jointPointsXinter[st_edge]);

										}
										else if ((st_edge_tmp == 4) || (st_edge_tmp == 20) || (st_edge_tmp == 30) || (st_edge_tmp == 46))
										{
											nx = -(nx);
											ny = -(ny);
										}
									}
									nx1 = nx / nMod;
									ny1 = ny / nMod;
									length = edge_length * 2;
									if (T_st == 2)
									{
										length = edge_length * 6;
									}
									//printf("the nmod is %f and nxq or ny1 is %F %f and length is %f \n", nMod, nx1, ny1, length);
									if (T_st < 4)
									{
										t_jointPointsXinter.insert(t_jointPointsXinter.begin() + (st_edge + 2), ((t_jointPointsXinter[st_edge + 1]) + (nx1 * length)));
										t_jointPointsYinter.insert(t_jointPointsYinter.begin() + (st_edge + 2), ((t_jointPointsYinter[st_edge + 1]) + (ny1 * length)));
									}
									else if (T_st == 4)
									{
										t_jointPointsXinter[st_edge + 2]= ((t_jointPointsXinter[st_edge + 1]) + (nx1 * length));
										t_jointPointsYinter[st_edge + 2]= ((t_jointPointsYinter[st_edge + 1]) + (ny1 * length));
									}
									st_edge++;

								}
								//break;
								T_jointSegSize += t_jointPointsXinter.size() - poly_circle_intersection.size() / 2;
							}

							//printf("The segment points are %f %f \n", INT2MM(t_jointPointsXinter[st_edge] / 100), INT2MM(t_jointPointsYinter[st_edge] / 100));

							//printf("the new points are %f %f and mod is %f with new nx and ny is %f %f \n", nx, ny, nMod, nx1, ny1);
							//printf("The segment points are %f %f and %f %f \n", INT2MM(poly_circle_intersection[st_edge].X / 100), INT2MM(poly_circle_intersection[st_edge].Y / 100), INT2MM(poly_circle_intersection[st_edge + 1].X / 100), INT2MM(poly_circle_intersection[st_edge + 1].Y / 100));

							//break;





					}







					for (int T_chk = 0; T_chk < t_jointPointsXinter.size() - 1; T_chk++)
					{
						//printf("The segment points are %f %f \n", INT2MM(t_jointPointsXinter[T_chk] / 100), INT2MM(t_jointPointsYinter[T_chk] / 100));
						if (T_chk == 0)
						{
							segment.start.X = t_jointPointsXinter[T_chk];
							segment.start.Y = t_jointPointsYinter[T_chk];
							T_Joint_inter.add(segment.start);
							segment.end.X = t_jointPointsXinter[T_chk + 1];
							segment.end.Y = t_jointPointsYinter[T_chk + 1];
							T_Joint_inter.add(segment.end);
						}
						else
						{
							segment.end.X = t_jointPointsXinter[T_chk + 1];
							segment.end.Y = t_jointPointsYinter[T_chk + 1];
							T_Joint_inter.add(segment.end);
						}


						//printf("The segment points are %f %f and %f %f and the material is %d and %d \n", INT2MM(poly_circle_A[st_edge].X / 100), INT2MM(poly_circle_A[st_edge].Y / 100), INT2MM(poly_circle_A[st_edge + 1].X / 100), INT2MM(poly_circle_A[st_edge + 1].Y / 100), poly_circle_A_material[st_edge], poly_circle_A_material[st_edge + 1]);
					}

				}

				*/

				//layers[layer_nr].polygons_Circle_inter.add(poly_circle_intersection);
				//layers[layer_nr].polygons_Circle_inter.add(T_Joint_inter);
				//poly_circle_intersection.clear();
				//std::cout<<"the polygons in the layer are %d \n",layers[layer_nr].
				//layer_nr++;
				//mesh_count = 0;
				//layers[layer_nr].polygons_Circle_inter.add(poly_circle_intersection);
				//if (layer_nr == 3) break;




	//		}



		//}
		
		
	}
	
	
	


