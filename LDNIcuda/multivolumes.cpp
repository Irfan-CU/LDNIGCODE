//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include "multiVolumes.h"
#include "slicer.h"
#include "EnumSettings.h"


void carveMultipleVolumes(std::vector<Slicer*> &volumes)
{
	//Go trough all the volumes, and remove the previous volume outlines from our own outline, so we never have overlapped areas.
	const bool alternate_carve_order = true;// Application::getInstance().current_slice->scene.current_mesh_group->settings.get<bool>("alternate_carve_order");
	printf("the volume size is %d \n", volumes.size());
	for (unsigned int volume_1_idx = 1; volume_1_idx < volumes.size(); volume_1_idx++)
	{
		printf("Inside crave multiple volume loop 1 \n");
		bool infill_mesh =false; // can be changed by used input thats introducing them here
		bool anti_overhang_mesh = false;
		bool support_mesh = false;
		ESurfaceMode magic_mesh_surface_mode= ESurfaceMode::NORMAL;
		
		Slicer& volume_1 = *volumes[volume_1_idx];
		if ((infill_mesh)
			||(anti_overhang_mesh)
			||(support_mesh)
			||(magic_mesh_surface_mode == ESurfaceMode::SURFACE)
			)
		{
			continue;
		}
		for (unsigned int volume_2_idx = 0; volume_2_idx < volume_1_idx; volume_2_idx++)
		{
			printf("Inside crave multiple volume loop 2 \n");
			Slicer& volume_2 = *volumes[volume_2_idx];
			if ((infill_mesh)
				|| (anti_overhang_mesh)
				|| (support_mesh)
				|| (magic_mesh_surface_mode == ESurfaceMode::SURFACE)
				)
			{
				continue;
			}
			/*
			if (!volume_1.mesh->getAABB().hit(volume_2.mesh->getAABB()))
			{
				continue;
			}
			for (unsigned int layerNr = 0; layerNr < volume_1.layers.size(); layerNr++)
			{
				SlicerLayer& layer1 = volume_1.layers[layerNr];
				SlicerLayer& layer2 = volume_2.layers[layerNr];
				if (alternate_carve_order && layerNr % 2 == 0)
				{
					layer2.polygons = layer2.polygons.difference(layer1.polygons);
				}
				else
				{
					layer1.polygons = layer1.polygons.difference(layer2.polygons);
				}
			}
			*/
		}
	}
}

void generateMultipleVolumesOverlap(std::vector<Slicer*> &volumes)
	{
		if (volumes.size() < 2)
		{
			//printf("the volumes size is %d \n", volumes.size());
			return;
			
		}
		
		int offset_to_merge_other_merged_volumes = 20;
		for (Slicer* volume : volumes)
		{
			coord_tIrfan overlap = MM2INT(0.000);// volume->mesh->settings.get<coord_tIrfan>("multiple_mesh_overlap");
			if (overlap == 0)
			{
				continue;
			}
			// expand to account for the case where two models and their bounding boxes are adjacent along the X or Y-direction
			//printf("the layers.size() is %d \n", volume->layers.size());
			for (unsigned int layer_nr = 0; layer_nr < volume->layers.size(); layer_nr++)
			{
				Polygons all_other_volumes;
				for (Slicer* other_volume : volumes)
				{
					if (other_volume == volume)
					{
						continue;
					}
					printf("*****Warning the code is at wrong place somehow at ine 40 in multivolumes.cpp \n");
					SlicerLayer& other_volume_layer = other_volume->layers[layer_nr];
					all_other_volumes = all_other_volumes.unionPolygons(other_volume_layer.polygons.offset(offset_to_merge_other_merged_volumes));
				}

				SlicerLayer& volume_layer = volume->layers[layer_nr];
				volume_layer.polygons = volume_layer.polygons.unionPolygons(all_other_volumes.intersection(volume_layer.polygons.offset(overlap / 2)));
			}
		}
	}



