//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "layerPart.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "EnumSettings.h" //For ESurfaceMode.


/*
The layer-part creation step is the first step in creating actual useful data for 3D printing.
It takes the result of the Slice step, which is an unordered list of polygons, and makes groups of polygons,
each of these groups is called a "part", which sometimes are also known as "islands". These parts represent
isolated areas in the 2D layer with possible holes.

Creating "parts" is an important step, as all elements in a single part should be printed before going to another part.
And all every bit inside a single part can be printed without the nozzle leaving the boundary of this part.

It's also the first step that stores the result in the "data storage" so all other steps can access it.
*/


	void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer)
	{
		//storageLayer.openPolyLines = layer->openPolylines;
		std::vector<PolygonsPart> result;
		const bool union_layers = true;
		
		result = layer->polygons.splitIntoParts(union_layers);	  //polygons in slicer list
		
																  //printf("the id of the contour is %d and  result size is %d annd %d \n", layer->polygons.getpolygons_matid(), result.size(), layer->polygons.size());

		//printf("the size of the parts are %d and the point count in the part is %d \n",result.size(),result.);
		//printf("the polygons size is %d ", layer->polygons.size());
		for (unsigned int i = 0; i < result.size(); i++)
		{
			storageLayer.parts.emplace_back();
			
			storageLayer.mat_parts.emplace_back(layer->polygons.polygons_matid[i]);
			storageLayer.parts[i].part_mat = layer->polygons.polygons_matid[i];
			storageLayer.parts[i].outline = result[i];
			storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
		//	printf("the size of the parts %d outline is %d \n",i, storageLayer.parts[i].outline.pointCount());
		}
		
	}
	void createLayerParts(SliceDataStorage& storage, Slicer* slicer)
	{
		const auto total_layers = slicer->layers.size();
		//printf("Parts the layers size is %d \n", total_layers);	//141
		assert(storage.Layers.size() == total_layers);
#pragma omp parallel for default(none) shared(storage, slicer) schedule(dynamic)
		// Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
		for (int layer_nr = 0; layer_nr < static_cast<int>(total_layers); layer_nr++)
		{
			SliceLayer& layer_storage = storage.Layers[layer_nr];
			SlicerLayer& slice_layer = slicer->layers[layer_nr];
			
			createLayerWithParts(layer_storage, &slice_layer);
			//printf("the parts are genrated for layer %d and the parts size is %d \n", layer_nr, layer_storage.parts.size());
		}

		for (int layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
		{
			SliceLayer& layer_storage = storage.Layers[layer_nr];
			for (int i = 0; i < layer_storage.parts.size(); i++)
			//printf("the layer nr is %d and the parts are %d and the materials is %d \n", layer_nr, i, layer_storage.mat_parts[i]);
			ESurfaceMode magic_mesh_surface_mode = ESurfaceMode::NORMAL;
		   /*
			if  (layer_storage.parts.size() > 0 || magic_mesh_surface_mode != ESurfaceMode::NORMAL && layer_storage.openPolyLines.size() > 0)
			{
				storage.layer_nr_max_filled_layer = layer_nr; // last set by the highest non-empty layer
				break;
			}
			*/
		}
	}

//namespace cura
