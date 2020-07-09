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
		std::vector<PolygonsPart> result_zigzag;
		const bool union_layers = true;
		
		result = layer->polygons.splitIntoParts(union_layers);	  //polygons in slicer list		
		
		if (!layer->polygons_Zigzag.empty())
		{
			result_zigzag = layer->polygons_Zigzag.splitIntoParts(union_layers);	  //polygons in slicer list
		}
		for (unsigned int i = 0; i < result.size(); i++)
		{
			storageLayer.parts.emplace_back();
			storageLayer.mat_parts.emplace_back(layer->polygons.polygons_matid[i]);
			storageLayer.parts[i].part_mat = layer->polygons.polygons_matid[i];
			storageLayer.parts[i].outline = result[i];
			storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
			if (!layer->polygons_Zigzag.empty())
			{
				storageLayer.parts_zigzag.emplace_back();
				storageLayer.parts_zigzag[i].outline_zigzag = result_zigzag[i];
				storageLayer.parts_zigzag[i].boundaryBox.calculate(storageLayer.parts_zigzag[i].outline_zigzag);
				
			}
		}
		
	}
	void createLayerParts(SliceDataStorage& storage, Slicer* slicer)
	{
		const auto total_layers = slicer->layers.size();
	
		assert(storage.Layers.size() == total_layers);
#pragma omp parallel for default(none) shared(storage, slicer) schedule(dynamic)
		
		for (int layer_nr = 0; layer_nr < static_cast<int>(total_layers); layer_nr++)
		{
			SliceLayer& layer_storage = storage.Layers[layer_nr];
			SlicerLayer& slice_layer = slicer->layers[layer_nr];
			createLayerWithParts(layer_storage, &slice_layer);
			
		}

		for (int layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
		{
			SliceLayer& layer_storage = storage.Layers[layer_nr];
		}
	}

//namespace cura
