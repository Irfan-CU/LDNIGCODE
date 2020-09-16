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
	std::vector<PolygonsPart> result_boundary;//Outer boundary material polygon result
	std::vector<PolygonsPart> result_inter_circle_A;//A material polygon result
	std::vector<PolygonsPart> result_inter_circle_B;//B material polygon result
	std::vector<PolygonsPart> result_circle_inter;//intersection parts result 
	std::vector<PolygonsPart> result_zigzag;
	std::vector<PolygonsPart> result_circle;
	const bool union_layers = true;

	printf("Inside the parts 0 \n");

	if (!layer->polygons_boundary.empty())
	{
		result = layer->polygons_boundary.splitIntoParts(union_layers);	  //polygons in slicer list		
	}

	if (!layer->polygons_Circle_interB.empty())
	{
		result_inter_circle_B = layer->polygons_Circle_interB.splitIntoParts(union_layers);
	}

	if (!layer->polygons_Circle_interA.empty())
	{
		result_inter_circle_A = layer->polygons_Circle_interA.splitIntoParts(union_layers);

	}
	if (!layer->polygons_Zigzag.empty())
	{
		result_zigzag = layer->polygons_Zigzag.splitIntoParts(union_layers);	  //polygons in slicer list
	}

	if (!layer->polygons_Circle_inter.empty())
	{
		result_circle = layer->polygons_Circle_inter.splitIntoParts(union_layers);	  //polygons in slicer list
	}

	printf("Inside the parts 1 \n");

	for (unsigned int i = 0; i < result.size(); i++)
	{
		storageLayer.parts.emplace_back();
		storageLayer.mat_parts.emplace_back(10);//(layer->polygons.polygons_matid[i]);
		storageLayer.parts[i].part_mat = 10;//layer->polygons.polygons_matid[i];
		storageLayer.parts[i].outline = result[i];
		storageLayer.parts[i].boundaryBox.calculate(storageLayer.parts[i].outline);
		storageLayer.parts[i].part_print_property = 2;

		//if (!layer->polygons_Zigzag.empty())
		/* For circle intersecting Model
		{
			storageLayer.parts_zigzag.emplace_back();
			storageLayer.parts_zigzag[i].outline_zigzag = result_zigzag[i];
			storageLayer.parts_zigzag[i].boundaryBox.calculate(storageLayer.parts_zigzag[i].outline_zigzag);

		}
		*/

	}
	printf("Inside the parts 2 \n");
	if (!layer->polygons_Circle_interA.empty())
	{
		for (unsigned int i = 0; i < result_inter_circle_A.size(); i++)
		{
			int id = storageLayer.parts.size();
			storageLayer.parts.emplace_back();
			storageLayer.mat_parts.emplace_back(3);
			storageLayer.parts[id].part_mat = 3;
			storageLayer.parts[id].outline = result_inter_circle_A[i];
			storageLayer.parts[id].boundaryBox.calculate(storageLayer.parts[id].outline);
			storageLayer.parts[id].part_print_property = 1;
		}
	}
	
	if (!layer->polygons_Circle_inter.empty())
	{
		for (unsigned int i = 0; i < result_circle.size(); i++)
		{
			
			int id = storageLayer.parts.size();
			storageLayer.parts.emplace_back();
			storageLayer.mat_parts.emplace_back(5);
			storageLayer.parts[id].part_mat = 5;
			storageLayer.parts[id].outline = result_circle[i];
			storageLayer.parts[id].boundaryBox.calculate(storageLayer.parts[id].outline);
			storageLayer.parts[id].part_print_property = 1;
		}
	}
	printf("Inside the parts 4 \n");
	if (!layer->polygons_Circle_interB.empty())
	{
		for (unsigned int i = 0; i < result_inter_circle_B.size(); i++)
		{


			int id = storageLayer.parts.size();
			storageLayer.parts.emplace_back();
			storageLayer.mat_parts.emplace_back(2);
			storageLayer.parts[id].part_mat = 2;
			storageLayer.parts[id].outline = result_inter_circle_B[i];

			storageLayer.parts[id].boundaryBox.calculate(storageLayer.parts[id].outline);
			storageLayer.parts[id].part_print_property = 1;

		}
	}
	printf("Inside the parts 5 \n");

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
			printf("the layer part size is %d \n", layer_storage.parts.size());

			
		}

		for (int layer_nr = total_layers - 1; layer_nr >= 0; layer_nr--)
		{
			SliceLayer& layer_storage = storage.Layers[layer_nr];
		}
	}

//namespace cura
