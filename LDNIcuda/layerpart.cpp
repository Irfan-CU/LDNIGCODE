#include "layerPart.h"
#include "sliceDataStorage.h"
#include "slicer.h"
#include "EnumSettings.h"


void createLayerWithParts(SliceLayer& storageLayer, SlicerLayer* layer)
{
	
	std::vector<vector<PolygonsPart>> LMIresult_circle;

	const bool union_layers = true;

	for (int i = 0; i < 3; i++)
	{
		std::vector<PolygonsPart> LDMIresult;
		if (!layer->LDMIpolygons[i].empty())
		{

			LDMIresult = layer->LDMIpolygons[i].splitIntoParts(union_layers);
			LMIresult_circle.push_back(LDMIresult);

		}

		
	}

	for (int j = 0; j < 3; j++)
	{
		if (!layer->LDMIpolygons[j].empty())
		{
			for (unsigned int i = 0; i < LMIresult_circle.at(j).size(); i++)
			{
				int id = storageLayer.parts.size();
				storageLayer.parts.emplace_back();
				storageLayer.mat_parts.emplace_back(3);
				storageLayer.parts[id].part_mat = 3;
				storageLayer.parts[id].setPolygonExtruders((layer->LDMIpolygons[j].getPolygonExtruders()));
				storageLayer.parts[id].outline = LMIresult_circle.at(j).at(i);
				storageLayer.parts[id].boundaryBox.calculate(storageLayer.parts[id].outline);
				storageLayer.parts[id].part_print_property = 1;
				storageLayer.parts[id].thirdMaterial = 0;
				
				
			}
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

