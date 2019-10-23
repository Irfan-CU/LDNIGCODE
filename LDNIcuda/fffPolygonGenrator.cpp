//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include <algorithm>
#include <map> // multimap (ordered map allowing duplicate keys)
#include <fstream> // ifstream.good()


#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "fffPolygonGenrator.h"
#include "infill.h"
#include "layerPart.h"

#include "multivolumes.h" 
#include "PrintFeature.h"
#include "raft.h"
#include "SkininfillAreaComputation.h"
#include "SkirtBrim.h"
#include "LDNIcudaOperation.h"
#include "sliceDataStorage.h"
#include "Slicer.h"
#include "Support.h"
//#include "TreeSupport.h"

#include "WallComputation.h"
#include "Progress.h"
#include "ProgressEstimator.h"
#include "ProgressStageEstimator.h"
#include "ProgressEstimatorLinear.h"
#include "algorithm.h"
#include "gettime.h"
#include "math.h"



void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage)
{
	// compute layer count and remove first empty layers
	// there is no separate progress stage for removeEmptyFisrtLayer (TODO)
	TimeKeeper time_keeper;
	time_keeper.restart();
	unsigned int slice_layer_count = 0;
	for (SliceLayer& layer : storage.Layers)
	{
			slice_layer_count = std::max<unsigned int>(slice_layer_count, storage.Layers.size());
	}

	std::vector<double> mesh_timings;

	for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
	{
		mesh_timings.push_back(1.0); // TODO: have a more accurate estimate of the relative time it takes per mesh, based on the height and number of polygons
	}

	ProgressStageEstimator inset_skin_progress_estimate(mesh_timings);
	Progress::messageProgressStage(Progress::Stage::INSET_SKIN, &time_keeper);

	std::vector<size_t> mesh_order;
	{ // compute mesh order
		std::multimap<int, size_t> order_to_mesh_indices;
		for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
		{
			order_to_mesh_indices.emplace((0), mesh_idx);
		}
		for (std::pair<const int, size_t>& order_and_mesh_idx : order_to_mesh_indices)
		{
			mesh_order.push_back(order_and_mesh_idx.second);
		}
	}
   
	for (int mesh_order_idx = 0; mesh_order_idx < 1; mesh_order_idx++)
	{
		processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);
		//Progress::messageProgress(Progress::Stage::INSET_SKIN, mesh_order_idx + 1, storage.meshes.size());
		
	}
	//current mesh group seeting are for the whole single 3D MESH
	if (isEmptyLayer(storage, 0) && !isEmptyLayer(storage, 1))
	{
		// the first layer is empty, the second is not empty, so remove the empty first layer as support isn't going to be generated under it.
		// Do this irrespective of the value of remove_empty_first_layers as that setting is hidden when support is enabled and so cannot be relied upon

		removeEmptyFirstLayers(storage, storage.print_layer_count); // changes storage.print_layer_count!
	}
	if (storage.print_layer_count == 0)
	{
		printf("Stopping process because there are no non-empty layers.\n");
		return;
	}
	
	printf("Print Layer count: %i\n", storage.print_layer_count);

	AreaSupport::generateOverhangAreas(storage);
	AreaSupport::generateSupportAreas(storage);
	//TreeSupport tree_support_generator(storage);
	//tree_support_generator.generateSupportAreas(storage);

	printf("the bool is %d \n", isEmptyLayer(storage, 0));
	if (!isEmptyLayer(storage, 0))
	{
		printf("Processing platform adhesion\n");
		processPlatformAdhesion(storage);
	}
	
	
	processDerivedWallsSkinInfill(storage);
	

	//printf("Processing gradual support\n");
	// generate gradual support
	AreaSupport::generateSupportInfillFeatures(storage);
}


void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
	
	Polygons first_layer_outline;
	coord_tIrfan primary_line_count;
	bool should_brim_prime_tower = false;// storage.primeTower.enabled && mesh_group_settings.get<bool>("prime_tower_brim_enable");
	
	primary_line_count = 17;// train.settings.get<size_t>("brim_line_count");
	SkirtBrim::getFirstLayerOutline(storage, primary_line_count, false, first_layer_outline);
	SkirtBrim::generate(storage, first_layer_outline, 0, primary_line_count);
	
}

/*
void FffPolygonGenerator::computePrintHeightStatistics(SliceDataStorage& storage)
{
	const size_t extruder_count = 1;

	std::vector<int>& max_print_height_per_extruder = storage.max_print_height_per_extruder;
	assert(max_print_height_per_extruder.size() == 0 && "storage.max_print_height_per_extruder shouldn't have been initialized yet!");
	max_print_height_per_extruder.resize(extruder_count, 0); //Initialize all as -1 (or lower in case of raft).
	{ // compute max_object_height_per_extruder
		//Height of the meshes themselves.
		
			for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
			{
				for (int layer_nr = static_cast<int>(storage.Layers.size()) - 1; layer_nr > max_print_height_per_extruder[extruder_nr]; layer_nr--)
				{
						assert(max_print_height_per_extruder[extruder_nr] <= layer_nr);
						max_print_height_per_extruder[extruder_nr] = layer_nr;
					
				}
			}
		

		//Height of where the support reaches.
		
		const size_t support_infill_extruder_nr = 0;// scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr; // TODO: Support extruder should be configurable per object.
		max_print_height_per_extruder[support_infill_extruder_nr] = std::max(max_print_height_per_extruder[support_infill_extruder_nr],storage.support.layer_nr_max_filled_layer);
		const size_t support_roof_extruder_nr = 0;// scene.current_mesh_group->settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr; // TODO: Support roof extruder should be configurable per object.
		max_print_height_per_extruder[support_roof_extruder_nr] =
			std::max(max_print_height_per_extruder[support_roof_extruder_nr],
				storage.support.layer_nr_max_filled_layer);
		const size_t support_bottom_extruder_nr = 0;// scene.current_mesh_group->settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr; //TODO: Support bottom extruder should be configurable per object.
		max_print_height_per_extruder[support_bottom_extruder_nr] =
			std::max(max_print_height_per_extruder[support_bottom_extruder_nr],
				storage.support.layer_nr_max_filled_layer);

		//Height of where the platform adhesion reaches.
		const size_t adhesion_extruder_nr = 0;// scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr;
			max_print_height_per_extruder[adhesion_extruder_nr] = std::max(0, max_print_height_per_extruder[adhesion_extruder_nr]);
		
	}

	storage.max_print_height_order = order(max_print_height_per_extruder);
	storage.max_print_height_second_to_last_extruder = -(Raft::getTotalExtraLayers() + 1);
	
}
*/
bool FffPolygonGenerator::sliceModel(GLKObList& meshlist, ContourMesh& c_mesh, SliceDataStorage& storage, int total_layers, std::vector<int>& meshin_layer, double rotBoundingBox[]) // slices the model
{
	
	storage.model_min.x = MM2INT(rotBoundingBox[0] * 30);
	storage.model_min.y = MM2INT(rotBoundingBox[4] * 30);
	storage.model_min.z = MM2INT(rotBoundingBox[2] * 30);
	storage.model_max.x = MM2INT(rotBoundingBox[1] * 30);
	storage.model_max.y = MM2INT(rotBoundingBox[5] * 30);
	storage.model_max.z = MM2INT(rotBoundingBox[3] * 30);

	storage.model_size = storage.model_max - storage.model_min;

	int slice_layer_count = total_layers;

	//slice_layer_count = (storage.model_max.z - layer_thickness) / ;
	coord_tIrfan layer_thickness;
	
	(layer_thickness) = (storage.model_max.z - slice_layer_count) / (slice_layer_count - 1);

	storage.setlayer_thickness(layer_thickness);

	printf("slice layer count is %d and %d \n", slice_layer_count, layer_thickness);


//	printf("the storage.model_min.x is %d and storage.model_min.y is %d  storage.model_min.z is %d storage.model_max.x is %d storage.model_max.y is %d  storage.model_max.z is %d \n", storage.model_min.x, storage.model_min.y, storage.model_min.z, storage.model_max.x, storage.model_max.y, storage.model_max.z);
	
	// regular layers
	//Use signed int because we need to subtract the initial layer in a calculation temporarily.
	
	// Initial layer height of 0 is not allowed. Negative layer height is nonsense.
	//coord_tIrfan initial_layer_thickness = MM2INT(0.01*27); //mesh_group_settings.get<coord_t>("layer_height_0");													initial_layer_thickness
	//printf("the initial_layer_thickness is %d \n", initial_layer_thickness);
	if (layer_thickness <= 0)
	{
		//logError("Initial layer height %i is disallowed.\n", initial_layer_thickness);
		return false;
	}

	// Layer height of 0 is not allowed. Negative layer height is nonsense.
	//const coord_tIrfan layer_thickness = MM2INT(0.01*15);// mesh_group_settings.get<coord_t>("layer_height");
	//printf("the layer_thickness is %d \n", layer_thickness);
	if (layer_thickness <= 0)
	{
		//logError("Layer height %i is disallowed.\n", layer_thickness);
		return false;
	}
	

	// variable layers
	//AdaptiveLayerHeights* adaptive_layer_heights = nullptr;
	const bool use_variable_layer_heights = false;// mesh_group_settings.get<bool>("adaptive_layer_height_enabled");
	slice_layer_count = total_layers; //(storage.model_max.z - initial_layer_thickness) / layer_thickness + 1;
	//printf("slice layer count is %d \n", slice_layer_count);
	
	//printf("the slice layer count is %d \n", total_layers);
	// Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
	if (slice_layer_count <= 0)
	{
		return true; // This is NOT an error state!
	}
	
	std::vector<Slicer*> slicerList;
	
	for (unsigned int mesh_idx = 0; mesh_idx < 1; mesh_idx++) //only one mesh
	{
		//printf("going for the slicer program \n");
		//Mesh& mesh = storage.meshes[mesh_idx];
		Slicer* slicer = new Slicer(storage, meshlist, c_mesh, layer_thickness, slice_layer_count, use_variable_layer_heights, meshin_layer);

		slicerList.push_back(slicer);
		
	}
	/*
	for (int i = 0; i < slicerList[0]->layers.size(); i++)
	{
		printf("the size of the polygons is %d in layer %d \n", slicerList[0]->layers[i].polygons.size(),i);
	}
	*/
   
	// No need for MOLD Process Checked requirements 
	//carveMultipleVolumes(slicerList);no need as the slicer list is only 1;Mesh is a single 3D mesh Mesh group is a group of meshes;
	generateMultipleVolumesOverlap(slicerList);
	
	storage.print_layer_count = 0;
	
	//printf("at line 225 \n");

	for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
	{
			Slicer* slicer = slicerList[meshIdx];
			bool anti_overhang_mesh = false;
			bool infill_mesh = false;
			bool cutting_mesh = false;

			if (!anti_overhang_mesh && ! infill_mesh && !cutting_mesh)
			{
					storage.print_layer_count = slicer->layers.size();
			}
						
	}

	storage.support.supportLayers.resize(storage.print_layer_count);
	storage.meshes.reserve(slicerList.size());

	/*
	for (int i = 0; i < slice_layer_count; i++)
	{
		printf("the polygons point count is %d \n", slicerList[0]->layers[i].polygons.pointCount());
	}
	*/

	for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
	{
		Slicer* slicer = slicerList[meshIdx];
		// I dont have any mesh in my code	storage.meshes.emplace_back(&meshgroup->meshes[meshIdx], slicer->layers.size()); // new mesh in storage had settings from the Mesh
			//SliceMeshStorage& meshStorage = storage.meshes.back();
			// only create layer parts for normal meshes
		const bool is_support_modifier = false;// AreaSupport::handleSupportModifierMesh(storage, mesh.settings, slicer);
		//printf ("the size of the polygons is %d \n",slicer->layers[0].polygons.size());
		if (!is_support_modifier)
		{
			createLayerParts(storage, slicer);
		}

		// Do not add and process support _modifier_ meshes further, and ONLY skip support _modifiers_. They have been
		// processed in AreaSupport::handleSupportModifierMesh(), but other helper meshes such as infill meshes are
		// processed in a later stage, except for support mesh itself, so an exception is made for that.
		/*
		if (is_support_modifier && !mesh.settings.get<bool>("support_mesh"))
		{
			storage.meshes.pop_back();
			continue;
		}*/
		// check one if raft offset is needed

		const bool has_raft = false;// mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT;

		// calculate the height at which each layer is actually printed (printZ)
		for (unsigned int layer_nr = 0; layer_nr < storage.Layers.size(); layer_nr++)
		{
			SliceLayer& layer = storage.Layers[layer_nr];

			storage.Layers[layer_nr].printZ = layer_thickness + (layer_nr * layer_thickness);

			if (layer_nr == 0)
			{
				storage.Layers[layer_nr].thickness = layer_thickness;
				//printf("the initial layer thickness is %d \n", initial_layer_thickness);
			}
			else
			{
				storage.Layers[layer_nr].thickness = layer_thickness;
			}

		}
		
		delete slicerList[meshIdx];

	}  

	return true;
}

void FffPolygonGenerator::processBasicWallsSkinInfill(SliceDataStorage& storage, const size_t mesh_order_idx, const std::vector<size_t>& mesh_order, ProgressStageEstimator& inset_skin_progress_estimate)
{
	size_t mesh_idx = mesh_order[mesh_order_idx];
	//printf("inside processBasicWallsSkinInfill\n");
	//size_t mesh_idx = 1;
	size_t storage_layer_count = storage.Layers.size();	  //141
	std::vector<double> walls_vs_skin_timing({ 22.953, 48.858 });
	ProgressStageEstimator* mesh_inset_skin_progress_estimator = new ProgressStageEstimator(walls_vs_skin_timing);
	inset_skin_progress_estimate.nextStage(mesh_inset_skin_progress_estimator); // the stage of this function call
	ProgressEstimatorLinear* inset_estimator = new ProgressEstimatorLinear(storage_layer_count);
	mesh_inset_skin_progress_estimator->nextStage(inset_estimator);

	// walls
	size_t processed_layer_count = 0;
#pragma omp parallel for default(none) shared(storage_layer_count, storage) schedule(dynamic)
	// Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
	
	for (int layer_number = 0; layer_number < static_cast<int>(storage.Layers.size()); layer_number++)
	{
				
		processInsets(storage, layer_number);
		
		// , layercheck->parts[0].insets[0].pointCount(), layercheck->parts[0].insets[1].pointCount(), layercheck->parts[2].insets[0].pointCount());
		//printf("Insets are successfully developed line 187 nof fffPolygonGenerator.cpp\n");
#ifdef _OPENMP
		//if (omp_get_thread_num() == 0)
#endif
		//{ // progress estimation is done only in one thread so that no two threads message progress at the same time
		//	int _processed_layer_count;
#if _OPENMP < 201107
#pragma omp critical
#else
#pragma omp atomic read
#endif
			//_processed_layer_count = processed_layer_count;
			//double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
			//Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
			//printf("Processed insets for layer %d \n", layer_number);

		//}
#pragma omp atomic
		processed_layer_count++;
	}
	//printf("the code is right now at the line 187 of fffpolygongenrator.cpp\n");
	ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(storage_layer_count);
	mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

	bool process_infill = true;// mesh.settings.get<coord_t>("infill_line_distance") > 0;
	/*
	if (!process_infill)
	{ // do process infill anyway if it's modified by modifier meshes
		const Scene& scene = Application::getInstance().current_slice->scene;
		for (size_t other_mesh_order_idx = mesh_order_idx + 1; other_mesh_order_idx < mesh_order.size(); ++other_mesh_order_idx)
		{
			const size_t other_mesh_idx = mesh_order[other_mesh_order_idx];
			SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
			if (other_mesh.settings.get<bool>("infill_mesh"))
			{
				AABB3D aabb = scene.current_mesh_group->meshes[mesh_idx].getAABB();
				AABB3D other_aabb = scene.current_mesh_group->meshes[other_mesh_idx].getAABB();
				if (aabb.hit(other_aabb))
				{
					process_infill = true;
				}
			}
		}
	}
	*/
	// skin & infill

	//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	size_t mesh_max_bottom_layer_count = 0;
	/*
	magic_spiralize = flase;
	if (mesh_group_settings.get<bool>("magic_spiralize"))
	{
		mesh_max_bottom_layer_count = std::max(mesh_max_bottom_layer_count, mesh.settings.get<size_t>("bottom_layers"));
	}
	*/
	processed_layer_count = 0;
#pragma omp parallel default(none) shared(storage_layer_count, mesh_max_bottom_layer_count, process_infill, processed_layer_count)
	{

#pragma omp for schedule(dynamic)
		// Use a signed type for the loop counter so MSVC compiles (because it uses OpenMP 2.0, an old version).
		for (int layer_number = 0; layer_number < static_cast<int>(storage.Layers.size()); layer_number++)
		{
			//logDebug("Processing skins and infill layer %i of %i\n", layer_number, mesh_layer_count);
			//printf("Processing skins and infill layer %i of %i\n", layer_number, storage_layer_count);
			bool magic_spiralize = false;

			if (!magic_spiralize || layer_number < static_cast<int>(mesh_max_bottom_layer_count))    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
			{
				processSkinsAndInfill(storage, layer_number, process_infill);
			}
			
#ifdef _OPENMP
			//if (omp_get_thread_num() == 0)
#endif
			{ // progress estimation is done only in one thread so that no two threads message progress at the same time
				//int _processed_layer_count;
#if _OPENMP < 201107
#pragma omp critical
#else
#pragma omp atomic read
#endif
				//_processed_layer_count = processed_layer_count;
				//double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
				//Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
				printf("Processed skins and infill layer %i of %i\n", layer_number, storage_layer_count);
		    }
#pragma omp atomic
			processed_layer_count++;
		}
	}
}

void FffPolygonGenerator::processSkinsAndInfill(SliceDataStorage& storage, const int layer_nr, bool process_infill)
{
	SkinInfillAreaComputation skin_infill_area_computation(layer_nr, storage, process_infill);
	skin_infill_area_computation.generateSkinsAndInfill();

}

bool FffPolygonGenerator::isEmptyLayer(SliceDataStorage& storage, const unsigned int layer_idx)
{
	SliceLayer& layer = storage.Layers[layer_idx];

	for (const SliceLayerPart& part : layer.parts)
	{
		if (part.print_outline.size() > 0)
		{
			return false;
		}
	}
	
	return true;
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, size_t& total_layers)
{
	int n_empty_first_layers = 0;
	for (size_t layer_idx = 0; layer_idx < total_layers; layer_idx++)
	{
		if (isEmptyLayer(storage, layer_idx))
		{
			n_empty_first_layers++;
		}
		else
		{
			break;
		}
	}

	if (n_empty_first_layers > 0)
	{
		printf("Removing %d layers because they are empty\n", n_empty_first_layers);
		const coord_tIrfan layer_height = 0.2;// Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
		std::vector<SliceLayer>& layers = storage.Layers;
			layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
			for (SliceLayer& layer : layers)
			{
				layer.printZ -= n_empty_first_layers * layer_height;
			}
			storage.layer_nr_max_filled_layer -= n_empty_first_layers;
		
		total_layers -= n_empty_first_layers;
		//storage.support.layer_nr_max_filled_layer -= n_empty_first_layers;
		//std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
		//support_layers.erase(support_layers.begin(), support_layers.begin() + n_empty_first_layers);
	}
}

void FffPolygonGenerator::processDerivedWallsSkinInfill(SliceDataStorage& storage)
{
	// generate spaghetti infill filling areas and volumes
	
		// create gradual infill areas
		SkinInfillAreaComputation::generateGradualInfill(storage);
		// Pre-compute Cross Fractal
		// combine infill

		printf("**********************************Starting Coming Infill**************************************\n");
		SkinInfillAreaComputation::combineInfillLayers(storage);
	}

void FffPolygonGenerator::processInsets(SliceDataStorage& storage, int layer_nr)
{
	SliceLayer* layer = &storage.Layers[layer_nr];
	WallsComputation walls_computation(layer_nr);
	walls_computation.generateInsets(layer);
	
	
}

	// fuzzy skin
	