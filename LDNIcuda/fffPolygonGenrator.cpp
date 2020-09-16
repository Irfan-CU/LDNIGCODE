//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#include <algorithm>
#include <map> // multimap (ordered map allowing duplicate keys)
#include <fstream> // ifstream.good()


#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "fffPolygonGenrator.h"
#include "Infill.h"
#include "layerPart.h"
#include "MeshGroup.h"
#include "multivolumes.h" 
#include "PrintFeature.h"
#include "raft.h"
#include "SkininfillAreaComputation.h"
#include "SkirtBrim.h"
#include "LDNIcudaOperation.h"
#include "sliceDataStorage.h"
#include "Slicer.h"
#include "Support.h"

#include "TreeSupporth.h"
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
	//Progress::messageProgressStage(Progress::Stage::INSET_SKIN, &time_keeper);

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
	}

	if (isEmptyLayer(storage, 0) && !isEmptyLayer(storage, 1))
	{
		removeEmptyFirstLayers(storage, storage.print_layer_count); 
	}
	if (storage.print_layer_count == 0)
	{
		printf("Stopping process because there are no non-empty layers.\n");
		return;
	}
	
	printf("Print Layer count: %i\n", storage.print_layer_count);

	AreaSupport::generateOverhangAreas(storage);
	AreaSupport::generateSupportAreas(storage);
	TreeSupport tree_support_generator(storage);
	tree_support_generator.generateSupportAreas(storage);
	
	//processOutlineGaps(storage);//										 the outline 
	processPerimeterGaps(storage);
	
	
	
	processDerivedWallsSkinInfill(storage);

	printf("Done with DerivedWallsSkinInfill \n");

	//AreaSupport::generateSupportInfillFeatures(storage);
}

void FffPolygonGenerator::processPerimeterGaps(SliceDataStorage& storage)
{
		constexpr int perimeter_gaps_extra_offset = 15; // extra offset so that the perimeter gaps aren't created everywhere due to rounding errors
		const bool fill_perimeter_gaps = true;
		bool filter_out_tiny_gaps = true;

		for (int layer_nr = 0; layer_nr < storage.Layers.size(); layer_nr++)
		{
			
			bool fill_gaps_between_inner_wall_and_skin_or_infill = false;
			SliceLayer& layer = storage.Layers[layer_nr];
			coord_tIrfan wall_line_width_0 = MM2INT(0.35);
			coord_tIrfan wall_line_width_x = MM2INT(0.30);
			coord_tIrfan skin_line_width = MM2INT(0.4);
			if (layer_nr == 0)
			{
				
				
				wall_line_width_0 *= 100 / 100;
				wall_line_width_x *= 100 / 100;
				skin_line_width *= 100 / 100;
			}
			for (SliceLayerPart& part : layer.parts)
			{
				// handle perimeter gaps of normal insets
				int line_width = wall_line_width_0;
				for (unsigned int inset_idx = 0; static_cast<int>(inset_idx) < static_cast<int>(part.insets.size()) - 1; inset_idx++)
				{
					const Polygons outer = part.insets[inset_idx].offset(-1 * line_width / 2 - perimeter_gaps_extra_offset);
					line_width = wall_line_width_x;

					Polygons inner = part.insets[inset_idx + 1].offset(line_width / 2);
					part.perimeter_gaps.add(outer.difference(inner));
				}

				if (filter_out_tiny_gaps) {
					part.perimeter_gaps.removeSmallAreas(2 * INT2MM(wall_line_width_0) * INT2MM(wall_line_width_0)); // remove small outline gaps to reduce blobs on outside of model
				}

				for (SkinPart& skin_part : part.skin_parts)
				{
					skin_part.SkinPart_mat = part.getpartMat();

					if (skin_part.insets.size() > 0)
					{
						// add perimeter gaps between the outer skin inset and the innermost wall
						const Polygons outer = skin_part.outline;
						const Polygons inner = skin_part.insets[0].offset(skin_line_width / 2 + perimeter_gaps_extra_offset);
						skin_part.perimeter_gaps.add(outer.difference(inner));

						for (unsigned int inset_idx = 1; inset_idx < skin_part.insets.size(); inset_idx++)
						{ // add perimeter gaps between consecutive skin walls
							const Polygons outer = skin_part.insets[inset_idx - 1].offset(-1 * skin_line_width / 2 - perimeter_gaps_extra_offset);
							const Polygons inner = skin_part.insets[inset_idx].offset(skin_line_width / 2);
							skin_part.perimeter_gaps.add(outer.difference(inner));
						}

						if (filter_out_tiny_gaps) {
							skin_part.perimeter_gaps.removeSmallAreas(2 * INT2MM(skin_line_width) * INT2MM(skin_line_width)); // remove small outline gaps to reduce blobs on outside of model
						}
					}
				}
			}
		}
	}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
	
	Polygons first_layer_outline;
	coord_tIrfan primary_line_count=17;// train.settings.get<size_t>("brim_line_count");
	bool should_brim_prime_tower = false;// storage.primeTower.enabled && mesh_group_settings.get<bool>("prime_tower_brim_enable");
	
	EPlatformAdhesion type = EPlatformAdhesion::BRIM;
	SkirtBrim::getFirstLayerOutline(storage, primary_line_count, false, first_layer_outline);
	SkirtBrim::generate(storage, first_layer_outline, 0, primary_line_count);
	//Skirt Polygons are accurate till here
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
	
	int scale = storage.get_scale(); // scaling for the input CAD gemotry scaling is different in X and Z;

	storage.model_min.x = MM2INT(rotBoundingBox[0] * scale);
	storage.model_min.y = MM2INT(rotBoundingBox[4] * scale);
	storage.model_min.z = MM2INT(rotBoundingBox[2] * 18);
	storage.model_max.x = MM2INT(rotBoundingBox[1] * scale);
	storage.model_max.y = MM2INT(rotBoundingBox[5] * scale);
	storage.model_max.z = MM2INT(rotBoundingBox[3] * 18);

	storage.model_size = storage.model_max - storage.model_min;
	
	int slice_layer_count = total_layers;
	storage.Layers.resize(slice_layer_count);

	coord_tIrfan layer_thickness;
	
	(layer_thickness) = (storage.model_max.z - storage.model_min.z) / (slice_layer_count);

	storage.setlayer_thickness(layer_thickness);

	if (layer_thickness <= 0)
	{
		printf("###Error layer height %i is disallowed.\n", layer_thickness);
		return false;
	}

	slice_layer_count = total_layers; 
	
	if (slice_layer_count <= 0)
	{
		printf("###Error slice_layer_count height %i is disallowed.\n", slice_layer_count);
		return false; // This is NOT an error state!
	}
	
	std::vector<Slicer*> slicerList;

	bool use_variable_layer_heights = false;
	
	for (unsigned int mesh_idx = 0; mesh_idx < 1; mesh_idx++) //only one mesh
	{
		
		
		Slicer* slicer = new Slicer(storage, meshlist, c_mesh, layer_thickness, slice_layer_count, use_variable_layer_heights, meshin_layer);

		slicerList.push_back(slicer);
		
		
	}
	generateMultipleVolumesOverlap(slicerList);
	
	storage.print_layer_count = 0;
	
	

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
		printf("Done with parts formation");
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
	
	size_t storage_layer_count = storage.Layers.size();	 
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
		
		
#ifdef _OPENMP
		
#endif
		
#if _OPENMP < 201107
#pragma omp critical
#else
#pragma omp atomic read
#endif
		

		
#pragma omp atomic
		processed_layer_count++;
	}
	//printf("the code is right now at the line 187 of fffpolygongenrator.cpp\n");
	ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(storage_layer_count);
	mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

	bool process_infill = true;// mesh.settings.get<coord_t>("infill_line_distance") > 0;
	
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
				//printf("Processed skins and infill layer %i of %i\n", layer_number, storage_layer_count);
				;
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

		
		SkinInfillAreaComputation::combineInfillLayers(storage);
	}

void FffPolygonGenerator::processInsets(SliceDataStorage& storage, int layer_nr)
{
	SliceLayer* layer = &storage.Layers[layer_nr];
	WallsComputation walls_computation(layer_nr);
	walls_computation.generateInsets(layer);
	
}

