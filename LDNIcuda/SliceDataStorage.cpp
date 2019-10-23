

//To create a mesh group with if none is provided.
#include "raft.h"
#include "SliceDataStorage.h"
//#include "Mesh.h"


SupportStorage::SupportStorage()
	: generated(false)
	, layer_nr_max_filled_layer(-1)

{
}

SupportStorage::~SupportStorage()
{
	supportLayers.clear();
	
}

Polygons& SliceLayerPart::getOwnInfillArea()
{
	return const_cast<Polygons&>(const_cast<const SliceLayerPart*>(this)->getOwnInfillArea());
}

const Polygons& SliceLayerPart::getOwnInfillArea() const
{
	if (infill_area_own)
	{
		return *infill_area_own;
	}
	else
	{
		return infill_area;
	}
	
	//return infill_area;
}

std::vector<bool> SliceDataStorage::getExtrudersUsed() const
{
	std::vector<bool> ret;
	ret.resize(1, false);
	ret[0] = true;
	 // process brim/skirt
	for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
	{
		if (skirt_brim[extruder_nr].size() > 0)
		{
			ret[extruder_nr] = true;
	    	continue;
		}
			
	}
		
	bool support_enable = false;
	bool support_tree_enable = true;
	bool support_mesh = false;

	if (support_enable || support_tree_enable || support_mesh)
	{
			ret[0] = true;
			//ret[0] = true;
			
	}

	return ret;
}

std::vector<WipeScriptConfig> SliceDataStorage::initializeWipeConfigs()
{
	std::vector<WipeScriptConfig> ret;
	ret.resize(1);
	return ret;
}

std::vector<RetractionConfig> SliceDataStorage::initializeRetractionConfigs()
{
	std::vector<RetractionConfig> ret;
	ret.resize(1); // initializes with constructor RetractionConfig()
	return ret;
}

 SliceDataStorage::SliceDataStorage()
	: print_layer_count(0),
	  wipe_config_per_extruder(initializeWipeConfigs())
	, retraction_config_per_extruder(initializeRetractionConfigs())
	, extruder_switch_retraction_config_per_extruder(initializeRetractionConfigs())
	, max_print_height_second_to_last_extruder(-1)
{
	Point3 machine_max(MM2INT(233), MM2INT(215), MM2INT(200));
	Point3 machine_min(0, 0, 0);
	bool machine_center_is_zero = false;
	if (machine_center_is_zero)
	{
		machine_max /= 2;
		machine_min -= machine_max;
	}
	machine_size.include(machine_min);
	machine_size.include(machine_max);
}

Polygons SliceDataStorage::getLayerOutlines(const int layer_nr, const bool include_support, const bool include_prime_tower, const bool external_polys_only) const
{
	if (layer_nr < 0 && layer_nr < -static_cast<int>(Raft::getFillerLayerCount()))
	{ // when processing raft
	    
		return Polygons();
	
	}
	else
	{
		printf("Inside the platformadhesion \n");
		Polygons total;
		coord_tIrfan maximum_resolution = std::numeric_limits<coord_tIrfan>::max();
		coord_tIrfan maximum_deviation = std::numeric_limits<coord_tIrfan>::max();
		if (layer_nr >= 0)
		{
				const SliceLayer& layer = Layers[layer_nr];
				printf("layer parts are %d \n", layer.parts.size());
				layer.getOutlines(total, external_polys_only);
				
				maximum_resolution = MM2INT(0.04);// std::min(maximum_resolution, mesh.settings.get<coord_t>("meshfix_maximum_resolution"));
				maximum_deviation =  MM2INT(0.05);// std::min(maximum_deviation, mesh.settings.get<coord_t>("meshfix_maximum_deviation"));
			
		}
		//total.simplify(maximum_resolution, maximum_deviation);
		return total;
	}
}

/*

SliceMeshStorage::SliceMeshStorage(Mesh* mesh, const size_t slice_layer_count)
	: mesh_name(mesh->mesh_name)
	, layer_nr_max_filled_layer(0)
	, bounding_box(mesh->getAABB())
{
	
}



SliceDataStorage::SliceDataStorage(const size_t slice_layer_count)
	: print_layer_count(0)
	, retraction_config_per_extruder(initializeRetractionConfigs())
	, extruder_switch_retraction_config_per_extruder(initializeRetractionConfigs())
	, max_print_height_second_to_last_extruder(-1)
{
	Layers.resize(slice_layer_count);
	Point3 machine_max(233, 215, 200);
	Point3 machine_min(0, 0, 0);
	machine_size.include(machine_min);
	machine_size.include(machine_max);
}

std::vector<RetractionConfig> SliceDataStorage::initializeRetractionConfigs()
{
	std::vector<RetractionConfig> ret;
	ret.resize(2); // initializes with constructor RetractionConfig()
	return ret;
}

 */

Polygon SliceDataStorage::getMachineBorder(bool adhesion_offset) const
{
	//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;

	Polygon border{};
	
		border = machine_size.flatten().toPolygon();
	
	if (!adhesion_offset) 
	{
		return border;
	}

	coord_tIrfan adhesion_size = 0; //Make sure there is enough room for the platform adhesion around support.
	//const ExtruderTrain& adhesion_extruder = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
	coord_tIrfan extra_skirt_line_width = 0;
	const std::vector<bool> is_extruder_used = getExtrudersUsed();
	for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
	{
		if (extruder_nr == 0) //Unused extruders and the primary adhesion extruder don't generate an extra skirt line.
		{
			continue;
		}
		extra_skirt_line_width += MM2INT(0.35) *120;
	}
	adhesion_size = MM2INT(0.35) * 120 * 17 + extra_skirt_line_width;
	return border.offset(-adhesion_size)[0];
}

Polygons SliceLayer::getOutlines(bool external_polys_only) const
{
	Polygons ret;
	getOutlines(ret, external_polys_only);
	return ret;
}

void SliceLayer::getOutlines(Polygons& result, bool external_polys_only) const
{
	for (const SliceLayerPart& part : parts)
	{
		if (external_polys_only)
		{
			result.add(part.outline.outerPolygon());
		}
		else
		{
			result.add(part.print_outline);
			printf("result size is %d \n", result.size());
		}
	}
}

SliceLayer::~SliceLayer()
{

}

curaIrfan::PointIrfan SliceDataStorage::getZSeamHint() const
{
	curaIrfan::PointIrfan pos(116.5, 645);
	return pos;
}

void SupportLayer::excludeAreasFromSupportInfillAreas(const Polygons& exclude_polygons, const AABB& exclude_polygons_boundary_box)
{
	// record the indexes that need to be removed and do that after
	std::list<size_t> to_remove_part_indices;  // LIFO for removing

	unsigned int part_count_to_check = support_infill_parts.size(); // note that support_infill_parts.size() changes during the computation below
	for (size_t part_idx = 0; part_idx < part_count_to_check; ++part_idx)
	{
		SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

		// if the areas don't overlap, do nothing
		if (!exclude_polygons_boundary_box.hit(support_infill_part.outline_boundary_box))
		{
			continue;
		}

		Polygons result_polygons = support_infill_part.outline.difference(exclude_polygons);

		// if no smaller parts get generated, this mean this part should be removed.
		if (result_polygons.empty())
		{
			to_remove_part_indices.push_back(part_idx);
			continue;
		}

		std::vector<PolygonsPart> smaller_support_islands = result_polygons.splitIntoParts();

		if (smaller_support_islands.empty())
		{ // extra safety guard in case result_polygons consists of too small polygons which are automatically removed in splitIntoParts
			to_remove_part_indices.push_back(part_idx);
			continue;
		}

		// there are one or more smaller parts.
		// we first replace the current part with one of the smaller parts,
		// the rest we add to the support_infill_parts (but after part_count_to_check)
		support_infill_part.outline = smaller_support_islands[0];

		for (size_t support_island_idx = 1; support_island_idx < smaller_support_islands.size(); ++support_island_idx)
		{
			const PolygonsPart& smaller_island = smaller_support_islands[support_island_idx];
			support_infill_parts.emplace_back(smaller_island, support_infill_part.support_line_width, support_infill_part.inset_count_to_generate);
		}
	}

	// remove the ones that need to be removed (LIFO)
	while (!to_remove_part_indices.empty())
	{
		const size_t remove_idx = to_remove_part_indices.back();
		to_remove_part_indices.pop_back();

		if (remove_idx < support_infill_parts.size() - 1)
		{ // move last element to the to-be-removed element so that we can erase the last place in the vector
			support_infill_parts[remove_idx] = std::move(support_infill_parts.back());
		}
		support_infill_parts.pop_back(); // always erase last place in the vector
	}
}

 void SliceDataStorage::setlayer_thickness(coord_tIrfan layer_thickness1) 
{
	 layer_thickness= layer_thickness1;
}

coord_tIrfan SliceDataStorage::getlayer_thickness() 
{
	return layer_thickness;
}



