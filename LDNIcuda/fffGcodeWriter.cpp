

#include <list>
#include <limits> // numeric_limits
#include <vector>

#include "Bridge.h"
#include "fffGcodeWriter.h"
#include "GcodeLayerThreader.h"
#include "Infill.h"
#include "InsetOrderOptimizer.h"
#include "LayerPlan.h"
#include "Raft.h"
#include "WallOverlap.h"
#include "SliceDataStorage.h"
#include "Slicer.h"



#define OMP_MAX_ACTIVE_LAYERS_PROCESSED 30

FffGcodeWriter::FffGcodeWriter()
	: max_object_height(0)
	, layer_plan_buffer(gcode)
{
	for (unsigned int extruder_nr = 0; extruder_nr < 16; extruder_nr++)
	{
		extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
	}
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage, bool start)
{
	const size_t start_extruder_nr = getStartExtruder(storage);
	gcode.preSetup(start_extruder_nr);

	if (start)
	{
		gcode.resetTotalPrintTimeAndFilament();
		gcode.setInitialAndBuildVolumeTemps(start_extruder_nr);
		max_object_height = MM2INT(0);
	}

	setConfigFanSpeedLayerTime();

	setConfigRetraction(storage);

	setConfigWipe(storage);

	coord_tIrfan layer_thickness = storage.getlayer_thickness();


	if (start)
	{
		processStartingCode(storage, start_extruder_nr);
		start = false;

	}
	else
	{
		processNextMeshGroupCode(storage);
	}

	size_t total_layers = 0;
	for (SliceLayer& layer_storage : storage.Layers)
	{

		total_layers = std::max(total_layers, storage.Layers.size());
		setInfillAndSkinAngles(storage);
	}

	gcode.writeLayerCountComment(total_layers);
	storage.bounding_box;
	int process_layer_starting_layer_nr = 0;
	const std::function<LayerPlan* (int)>& produce_item = [&storage, total_layers, this](int layer_nr)
	{

		LayerPlan& gcode_layer = processLayer(storage, layer_nr, total_layers);
		return &gcode_layer;
	};

	const std::function<void(LayerPlan*)>& consume_item =
		[this, total_layers](LayerPlan* gcode_layer)
	{

		layer_plan_buffer.handle(*gcode_layer, gcode);
	};
	const unsigned int max_task_count = OMP_MAX_ACTIVE_LAYERS_PROCESSED;
	GcodeLayerThreader<LayerPlan> threader(
		process_layer_starting_layer_nr
		, static_cast<int>(total_layers)
		, produce_item
		, consume_item
		, max_task_count
	);


	threader.run();

	layer_plan_buffer.flush();
	max_object_height = std::max(max_object_height, storage.model_max.z);


	constexpr bool force = true;

	gcode.writeRetraction(storage.retraction_config_per_extruder[0]);


}



std::vector<size_t> FffGcodeWriter::getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const size_t start_extruder, const int& layer_nr) const
{
	size_t extruder_count = 3;
	assert(static_cast<int>(extruder_count) > 0);
	std::vector<size_t> ret;
	ret.push_back(start_extruder);
	std::vector<bool> extruder_is_used_on_this_layer = storage.getExtrudersUsed(layer_nr);
	bool prime_tower_enable = false;

	//The outermost prime tower extruder is always used if there is a prime tower.
	if (prime_tower_enable && layer_nr <= storage.max_print_height_second_to_last_extruder)
	{
		extruder_is_used_on_this_layer[storage.primeTower.extruder_order[0]] = true;
	}


	// check if we are on the first layer
	if (layer_nr == 0)
	{
		// check if we need prime blob on the first layer
		for (size_t used_idx = 0; used_idx < extruder_is_used_on_this_layer.size(); used_idx++)
		{
			if (getExtruderNeedPrimeBlobDuringFirstLayer(storage, used_idx))
			{
				extruder_is_used_on_this_layer[used_idx] = true;
			}
		}
	}

	for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
	{
		if (extruder_nr == start_extruder)
		{ // skip the current extruder, it's the one we started out planning
			continue;
		}
		if (!extruder_is_used_on_this_layer[extruder_nr])
		{
			continue;
		}
		ret.push_back(extruder_nr);
	}
	assert(ret.size() <= (size_t)extruder_count && "Not more extruders may be planned in a layer than there are extruders!");
	return ret;
}

bool FffGcodeWriter::getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, const size_t extruder_nr) const
{
	bool need_prime_blob = false;
	need_prime_blob = true;

	// check the settings if the prime blob is disabled
	if (need_prime_blob)
	{
		const bool is_extruder_used_overall = storage.getExtrudersUsed()[extruder_nr];
		const bool extruder_prime_blob_enabled = storage.getExtruderPrimeBlobEnabled(extruder_nr);

		need_prime_blob = is_extruder_used_overall && extruder_prime_blob_enabled;
	}

	return need_prime_blob;
}

bool SliceDataStorage::getExtruderPrimeBlobEnabled(const size_t extruder_nr) const
{
	if (extruder_nr >= 1)
	{
		return false;
	}

	return true;
}

LayerPlan& FffGcodeWriter::processLayer(SliceDataStorage& storage, int layer_nr, const size_t total_layers) const
{
	coord_tIrfan z;
	bool include_helper_parts = true;

	z = storage.Layers[layer_nr].printZ; // stub default   //storage.Layers[layer_nr].printZ = initial_layer_thickness + (layer_nr * layer_thickness);
	coord_tIrfan layer_thickness = storage.Layers[layer_nr].thickness;
	coord_tIrfan avoid_distance = 0; // minimal avoid distance is zero
	const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
	for (size_t extruder_nr = 0; extruder_nr <= 2; extruder_nr++)
	{
		if (extruder_is_used[extruder_nr])
		{
			avoid_distance = MM2INT(3);// std::max(avoid_distance, extruder.settings.get<coord_t>("travel_avoid_distance"));
		}
	}

	//double max_inner_wall_width = 0.3;
	coord_tIrfan max_inner_wall_width = MM2INT(0.30);
	if (layer_nr == 0)
	{
		max_inner_wall_width *= Ratio(100);
	}

	const coord_tIrfan comb_offset_from_outlines = max_inner_wall_width * 2;// inner_wall_width * 2; defalut is 120
	//no need for extruder order as I ma using only extruder

	const std::vector<size_t>& extruder_order = extruder_order_per_layer[layer_nr];

	coord_tIrfan first_outer_wall_line_width = MM2INT(0.35);


	LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_thickness, 0, fan_speed_layer_time_settings_per_extruder, comb_offset_from_outlines, first_outer_wall_line_width, avoid_distance);


	const size_t support_roof_extruder_nr = 0;
	const size_t support_bottom_extruder_nr = 0;
	const size_t support_infill_extruder_nr = 0;
	

	bool disable_path_optimisation = true;
	int mesh_idx = 0;


	const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
	const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
	addMeshLayerToGCode(storage, mesh, 0, mesh_config, gcode_layer);



	//if (layer_nr % 2 != 0)
	//{
	//	for (int extruder_nr = 2; extruder_nr >= 0; extruder_nr--)
	//	{
	//		if (layer_nr >= 0)
	//		{

	//			const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
	//			const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
	//			addMeshLayerToGCode(storage, mesh, extruder_nr, mesh_config, gcode_layer);

	//		}
	//	}

	//}
	//else
	//{
	//	for (int extruder_nr = 0; extruder_nr <= 2; extruder_nr++)
	//	{
	//		if (layer_nr >= 0)
	//		{

	//			const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
	//			const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
	//			addMeshLayerToGCode(storage, mesh, extruder_nr, mesh_config, gcode_layer);

	//		}
	//	}

	//}





	if (!disable_path_optimisation)
	{

		gcode_layer.optimizePaths(gcode.getPositionXY());

	}

	return gcode_layer;
}

void FffGcodeWriter::processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr) const
{

	coord_tIrfan layer_thickness = storage.layer_thickness;
	//printf("inside skirt brim\n");
	if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
	{
		//printf("not processing brim \n");
		return;
	}

	const Polygons& skirt_brim = storage.skirt_brim[extruder_nr];
	gcode_layer.setSkirtBrimIsPlanned(extruder_nr);
	if (skirt_brim.size() == 0)
	{
		return;
	}
	// Start brim close to the prime location
	//const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
	curaIrfan::PointIrfan start_close_to;
	const bool prime_pos_is_abs = true;
	const curaIrfan::PointIrfan prime_pos(MM2INT(0.0), MM2INT(0.0));
	start_close_to = prime_pos;
	gcode_layer.addTravel(layer_thickness, gcode_layer.getLayerNr(), skirt_brim.back().closestPointTo(start_close_to));
	gcode_layer.addPolygonsByOptimizer(layer_thickness, gcode_layer.getLayerNr(), skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);

}

unsigned int FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{

	size_t start_extruder_nr = 0;	//adhesion extruder number
	return start_extruder_nr;

}

void FffGcodeWriter::setConfigFanSpeedLayerTime()
{
	for (int extruders = 0; extruders < 1; extruders++)
	{

		fan_speed_layer_time_settings_per_extruder.emplace_back();

		FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();

		fan_speed_layer_time_settings.cool_min_layer_time = 5.000;
		fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = 10.000;
		fan_speed_layer_time_settings.cool_fan_speed_0 = Ratio(0.000 / 100) * 100.0;
		fan_speed_layer_time_settings.cool_fan_speed_min = Ratio(50.0000 / 100) * 100.0;
		fan_speed_layer_time_settings.cool_fan_speed_max = Ratio(100.00 / 100) * 100.0;
		fan_speed_layer_time_settings.cool_min_speed = 5.000;
		fan_speed_layer_time_settings.cool_fan_full_layer = 6.000;
	}



}

void FffGcodeWriter::setConfigRetraction(SliceDataStorage& storage)
{

	for (size_t extruder_index = 0; extruder_index < 1; extruder_index++)
	{
		//ExtruderTrain& train = storage.extruders[extruder_index];
		RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_index];
		retraction_config.distance = 6.5;// (train.settings.get<bool>("retraction_enable")) ? train.settings.get<double>("retraction_amount") : 0; //Retraction distance in mm.
		retraction_config.prime_volume = 0;// train.settings.get<double>("retraction_extra_prime_amount"); //Extra prime volume in mm^3.
		retraction_config.speed = 25;// train.settings.get<Velocity>("retraction_retract_speed");
		retraction_config.primeSpeed = 15;//train.settings.get<Velocity>("retraction_prime_speed");
		retraction_config.zHop = MM2INT(2);// train.settings.get<coord_t>("retraction_hop");
		retraction_config.retraction_min_travel_distance = MM2INT(5);// train.settings.get<coord_t>("retraction_min_travel");
		retraction_config.retraction_extrusion_window = 1.0;// train.settings.get<double>("retraction_extrusion_window"); //Window to count retractions in in mm of extruded filament.
		retraction_config.retraction_count_max = 10;// train.settings.get<size_t>("retraction_count_max");

		RetractionConfig& switch_retraction_config = storage.extruder_switch_retraction_config_per_extruder[extruder_index];
		switch_retraction_config.distance = 8;// train.settings.get<double>("switch_extruder_retraction_amount"); //Retraction distance in mm.
		switch_retraction_config.prime_volume = 0.0;
		switch_retraction_config.speed = 20;// train.settings.get<Velocity>("switch_extruder_retraction_speed");
		switch_retraction_config.primeSpeed = 15;// train.settings.get<Velocity>("switch_extruder_prime_speed");
		switch_retraction_config.zHop = MM2INT(1);// train.settings.get<coord_t>("retraction_hop_after_extruder_switch_height");
		switch_retraction_config.retraction_min_travel_distance = MM2INT(0); // no limitation on travel distance for an extruder switch retract
		switch_retraction_config.retraction_extrusion_window = 99999.9; // so that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions)
		switch_retraction_config.retraction_count_max = 9999999; // extruder switch retraction is never limited
	}
}

void FffGcodeWriter::setConfigWipe(SliceDataStorage& storage)
{
	for (size_t extruder_index = 0; extruder_index < 1; extruder_index++)
	{

		WipeScriptConfig& wipe_config = storage.wipe_config_per_extruder[extruder_index];

		wipe_config.retraction_enable = true;// train.settings.get<bool>("wipe_retraction_enable");
		wipe_config.retraction_config.distance = 1.0;// train.settings.get<double>("wipe_retraction_amount");
		wipe_config.retraction_config.speed = 3.0;// train.settings.get<Velocity>("wipe_retraction_retract_speed");
		wipe_config.retraction_config.primeSpeed = 2.0;// train.settings.get<Velocity>("wipe_retraction_prime_speed");
		wipe_config.retraction_config.prime_volume = 0.0;// train.settings.get<double>("wipe_retraction_extra_prime_amount");
		wipe_config.retraction_config.retraction_min_travel_distance = 0;
		wipe_config.retraction_config.retraction_extrusion_window = std::numeric_limits<double>::max();
		wipe_config.retraction_config.retraction_count_max = std::numeric_limits<size_t>::max();

		wipe_config.pause = 0.0;// train.settings.get<Duration>("wipe_pause");

		wipe_config.hop_enable = true;// train.settings.get<bool>("wipe_hop_enable");
		wipe_config.hop_amount = MM2INT(1.0);// train.settings.get<coord_t>("wipe_hop_amount");
		wipe_config.hop_speed = 100;// train.settings.get<Velocity>("wipe_hop_speed");

		wipe_config.brush_pos_x = MM2INT(100);// train.settings.get<coord_t>("wipe_brush_pos_x");
		wipe_config.repeat_count = 5;// train.settings.get<size_t>("wipe_repeat_count");
		wipe_config.move_distance = MM2INT(20.0);// train.settings.get<coord_t>("wipe_move_distance");
		wipe_config.move_speed = 120.0;// train.settings.get<Velocity>("speed_travel");
		wipe_config.max_extrusion_mm3 = 10.0;// train.settings.get<double>("max_extrusion_before_wipe");
		wipe_config.clean_between_layers = false;// train.settings.get<bool>("clean_between_layers");
	}
}

void FffGcodeWriter::processStartingCode(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
	std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
	//printf(" the code is at the line 220 \n ");
	std::string prefix = gcode.getFileHeader(extruder_is_used);
	gcode.writeCode(prefix.c_str());
	std::ostringstream tmp;
	tmp << "T" << start_extruder_nr;
	gcode.writeLine(tmp.str().c_str());
	gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
	std::string machinegcode = "G92 E0";
	gcode.writeCode(machinegcode.c_str());
	const double volume_temprature = 0.0;
	prefix = " ";
	gcode.writeCode(prefix.c_str());
	gcode.startExtruder(start_extruder_nr);
	processInitialLayerTemperature(storage, start_extruder_nr);
	double speed_travel = 250.0;
	coord_tIrfan thickness = storage.Layers[0].thickness;
	gcode.writePrimeTrain(speed_travel, thickness);
	extruder_prime_layer_nr[start_extruder_nr] = std::numeric_limits<int>::min(); // set to most negative number so that layer processing never primes this extruder any more.
	const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[start_extruder_nr];
	gcode.writeRetraction(retraction_config);
	gcode.setExtruderFanNumber(start_extruder_nr);
	

}


void FffGcodeWriter::processInitialLayerTemperature(const SliceDataStorage& storage, const size_t start_extruder_nr)
{
	std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
	//Scene& scene = Application::getInstance().current_slice->scene;
	const size_t num_extruders = 2;// scene.extruders.size();
	constexpr bool wait = true;
	const double print_temp_0 = 205;// ("material_print_temperature_layer_0");
	const double print_temp_here = 210;// ("material_print_temperature");
	gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);

}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
	SliceDataStorage* storage1;
	coord_tIrfan layer_thicknees = storage1->getlayer_thickness();
	gcode.writeFanCommand(0);
	gcode.setZ(max_object_height + 5000);
	gcode.writeTravel(gcode.getPositionXY(), 250.0, layer_thicknees);
	curaIrfan::PointIrfan start_pos(storage.model_min.x, storage.model_min.y);
	gcode.writeTravel(start_pos, 250.0, layer_thicknees);


	/*const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	if (mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0") != 0)
	{
		constexpr bool wait = true;
		gcode.writeBedTemperatureCommand(mesh_group_settings.get<Temperature>("material_bed_temperature_layer_0"), wait);
	}
	*/
	processInitialLayerTemperature(storage, gcode.getExtruderNr());
}

void FffGcodeWriter::setInfillAndSkinAngles(SliceDataStorage& storage)
{
	storage.infill_angles.push_back(45.0);
	storage.roofing_angles.push_back(45.0);
	storage.roofing_angles.push_back(135.0);
	storage.skin_angles.push_back(45.0);
	storage.skin_angles.push_back(135.0);
	// generally all infill patterns use 45 degrees
}

void FffGcodeWriter::addMeshLayerToGCode(SliceDataStorage& storage, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
	int mat;

	SliceLayer& layer = storage.Layers[gcode_layer.getLayerNr()];

	if (layer.parts.size() == 0)
	{
		return;
	}

	gcode_layer.setMesh("Mesh1");

	ZSeamConfig z_seam_config(EZSeamType::SHARPEST_CORNER, storage.getZSeamHint(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY);
	const curaIrfan::PointIrfan layer_start_position(MM2INT(213.0), MM2INT(198.0));
	PathOrderOptimizer part_order_optimizer(layer_start_position, z_seam_config);

	for (unsigned int part_idx = 0; part_idx < layer.parts.size(); part_idx++)
	{
		const SliceLayerPart& part = layer.parts[part_idx];
		ConstPolygonRef part_representative = (part.insets.size() > 0) ? part.insets[0][0] : part.outline[0];
		part_order_optimizer.addPolygon(part_representative);

	}

	part_order_optimizer.optimize();

	for (int part_idx : part_order_optimizer.polyOrder)
	{
			int mat = layer.mat_parts[part_idx];
			{
				SliceLayerPart& part = layer.parts[part_idx];
				
				gcode_layer.layer_parts.push_back(part);
				gcode_layer.layer_parts_mat.push_back(mat);
				
				char* extruders = layer.parts[part_idx].getPolygonExtruders();
				
				std::vector<int>LDMIExtruders;
				
				for (extruders; *extruders != '\0'; ++extruders)
				{
					char extrud = *extruders;
					int ext = (int)extrud - 48;

					LDMIExtruders.push_back(ext);

				}

				float lineDistance = 5.0;
				
				addMeshPartToGCode(storage, mesh, mesh_config, part, gcode_layer, LDMIExtruders);
			}
	}
	processIroning(layer, gcode_layer);
	gcode_layer.setMesh("NONMESH");
}

void FffGcodeWriter::setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr) const
{
	const size_t outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];
	bool  prime_blob_enable = true;
	bool extruder_prime_pos_abs = true;
	const size_t previous_extruder = gcode_layer.getExtruder();
	if (previous_extruder == extruder_nr && !(extruder_nr == outermost_prime_tower_extruder
		&& gcode_layer.getLayerNr() >= -static_cast<int>(Raft::getFillerLayerCount()))) //No unnecessary switches, unless switching to extruder for the outer shell of the prime tower.
	{
		return;
	}
	const bool extruder_changed = gcode_layer.setExtruder(extruder_nr);

	if (extruder_changed)
	{
		if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
		{


			// We always prime an extruder, but whether it will be a prime blob/poop depends on if prime blob is enabled.
			// This is decided in GCodeExport::writePrimeTrain().
			if (prime_blob_enable) // Don't travel to the prime-blob position if not enabled though.
			{
				bool prime_pos_is_abs = extruder_prime_pos_abs;
				curaIrfan::PointIrfan prime_pos = curaIrfan::PointIrfan(MM2INT(0), MM2INT(0));

				gcode_layer.addTravel(storage.layer_thickness, gcode_layer.getLayerNr(), prime_pos);
				gcode_layer.planPrime();
			}
		}

		if (gcode_layer.getLayerNr() == 0 && !gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
		{
			processSkirtBrim(storage, gcode_layer, extruder_nr);
		}
	}

	// The first layer of the prime tower is printed with one material only, so do not prime another material on the
	// first layer again.
	if (((extruder_changed && gcode_layer.getLayerNr() > 0) || extruder_nr == outermost_prime_tower_extruder) && gcode_layer.getLayerNr() >= -1) //Always print a prime tower with outermost extruder.
	{
		addPrimeTower(storage, gcode_layer, previous_extruder);
	}
}

bool FffGcodeWriter::processIroning(const SliceLayer& layer, LayerPlan& gcode_layer) const
{
	bool added_something = false;
	return added_something;
}

void FffGcodeWriter::addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcode_layer, int prev_extruder) const
{
	bool prime_tower_enable = false;

	if (prime_tower_enable)
	{
		return;
	}

	const size_t outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];
	if (gcode_layer.getLayerNr() == 0 && outermost_prime_tower_extruder != gcode_layer.getExtruder())
	{
		return;
	}

	storage.primeTower.addToGcode(storage, gcode_layer, prev_extruder, gcode_layer.getExtruder());
}


void FffGcodeWriter::addMeshPartToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, SliceLayerPart& part, LayerPlan& gcode_layer, std::vector<int>extruder_nr) const
{

	bool added_something = false;

	if ((part.part_print_property == 1)&&(part.thirdMaterial!=1))
	{
		if (extruder_nr.size()==1)
		{
			added_something = added_something | processInsets(storage, gcode_layer, extruder_nr, mesh_config, part);
			processOutlineGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, added_something);
			added_something = added_something | processInfill(storage, mesh, gcode_layer, extruder_nr, mesh_config, part);
			added_something = added_something | processSkinAndPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
		}
		else
		{

			added_something = added_something | processInfill(storage, mesh, gcode_layer, extruder_nr, mesh_config, part);

		}
	}

	coord_tIrfan layer_thickness = storage.Layers[0].thickness;
	bool  magic_spiralize = false;

	int bottom_layers = 6;

	if (added_something && (!magic_spiralize) || gcode_layer.getLayerNr() < bottom_layers)
	{
		coord_tIrfan innermost_wall_line_width = MM2INT(0.3);// mesh.settings.get<coord_tIrfan>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
		if (gcode_layer.getLayerNr() == 0)
		{
			innermost_wall_line_width *= Ratio(120 / 100);// mesh.settings.get<Ratio>("initial_layer_line_width_factor");
		}
		gcode_layer.moveInsideCombBoundary(gcode_layer.getLayerNr(), innermost_wall_line_width);
	}
	gcode_layer.setIsInside(false);
}

void FffGcodeWriter::processOutlineGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int>extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, bool& added_something) const
{
	/*
	size_t wall_0_extruder_nr = extruder_nr;
	bool fill_outline_gaps = false;
	if (extruder_nr != wall_0_extruder_nr || !fill_outline_gaps)
	{
		return;
	}
	
	const coord_t perimeter_gaps_line_width = mesh_config.perimeter_gap_config.getLineWidth();
	int skin_angle = 45;
	if (mesh.skin_angles.size() > 0)
	{
		skin_angle = mesh.skin_angles.at(gcode_layer.getLayerNr() % mesh.skin_angles.size());
	}
	Polygons gap_polygons; // unused
	Polygons gap_lines; // soon to be generated gap filler lines
	int offset = 0;
	int extra_infill_shift = 0;
	constexpr coord_t outline_gap_overlap = 0;
	constexpr int infill_multiplier = 1;
	constexpr bool zig_zaggify_infill = false;
	constexpr bool connect_polygons = false; // not applicable

	constexpr int wall_line_count = 0;
	const Point& infill_origin = Point();
	Polygons* perimeter_gaps = nullptr;
	constexpr bool connected_zigzags = false;
	constexpr bool use_endpieces = true;
	constexpr bool skip_some_zags = false;
	constexpr int zag_skip_count = 0;
	constexpr coord_t pocket_size = 0;

	Infill infill_comp(
		EFillMethod::LINES, zig_zaggify_infill, connect_polygons, part.outline_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, outline_gap_overlap, infill_multiplier, skin_angle, gcode_layer.z, extra_infill_shift,
		wall_line_count, infill_origin, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
	);
	infill_comp.generate(gap_polygons, gap_lines);

	if (gap_lines.size() > 0)
	{
		assert(extruder_nr == wall_0_extruder_nr); // Should already be the case because of fill_perimeter_gaps check
		added_something = true;
		setExtruder_addPrime(storage, gcode_layer, extruder_nr);
		gcode_layer.setIsInside(false); // going to print stuff outside print object
		gcode_layer.addLinesByOptimizer(gap_lines, mesh_config.perimeter_gap_config, SpaceFillType::Lines);
	}
	*/
}




bool FffGcodeWriter::processInfill(const SliceDataStorage& storage, const SliceMeshStorage& mesh, LayerPlan& gcode_layer, std::vector<int>extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{

	//bool added_something = processMultiLayerInfill(storage, mesh, gcode_layer, extruder_nr, mesh_config, part, mat);
	bool added_something = processSingleLayerInfill(storage, mesh_config, gcode_layer, extruder_nr, part);
	return added_something;


}

bool FffGcodeWriter::processMultiLayerInfill(const SliceDataStorage& storage, const SliceMeshStorage& mesh, LayerPlan& gcode_layer, std::vector<int>extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, std::vector<int>mat) const
{

	//extruder_nr == 0;
	const coord_tIrfan infill_line_distance = MM2INT(1.3);
	const coord_tIrfan infill_overlap = MM2INT(0.04);// mesh.settings.get<coord_t>("infill_overlap_mm");
	double infill_angle = 45.0; //Original default. This will get updated to an element from mesh->infill_angles.

	if (!storage.infill_angles.empty())
	{
		const size_t combined_infill_layers = std::max(unsigned(1), round_divide(MM2INT(0.2), std::max(MM2INT(0.01 * 15), coord_tIrfan(1))));
		infill_angle = storage.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % storage.infill_angles.size());
	}


	Point3 mesh_middle = storage.bounding_box.getMiddle();
	const curaIrfan::PointIrfan infill_origin(mesh_middle.x, mesh_middle.y);
	EFillMethod infill_pattern = EFillMethod::TRIANGLES;

	bool added_something = false;


	for (unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
	{

		int layernum = gcode_layer.getLayerNr();
		const coord_tIrfan layer_thickness = storage.Layers[0].thickness;
		const coord_tIrfan infill_line_width = mesh_config.infill_config[combine_idx].getLineWidth();
		const EFillMethod infill_pattern = EFillMethod::TRIANGLES;
		const bool zig_zaggify_infill = false;// mesh.settings.get<bool>("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
		const bool connect_polygons = true;// mesh.settings.get<bool>("connect_infill_polygons");
		const size_t infill_multiplier = 1;// mesh.settings.get<size_t>("infill_multiplier");
		Polygons infill_polygons;
		Polygons infill_lines;
		for (size_t density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
		{
			size_t density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
			coord_tIrfan infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
			coord_tIrfan infill_shift = infill_line_distance_here / 2;
			if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
			{
				infill_line_distance_here /= 2;
			}
			constexpr size_t wall_line_count = 0; // wall lines are always single layer
			Polygons* perimeter_gaps = nullptr;
			constexpr bool connected_zigzags = false;
			constexpr bool use_endpieces = true;
			constexpr bool skip_some_zags = false;
			constexpr size_t zag_skip_count = 0;
			//printf("line 422 in fffGCODEwriter.cpp \n");
			Infill infill_comp(infill_pattern, zig_zaggify_infill, connect_polygons, part.infill_area_per_combine_per_density[density_idx][combine_idx], /*outline_offset =*/ 0
				, infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
				, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count
				, MM2INT(2.0));

			infill_comp.generate(infill_polygons, infill_lines);
		}

		if (!infill_lines.empty())
		{
			added_something = true;
			//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
			gcode_layer.setIsInside(true); // going to print stuff inside print object
			const bool enable_travel_optimization = false;//mesh.settings.get<bool>("infill_enable_travel_optimization");
			//gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], zig_zaggify_infill ? SpaceFillType::PolyLines : SpaceFillType::Lines, enable_travel_optimization);
			//gcode_layer.addLinesByOptimizer(layer_thickness, mesh_config.infill_config[combine_idx], infill_lines, layernum, 2, SpaceFillType::Lines, enable_travel_optimization);
		}
	}
	//printf("line  in 445 fffGCODEwriter.cpp %d \n", part.infill_area_per_combine_per_density.size());
	return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(const SliceDataStorage& storage, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer, std::vector<int>extruder_nr, const SliceLayerPart& part) const
{

	if (part.infill_area_per_combine_per_density[0].size() == 0)
	{
		printf("@@error in single layer infill for the layer %d \n", gcode_layer.getLayerNr());
		return false;
	}
	bool added_something = false;
	const coord_tIrfan infill_line_width = mesh_config.infill_config[0].getLineWidth();

	const EFillMethod pattern = EFillMethod::TRIANGLES;
	const bool zig_zaggify_infill = true;// mesh.settings.get<bool>("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
	const bool connect_polygons = false;// mesh.settings.get<bool>("connect_infill_polygons");
	const coord_tIrfan infill_overlap = MM2INT(0.0);//sh.settings.get<coord_tIrfan>("infill_overlap_mm");
	const size_t infill_multiplier = 1;// mesh.settings.get<size_t>("infill_multiplier");
	const size_t wall_line_count = 0;
	double infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.

	//if (storage.infill_angles.size() > 0)
	//{
	//	const size_t combined_infill_layers = 1;// std::max(unsigned(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
	//	infill_angle = storage.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % storage.infill_angles.size());
	//}

	const Point3 mesh_middle = storage.bounding_box.getMiddle();
	curaIrfan::PointIrfan infill_origin(mesh_middle.x, mesh_middle.y);
	for (unsigned int density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
	{

		//int infill_line_distance_here = infill_line_distance << (density_idx + 1); // the highest density infill combines with the next to create a grid with density_factor 1
		//int infill_shift = infill_line_distance_here / 2;

		//if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
		//{
		//	infill_line_distance_here /= 2;
		//}

		//Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];	 //part.outline;

		//const coord_tIrfan circumference = in_outline.polygonLength();
		for (int ext = 1; ext <= extruder_nr.size(); ext++)
		{

			Polygons infill_polygons;
			Polygons infill_lines;
			coord_tIrfan infill_line_distance;
			if (extruder_nr.size() == 1)
			{
				infill_line_distance = MM2INT(2.5);
			}	
			else
			{
				infill_line_distance = MM2INT(extruder_nr.size()*2.5);
			}

			int infill_line_distance_here = infill_line_distance << (density_idx + 1);
			
			int infill_shift = infill_line_distance_here / 2;
			
			

			if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
			{
				infill_line_distance_here /= 2;
			}

			Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];	 //part.outline;

			const coord_tIrfan circumference = in_outline.polygonLength();

			const double minimum_small_area = 0.4 * 0.4 * circumference / 40000;
			in_outline.removeSmallAreas(minimum_small_area);
			coord_tIrfan outline_offset = 0;
			
			if ((extruder_nr.size() == 2) && (ext ==extruder_nr.at(1)))
			{
				
				infill_shift += infill_line_distance_here / 2;


			}
			if ((extruder_nr.size() == 3) && (ext == extruder_nr.at(1)))
			{
				infill_shift += infill_line_distance_here / 1.6;
			
			}
			if ((extruder_nr.size() == 3) && (ext == extruder_nr.at(2)))
			{
				infill_shift += infill_line_distance_here/3.4;
				
			}
			
			Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, in_outline, 0, infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
				, /*Polygons* perimeter_gaps =*/ false
				, /*bool connected_zigzags =*/ false
				, /*bool use_endpieces =*/ false
				, /*bool skip_some_zags =*/ false
				, /*int zag_skip_count =*/ 0
				, MM2INT(2.0));
			infill_comp.generate(infill_polygons, infill_lines);
			infill_lines.print_extruder = extruder_nr.at(ext-1);
			coord_tIrfan layer_thickness = mesh_config.infill_config[0].getLayerThickness();
			int layernum = gcode_layer.getLayerNr();

			//printf("the infill lines size is %d and the layer number is %d \n", infill_lines.size(), gcode_layer.getLayerNr());
			if (infill_lines.size() > 0)
			{
				added_something = true;
				//setExtruder_addPrime(storage, gcode_layer, extruder_nr);	not needed
				gcode_layer.setIsInside(true); // going to print stuff inside print object
				const bool enable_travel_optimization = false;// mesh.settings.get<bool>("infill_enable_travel_optimization");
				//printf("outside the lineoptimizer \n");
				
				gcode_layer.addLinesByOptimizer(layer_thickness, mesh_config.infill_config[0], infill_lines, layernum, SpaceFillType::Lines, enable_travel_optimization);
				//gcode_layer.addLinesByOptimizer(in/fill_lines,SpaceFillType::Lines, enable_travel_optimization,0);

			}
			infill_shift = 0;
			

		}
		
		
		//else if ((part.part_mat == 1) || (part.part_mat == 2) || (part.part_mat == 3))
		//{
		//	Polygons infill_polygons;
		//	Polygons infill_lines;

		//	coord_tIrfan infill_line_distance = MM2INT(2.5);
		//	int infill_line_distance_here = infill_line_distance << (density_idx + 1); // the highest density infill combines with the next to create a grid with density_factor 1
		//	int infill_shift = infill_line_distance_here / 2;

		//	if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
		//	{
		//		infill_line_distance_here /= 2;
		//	}

		//	Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];	 //part.outline;

		//	const coord_tIrfan circumference = in_outline.polygonLength();

		//	const double minimum_small_area = 0.4 * 0.4 * circumference / 40000;
		//	in_outline.removeSmallAreas(minimum_small_area);
		//	coord_tIrfan outline_offset = 0;

		//	Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, in_outline, 0, infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
		//		, /*Polygons* perimeter_gaps =*/ false
		//		, /*bool connected_zigzags =*/ false
		//		, /*bool use_endpieces =*/ false
		//		, /*bool skip_some_zags =*/ false
		//		, /*int zag_skip_count =*/ 0
		//		, MM2INT(2.0));
		//	infill_comp.generate(infill_polygons, infill_lines);
		//	if (part.part_mat == 3)
		//	{
		//		infill_lines.print_extruder = 1;// Assignment of the extruder to the infill paths
		//	}
		//	if ((part.part_mat == 2) && (extruder_nr == 2))
		//	{
		//		infill_lines.print_extruder = 2;// Assignment of the extruder to the infill paths
		//	}
		//	if ((part.part_mat == 2) || (part.part_mat == 1))
		//	{
		//		infill_lines.print_extruder = 0;// Assignment of the extruder to the infill paths
		//	}

		//	//infill_lines.id = 0;
		//	coord_tIrfan layer_thickness = mesh_config.infill_config[0].getLayerThickness();
		//	int layernum = gcode_layer.getLayerNr();

		//	//printf("the infill lines size is %d and the layer number is %d \n", infill_lines.size(), gcode_layer.getLayerNr());
		//	if (infill_lines.size() > 0)
		//	{
		//		added_something = true;
		//		//setExtruder_addPrime(storage, gcode_layer, extruder_nr);	not needed
		//		gcode_layer.setIsInside(true); // going to print stuff inside print object
		//		const bool enable_travel_optimization = false;// mesh.settings.get<bool>("infill_enable_travel_optimization");
		//		//printf("outside the lineoptimizer \n");
		//		gcode_layer.addLinesByOptimizer(layer_thickness, mesh_config.infill_config[0], infill_lines, layernum, mat, SpaceFillType::Lines, mat, enable_travel_optimization);
		//		//gcode_layer.addLinesByOptimizer(in/fill_lines,SpaceFillType::Lines, enable_travel_optimization,0);

		//	}
		//}


	}



	return added_something;
}

bool FffGcodeWriter::processInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, std::vector<int> extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
	/*if (extruder_nr != 0)
	{
		return false;
	}*/

	bool added_something = false;
	const bool compensate_overlap_0 = true;// mesh.settings.get<bool>("travel_compensate_overlapping_walls_0_enabled");
	const bool compensate_overlap_x = true;// mesh.settings.get<bool>("travel_compensate_overlapping_walls_x_enabled");
	const bool retract_before_outer_wall = true;// mesh.settings.get<bool>("travel_retract_before_outer_wall");
	size_t wall_line_count = 1;
	if (wall_line_count > 0)
	{

		if (gcode_layer.getLayerNr() > 0)
		{
			Polygons outlines_below;
			AABB boundaryBox(part.outline);
			for (const SliceMeshStorage& m : storage.meshes)
			{
				for (const SliceLayerPart& prevLayerPart : storage.Layers[gcode_layer.getLayerNr() - 1].parts)
				{
					if (boundaryBox.hit(prevLayerPart.boundaryBox))
					{
						outlines_below.add(prevLayerPart.outline);
					}
				}

			}

			const coord_tIrfan layer_height = mesh_config.inset0_config.getLayerThickness();

			// if support is enabled, add the support outlines also so we don't generate bridges over support

			const coord_tIrfan z_distance_top = MM2INT(0.4);//	mesh.settings.get<coord_t>("support_top_distance");
			const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1;
			const int support_layer_nr = gcode_layer.getLayerNr() - z_distance_top_layers;

			if (support_layer_nr > 0)
			{
				const SupportLayer& support_layer = storage.support.supportLayers[support_layer_nr];

				if (!support_layer.support_roof.empty())
				{
					AABB support_roof_bb(support_layer.support_roof);
					if (boundaryBox.hit(support_roof_bb))
					{
						outlines_below.add(support_layer.support_roof);
					}
				}
				else
				{
					for (const SupportInfillPart& support_part : support_layer.support_infill_parts)
					{
						AABB support_part_bb(support_part.getInfillArea());
						if (boundaryBox.hit(support_part_bb))
						{
							outlines_below.add(support_part.getInfillArea());
						}
					}
				}
			}


			const int half_outer_wall_width = mesh_config.inset0_config.getLineWidth() / 2;

			// remove those parts of the layer below that are narrower than a wall line width as they will not be printed

			outlines_below = outlines_below.offset(-half_outer_wall_width).offset(half_outer_wall_width);
			gcode_layer.setBridgeWallMask(Polygons());

			const double overhang_angle = 90;			//Angle Degrees means angle in degrees
			if (overhang_angle >= 90)
			{
				// clear to disable overhang detection
				gcode_layer.setOverhangMask(Polygons());
			}

		}
		else
		{
			// clear to disable use of bridging settings
			gcode_layer.setBridgeWallMask(Polygons());
			// clear to disable overhang detection
			gcode_layer.setOverhangMask(Polygons());
		}

		// Only spiralize the first part in the mesh, any other parts will be printed using the normal, non-spiralize codepath.
		// This sounds weird but actually does the right thing when you have a model that has multiple parts at the bottom that merge into
		// one part higher up. Once all the parts have merged, layers above that level will be spiralized
		/*
		if (InsetOrderOptimizer::optimizingInsetsIsWorthwhile(part))
		{
			InsetOrderOptimizer ioo(*this, storage, gcode_layer, extruder_nr, mesh_config, part, gcode_layer.getLayerNr());
			return ioo.processInsetsWithOptimizedOrdering();
		}
*/		//else
		{
			bool outer_inset_first = false;
			outer_inset_first = outer_inset_first || (gcode_layer.getLayerNr() == 0);
			int processed_inset_number = -1;

			for (int inset_number = part.insets.size() - 1; inset_number > -1; inset_number--)
			{
				processed_inset_number = inset_number;
				if (outer_inset_first)
				{
					processed_inset_number = part.insets.size() - 1 - inset_number;
				}
				// Outer wall is processed
				if (processed_inset_number == 0)
				{
					constexpr float flow = 1.0;
					if (part.insets[0].size() > 0)
					{
						added_something = true;
						//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
						gcode_layer.setIsInside(true); // going to print stuff inside print object
						ZSeamConfig z_seam_config(EZSeamType::SHARPEST_CORNER, storage.getZSeamHint(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER);
						Polygons outer_wall;
						if (part.insets.size() > 3)
						{
							Polygons outer_wall = part.insets[3];
						}
						if (part.insets.size() <= 3)
						{
							Polygons outer_wall = part.insets[0];
						}
						if (!compensate_overlap_0)
						{
							WallOverlapComputation* wall_overlap_computation(nullptr);
							gcode_layer.addWalls(outer_wall, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlap_computation,extruder_nr, z_seam_config, MM2INT(0.2), flow, retract_before_outer_wall);
						}
						else
						{

							WallOverlapComputation wall_overlap_computation(outer_wall, mesh_config.inset0_config.getLineWidth());
							if (gcode_layer.layer_parts_mat.back() == 5)
							{
								outer_wall.print_extruder == 1;
							}
							gcode_layer.addWalls(outer_wall, mesh_config.inset0_config, mesh_config.bridge_inset0_config, &wall_overlap_computation, extruder_nr, z_seam_config, MM2INT(0.2), flow, retract_before_outer_wall);
						}
					}
				}
				// Inner walls are processed
				else if (!part.insets[processed_inset_number].empty())
				{
					added_something = true;
					//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
					gcode_layer.setIsInside(true); // going to print stuff inside print object
					ZSeamConfig z_seam_config(EZSeamType::SHARPEST_CORNER, storage.getZSeamHint(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER);
					Polygons inner_wall = part.insets[processed_inset_number];
					if (!compensate_overlap_x)
					{
						WallOverlapComputation* wall_overlap_computation(nullptr);
						gcode_layer.addWalls(part.insets[processed_inset_number], mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlap_computation, extruder_nr, z_seam_config );
					}
					else
					{
						if (gcode_layer.layer_parts_mat.back() == 5)
						{
							inner_wall.print_extruder == 1;
						}
						WallOverlapComputation wall_overlap_computation(inner_wall, mesh_config.inset0_config.getLineWidth());
						gcode_layer.addWalls(inner_wall, mesh_config.insetX_config, mesh_config.bridge_insetX_config, &wall_overlap_computation, extruder_nr, z_seam_config );
					}
				}
			}
		}
	}
	return added_something;
}

bool FffGcodeWriter::processSkinAndPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int>extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
	const size_t top_bottom_extruder_nr = extruder_nr.at(0);
	const size_t roofing_extruder_nr = extruder_nr.at(0);
	const size_t wall_0_extruder_nr = extruder_nr.at(0);
	const size_t roofing_layer_count = extruder_nr.at(0);
	/*if (extruder_nr != top_bottom_extruder_nr && extruder_nr.at(0) != wall_0_extruder_nr
		&& (extruder_nr != roofing_extruder_nr || roofing_layer_count <= 0))
	{
		return false;
	}
	*/
	bool added_something = false;
	bool spiralize = false;
	const bool fill_perimeter_gaps = true;		  // extruder_nr can be changed for different settigs (Hardcoding)


	PathOrderOptimizer part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());



	for (unsigned int skin_part_idx = 0; skin_part_idx < part.skin_parts.size(); skin_part_idx++)
	{
		const PolygonsPart& outline = part.skin_parts[skin_part_idx].outline;
		part_order_optimizer.addPolygon(outline.outerPolygon());
	}
	part_order_optimizer.optimize();

	for (int ordered_skin_part_idx : part_order_optimizer.polyOrder)
	{
		const SkinPart& skin_part = part.skin_parts[ordered_skin_part_idx];

		processSkinInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);
		//printf("Done with the process Skin insets \n");
		added_something = added_something | processSkinPart(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part);
		//printf("Done with the process Skin parts for this layer \n");

	}
	//printf("Done with the process Skin Parts \n");

	if (fill_perimeter_gaps)
	{ // handle perimeter gaps of normal insets
		assert(extruder_nr.at(0) == wall_0_extruder_nr); // Should already be the case because of fill_perimeter_gaps check
		processPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, part.perimeter_gaps, mesh_config.perimeter_gap_config, added_something);

	}

	return added_something;
}

bool FffGcodeWriter::processSkinPart(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int> extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part) const
{
	bool added_something = false;

	gcode_layer.mode_skip_agressive_merge = true;

	// add roofing
	Polygons roofing_concentric_perimeter_gaps; // the perimeter gaps of the insets of concentric skin pattern of this skin part
	//processRoofing(storage, gcode_layer, mesh, extruder_nr,mat, mesh_config, skin_part, roofing_concentric_perimeter_gaps, added_something);

	// add normal skinfill
	Polygons top_bottom_concentric_perimeter_gaps; // the perimeter gaps of the insets of concentric skin pattern of this skin part
	processTopBottom(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, top_bottom_concentric_perimeter_gaps, added_something);

	// handle perimeter_gaps of concentric skin
	{
		Polygons perimeter_gaps = top_bottom_concentric_perimeter_gaps;
		//perimeter_gaps.add(roofing_concentric_perimeter_gaps);
		//if (extruder_nr == 0)
		{
			perimeter_gaps.add(skin_part.perimeter_gaps);
		}
		perimeter_gaps.unionPolygons();

		processPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, perimeter_gaps, mesh_config.perimeter_gap_config, added_something);
	}

	gcode_layer.mode_skip_agressive_merge = false;
	return added_something;
}

void FffGcodeWriter::processSkinInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int> extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
	const size_t skin_extruder_nr = extruder_nr.at(0);
	// add skin walls aka skin perimeters
	//if (extruder_nr == skin_extruder_nr)
	{
		for (const Polygons& skin_perimeter : skin_part.insets)
		{
			if (skin_perimeter.size() > 0)
			{
				added_something = true;
				//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
				gcode_layer.setIsInside(true); // going to print stuff inside print object
				gcode_layer.addWalls(skin_perimeter, mesh_config.skin_config, mesh_config.bridge_skin_config, nullptr,extruder_nr); // add polygons to gcode in inward order
			}
		}
	}
}



void FffGcodeWriter::processTopBottom(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int> extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, Polygons& concentric_perimeter_gaps, bool& added_something) const
{
	const size_t top_bottom_extruder_nr = extruder_nr.at(0);
	if (extruder_nr.at(0) != top_bottom_extruder_nr)
	{
		return;
	}


	const bool generate_perimeter_gaps = true;
	//mesh.settings.get<FillPerimeterGapMode>("fill_perimeter_gaps") != FillPerimeterGapMode::NOWHERE
	//&& !mesh_group_settings.get<bool>("magic_spiralize");

	const size_t layer_nr = gcode_layer.getLayerNr();

	EFillMethod pattern = EFillMethod::LINES;
	AngleDegrees skin_angle = 45;
	bool skin_alternate_rotation = true;
	size_t top_layers = 4;
	size_t bottom_layers = 6;
	skin_alternate_rotation = skin_alternate_rotation && (top_layers >= 2 || bottom_layers >= 2);
	if (skin_alternate_rotation && (layer_nr / 2) & 1)
	{
		skin_angle -= 45;
	}

	// generate skin_polygons and skin_lines (and concentric_perimeter_gaps if needed)
	const GCodePathConfig* skin_config = &mesh_config.skin_config;
	Ratio skin_density = 1.0;

	coord_tIrfan skin_overlap = MM2INT(0.25);// mesh.settings.get<coord_t>("skin_overlap_mm");
	const coord_tIrfan more_skin_overlap = std::max(skin_overlap, (coord_tIrfan)(mesh_config.insetX_config.getLineWidth() / 2)); // force a minimum amount of skin_overlap
	const bool bridge_settings_enabled = false;// mesh.settings.get<bool>("bridge_settings_enabled");
	bool bridge_enable_more_layers = true;
	bridge_enable_more_layers = bridge_settings_enabled && bridge_enable_more_layers;
	const Ratio support_threshold = bridge_settings_enabled ? Ratio(50 / 100) : 0.0_r;

	bool support_tree_enable = false;
	bool support_enable = false;

	// if support is enabled, consider the support outlines so we don't generate bridges over support

	int support_layer_nr = -1;
	const SupportLayer* support_layer = nullptr;

	if (support_enable || support_tree_enable)
	{
		const coord_tIrfan layer_height = mesh_config.inset0_config.getLayerThickness();
		const coord_tIrfan z_distance_top = MM2INT(0.4);// mesh.settings.get<coord_t>("support_top_distance");
		const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1;
		support_layer_nr = layer_nr - z_distance_top_layers;
	}

	// helper function that detects skin regions that have no support and modifies their print settings (config, line angle, density, etc.)

	/*
	auto handle_bridge_skin = [&](const int bridge_layer, const GCodePathConfig* config, const float density) // bridge_layer = 1, 2 or 3
	{
		if (support_layer_nr >= (bridge_layer - 1))
		{
			support_layer = &storage.support.supportLayers[support_layer_nr - (bridge_layer - 1)];
		}

		// for upper bridge skins, outline used is union of current skin part and those skin parts from the 1st bridge layer that overlap the curent skin part

		// this is done because if we only use skin_part.outline for this layer and that outline is different (i.e. smaller) than
		// the skin outline used to compute the bridge angle for the first skin, the angle computed for this (second) skin could
		// be different and we would prefer it to be the same as computed for the first bridge layer
		Polygons skin_outline(skin_part.outline);

		if (bridge_layer > 1)
		{
			for (const SliceLayerPart& layer_part : storage.Layers[layer_nr - (bridge_layer - 1)].parts)
			{
				for (const SkinPart& other_skin_part : layer_part.skin_parts)
				{
					if (PolygonUtils::polygonsIntersect(skin_part.outline.outerPolygon(), other_skin_part.outline.outerPolygon()))
					{
						skin_outline = skin_outline.unionPolygons(other_skin_part.outline);
					}
				}
			}
		}

		Polygons supported_skin_part_regions;

		const int angle = bridgeAngle(skin_part.outline, storage, layer_nr - bridge_layer, support_layer, supported_skin_part_regions);

		if (angle > -1 || (supported_skin_part_regions.area() / (skin_part.outline.area() + 1) < support_threshold))
		{
			if (angle > -1)
			{
				switch (bridge_layer)
				{
				default:
				case 1:
					skin_angle = angle;
					break;

				case 2:
					if (bottom_layers > 2)
					{
						// orientate second bridge skin at +45 deg to first
						skin_angle = angle + 45;
					}
					else
					{
						// orientate second bridge skin at 90 deg to first
						skin_angle = angle + 90;
					}
					break;

				case 3:
					// orientate third bridge skin at 135 (same result as -45) deg to first
					skin_angle = angle + 135;
					break;
				}
			}
			pattern = EFillMethod::LINES; // force lines pattern when bridging
			if (bridge_settings_enabled)
			{
				skin_config = config;
				skin_overlap = more_skin_overlap;
				skin_density = density;
			}
			return true;
		}

		return false;
	};

	bool is_bridge_skin = false;
	if (layer_nr > 0)
	{
		is_bridge_skin = handle_bridge_skin(1, &mesh_config.bridge_skin_config, Ratio(100 / 100));// mesh.settings.get<Ratio>("bridge_skin_density"));
	}
	if (bridge_enable_more_layers && !is_bridge_skin && layer_nr > 1 && bottom_layers > 1)
	{
		is_bridge_skin = handle_bridge_skin(2, &mesh_config.bridge_skin_config2, Ratio(75 / 100)); //mesh.settings.get<Ratio>("bridge_skin_density_2"));

		if (!is_bridge_skin && layer_nr > 2 && bottom_layers > 2)
		{
			is_bridge_skin = handle_bridge_skin(3, &mesh_config.bridge_skin_config3, Ratio(80 / 100)); //mesh.settings.get<Ratio>("bridge_skin_density_3"));
		}
	}
	*/
	double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
	bool support_fan_enable = false;
	/*
	if (layer_nr > 0 && skin_config == &mesh_config.skin_config && support_layer_nr >= 0 && support_fan_enable)
	{
		// skin isn't a bridge but is it above support and we need to modify the fan speed?

		AABB skin_bb(skin_part.outline);

		support_layer = &storage.support.supportLayers[support_layer_nr];

		bool supported = false;

		if (!support_layer->support_roof.empty())
		{
			AABB support_roof_bb(support_layer->support_roof);
			if (skin_bb.hit(support_roof_bb))
			{
				supported = !skin_part.outline.intersection(support_layer->support_roof).empty();
			}
		}
		else
		{
			for (auto support_part : support_layer->support_infill_parts)
			{
				AABB support_part_bb(support_part.getInfillArea());
				if (skin_bb.hit(support_part_bb))
				{
					supported = !skin_part.outline.intersection(support_part.getInfillArea()).empty();

					if (supported)
					{
						break;
					}
				}
			}
		}

		if (supported)
		{
			fan_speed = Ratio(100/100) * 100.0;
		}
	}
	*/
	// calculate polygons and lines
	Polygons* perimeter_gaps_output = (generate_perimeter_gaps) ? &concentric_perimeter_gaps : nullptr;

	int skin_part_mat = skin_part.SkinPart_mat;
	processSkinPrintFeature(storage, gcode_layer, mesh, extruder_nr, skin_part_mat, skin_part.inner_infill, *skin_config, pattern, skin_angle, skin_overlap, skin_density, perimeter_gaps_output, added_something, fan_speed);

}

void FffGcodeWriter::processSkinPrintFeature(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int> extruder_nr, int skin_part_mat, const Polygons& area, const GCodePathConfig& config, EFillMethod pattern, const AngleDegrees skin_angle, const coord_tIrfan skin_overlap, const Ratio skin_density, Polygons* perimeter_gaps_output, bool& added_something, double fan_speed) const
{
	Polygons skin_polygons;
	Polygons skin_lines;

	constexpr int infill_multiplier = 1;
	constexpr int extra_infill_shift = 0;
	constexpr int wall_line_count = 0;
	constexpr coord_tIrfan offset_from_inner_skin_infill = 0;
	const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
	const bool connect_polygons = false;
	const curaIrfan::PointIrfan infill_origin;
	constexpr bool connected_zigzags = false;
	constexpr bool use_endpieces = true;
	constexpr bool skip_some_zags = false;
	constexpr int zag_skip_count = 0;
	constexpr coord_tIrfan pocket_size = 0;
	
	Infill infill_comp(
		pattern, zig_zaggify_infill, connect_polygons, area, offset_from_inner_skin_infill, 350, (350) / skin_density, skin_overlap, infill_multiplier, skin_angle, gcode_layer.z, extra_infill_shift, wall_line_count, infill_origin, perimeter_gaps_output,
		connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
	);
	infill_comp.generate(skin_polygons, skin_lines);
	
	// add paths
	if (skin_polygons.size() > 0 || skin_lines.size() > 0)
	{
		added_something = true;
		//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
		gcode_layer.setIsInside(true); // going to print stuff inside print object
		if (!skin_polygons.empty())
		{
			constexpr bool force_comb_retract = false;
			gcode_layer.addTravel(storage.layer_thickness, gcode_layer.getLayerNr(), skin_polygons[0][0], force_comb_retract);
			gcode_layer.addPolygonsByOptimizer(storage.layer_thickness, gcode_layer.getLayerNr(), skin_polygons, config);
		}

		std::optional<curaIrfan::PointIrfan> near_start_location;
		const EFillMethod pattern = EFillMethod::LINES;
		if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
		{ // update near_start_location to a location which tries to avoid seams in skin
			near_start_location = getSeamAvoidingLocation(area, skin_angle, gcode_layer.getLastPlannedPositionOrStartingPosition());
		}

		constexpr bool enable_travel_optimization = false;
		constexpr float flow = 1.0;

		if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
		{
			if (skin_part_mat == 5)	 //5 is the index of the iterferece material
			{
				skin_lines.print_extruder = extruder_nr.at(0);	  // extruder will be 1 if the material for the part is interference material
			}

			gcode_layer.addLinesByOptimizer(storage.layer_thickness, config, skin_lines, gcode_layer.getLayerNr(), SpaceFillType::Lines, enable_travel_optimization, /*infill_wipe_dist*/ 0, flow, near_start_location, fan_speed);
		}
		else
		{
			SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
			constexpr coord_tIrfan wipe_dist = 0;

			gcode_layer.addLinesByOptimizer(storage.layer_thickness, config, skin_lines, gcode_layer.getLayerNr(), space_fill_type, enable_travel_optimization, wipe_dist, flow, near_start_location, fan_speed);
		}
	}
}

std::optional<curaIrfan::PointIrfan> FffGcodeWriter::getSeamAvoidingLocation(const Polygons& filling_part, int filling_angle, curaIrfan::PointIrfan last_position) const
{
	if (filling_part.empty())
	{
		return std::optional<curaIrfan::PointIrfan>();
	}
	// start with the BB of the outline
	AABB skin_part_bb(filling_part);
	curaIrfan::PointMatrix rot((double)((-filling_angle + 90) % 360)); // create a matrix to rotate a vector so that it is normal to the skin angle
	const curaIrfan::PointIrfan bb_middle = skin_part_bb.getMiddle();
	// create a vector from the middle of the BB whose length is such that it can be rotated
	// around the middle of the BB and the end will always be a long way outside of the part's outline
	// and rotate the vector so that it is normal to the skin angle
	const curaIrfan::PointIrfan vec = rot.apply(curaIrfan::PointIrfan(0, curaIrfan::vSize(curaIrfan::operator-(skin_part_bb.max, bb_middle)) * 100));
	// find the vertex in the outline that is closest to the end of the rotated vector
	const PolygonsPointIndex pa = PolygonUtils::findNearestVert(curaIrfan::operator+(bb_middle, vec), filling_part);
	// and find another outline vertex, this time using the vector + 180 deg
	const  PolygonsPointIndex pb = PolygonUtils::findNearestVert(curaIrfan::operator-(bb_middle, vec), filling_part);
	if (!pa.initialized() || !pb.initialized())
	{
		return std::optional<curaIrfan::PointIrfan>();
	}
	// now go to whichever of those vertices that is closest to where we are now
	if (curaIrfan::vSize2(curaIrfan::operator-(pa.p(), last_position)) < curaIrfan::vSize2(curaIrfan::operator-(pb.p(), last_position)))
	{
		bool bs_arg = true;
		return std::optional<curaIrfan::PointIrfan>(bs_arg, pa.p());
	}
	else
	{
		bool bs_arg = true;
		return std::optional<curaIrfan::PointIrfan>(bs_arg, pb.p());
	}
}

void FffGcodeWriter::processPerimeterGaps(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, std::vector<int> extruder_nr, const Polygons& perimeter_gaps, const GCodePathConfig& perimeter_gap_config, bool& added_something) const
{
	const coord_tIrfan perimeter_gaps_line_width = perimeter_gap_config.getLineWidth();


	assert(storage.roofing_angles.size() > 0);
	const bool zig_zaggify_infill = false;
	const bool connect_polygons = false; // not applicable
	int perimeter_gaps_angle = 45; // use roofing angles for perimeter gaps
	const size_t layer_nr = gcode_layer.getLayerNr();
	bool perimeter_gaps_alternate_rotation = true;
	size_t top_layers = 4;
	size_t bottom_layers = 6;
	perimeter_gaps_alternate_rotation = perimeter_gaps_alternate_rotation && (top_layers >= 2 || bottom_layers >= 2);
	if (perimeter_gaps_alternate_rotation && (layer_nr / 2) & 1)
	{
		perimeter_gaps_angle -= 45;
	}

	Polygons gap_polygons; // will remain empty
	Polygons gap_lines;
	constexpr int offset = 0;
	constexpr int infill_multiplier = 1;
	constexpr int extra_infill_shift = 0;
	const coord_tIrfan skin_overlap = MM2INT(0.25);
	constexpr int wall_line_count = 0;
	const curaIrfan::PointIrfan& infill_origin = curaIrfan::PointIrfan();
	constexpr Polygons* perimeter_gaps_polyons = nullptr;
	constexpr bool connected_zigzags = false;
	constexpr bool use_endpieces = true;
	constexpr bool skip_some_zags = false;
	constexpr int zag_skip_count = 0;
	constexpr coord_tIrfan pocket_size = 0;

	gcode_layer.mode_skip_agressive_merge = false;

	Infill infill_comp(
		EFillMethod::LINES, zig_zaggify_infill, connect_polygons, perimeter_gaps, offset, perimeter_gaps_line_width, perimeter_gaps_line_width, skin_overlap, infill_multiplier, perimeter_gaps_angle, gcode_layer.z, extra_infill_shift,
		wall_line_count, infill_origin, perimeter_gaps_polyons, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
	infill_comp.generate(gap_polygons, gap_lines);
	if (gap_lines.size() > 0)
	{
		added_something = true;
		setExtruder_addPrime(storage, gcode_layer, extruder_nr.at(0));//Extrduer should be changed here as its not 0 always
		gcode_layer.setIsInside(true); // going to print stuff inside print object
		gap_lines.print_extruder = extruder_nr.at(0);
		gcode_layer.addLinesByOptimizer(storage.layer_thickness, perimeter_gap_config, gap_lines, gcode_layer.getLayerNr(),  SpaceFillType::Lines);
	}

	gcode_layer.mode_skip_agressive_merge = true;
}


















