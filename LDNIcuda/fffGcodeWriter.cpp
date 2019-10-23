

#include <list>
#include <limits> // numeric_limits
#include <vector>

#include "fffGcodeWriter.h"
#include "GcodeLayerThreader.h"
#include "Infill.h"
#include "LayerPlan.h"
#include "Raft.h"
#include "SliceDataStorage.h"
#include "Slicer.h"



#define OMP_MAX_ACTIVE_LAYERS_PROCESSED 30

FffGcodeWriter::FffGcodeWriter()
	: max_object_height(0)
	, layer_plan_buffer(gcode)
{
	for (unsigned int extruder_nr = 0; extruder_nr < 16; extruder_nr++)
	{ // initialize all as max layer_nr, so that they get updated to the lowest layer on which they are used.
		extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
	}
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage,bool start)
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
	printf("the layer thickness is %d \n", layer_thickness);
	
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
	for (SliceLayer& layer_storage: storage.Layers)
	{
		
		total_layers = std::max(total_layers, storage.Layers.size());
		setInfillAndSkinAngles(storage);
	}
	gcode.writeLayerCountComment(total_layers);
	std::string current_mesh = "NONMESH";
	total_layers = 15;
	int process_layer_starting_layer_nr = 0;
	/*
	{ // calculate the mesh order for each extruder
		const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
		mesh_order_per_extruder.reserve(extruder_count);
		for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
		{
			mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
		}
	}
	*/
	//calculateExtruderOrderPerLayer(storage);
	/*
	const bool has_raft = true;
	if (has_raft)
	{
		processRaft(storage);
		// process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
		// only process the filler layers if there is anything to print in them.
		for (bool extruder_is_used_in_filler_layers : storage.getExtrudersUsed(-1))
		{
			if (extruder_is_used_in_filler_layers)
			{
				process_layer_starting_layer_nr = -Raft::getFillerLayerCount();
				break;
			}
		}
	}
   */
	printf("the program is at line 98\n");
	//LayerPlan& gcode_layer = processLayer(storage, process_layer_starting_layer_nr, total_layers);
	total_layers = 10;
	const std::function<LayerPlan* (int)>& produce_item = [&storage, total_layers, this](int layer_nr)
	{
		printf("starting the layer plan for the layer %d \n", layer_nr);
		LayerPlan& gcode_layer = processLayer(storage, layer_nr, total_layers);
		printf("got the Gcode for the layer plan %d \n", layer_nr);
		return &gcode_layer;
	};
	 
	const std::function<void(LayerPlan*)>& consume_item =
		[this, total_layers](LayerPlan* gcode_layer)
	{
		//Progress::messageProgress(Progress::Stage::EXPORT, std::max(0, gcode_layer->getLayerNr()) + 1, total_layers);
		printf("processing the layer number %d \n", gcode_layer->getLayerNr());
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

	printf("Done with middel processses for layer %d\n");
	// process all layers, process buffer for preheating and minimal layer time etc, write layers to gcode:
	threader.run();

	layer_plan_buffer.flush();
	printf("flushed the layer \n");
	//Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

	//Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
	max_object_height = std::max(max_object_height, storage.model_max.z);


	constexpr bool force = true;
	printf("going for the retractions \n");
	gcode.writeRetraction(storage.retraction_config_per_extruder[0]); // retract after finishing each meshgroup
	printf("wrote the layer \n");

}



LayerPlan& FffGcodeWriter::processLayer(SliceDataStorage& storage, int layer_nr, const size_t total_layers) const
{
	
	//printf("The program is at line 142 and the layerthickness is %f for the layer %d \n",layer_thickness,layer_nr);
	coord_tIrfan z;
	bool include_helper_parts = true;

	z = storage.Layers[layer_nr].printZ; // stub default   //storage.Layers[layer_nr].printZ = initial_layer_thickness + (layer_nr * layer_thickness);
	coord_tIrfan layer_thickness = storage.Layers[layer_nr].thickness;
	coord_tIrfan avoid_distance = 0; // minimal avoid distance is zero
	const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
	for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
	{
		if (extruder_is_used[extruder_nr])
		{
			
			avoid_distance = MM2INT(3);// std::max(avoid_distance, extruder.settings.get<coord_t>("travel_avoid_distance"));
		}
	}

	//double max_inner_wall_width = 0.3;
	coord_tIrfan max_inner_wall_width = MM2INT(0.3);
	if (layer_nr == 0)
	{
		max_inner_wall_width *= 120;
	}
	const coord_tIrfan comb_offset_from_outlines = max_inner_wall_width * 2;// inner_wall_width * 2; defalut is 120
	//no need for extruder order as I ma using only extruder
	
	coord_tIrfan first_outer_wall_line_width = MM2INT(0.35);
	//size_t extruder_order = ;
	size_t extruder_nr = 0;
	LayerPlan& gcode_layer = *new LayerPlan(storage, layer_nr, z, layer_thickness, extruder_nr, fan_speed_layer_time_settings_per_extruder, comb_offset_from_outlines, first_outer_wall_line_width, avoid_distance);
	printf("set the layer plan \n");
	if (include_helper_parts && layer_nr == 0)
	{ // process the skirt or the brim of the starting extruder.
		int extruder_nr = 0;
		processSkirtBrim(storage, gcode_layer, extruder_nr);
		
	}
	
	bool disable_path_optimisation = false;
	int mesh_idx = 0;
	
	
	for (int extruder_nr = 0; extruder_nr <= 1; extruder_nr++)
	{

		if (layer_nr >= 0)
		{
			addMeshLayerToGCode(storage, extruder_nr,gcode_layer);
			//printf("the code is at the line 181of th eprogram \n");
		}
		// ensure we print the prime tower with this extruder, because the next layer begins with this extruder!
		// If this is not performed, the next layer might get two extruder switches...
		//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
	}
	
	if (!disable_path_optimisation)	
	{
		printf("going for optimizing the paths \n");
		gcode_layer.optimizePaths(gcode.getPositionXY());
		
	}
	
	return gcode_layer;
}

void FffGcodeWriter::processSkirtBrim(SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr) const
{
	coord_tIrfan layer_thickness = storage.getlayer_thickness();
	printf("inside skirt brim\n");
	if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
	{
		printf("not processing brim \n");
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
	const bool prime_pos_is_abs =true;
	const curaIrfan::PointIrfan prime_pos(MM2INT(0.0),MM2INT(0.0));
	start_close_to = prime_pos;
	gcode_layer.addTravel(layer_thickness, gcode_layer.getLayerNr(), skirt_brim.back().closestPointTo(start_close_to));
	gcode_layer.addPolygonsByOptimizer(layer_thickness, gcode_layer.getLayerNr(),skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
	printf("skirt brim is processed \n");
}

unsigned int FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{
	
	size_t start_extruder_nr = 0;
	std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
	assert(start_extruder_nr < 1);
	return start_extruder_nr;
}
/*
void FffGcodeWriter::processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr) const
{
	if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
	{
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
	bool prime_blob_enable = true;
	if (prime_blob_enable)
	{
		const bool extruder_prime_pos_abs = true;
		const bool prime_pos_is_abs = extruder_prime_pos_abs;
		coord_tIrfan extruder_prime_pos_x = MM2INT(0);
		coord_tIrfan extruder_prime_pos_y = MM2INT(0);
		const curaIrfan::PointIrfan prime_pos(extruder_prime_pos_x, extruder_prime_pos_y);
		start_close_to = prime_pos;
	}
	else
	{
		start_close_to = gcode_layer.getLastPlannedPositionOrStartingPosition();
	}
	bool brim_outside_only = true;
	if (brim_outside_only)
	{
		gcode_layer.addTravel(skirt_brim.back().closestPointTo(start_close_to));
		gcode_layer.addPolygonsByOptimizer(skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
	}
	else
	{
		Polygons outer_brim, inner_brim;
		for (unsigned int index = 0; index < skirt_brim.size(); index++)
		{
			ConstPolygonRef polygon = skirt_brim[index];
			if (polygon.area() > 0)
			{
				outer_brim.add(polygon);
			}
			else
			{
				inner_brim.add(polygon);
			}
		}
		gcode_layer.addTravel(outer_brim.back().closestPointTo(start_close_to));
		gcode_layer.addPolygonsByOptimizer(outer_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);

		//Add polygon in reverse order
		const coord_tIrfan wall_0_wipe_dist = 0;
		const bool spiralize = false;
		const float flow_ratio = 1.0;
		const bool always_retract = false;
		const bool reverse_order = true;
		gcode_layer.addPolygonsByOptimizer(inner_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr], nullptr, ZSeamConfig(), wall_0_wipe_dist, spiralize, flow_ratio, always_retract, reverse_order);
	}
}
*/

void FffGcodeWriter::setConfigFanSpeedLayerTime()
{
	
	   
		fan_speed_layer_time_settings_per_extruder.emplace_back();
		
		FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();
		
		fan_speed_layer_time_settings.cool_min_layer_time = 5.000;
		fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = 10.000;
		fan_speed_layer_time_settings.cool_fan_speed_0 = 0.000 * 100.0;
		fan_speed_layer_time_settings.cool_fan_speed_min = 50.0000 * 100.0;
		fan_speed_layer_time_settings.cool_fan_speed_max = 100.00 * 100.0;
		fan_speed_layer_time_settings.cool_min_speed = 5.000;
		fan_speed_layer_time_settings.cool_fan_full_layer = 6.000;
		
	
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
	std::string machinegcode = "G28 ;Home\nG1 Z15.0 F6000 ;Move the platform down 15mm\n;Prime the extruder\nG92 E0\nG1 F200 E3\nG92 E0";
	gcode.writeCode(machinegcode.c_str());
	const double volume_temprature = 0.0;
	prefix= " ";
	gcode.writeCode(prefix.c_str());
	gcode.startExtruder(start_extruder_nr);
	processInitialLayerTemperature(storage, start_extruder_nr);
	double speed_travel = 250.0;
	printf("Outside prime train \n");
	coord_tIrfan thickness = storage.Layers[0].thickness;
	gcode.writePrimeTrain(speed_travel, thickness);
	printf("Processed prime train \n");
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
	const double print_temp_here = 200;// ("material_print_temperature");
	gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
	SliceDataStorage* storage1;
	coord_tIrfan layer_thicknees= storage1->getlayer_thickness();
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
	
	storage.infill_angles.push_back(45.0);
	// generally all infill patterns use 45 degrees
}
/*
void FffGcodeWriter::calculateExtruderOrderPerLayer(const SliceDataStorage& storage)
{
	size_t last_extruder=0;
	// set the initial extruder of this meshgroup
	
	for (int layer_nr = -Raft::getTotalExtraLayers(); layer_nr < static_cast<int>(storage.print_layer_count); layer_nr++)
	{
		std::vector<std::vector<size_t>>& extruder_order_per_layer_here = (layer_nr < 0) ? extruder_order_per_layer_negative_layers : extruder_order_per_layer;
		extruder_order_per_layer_here.push_back(getUsedExtrudersOnLayerExcludingStartingExtruder(storage, last_extruder, layer_nr));
		last_extruder = extruder_order_per_layer_here.back().back();
		extruder_prime_layer_nr[last_extruder] = std::min(extruder_prime_layer_nr[last_extruder], layer_nr);
	}
}
*/
void FffGcodeWriter::addMeshLayerToGCode(const SliceDataStorage& storage, const size_t extruder_nr, LayerPlan& gcode_layer) const
{	
	const SliceLayer& layer = storage.Layers[gcode_layer.getLayerNr()];	 // get layer # function tells that which layer to process

	//printf("inside addMeshLayerToGCode for the layer %d and the parts are %d \n   ", gcode_layer.getLayerNr(), layer.parts.size());

	if (layer.parts.size() == 0)  //simple condition 
	{
		return;
	}

	gcode_layer.setMesh("Mesh1");
	printf("**@497 of addmeshlayertogcode \n");
	ZSeamConfig z_seam_config(EZSeamType::SHARPEST_CORNER , storage.getZSeamHint(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY);
	const curaIrfan::PointIrfan layer_start_position(0.0, 0.0);
	PathOrderOptimizer part_order_optimizer(layer_start_position, z_seam_config);
	for (unsigned int part_idx = 0; part_idx < layer.parts.size(); part_idx++)
	{
		const SliceLayerPart& part = layer.parts[part_idx];
	//	printf("the code is at the line 345 and the parts insets size is %d \n", part.insets.size());
		ConstPolygonRef part_representative = (part.insets.size() > 0) ? part.insets[0][0] : part.outline[0];
		//printf("the code is at the line 347 %d \n ", part.outline[part_idx].size());
		part_order_optimizer.addPolygon(part_representative);
		//printf("the polygons is added 347 %d \n ", part.outline[part_idx].size());
	}
	//printf("the code is at the line 350 \n ");
	part_order_optimizer.optimize();

	for (int part_idx : part_order_optimizer.polyOrder)
	{
		//printf("the code is at the line 355 \n %d",part_idx);
		const SliceLayerPart& part = layer.parts[part_idx];
		addMeshPartToGCode(storage, extruder_nr, part, gcode_layer);//part n layer being added to the GCode
	}
	processIroning(layer,  gcode_layer);

	gcode_layer.setMesh("NONMESH");
}

bool FffGcodeWriter::processIroning(const SliceLayer& layer,  LayerPlan& gcode_layer) const
{
	bool added_something = false;
	return added_something;
}

void FffGcodeWriter::addMeshPartToGCode(const SliceDataStorage&storage, const size_t extruder_nr, const SliceLayerPart& part, LayerPlan& gcode_layer) const
{
	printf("inside addMeshPartToGCode for the layer %d and the parts are \n ", gcode_layer.getLayerNr());

	bool added_something = false;		

	added_something = added_something | processInfill(storage, gcode_layer, extruder_nr, part);
	
	//added_something = added_something | processInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

	//processOutlineGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, added_something);

	//added_something = added_something | processSkinAndPerimeterGaps(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
	coord_tIrfan layer_thickness = storage.Layers[0].thickness;
	bool  magic_spiralize = false;

	int bottom_layers = 5;

	if (added_something && (!magic_spiralize) || gcode_layer.getLayerNr() < bottom_layers)
	{
		coord_tIrfan innermost_wall_line_width = MM2INT(0.3);// mesh.settings.get<coord_tIrfan>((mesh.settings.get<size_t>("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
		if (gcode_layer.getLayerNr() == 0)
		{
			innermost_wall_line_width *= 120;// mesh.settings.get<Ratio>("initial_layer_line_width_factor");
		}
		gcode_layer.moveInsideCombBoundary(gcode_layer.getLayerNr(), innermost_wall_line_width);
	}																
	gcode_layer.setIsInside(false);
}

bool FffGcodeWriter::processInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr, const SliceLayerPart& part) const
{
	printf("inside processInfill \n ");
	//bool added_something = processMultiLayerInfill(storage, gcode_layer, extruder_nr, part);
	bool added_something = processSingleLayerInfill(storage, gcode_layer, extruder_nr, part);
	printf("the boolean added_something is %d \n", added_something);
	return added_something;
}

bool FffGcodeWriter::processMultiLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr, const SliceLayerPart& part) const
{
	printf("inside processMultiLayerInfill \n ");
	const coord_tIrfan infill_line_distance = MM2INT(6.3);// mesh.settings.get<coord_t>("infill_line_distance");
	const coord_tIrfan infill_overlap = MM2INT(0.00);// mesh.settings.get<coord_t>("infill_overlap_mm");
	double infill_angle = 45.0; //Original default. This will get updated to an element from mesh->infill_angles.
	
	if (storage.infill_angles.empty())
	{
		const size_t combined_infill_layers = std::max(unsigned(1), round_divide(MM2INT(0.2), std::max(MM2INT(0.01 * 15), coord_tIrfan(1))));
		printf("the combined layer are %d \n", combined_infill_layers);
		infill_angle = storage.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % storage.infill_angles.size());
	}
	
	
	Point3 mesh_middle = storage.bounding_box.getMiddle();
	const curaIrfan::PointIrfan infill_origin(mesh_middle.x, mesh_middle.y);
	EFillMethod infill_pattern = EFillMethod::TRIANGLES;
	//Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
	bool added_something = false;
	printf("in multilayerinfill 401 fffGCODEwriter.cpp %d \n", part.infill_area_per_combine_per_density[0].size());

	for (unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
	{
		printf("line 404 in fffGCODEwriter.cpp \n");
		const coord_tIrfan infill_line_width = MM2INT(0.42);// 0.42;
		const EFillMethod infill_pattern = EFillMethod::TRIANGLES;
		const bool zig_zaggify_infill = true;// mesh.settings.get<bool>("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
		const bool connect_polygons = false;// mesh.settings.get<bool>("connect_infill_polygons");
		const size_t infill_multiplier = 0;// mesh.settings.get<size_t>("infill_multiplier");
		Polygons infill_polygons;
		Polygons infill_lines;
		for (size_t density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
		{ // combine different density infill areas (for gradual infill)
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
			printf("line 422 in fffGCODEwriter.cpp \n");
			Infill infill_comp(infill_pattern, zig_zaggify_infill, connect_polygons, part.infill_area_per_combine_per_density[density_idx][combine_idx], /*outline_offset =*/ 0
				, infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
				, perimeter_gaps, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count
				, MM2INT(0.42));
			
			infill_comp.generate(infill_lines, part);
			printf("line 429 in fffGCODEwriter.cpp \n");
		}

		if (!infill_lines.empty())
		{
			added_something = true;
			//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
			gcode_layer.setIsInside(true); // going to print stuff inside print object
			const bool enable_travel_optimization = false;//mesh.settings.get<bool>("infill_enable_travel_optimization");
			//gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], zig_zaggify_infill ? SpaceFillType::PolyLines : SpaceFillType::Lines, enable_travel_optimization);
		}
	}
	printf("line  in 445 fffGCODEwriter.cpp %d \n", part.infill_area_per_combine_per_density.size());
	return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const size_t extruder_nr, const SliceLayerPart& part) const
{
	printf("inside processSingleLayerInfill \n ");
	
	const coord_tIrfan infill_line_distance = MM2INT(6.3);
	printf("the infill line distance is %d \n", infill_line_distance);
	printf("inside processSingleLayerInfill @ 634\n");
	bool added_something = false;
	PathConfigStorage::MeshPathConfigs *meshpath;
	const coord_tIrfan infill_line_width = MM2INT(0.42);
		
	Polygons infill_lines;
	//const double infill_line_width = 0.42;
	//const coord_tIrfan infill_line_width_int = MM2INT(infill_line_width);
	//printf("Inside Singel Layer Infill and the infill area per is %d \n", part.infill_area_per_combine_per_density[0].size());
	if (infill_line_distance== 0 || part.infill_area_per_combine_per_density[0].size() == 0)
	{
		printf("error in single layer infill \n");
		return false;
	}


	//Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
	

	const EFillMethod pattern = EFillMethod::TRIANGLES;
	const bool zig_zaggify_infill = true;// mesh.settings.get<bool>("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
	const bool connect_polygons = false;// mesh.settings.get<bool>("connect_infill_polygons");
	const coord_tIrfan infill_overlap =0 ;//sh.settings.get<coord_tIrfan>("infill_overlap_mm");
	const size_t infill_multiplier = 1;// mesh.settings.get<size_t>("infill_multiplier");
	const size_t wall_line_count = 0;
	double infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.
	PrintFeatureType feature = PrintFeatureType::Infill;
	coord_tIrfan layer_height = gcode_layer.z;
	
	coord_tIrfan layer_thickness = storage.layer_thickness;
	int layernum = gcode_layer.getLayerNr();
	//printf("inside the singellayerinfill line 474 \n");
	if (storage.infill_angles.size() > 0)
	{
		const size_t combined_infill_layers = 1;// std::max(unsigned(1), round_divide(mesh.settings.get<coord_t>("infill_sparse_thickness"), std::max(mesh.settings.get<coord_t>("layer_height"), coord_t(1))));
		infill_angle = storage.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % storage.infill_angles.size());
	}
	//printf("inside the singellayerinfill loop 473\n");
	const Point3 mesh_middle = storage.bounding_box.getMiddle();
	const curaIrfan::PointIrfan infill_origin(mesh_middle.x , mesh_middle.y);
	for (unsigned int density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
	{
		//printf("inside the singellayerinfill loop\n");
		int infill_line_distance_here = infill_line_distance<<(density_idx +1) ; // the highest density infill combines with the next to create a grid with density_factor 1
		int infill_shift = infill_line_distance_here / 2;
		// infill shift explanation: [>]=shift ["]=line_dist
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>"""""
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>>>"""""""""
// :       |       :       |       :       |       :       |         > furthest from top
// :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
// : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
// >>>>>>>>"""""""""""""""""
		SlicerLayer slicerlayer;
	//	printf("inside the singellayerinfill loop 494\n");
		
		Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];	 //part.outline;
		
		if (in_outline.empty())
		{
			printf("*********************the outnline is empty******************************* \n");
		}
		
		const coord_tIrfan circumference = in_outline.polygonLength();
		//printf("the polygn length is %d\n ",circumference);
		//Originally an area of 0.4*0.4*2 (2 line width squares) was found to be a good threshold for removal.
		//However we found that this doesn't scale well with polygons with larger circumference (https://github.com/Ultimaker/Cura/issues/3992).
		//Given that the original test worked for approximately 2x2cm models, this scaling by circumference should make it work for any size.
		const double minimum_small_area = 0.4 * 0.4 * circumference / 40000;

		// This is only for density infill, because after generating the infill might appear unnecessary infill on walls
		// especially on vertical surfaces
		//printf("inside the singellayerinfill loop 520\n");
		//in_outline.removeSmallAreas(minimum_small_area);
		
		coord_tIrfan outline_offset = 0 ;

		Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, in_outline, outline_offset, infill_line_width, infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle, gcode_layer.z, infill_shift, wall_line_count, infill_origin
			, /*Polygons* perimeter_gaps =*/ nullptr
			, /*bool connected_zigzags =*/ false
			, /*bool use_endpieces =*/ false
			, /*bool skip_some_zags =*/ false
			, /*int zag_skip_count =*/ 0
			, MM2INT(0.42));
		infill_comp.generate(infill_lines, part);	 
	}
	printf("the infill lines size is %d and the layer number is %d \n", infill_lines.size(), gcode_layer.getLayerNr());
	if (infill_lines.size() > 0)
	{
		added_something = true;
		//setExtruder_addPrime(storage, gcode_layer, extruder_nr);
		gcode_layer.setIsInside(true); // going to print stuff inside print object
		const bool enable_travel_optimization = false;// mesh.settings.get<bool>("infill_enable_travel_optimization");
		printf("outside the lineoptimizer \n");
		gcode_layer.addLinesByOptimizer(layer_thickness, infill_lines, layernum, SpaceFillType::Lines, enable_travel_optimization);
		//gcode_layer.addLinesByOptimizer(in/fill_lines,SpaceFillType::Lines, enable_travel_optimization,0);
		
	}
	

	return added_something;
}
 /*
bool FffGcodeWriter::processInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer,  const size_t extruder_nr, const SliceLayerPart& part) const
{
	bool added_something = false;
	const bool compensate_overlap_0 = true;// mesh.settings.get<bool>("travel_compensate_overlapping_walls_0_enabled");
	const bool compensate_overlap_x = true;// mesh.settings.get<bool>("travel_compensate_overlapping_walls_x_enabled");
	const bool retract_before_outer_wall = true;// mesh.settings.get<bool>("travel_retract_before_outer_wall");
	size_t wall_line_count = 3;
	if (wall_line_count > 0)
	{
		bool spiralize = false;
		if (!spiralize && gcode_layer.getLayerNr() > 0)
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
		if (InsetOrderOptimizer::optimizingInsetsIsWorthwhile(part))
		{
			InsetOrderOptimizer ioo(*this, storage, gcode_layer, mesh, extruder_nr, mesh_config, part, gcode_layer.getLayerNr());
			return ioo.processInsetsWithOptimizedOrdering();
		}
		else
		{
			const bool outer_inset_first = mesh.settings.get<bool>("outer_inset_first")
				|| (gcode_layer.getLayerNr() == 0 && mesh.settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::BRIM);
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
					if (part.insets[0].size() > 0 && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr)
					{
						added_something = true;
						setExtruder_addPrime(storage, gcode_layer, extruder_nr);
						gcode_layer.setIsInside(true); // going to print stuff inside print object
						ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
						Polygons outer_wall = part.insets[0];
						if (!compensate_overlap_0)
						{
							WallOverlapComputation* wall_overlap_computation(nullptr);
							gcode_layer.addWalls(outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, wall_overlap_computation, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), flow, retract_before_outer_wall);
						}
						else
						{
							WallOverlapComputation wall_overlap_computation(outer_wall, mesh_config.inset0_config.getLineWidth());
							gcode_layer.addWalls(outer_wall, mesh, mesh_config.inset0_config, mesh_config.bridge_inset0_config, &wall_overlap_computation, z_seam_config, mesh.settings.get<coord_t>("wall_0_wipe_dist"), flow, retract_before_outer_wall);
						}
					}
				}
				// Inner walls are processed
				else if (!part.insets[processed_inset_number].empty() && extruder_nr == mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr)
				{
					added_something = true;
					setExtruder_addPrime(storage, gcode_layer, extruder_nr);
					gcode_layer.setIsInside(true); // going to print stuff inside print object
					ZSeamConfig z_seam_config(mesh.settings.get<EZSeamType>("z_seam_type"), mesh.getZSeamHint(), mesh.settings.get<EZSeamCornerPrefType>("z_seam_corner"));
					Polygons inner_wall = part.insets[processed_inset_number];
					if (!compensate_overlap_x)
					{
						WallOverlapComputation* wall_overlap_computation(nullptr);
						gcode_layer.addWalls(part.insets[processed_inset_number], mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, wall_overlap_computation, z_seam_config);
					}
					else
					{
						WallOverlapComputation wall_overlap_computation(inner_wall, mesh_config.insetX_config.getLineWidth());
						gcode_layer.addWalls(inner_wall, mesh, mesh_config.insetX_config, mesh_config.bridge_insetX_config, &wall_overlap_computation, z_seam_config);
					}
				}
			}
		}
	}
	return added_something;
}
*/













