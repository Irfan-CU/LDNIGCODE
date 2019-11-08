
#include <cstring>
#include <fstream>

#include "ExtruderTrain.h"
#include "LayerPlan.h"
#include "MergeInfillLines.h"
#include "Raft.h"
#include "SliceDataStorage.h"
#include "WallOverlap.h"
#include "Ratio.h"
#include "Polygonutils.h"
//#include "fffGcodewriter.h"



ExtruderPlan::ExtruderPlan(const size_t extruder, const int layer_nr, const bool is_initial_layer, const bool is_raft_layer, const coord_tIrfan layer_thickness, const FanSpeedLayerTimeSettings& fan_speed_layer_time_settings, const RetractionConfig& retraction_config)
	: heated_pre_travel_time(0)
	, required_start_temperature(-1)
	, extruder_nr(extruder)
	, layer_nr(layer_nr)
	, is_initial_layer(is_initial_layer)
	, is_raft_layer(is_raft_layer)
	, layer_thickness(layer_thickness)
	, fan_speed_layer_time_settings(fan_speed_layer_time_settings)
	, retraction_config(retraction_config)
	, extrudeSpeedFactor(1.0)
	, extraTime(0.0)
	, totalPrintTime(0)
{
}

double ExtruderPlan::getFanSpeed()
{
	return 100;//;
}
 
GCodePath* LayerPlan::getLatestPathWithConfig(coord_tIrfan layer_thickness, const GCodePathConfig& config, SpaceFillType space_fill_type, const Ratio flow, bool spiralize, const Ratio speed_factor)
{
	
	std::vector<GCodePath>& paths = extruder_plans.back().paths;
	
	
	if (paths.size() > 0 && paths.back().config == &config && !paths.back().done && paths.back().flow == flow && paths.back().speed_factor == speed_factor && paths.back().mesh_id == current_mesh) // spiralize can only change when a travel path is in between
	{
		
		return &paths.back();
	}
	paths.emplace_back(config, current_mesh, space_fill_type, flow, spiralize, speed_factor);
	GCodePath* ret = &paths.back();
	ret->skip_agressive_merge_hint = mode_skip_agressive_merge;
	return ret;



}

void LayerPlan::forceNewPathStart()
{
	std::vector<GCodePath>& paths = extruder_plans.back().paths;
	if (paths.size() > 0)
		paths[paths.size() - 1].done = true;
}

LayerPlan::LayerPlan(const SliceDataStorage& storage, int layer_nr, coord_tIrfan z, coord_tIrfan layer_thickness, size_t start_extruder, const std::vector<FanSpeedLayerTimeSettings>& fan_speed_layer_time_settings_per_extruder, coord_tIrfan comb_boundary_offset, coord_tIrfan comb_move_inside_distance, coord_tIrfan travel_avoid_distance)
	: storage(storage)
	, configs_storage(storage, layer_nr, layer_thickness)
	, processconfigs(ProcessCofigs_Storage())
	, z(z)
	, final_travel_z(z)
	, mode_skip_agressive_merge(false)
	, layer_nr(layer_nr)
	, is_initial_layer((layer_nr == 0))
	, is_raft_layer(layer_nr < 0)// layer_nr < 0 - static_cast<int>(Raft::getFillerLayerCount()))
	, layer_thickness(layer_thickness)
	, current_mesh("NONMESH")
	, last_extruder_previous_layer(start_extruder)
	, last_planned_extruder(0)
	, first_travel_destination_is_inside(false) // set properly when addTravel is called for the first time (otherwise not set properly)
	, comb_boundary_inside1(computeCombBoundaryInside(1)) //no combing mode 
	, comb_boundary_inside2(computeCombBoundaryInside(2)) //no combing mode 
	, comb_move_inside_distance(comb_move_inside_distance)
	, fan_speed_layer_time_settings_per_extruder(fan_speed_layer_time_settings_per_extruder)

{
	size_t current_extruder = start_extruder;
	was_inside = true; // not used, because the first travel move is bogus
	is_inside = false; // assumes the next move will not be to inside a layer part (overwritten just before going into a layer part)
	comb = nullptr;
	layer_start_pos_per_extruder.emplace_back(MM2INT(50), MM2INT(50));
	extruder_plans.reserve(1);
	extruder_plans.emplace_back(current_extruder, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[current_extruder], storage.retraction_config_per_extruder[current_extruder]);

	for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
	{ //Skirt and brim.
		skirt_brim_is_processed[extruder_nr] = false;
	}
}

LayerPlan::~LayerPlan()
{
	bool *comb = false;
	if (comb)
		delete comb;
}

void LayerPlan::setIsInside(bool _is_inside)
{
	is_inside = _is_inside;
}

Polygons LayerPlan::computeCombBoundaryInside(const size_t max_inset)
{
	const CombingMode combing_mode = CombingMode::OFF;
	if (combing_mode == CombingMode::OFF)
	{
		return Polygons();
	}
}
 
void LayerPlan::addLinesByOptimizer(coord_tIrfan layer_thickness , const GCodePathConfig& config, const Polygons& polygons, int layernum, SpaceFillType space_fill_type, bool enable_travel_optimization, int wipe_dist, float flow_ratio, std::optional<curaIrfan::PointIrfan> near_start_location, double fan_speed)
{
	
	Polygons boundary;
	
	curaIrfan::PointIrfan startPoint = near_start_location.value_or(getLastPlannedPositionOrStartingPosition());
	//printf("near_start_location.value_or(getLastPlannedPositionOrStartingPosition() is %d and %d \n",startPoint.X,startPoint.Y);
	LineOrderOptimizer orderOptimizer(startPoint, &boundary);
	//printf("the polygon sizse for the line order optimizer is %d \n", polygons.size());
	
	for (unsigned int line_idx = 0; line_idx < polygons.size(); line_idx++)
	{
		orderOptimizer.addPolygon(polygons[line_idx]);
	}
	orderOptimizer.optimize();
	for (unsigned int order_idx = 0; order_idx < orderOptimizer.polyOrder.size(); order_idx++)
	{
		const unsigned int poly_idx = orderOptimizer.polyOrder[order_idx];
		ConstPolygonRef polygon = polygons[poly_idx];
		
		//printf("the pints size is %d \n", polygon.size());
		const size_t start = orderOptimizer.polyStart[poly_idx];
		const size_t end = 1 - start;
		const curaIrfan::PointIrfan& p0 = polygon[start];
		addTravel(layer_thickness, layernum,p0);
		//printf("@@the polygon starting point is %d and %d the start id is %d  \n", p0.X, p0.Y,start);
		const curaIrfan::PointIrfan& p1 = polygon[end];
		//printf("@@the polygon ending point is %d and %d end id is %d \n", p1.X,p1.Y,end );
		//const GCodePathConfig *config;
		//addExtrusionMove(p1,space_fill_type, flow_ratio, false, 1.0, fan_speed);
		addExtrusionMove(layer_thickness, config, p1, layernum, space_fill_type, flow_ratio, false, 1.0, fan_speed);
	}
	//printf("***** the extruder plans and paths size are %d \n", extruder_plans[0].paths.size());
	
}

void LayerPlan::addExtrusionMove(coord_tIrfan layer_thickness, const GCodePathConfig& config, curaIrfan::PointIrfan p, int layernum, SpaceFillType space_fill_type, const Ratio& flow, bool spiralize, Ratio speed_factor, double fan_speed)
{
	
	GCodePath* path = getLatestPathWithConfig(layer_thickness, config, space_fill_type, flow, spiralize, speed_factor);
	path->points.push_back(p);
	//printf("@@the points size is %d \n", path->points.size());
	path->setFanSpeed(fan_speed);
	last_planned_position = p;
	
}
 
void LayerPlan::setMesh(const std::string mesh_id)
{
	current_mesh = mesh_id;
}

ExtruderTrain* LayerPlan::getLastPlannedExtruderTrain()
{
	return last_planned_extruder;
}
void LayerPlan::planPrime()
{
	forceNewPathStart();
	constexpr float prime_blob_wipe_length = 10.0;
	GCodePath& prime_travel = addTravel_simple(layer_nr, curaIrfan::operator+(getLastPlannedPositionOrStartingPosition() , curaIrfan::PointIrfan(0, MM2INT(prime_blob_wipe_length))));
	prime_travel.retract = false;
	prime_travel.perform_z_hop = false;
	prime_travel.perform_prime = true;
	forceNewPathStart();
}


bool LayerPlan::setExtruder(const size_t extruder_nr)
{
	if (extruder_nr == getExtruder())
	{
		return false;
	}
	setIsInside(false);
	{ // handle end position of the prev extruder
		ExtruderTrain* extruder = getLastPlannedExtruderTrain();
		const bool end_pos_absolute = true;// extruder->settings.get<bool>("machine_extruder_end_pos_abs");
		curaIrfan::PointIrfan end_pos(MM2INT(0.0), MM2INT(0.0));// extruder->settings.get<coord_t>("machine_extruder_end_pos_y"));
		if (!end_pos_absolute)
		{
			curaIrfan::operator+=(end_pos , getLastPlannedPositionOrStartingPosition());
		}
		else
		{
			const curaIrfan::PointIrfan extruder_offset(MM2INT(0.0), MM2INT(0.0));// extruder->settings.get<coord_t>("machine_nozzle_offset_x"), extruder->settings.get<coord_t>("machine_nozzle_offset_y"));
			curaIrfan::operator+=(end_pos , extruder_offset); // absolute end pos is given as a head position
		}
		if (end_pos_absolute || last_planned_position)
		{
			addTravel(layer_thickness, layer_nr, end_pos); //  + extruder_offset cause it
		}
	}
	if (extruder_plans.back().paths.empty() && extruder_plans.back().inserts.empty())
	{ // first extruder plan in a layer might be empty, cause it is made with the last extruder planned in the previous layer
		extruder_plans.back().extruder_nr = extruder_nr;
	}
	else
	{
		extruder_plans.emplace_back(extruder_nr, layer_nr, is_initial_layer, is_raft_layer, layer_thickness, fan_speed_layer_time_settings_per_extruder[extruder_nr], storage.retraction_config_per_extruder[extruder_nr]);
		assert(extruder_plans.size() <= 1 && "Never use the same extruder twice on one layer!");
	}
	

	{ // handle starting pos of the new extruder
		ExtruderTrain* extruder = getLastPlannedExtruderTrain();
		const bool start_pos_absolute =true;
		curaIrfan::PointIrfan start_pos(MM2INT(2.0), MM2INT(2.0));
		
			curaIrfan::PointIrfan extruder_offset(MM2INT(0), MM2INT(0));
			curaIrfan::operator+=(start_pos , extruder_offset); // absolute start pos is given as a head position
		
		if (start_pos_absolute || last_planned_position)
		{
			last_planned_position = start_pos;
		}
	}
	return true;
}

void LayerPlan::writeGCode(GCodeExport& gcode)
{
//	printf("}}}}{}}{}{}{}{}}}{}{}}}{}{}}{}{}}{}{}}{}{}{}{}{}}{}{}{}{}{}{Inside Gcode layer is %d \n", layer_nr);
	coord_tIrfan layer_thicnkess = storage.Layers[0].thickness;
	gcode.setLayerNr(layer_nr);
	gcode.writeLayerComment(layer_nr);


	// flow-rate compensation
	//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	//const Settings& mesh_group_settings = Applicataion::getInstance().current_slice->scene.current_mesh_group->settings;
	gcode.setFlowRateExtrusionSettings(MM2INT(0), Ratio(100/100)); //Offset is in mm.

	if (layer_nr == 0 ) //- true and 60;........static_cast<LayerIndex>(Raft::getTotalExtraLayers()) && mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature") != 0)
	{
		constexpr bool wait = false;
		gcode.writeBedTemperatureCommand(60, wait);
	}

	
	gcode.setZ(z);
	
	const GCodePathConfig* last_extrusion_config= nullptr; // used to check whether we need to insert a TYPE comment in the gcode.
	size_t extruder_nr = gcode.getExtruderNr();
	
	const bool acceleration_enabled = true;// mesh_group_settings.get<bool>("acceleration_enabled");
	const bool jerk_enabled = true;// mesh_group_settings.get<bool>("jerk_enabled");
	bool retraction_hop_after_extruder_switch = true;
	bool retraction_enable = false;

	std::string current_mesh = "NONMESH";

	for (size_t extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
	{
		ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
		const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_plan.extruder_nr];
		coord_tIrfan z_hop_height = retraction_config.zHop;
		
		if (extruder_nr != extruder_plan.extruder_nr)
		{
			int prev_extruder = extruder_nr;
			extruder_nr = extruder_plan.extruder_nr;

			gcode.ResetLastEValueAfterWipe(prev_extruder);

			
			if (retraction_hop_after_extruder_switch)
			{
				z_hop_height = storage.extruder_switch_retraction_config_per_extruder[prev_extruder].zHop;
				gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder], z_hop_height);
			}
			else
			{
				gcode.switchExtruder(extruder_nr, storage.extruder_switch_retraction_config_per_extruder[prev_extruder]);
			}

			

			{ // require printing temperature to be met
				constexpr bool wait = true;
				gcode.writeTemperatureCommand(extruder_nr, extruder_plan.required_start_temperature, wait);
			}

			if (extruder_plan.prev_extruder_standby_temp)
			{ // turn off previous extruder
				constexpr bool wait = false;
				float prev_extruder_temp = *extruder_plan.prev_extruder_standby_temp;
				const int prev_layer_nr = (extruder_plan_idx == 0) ? layer_nr - 1 : layer_nr;
				if (prev_layer_nr == storage.max_print_height_per_extruder[prev_extruder])
				{
					prev_extruder_temp = 0; // TODO ? should there be a setting for extruder_off_temperature ?
				}
				gcode.writeTemperatureCommand(prev_extruder, prev_extruder_temp, wait);
			}

			const double extra_prime_amount = MM2INT(0.0);
			//gcode.addExtraPrimeAmount(extra_prime_amount);
		}
		
		if (extruder_plan_idx == 0)
		{
			const WipeScriptConfig& wipe_config = storage.wipe_config_per_extruder[extruder_plan.extruder_nr];
			if (wipe_config.clean_between_layers && gcode.getExtrudedVolumeAfterLastWipe(extruder_nr) > wipe_config.max_extrusion_mm3)
			{
				gcode.insertWipeScript(wipe_config, layer_thicnkess);
				gcode.ResetLastEValueAfterWipe(extruder_nr);
			}
			
		}
		gcode.writeFanCommand(extruder_plan.getFanSpeed());
		std::vector<GCodePath>& paths = extruder_plan.paths;
	   	extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool
		{
			return  a.path_idx < b.path_idx;
		});
		//printf("sorted the paths here and the sorted paths size is %d and layer nr is %d \n",paths.size(),layer_nr);
		bool update_extrusion_offset = true;

		for (unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
		{
			
			extruder_plan.handleInserts(path_idx, gcode);

			GCodePath& path = paths[path_idx];

			if (!path.retract && path.isTravelPath() && path.points.size() == 1 && path.points[0] == gcode.getPositionXY() && z == gcode.getPositionZ())
			{
				
				//printf("here in wrong place 356\n");
				continue;
			}

			if (acceleration_enabled)
			{
				
				if (path.isTravelPath())
				{
					//printf("writing travel accelrations \n");
					gcode.writeTravelAcceleration(path.config->getAcceleration());
				}
				else
				{
					//printf("writing print accelrations \n");
					gcode.writePrintAcceleration(path.config->getAcceleration());
				}
			}

			if (jerk_enabled)
			{
				gcode.writeJerk(path.config->getJerk());
			}
			/*
			if (path.retract)
			{
				gcode.writeRetraction(retraction_config);
				if (path.perform_z_hop)
				{
					gcode.writeZhopStart(z_hop_height);
					z_hop_height = retraction_config.zHop; // back to normal z hop
				}
				else
				{
					gcode.writeZhopEnd();
				}
			}
			 */
			if (!path.isTravelPath() && last_extrusion_config != path.config)
			{
				gcode.writeTypeComment(path.config->type);
				if (path.config->isBridgePath())
				{
					gcode.writeComment("BRIDGE");
				}
				last_extrusion_config = path.config;
				update_extrusion_offset = true;
			}
			else
			{
				update_extrusion_offset = false;
			}

			double speed = path.config->getSpeed();

			// for some movements such as prime tower purge, the speed may get changed by this factor
			speed *= path.speed_factor;

			//Apply the extrusion speed factor if it's an extrusion move.
			if (!path.isTravelPath())
			{
				speed *= extruder_plan.getExtrudeSpeedFactor();
			}
			if (path.mesh_id != current_mesh)
			{
				current_mesh = path.mesh_id;
				std::stringstream ss;
				ss << "MESH:" << current_mesh;
				gcode.writeComment(ss.str());
			}
			if (path.isTravelPath())
			{ // early comp for travel paths, which are handled more simply
				if (!path.perform_z_hop && final_travel_z != z && extruder_plan_idx == (extruder_plans.size() - 1) && path_idx == (paths.size() - 1))
				{
					// Before the final travel, move up to the next layer height, on the current spot, with a sensible speed.
					Point3 current_position = gcode.getPosition();
					current_position.z = final_travel_z;
					
					gcode.writeTravel(current_position, 10, layer_thicnkess);

					// Prevent the final travel(s) from resetting to the 'previous' layer height.
					
					gcode.setZ(final_travel_z);
				}
				for (unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
				{
					gcode.writeTravel(path.points[point_idx], speed, layer_thicnkess);
				}
				continue;
			}

			bool spiralize = false;
					
			if (!spiralize) // normal (extrusion) move (with coasting
			{
				// if path provides a valid (in range 0-100) fan speed, use it
				const double path_fan_speed = path.getFanSpeed();
				gcode.writeFanCommand(path_fan_speed != GCodePathConfig::FAN_SPEED_DEFAULT ? path_fan_speed : extruder_plan.getFanSpeed());

				bool coasting = false;
				if (coasting)
				{
					//coasting = writePathWithCoasting(gcode, extruder_plan_idx, path_idx, layer_thickness);
				}
				if (!coasting) // not same as 'else', cause we might have changed [coasting] in the line above...
				{ // normal path to gcode algorithm
					for (unsigned int point_idx = 0; point_idx < path.points.size(); point_idx++)
					{
						//communication->sendLineTo(path.config->type, path.points[point_idx], path.getLineWidthForLayerView(), path.config->getLayerThickness(), speed);
						gcode.writeExtrusion(path.points[point_idx], layer_thicnkess, speed, path.getExtrusionMM3perMM(), path.config->type, update_extrusion_offset);	   
						
					}
				}
				
			}
		}
		extruder_plan.handleAllRemainingInserts(gcode);

			
	} // paths for this extruder /\  .

}

double ExtruderPlan::getExtrudeSpeedFactor()
{
	return 1.0;
}
void GCodeExport::writeJerk(const double& jerk)
{
	*output_stream << "M205 X" << jerk << " Y" << jerk << new_line;
	current_jerk = jerk;
		
}

void GCodeExport::writeZhopStart(const coord_tIrfan hop_height, double speed/*= 0*/)
{
	if (hop_height > 0)
	{
		if (speed == 0)
		{
			speed =10.0;
		}
		is_z_hopped = hop_height;
		currentSpeed = speed;
		*output_stream << "G1 F" << speed * 60 << " Z" << new_line;// MMtoStream{ current_layer_z + is_z_hopped } << new_line;
		total_bounding_box.includeZ(current_layer_z + is_z_hopped);
		assert(speed > 0.0 && "Z hop speed should be positive.");
	}
}



void GCodeExport::writeTypeComment(const PrintFeatureType& type)
{
	switch (type)
	{
	case PrintFeatureType::OuterWall:
		*output_stream << ";TYPE:WALL-OUTER" << new_line;
		break;
	case PrintFeatureType::InnerWall:
		*output_stream << ";TYPE:WALL-INNER" << new_line;
		break;
	case PrintFeatureType::Skin:
		*output_stream << ";TYPE:SKIN" << new_line;
		break;
	case PrintFeatureType::Support:
		*output_stream << ";TYPE:SUPPORT" << new_line;
		break;
	case PrintFeatureType::SkirtBrim:
		*output_stream << ";TYPE:SKIRT" << new_line;
		break;
	case PrintFeatureType::Infill:
		*output_stream << ";TYPE:FILL" << new_line;
		break;
	case PrintFeatureType::SupportInfill:
		*output_stream << ";TYPE:SUPPORT" << new_line;
		break;
	case PrintFeatureType::SupportInterface:
		*output_stream << ";TYPE:SUPPORT-INTERFACE" << new_line;
		break;
	case PrintFeatureType::PrimeTower:
		*output_stream << ";TYPE:PRIME-TOWER" << new_line;
		break;
	case PrintFeatureType::MoveCombing:
	case PrintFeatureType::MoveRetraction:
	case PrintFeatureType::NoneType:
	case PrintFeatureType::NumPrintFeatureTypes:
		// do nothing
		break;
	}
}


std::optional<std::pair<curaIrfan::PointIrfan, bool>> LayerPlan::getFirstTravelDestinationState() const
{
	std::optional<std::pair<curaIrfan::PointIrfan, bool>> ret;
	if (first_travel_destination)
	{
		ret = std::make_pair(*first_travel_destination, first_travel_destination_is_inside);
	}
	return ret;
}

void LayerPlan::addPolygon(coord_tIrfan layer_thickness, int layer_nr, ConstPolygonRef polygon, int start_idx, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation, coord_tIrfan wall_0_wipe_dist, bool spiralize, const double& flow_ratio, bool always_retract)
{
	curaIrfan::PointIrfan p0 = polygon[start_idx];
	addTravel(layer_thickness, layer_nr, p0, always_retract);
	for (unsigned int point_idx = 1; point_idx < polygon.size(); point_idx++)
	{
		curaIrfan::PointIrfan p1 = polygon[(start_idx + point_idx) % polygon.size()];
		const Ratio flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
		addExtrusionMove(layer_thickness, config, p1, layer_nr, SpaceFillType::Polygons, flow, spiralize);
		p0 = p1;
	}
	if (polygon.size() > 2)
	{
		const curaIrfan::PointIrfan& p1 = polygon[start_idx];
		const Ratio flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
		addExtrusionMove(layer_thickness, config, p1, layer_nr, SpaceFillType::Polygons, flow, spiralize);

		if (wall_0_wipe_dist > 0)
		{ // apply outer wall wipe
			p0 = polygon[start_idx];
			int distance_traversed = 0;
			for (unsigned int point_idx = 1; ; point_idx++)
			{
				curaIrfan::PointIrfan p1 = polygon[(start_idx + point_idx) % polygon.size()];
				int p0p1_dist = curaIrfan::vSize(curaIrfan::operator-(p1 , p0));
				if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
				{
					curaIrfan::PointIrfan vector = curaIrfan::operator-(p1 , p0);
					curaIrfan::PointIrfan half_way = curaIrfan::operator+(p0 , curaIrfan::normal(vector, wall_0_wipe_dist - distance_traversed));
					addTravel_simple(layer_nr, half_way);
					break;
				}
				else
				{
					addTravel_simple(layer_nr, p1);
					distance_traversed += p0p1_dist;
				}
				p0 = p1;
			}
			forceNewPathStart();
		}
	}
	else
	{
		printf("WARNING: line added as polygon! (LayerPlan)\n");
	}
}

static const float max_non_bridge_line_volume = 100000.0f;
unsigned LayerPlan::locateFirstSupportedVertex(ConstPolygonRef wall, const unsigned start_idx) const
{
	if (bridge_wall_mask.empty() && overhang_mask.empty())
	{
		return start_idx;
	}

	Polygons air_below(bridge_wall_mask.unionPolygons(overhang_mask));

	unsigned curr_idx = start_idx;

	while (true)
	{
		const curaIrfan::PointIrfan & vertex = wall[curr_idx];
		if (!air_below.inside(vertex, true))
		{
			// vertex isn't above air so it's OK to use
			return curr_idx;
		}

		if (++curr_idx >= wall.size())
		{
			curr_idx = 0;
		}

		if (curr_idx == start_idx)
		{
			// no vertices are supported so just return the original index
			return start_idx;
		}
	}
}

void LayerPlan::addWallLine(const curaIrfan::PointIrfan& p0, const curaIrfan::PointIrfan& p1, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, float flow, float& non_bridge_line_volume, Ratio speed_factor, double distance_to_bridge_start)
{
	const coord_tIrfan min_line_len = 5; // we ignore lines less than 5um long
	const double acceleration_segment_len = 1000; // accelerate using segments of this length
	const double acceleration_factor = 0.85; // must be < 1, the larger the value, the slower the acceleration
	const bool spiralize = false;

	const coord_tIrfan min_bridge_line_len = MM2INT(5); 
	const Ratio bridge_wall_coast = Ratio(100 / 100);//mesh.settings.get<Ratio>("bridge_wall_coast");
	const Ratio overhang_speed_factor = Ratio(1);// mesh.settings.get<Ratio>("wall_overhang_speed_factor");

	curaIrfan::PointIrfan cur_point = p0;

	// helper function to add a single non-bridge line

	// If the line precedes a bridge line, it may be coasted to reduce the nozzle pressure before the bridge is reached

	// alternatively, if the line follows a bridge line, it may be segmented and the print speed gradually increased to reduce under-extrusion

	auto addNonBridgeLine = [&](const curaIrfan::PointIrfan& line_end)
	{										
		coord_tIrfan distance_to_line_end = curaIrfan::vSize(curaIrfan::operator-(cur_point , line_end));

		while (distance_to_line_end > min_line_len)
		{
			// if we are accelerating after a bridge line, the segment length is less than the whole line length
			curaIrfan::PointIrfan segment_end = (speed_factor == 1 || distance_to_line_end < acceleration_segment_len) ? line_end : curaIrfan::operator+( cur_point , (curaIrfan::operator*(curaIrfan::operator-(line_end, cur_point), acceleration_segment_len / distance_to_line_end)));

			// flow required for the next line segment - when accelerating after a bridge segment, the flow is increased in inverse proportion to the speed_factor
			// so the slower the feedrate, the greater the flow - the idea is to get the extruder back to normal pressure as quickly as possible
			const float segment_flow = (speed_factor < 1) ? flow * (1 / speed_factor) : flow;

			// if a bridge is present in this wall, this particular segment may need to be partially or wholely coasted
			if (distance_to_bridge_start > 0)
			{
				// speed_flow_factor approximates how the extrusion rate alters between the non-bridge wall line and the following bridge wall line
				// if the extrusion rates are the same, its value will be 1, if the bridge config extrusion rate is < the non-bridge config extrusion rate, the value is < 1

				const Ratio speed_flow_factor((bridge_config.getSpeed() * bridge_config.getFlowRatio()) / (non_bridge_config.getSpeed() * non_bridge_config.getFlowRatio()));

				// coast distance is proportional to distance, speed and flow of non-bridge segments just printed and is throttled by speed_flow_factor
				const double coast_dist = std::min(non_bridge_line_volume, max_non_bridge_line_volume) * (1 - speed_flow_factor) * bridge_wall_coast / 40;

				if ((distance_to_bridge_start - distance_to_line_end) <= coast_dist)
				{
					// coast takes precedence over acceleration
					segment_end = line_end;
				}

				const coord_tIrfan len = curaIrfan::vSize(curaIrfan::operator-(cur_point , segment_end));
				if (coast_dist > 0 && ((distance_to_bridge_start - len) <= coast_dist))
				{
					if ((len - coast_dist) > min_line_len)
					{
						// segment is longer than coast distance so extrude using non-bridge config to start of coast
						curaIrfan::PointIrfan point=  curaIrfan::operator+(segment_end, curaIrfan::operator*(coast_dist, curaIrfan::operator/((curaIrfan::operator-(cur_point, segment_end)), len)));
										
						addExtrusionMove(layer_thickness, non_bridge_config, point, layer_nr, SpaceFillType::Polygons, segment_flow, spiralize, speed_factor);
					}
					// then coast to start of bridge segment
					addExtrusionMove(layer_thickness, non_bridge_config, segment_end, layer_nr, SpaceFillType::Polygons, 0, spiralize, speed_factor);
				}
				else
				{
					// no coasting required, just normal segment using non-bridge config
					addExtrusionMove(layer_thickness, non_bridge_config, segment_end, layer_nr, SpaceFillType::Polygons, segment_flow, spiralize,
						(overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? speed_factor : overhang_speed_factor);
				}

				distance_to_bridge_start -= len;
			}
			else
			{
				// no coasting required, just normal segment using non-bridge config
				addExtrusionMove(layer_thickness, non_bridge_config, segment_end, layer_nr, SpaceFillType::Polygons, segment_flow, spiralize,
					(overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? speed_factor : overhang_speed_factor);
			}
			curaIrfan::PointIrfan flow_multiplier= curaIrfan::operator*(curaIrfan::operator*(segment_flow, speed_factor), non_bridge_config.getSpeed());
			non_bridge_line_volume += curaIrfan::vSize(curaIrfan::operator-(cur_point , segment_end)) * segment_flow * speed_factor * non_bridge_config.getSpeed();
			cur_point = segment_end;
			speed_factor = 1 - (1 - speed_factor) * acceleration_factor;
			distance_to_line_end = curaIrfan::vSize(curaIrfan::operator-(cur_point , line_end));
		}
	};

	if (bridge_wall_mask.empty())
	{
		// no bridges required
		addExtrusionMove(layer_thickness, non_bridge_config, p1, layer_nr, SpaceFillType::Polygons, flow, spiralize,
			(overhang_mask.empty() || (!overhang_mask.inside(p0, true) && !overhang_mask.inside(p1, true))) ? 1.0_r : overhang_speed_factor);
	}
	else
	{
		// bridges may be required
		if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0, p1))
		{
			// the line crosses the boundary between supported and non-supported regions so one or more bridges are required

			// determine which segments of the line are bridges

			Polygon line_poly;
			line_poly.add(p0);
			line_poly.add(p1);
			Polygons line_polys;
			line_polys.add(line_poly);
			line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

			// line_polys now contains the wall lines that need to be printed using bridge_config

			while (line_polys.size() > 0)
			{
				// find the bridge line segment that's nearest to the current point
				int nearest = 0;
				float smallest_dist2 = curaIrfan::vSize2f(curaIrfan::operator-(cur_point , line_polys[0][0]));
				for (unsigned i = 1; i < line_polys.size(); ++i)
				{
					float dist2 = curaIrfan::vSize2f(curaIrfan::operator-(cur_point , line_polys[i][0]));
					if (dist2 < smallest_dist2)
					{
						nearest = i;
						smallest_dist2 = dist2;
					}
				}
				ConstPolygonRef bridge = line_polys[nearest];

				// set b0 to the nearest vertex and b1 the furthest
				curaIrfan::PointIrfan b0 = bridge[0];
				curaIrfan::PointIrfan b1 = bridge[1];

				if (curaIrfan::vSize2f(curaIrfan::operator-(cur_point , b1)) < curaIrfan::vSize2f(curaIrfan::operator-(cur_point , b0)))
				{
					// swap vertex order
					b0 = bridge[1];
					b1 = bridge[0];
				}

				// extrude using non_bridge_config to the start of the next bridge segment

				addNonBridgeLine(b0);

				const double bridge_line_len = curaIrfan::vSize(curaIrfan::operator-(b1 , cur_point));

				if (bridge_line_len >= min_bridge_line_len)
				{
					// extrude using bridge_config to the end of the next bridge segment

					if (bridge_line_len > min_line_len)
					{
						addExtrusionMove(layer_thickness , bridge_config, b1, layer_nr, SpaceFillType::Polygons, flow);
						non_bridge_line_volume = 0;
						cur_point = b1;
						// after a bridge segment, start slow and accelerate to avoid under-extrusion due to extruder lag
						speed_factor = std::min(Ratio(bridge_config.getSpeed() / non_bridge_config.getSpeed()), 1.0_r);
					}
				}
				else
				{
					// treat the short bridge line just like a normal line

					addNonBridgeLine(b1);
				}

				// finished with this segment
				line_polys.remove(nearest);
			}

			// if we haven't yet reached p1, fill the gap with non_bridge_config line
			addNonBridgeLine(p1);
		}
		else if (bridge_wall_mask.inside(p0, true) && curaIrfan::vSize(curaIrfan::operator-(p0 , p1)) >= min_bridge_line_len)
		{
			// both p0 and p1 must be above air (the result will be ugly!)
			addExtrusionMove(layer_thickness, bridge_config, p1, layer_nr ,SpaceFillType::Polygons, flow);
			non_bridge_line_volume = 0;
		}
		else
		{
			// no part of the line is above air or the line is too short to print as a bridge line
			addNonBridgeLine(p1);
		}
	}
}

void LayerPlan::addWall(ConstPolygonRef wall, int start_idx, const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, coord_tIrfan wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
	// make sure wall start point is not above air!
	start_idx = locateFirstSupportedVertex(wall, start_idx);

	float non_bridge_line_volume = max_non_bridge_line_volume; // assume extruder is fully pressurised before first non-bridge line is output
	double speed_factor = 1.0; // start first line at normal speed
	coord_tIrfan distance_to_bridge_start = 0; // will be updated before each line is processed

	const coord_tIrfan min_bridge_line_len = MM2INT(5);// mesh.settings.get<coord_tIrfan>("bridge_wall_min_length");
	const Ratio wall_min_flow = 0;// mesh.settings.get<Ratio>("wall_min_flow");
	const bool wall_min_flow_retract = false;// mesh.settings.get<bool>("wall_min_flow_retract");

	// helper function to calculate the distance from the start of the current wall line to the first bridge segment

	auto computeDistanceToBridgeStart = [&](unsigned current_index)
	{
		distance_to_bridge_start = 0;

		if (!bridge_wall_mask.empty())
		{
			// there is air below the part so iterate through the lines that have not yet been output accumulating the total distance to the first bridge segment
			for (unsigned point_idx = current_index; point_idx < wall.size(); ++point_idx)
			{
				const curaIrfan::PointIrfan& p0 = wall[point_idx];
				const curaIrfan::PointIrfan& p1 = wall[(point_idx + 1) % wall.size()];

				if (PolygonUtils::polygonCollidesWithLineSegment(bridge_wall_mask, p0, p1))
				{
					// the line crosses the boundary between supported and non-supported regions so it will contain one or more bridge segments

					// determine which segments of the line are bridges

					Polygon line_poly;
					line_poly.add(p0);
					line_poly.add(p1);
					Polygons line_polys;
					line_polys.add(line_poly);
					line_polys = bridge_wall_mask.intersectionPolyLines(line_polys);

					while (line_polys.size() > 0)
					{
						// find the bridge line segment that's nearest to p0
						int nearest = 0;
						float smallest_dist2 = curaIrfan::vSize2f(curaIrfan::operator-(p0 , line_polys[0][0]));
						for (unsigned i = 1; i < line_polys.size(); ++i)
						{
							float dist2 = curaIrfan::vSize2f(curaIrfan::operator-(p0 , line_polys[i][0]));
							if (dist2 < smallest_dist2)
							{
								nearest = i;
								smallest_dist2 = dist2;
							}
						}
						ConstPolygonRef bridge = line_polys[nearest];

						// set b0 to the nearest vertex and b1 the furthest
						curaIrfan::PointIrfan b0 = bridge[0];
						curaIrfan::PointIrfan b1 = bridge[1];

						if (curaIrfan::vSize2f(curaIrfan::operator-(p0 , b1)) < curaIrfan::vSize2f(curaIrfan::operator-(p0 , b0)))
						{
							// swap vertex order
							b0 = bridge[1];
							b1 = bridge[0];
						}

						distance_to_bridge_start += curaIrfan::vSize(curaIrfan::operator-(b0 , p0));

						const double bridge_line_len = curaIrfan::vSize(curaIrfan::operator-(b1 , b0));

						if (bridge_line_len >= min_bridge_line_len)
						{
							// job done, we have found the first bridge line
							return;
						}

						distance_to_bridge_start += bridge_line_len;

						// finished with this segment
						line_polys.remove(nearest);
					}
				}
				else if (!bridge_wall_mask.inside(p0, true))
				{
					// none of the line is over air
					distance_to_bridge_start += curaIrfan::vSize(curaIrfan::operator-(p1 , p0));
				}
			}

			// we have got all the way to the end of the wall without finding a bridge segment so disable coasting by setting distance_to_bridge_start back to 0

			distance_to_bridge_start = 0;
		}
	};

	bool travel_required = false; // true when a wall has been omitted due to its flow being less than the minimum required

	bool first_line = true;

	curaIrfan::PointIrfan p0 = wall[start_idx];

	for (unsigned int point_idx = 1; point_idx < wall.size(); point_idx++)
	{
		const curaIrfan::PointIrfan& p1 = wall[(start_idx + point_idx) % wall.size()];
		const float flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;

		if (!bridge_wall_mask.empty())
		{
			computeDistanceToBridgeStart((start_idx + point_idx - 1) % wall.size());
		}

		if (flow >= wall_min_flow)
		{
			if (first_line || travel_required)
			{
				addTravel(layer_thickness,layer_nr, p0, (first_line) ? always_retract : wall_min_flow_retract);
				first_line = false;
				travel_required = false;
			}
			addWallLine(p0, p1,non_bridge_config, bridge_config, flow, non_bridge_line_volume, speed_factor, distance_to_bridge_start);
		}
		else
		{
			travel_required = true;
		}

		p0 = p1;
	}

	if (wall.size() > 2)
	{
		const curaIrfan::PointIrfan& p1 = wall[start_idx];
		const float flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;

		if (!bridge_wall_mask.empty())
		{
			computeDistanceToBridgeStart((start_idx + wall.size() - 1) % wall.size());
		}

		if (flow >= wall_min_flow)
		{
			if (travel_required)
			{
				addTravel(layer_thickness, layer_nr, p0, wall_min_flow_retract);
			}
			addWallLine(p0, p1, non_bridge_config, bridge_config, flow, non_bridge_line_volume, speed_factor, distance_to_bridge_start);

			if (wall_0_wipe_dist > 0)
			{ // apply outer wall wipe
				p0 = wall[start_idx];
				int distance_traversed = 0;
				for (unsigned int point_idx = 1; ; point_idx++)
				{
					curaIrfan::PointIrfan p1 = wall[(start_idx + point_idx) % wall.size()];
					int p0p1_dist = curaIrfan::vSize(curaIrfan::operator-( p1 , p0));
					if (distance_traversed + p0p1_dist >= wall_0_wipe_dist)
					{
						curaIrfan::PointIrfan vector = curaIrfan::operator-(p1, p0);
						curaIrfan::PointIrfan half_way = curaIrfan::operator+(p0 , curaIrfan::normal(vector, wall_0_wipe_dist - distance_traversed));
						addTravel_simple(layer_nr, half_way);
						break;
					}
					else
					{
						addTravel_simple(layer_nr, p1);
						distance_traversed += p0p1_dist;
					}
					p0 = p1;
				}
				forceNewPathStart();
			}
		}
	}
	else
	{
		printf("WARNING: line added as polygon! (LayerPlan)\n");
	}
}

void LayerPlan::addWalls(const Polygons& walls,const GCodePathConfig& non_bridge_config, const GCodePathConfig& bridge_config, WallOverlapComputation* wall_overlap_computation, const ZSeamConfig& z_seam_config, coord_tIrfan wall_0_wipe_dist, float flow_ratio, bool always_retract)
{
	printf("the walls size is %d \for the layer number %d \n", walls.size(), layer_nr);
	PathOrderOptimizer orderOptimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config);
	for (unsigned int poly_idx = 0; poly_idx < walls.size(); poly_idx++)
	{
		orderOptimizer.addPolygon(walls[poly_idx]);
	}
	orderOptimizer.optimize();
	for (unsigned int poly_idx : orderOptimizer.polyOrder)
	{
		addWall(walls[poly_idx], orderOptimizer.polyStart[poly_idx], non_bridge_config, bridge_config, wall_overlap_computation, wall_0_wipe_dist, flow_ratio, always_retract);
	}
}

void LayerPlan::moveInsideCombBoundary(int layernum,const coord_tIrfan distance)
{
	constexpr coord_tIrfan max_dist2 = MM2INT(2.0) * MM2INT(2.0); // if we are further than this distance, we conclude we are not inside even though we thought we were.
	// this function is to be used to move from the boudary of a part to inside the part
	curaIrfan::PointIrfan p = getLastPlannedPositionOrStartingPosition(); // copy, since we are going to move p
	
	if (PolygonUtils::moveInside(comb_boundary_inside2, p, distance, max_dist2) != NO_INDEX)
	{
		//Move inside again, so we move out of tight 90deg corners
		PolygonUtils::moveInside(comb_boundary_inside2, p, distance, max_dist2);
		if (comb_boundary_inside2.inside(p))
		{
			addTravel_simple(layernum, p);
			//Make sure the that any retraction happens after this move, not before it by starting a new move path.
			forceNewPathStart();
		}
	}
}
             


void LayerPlan::addPolygonsByOptimizer(coord_tIrfan layer_thickness, int layer_nr,const Polygons& polygons, const GCodePathConfig& config, WallOverlapComputation* wall_overlap_computation, const ZSeamConfig& z_seam_config, coord_tIrfan wall_0_wipe_dist, bool spiralize, const double flow_ratio, bool always_retract, bool reverse_order)
{
	if (polygons.size() == 0)
	{
		return;
	}
	PathOrderOptimizer orderOptimizer(getLastPlannedPositionOrStartingPosition(), z_seam_config);
	for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
	{
		orderOptimizer.addPolygon(polygons[poly_idx]);
	}
	orderOptimizer.optimize();

	if (reverse_order == false)
	{
		for (unsigned int poly_idx : orderOptimizer.polyOrder)
		{
			addPolygon(layer_thickness, layer_nr, polygons[poly_idx], orderOptimizer.polyStart[poly_idx], config, wall_overlap_computation, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
		}
	}
	else
	{
		for (int index = orderOptimizer.polyOrder.size() - 1; index >= 0; --index)
		{
			int poly_idx = orderOptimizer.polyOrder[index];
			addPolygon(layer_thickness, layer_nr, polygons[poly_idx], orderOptimizer.polyStart[poly_idx], config, wall_overlap_computation, wall_0_wipe_dist, spiralize, flow_ratio, always_retract);
		}
	}
}



GCodePath& LayerPlan::addTravel(coord_tIrfan layer_thickness, int layernum, curaIrfan::PointIrfan  p, bool force_comb_retract)
{
	const GCodePathConfig& travel_config = configs_storage.travel_config_per_extruder[getExtruder()];
	const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];
	
	
	GCodePath* path = getLatestPathWithConfig(layer_thickness, travel_config, SpaceFillType::None);
	
	bool combed = false;
	bool retraction_hop_after_extruder_switch = true;
//	const ExtruderTrain* extruder = getLastPlannedExtruderTrain();

	const coord_tIrfan maximum_travel_resolution = MM2INT(0.2857142857142857);//extruder->settings.get<coord_t>("meshfix_maximum_travel_resolution");

	const bool is_first_travel_of_extruder_after_switch = extruder_plans.back().paths.size() == 1 && (extruder_plans.size() > 1 || last_extruder_previous_layer != getExtruder());
	bool bypass_combing = is_first_travel_of_extruder_after_switch && retraction_hop_after_extruder_switch;
	const bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
	if (is_first_travel_of_layer)
	{
		
		bypass_combing = true; // first travel move is bogus; it is added after this and the previous layer have been planned in LayerPlanBuffer::addConnectingTravelMove
		first_travel_destination = p;
		first_travel_destination_is_inside = is_inside;
		forceNewPathStart(); // force a new travel path after this first bogus move
	}
	
	if (comb != nullptr && !bypass_combing)
	{
		printf("*****************BIGWARNING INSIDE COMB*************\n");
		
		/*
		CombPaths combPaths;

		// Divide by 2 to get the radius
		// Multiply by 2 because if two lines start and end points places very close then will be applied combing with retractions. (Ex: for brim)
		const coord_t max_distance_ignored = extruder->settings.get<coord_t>("machine_nozzle_tip_outer_diameter") / 2 * 2;

		combed = comb->calc(*extruder, *last_planned_position, p, combPaths, was_inside, is_inside, max_distance_ignored);
		if (combed)
		{
			bool retract = path->retract || combPaths.size() > 1;
			if (!retract)
			{ // check whether we want to retract
				if (combPaths.throughAir)
				{
					retract = true;
				}
				else
				{
					for (CombPath& combPath : combPaths)
					{ // retract when path moves through a boundary
						if (combPath.cross_boundary)
						{
							retract = true;
							break;
						}
					}
				}
				if (combPaths.size() == 1)
				{
					CombPath comb_path = combPaths[0];
					if (extruder->settings.get<bool>("limit_support_retractions") &&
						combPaths.throughAir && !comb_path.cross_boundary && comb_path.size() == 2 && comb_path[0] == *last_planned_position && comb_path[1] == p)
					{ // limit the retractions from support to support, which didn't cross anything
						retract = false;
					}
				}
			}

			coord_t distance = 0;
			Point last_point((last_planned_position) ? *last_planned_position : Point(0, 0));
			for (CombPath& combPath : combPaths)
			{ // add all comb paths (don't do anything special for paths which are moving through air)
				if (combPath.size() == 0)
				{
					continue;
				}
				for (Point& comb_point : combPath)
				{
					if (path->points.empty() || vSize2(path->points.back() - comb_point) > maximum_travel_resolution * maximum_travel_resolution)
					{
						path->points.push_back(comb_point);
						distance += vSize(last_point - comb_point);
						last_point = comb_point;
					}
				}
				last_planned_position = combPath.back();
				distance += vSize(last_point - p);
				const coord_t retract_threshold = extruder->settings.get<coord_t>("retraction_combing_max_distance");
				path->retract = retract || (retract_threshold > 0 && distance > retract_threshold);
				// don't perform a z-hop
			}
		}
		*/
	}
	
	// no combing? retract only when path is not shorter than minimum travel distance
	if (!combed && !is_first_travel_of_layer && last_planned_position && !curaIrfan::shorterThen(curaIrfan::operator-(*last_planned_position, p), retraction_config.retraction_min_travel_distance))
	{
		if (was_inside) // when the previous location was from printing something which is considered inside (not support or prime tower etc)
		{               // then move inside the printed part, so that we don't ooze on the outer wall while retraction, but on the inside of the print.
			//assert(extruder != nullptr);
			size_t wall_line_count = 3;
			coord_tIrfan wall_line_width_x = MM2INT(0.3);
			coord_tIrfan wall_line_width_0 = MM2INT(0.35);
			coord_tIrfan innermost_wall_line_width = ((wall_line_count > 1) ? wall_line_width_x : wall_line_width_0);
			if (layer_nr == 0)
			{
				Ratio initial_layer_line_width_factor = Ratio(120/100);
			}
			moveInsideCombBoundary(layernum, innermost_wall_line_width);
		}
		path->retract = true;
		path->perform_z_hop = true;
	}

	GCodePath& ret = addTravel_simple(layernum, p, path);
	was_inside = is_inside;
	return ret;
}

GCodePath& LayerPlan::addTravel_simple(int layer_nr,curaIrfan::PointIrfan p, GCodePath* path)
{
	
	bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
	if (is_first_travel_of_layer)
	{ // spiralize calls addTravel_simple directly as the first travel move in a layer
		first_travel_destination = p;
		first_travel_destination_is_inside = is_inside;
	}
	
	if (path == nullptr)
	{
		path = getLatestPathWithConfig(layer_thickness, configs_storage.travel_config_per_extruder[getExtruder()], SpaceFillType::None);
	}
	path->points.push_back(p);
	last_planned_position = p;
	return *path;
}

void LayerPlan::optimizePaths(const curaIrfan::PointIrfan& starting_position)
{
	for (ExtruderPlan& extr_plan : extruder_plans)
	{
		//Merge paths whose endpoints are very close together into one line.
		//printf("extruder_plans has the paths of size %d \n", extruder_plans[0].paths.size());
		//create the gcodepath config setup for this function
		MergeInfillLines merger(extr_plan);
		merger.mergeInfillLines(extr_plan.paths, starting_position);
		//printf("succesffullyad  optimized the paths \n");
	}
}

bool LayerPlan::ProcessCofigs_Storage()
{
	PathConfigStorage::PathConfigStorage(storage, layer_nr, layer_thickness);
	return true;
}
