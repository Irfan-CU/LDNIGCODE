
#include <cstring>

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
 
GCodePath* LayerPlan::getLatestPathWithConfig(coord_tIrfan layer_thickness, int layernum, SpaceFillType space_fill_type, const double flow, bool spiralize, const double speed_factor)
{

	std::vector<GCodePath>& paths = extruder_plans.back().paths;
	if (paths.size() > 0 && !paths.back().done && paths.back().flow1 == flow && paths.back().speed_factor == speed_factor && paths.back().mesh_id == current_mesh) // spiralize can only change when a travel path is in between
	{
		return &paths.back();
	}

	double speed = 60;
	double accelration = 1300;
	double Jerk = 25;
	PrintFeatureType feature = PrintFeatureType::Infill;
	coord_tIrfan line_width = MM2INT(0.42);
	
	double flowconfig = 100 * ((layer_nr == 0) ? 100 : double(1.0));
	std::string current_mesh = "Mesh1";
	
	paths.emplace_back(speed, accelration, Jerk, feature, line_width, layer_thickness,flowconfig, current_mesh, space_fill_type, flow, spiralize, layernum, speed_factor);
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
	bool comb = false;
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
 
void LayerPlan::addLinesByOptimizer(coord_tIrfan layer_thickness , const Polygons& polygons, int layernum, SpaceFillType space_fill_type, bool enable_travel_optimization, int wipe_dist, float flow_ratio, std::optional<curaIrfan::PointIrfan> near_start_location, double fan_speed)
{
	printf("inside the add Linesoptimer \n");
	Polygons boundary;
	LineOrderOptimizer orderOptimizer(near_start_location.value_or(getLastPlannedPositionOrStartingPosition()), &boundary);
	printf("the polygon sizse for the line order optimizer is %d \n", polygons.size());
	
	for (unsigned int line_idx = 0; line_idx < polygons.size(); line_idx++)
	{
		orderOptimizer.addPolygon(polygons[line_idx]);
	}
	orderOptimizer.optimize();
	printf("the loop after line order optimizer is satisfied and polyorder size is %d \n", orderOptimizer.polyOrder.size());
	for (unsigned int order_idx = 0; order_idx < orderOptimizer.polyOrder.size(); order_idx++)
	{
		const unsigned int poly_idx = orderOptimizer.polyOrder[order_idx];
		ConstPolygonRef polygon = polygons[poly_idx];
		const size_t start = orderOptimizer.polyStart[poly_idx];
		const size_t end = 1 - start;
		const curaIrfan::PointIrfan& p0 = polygon[start];
		addTravel(layer_thickness, layernum,p0);
		const curaIrfan::PointIrfan& p1 = polygon[end];
		//const GCodePathConfig *config;
		//addExtrusionMove(p1,space_fill_type, flow_ratio, false, 1.0, fan_speed);
		addExtrusionMove(layer_thickness,p1, layernum, space_fill_type, flow_ratio, false, 1.0, fan_speed);
	}
	printf("***** the extruder plans and paths size are %d ", extruder_plans[0].paths.size());
	
}

void LayerPlan::addExtrusionMove(coord_tIrfan layer_thickness,curaIrfan::PointIrfan p, int layernum, SpaceFillType space_fill_type, const double& flow, bool spiralize, double speed_factor, double fan_speed)
{
	
	GCodePath* path = getLatestPathWithConfig(layer_thickness, layernum,space_fill_type, flow, spiralize, speed_factor);
	path->points.push_back(p);
	path->setFanSpeed(fan_speed);
	last_planned_position = p;
	
}
 
void LayerPlan::setMesh(const std::string mesh_id)
{
	current_mesh = mesh_id;
}

void LayerPlan::writeGCode(GCodeExport& gcode)
{
	coord_tIrfan layer_thicnkess = storage.Layers[0].thickness;
	gcode.setLayerNr(layer_nr);
	printf("done with  setLayerNr\n");
	gcode.writeLayerComment(layer_nr);

	// flow-rate compensation
	//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	gcode.setFlowRateExtrusionSettings(0, Ratio(100)); //Offset is in mm.

	if (layer_nr == 0 ) //- true and 60;........static_cast<LayerIndex>(Raft::getTotalExtraLayers()) && mesh_group_settings.get<bool>("machine_heated_bed") && mesh_group_settings.get<Temperature>("material_bed_temperature") != 0)
	{
		constexpr bool wait = false;
		gcode.writeBedTemperatureCommand(60, wait);
	}

	gcode.setZ(z);
	printf("done with  setz\n");
	PrintFeatureType last_extrusion_config; // used to check whether we need to insert a TYPE comment in the gcode.

	size_t extruder_nr = gcode.getExtruderNr();
	const bool acceleration_enabled = true;// mesh_group_settings.get<bool>("acceleration_enabled");
	const bool jerk_enabled = true;// mesh_group_settings.get<bool>("jerk_enabled");
	
	for (size_t extruder_plan_idx = 0; extruder_plan_idx < extruder_plans.size(); extruder_plan_idx++)
	{
		ExtruderPlan& extruder_plan = extruder_plans[extruder_plan_idx];
		printf("got the extruder plan \n");
		const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder_plan.extruder_nr];
		coord_tIrfan z_hop_height = retraction_config.zHop;
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
		printf("wrote the fan command and going for the extruderplan.path\n");
		std::vector<GCodePath>& paths = extruder_plan.paths;
		printf("extruderplan.path size is %d \n", extruder_plan.paths.size());
		extruder_plan.inserts.sort([](const NozzleTempInsert& a, const NozzleTempInsert& b) -> bool
		{
			return  a.path_idx < b.path_idx;
		});
		printf("sorted the paths here and the sorted paths size is %d \n",paths.size());
		//const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[extruder_nr];

		bool update_extrusion_offset = true;

		for (unsigned int path_idx = 0; path_idx < paths.size(); path_idx++)
		{
			extruder_plan.handleInserts(path_idx, gcode);

			GCodePath& path = paths[path_idx];

			if (path.perform_prime)
			{
				gcode.writePrimeTrain(250, layer_thicnkess);
				gcode.writeRetraction(retraction_config);
			}

			if (!path.retract && path.isTravelPath() && path.points.size() == 1 && path.points[0] == gcode.getPositionXY() && z == gcode.getPositionZ())
			{
				printf(" warning the condition at line 203 is correct");
				// ignore travel moves to the current location to avoid needless change of acceleration/jerk
				continue;
			}

			if (acceleration_enabled)
			{
				printf("writing print accelrations \n");
				if (path.isTravelPath())
				{
					gcode.writeTravelAcceleration(path.getAccelration());
				}
				else
				{
					gcode.writePrintAcceleration(path.getAccelration());
				}
			}

			if (jerk_enabled)
			{
				gcode.writeJerk(path.getJerk());
			}

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
			if (!path.isTravelPath() && last_extrusion_config != path.type)
			{
				PrintFeatureType type = PrintFeatureType::Infill;

				gcode.writeTypeComment(type);
				last_extrusion_config = path.type;
				update_extrusion_offset = true;
			}
			else
			{
				update_extrusion_offset = false;
			}

			double speed = path.getSpeed();

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
						gcode.writeExtrusion(path.points[point_idx], speed, path.getExtrusionMM3perMM(), path.type, update_extrusion_offset, layer_thicnkess);
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

void GCodeExport::writeZhopEnd(double speed/*= 0*/)
{
	if (is_z_hopped)
	{
		if (speed == 0)
		{
			//const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[current_extruder];
			speed = 10.0;
		}
		is_z_hopped = 0;
		currentPosition.z = current_layer_z;
		currentSpeed = speed;
		*output_stream << "G1 F" << speed * 60 << " Z" << new_line; //MMtoStream{ current_layer_z } << new_line;
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
		addExtrusionMove(layer_thickness, p1, layer_nr, SpaceFillType::Polygons, flow, spiralize);
		p0 = p1;
	}
	if (polygon.size() > 2)
	{
		const curaIrfan::PointIrfan& p1 = polygon[start_idx];
		const Ratio flow = (wall_overlap_computation) ? flow_ratio * wall_overlap_computation->getFlow(p0, p1) : flow_ratio;
		addExtrusionMove(layer_thickness, p1, layer_nr, SpaceFillType::Polygons, flow, spiralize);

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
	printf("inside add travel \n");
	const GCodePathConfig& travel_config = configs_storage.travel_config_per_extruder[getExtruder()];
	const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[getExtruder()];
	
	GCodePath* path = getLatestPathWithConfig(layer_thickness, layernum, SpaceFillType::None);
	
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
	else if (force_comb_retract && last_planned_position && !curaIrfan::shorterThen(curaIrfan::operator-(*last_planned_position,p), retraction_config.retraction_min_travel_distance))
	{
		printf("inside a very wrong place \n");
		/*
		// path is not shorter than min travel distance, force a retraction
		path->retract = true;
		if (comb == nullptr)
		{
			path->perform_z_hop = extruder->settings.get<bool>("retraction_hop_enabled");
		}
		*/
	}
	/*
	if (comb != nullptr && !bypass_combing)
	{
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
	}
	*/
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
				double initial_layer_line_width_factor = 120;
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

GCodePath& LayerPlan::addTravel_simple(int layernum, curaIrfan::PointIrfan p, GCodePath* path)
{
	printf("inside att travel simple \n");
	bool is_first_travel_of_layer = !static_cast<bool>(last_planned_position);
	if (is_first_travel_of_layer)
	{ // spiralize calls addTravel_simple directly as the first travel move in a layer
		first_travel_destination = p;
		first_travel_destination_is_inside = is_inside;
	}
	
	if (path == nullptr)
	{
		path = getLatestPathWithConfig(layer_thickness, layernum,SpaceFillType::None);
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
		printf("extruder_plans has the paths of size %d \n", extruder_plans[0].paths.size());
		//create the gcodepath config setup for this function
		//	MergeInfillLines merger(extr_plan);
		//merger.mergeInfillLines(extr_plan.paths, starting_position);
		//printf("succesffullyad  optimized the paths \n");
	}
}
