//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.



#include "gcodeExport.h"
#include "LayerPlan.h"
#include "LayerBufferPlan.h"

	constexpr double LayerPlanBuffer::extra_preheat_time;

	void LayerPlanBuffer::push(LayerPlan& layer_plan)
	{
		buffer.push_back(&layer_plan);
	}

	void LayerPlanBuffer::handle(LayerPlan& layer_plan, GCodeExport& gcode)
	{
		push(layer_plan);
		
		int layernum = layer_plan.getLayerNr();
	
		coord_tIrfan layer_thickness = layer_plan.layer_thickness;
		layer_plan.layer_parts;
		
		LayerPlan* to_be_written = processBuffer(layernum, layer_thickness);
		
		if (to_be_written)
		{
			
			to_be_written->writeGCode(gcode);
			delete to_be_written;
		}
	}

	LayerPlan* LayerPlanBuffer::processBuffer(int layernum, coord_tIrfan layer_thickness)
	{
		
		
		if (buffer.empty())
		{
			
			return nullptr;
		}
		processFanSpeedLayerTime();
		
		if (buffer.size() >= 2)
		{
			addConnectingTravelMove(layer_thickness, layernum, *--(--buffer.end()), *--buffer.end());
		}
		if (buffer.size() > 0)
		{
			insertTempCommands(); 
		}
		if (buffer.size() > buffer_size)
		{
			LayerPlan* ret = buffer.front();
			
			buffer.pop_front();
			
			return ret;
		}
		return nullptr;
		
	}

	
	void LayerPlanBuffer::flush()
	{
		
		if (buffer.size() > 0)
		{
			insertTempCommands(); 
		}
		while (!buffer.empty())
		{
			buffer.front()->writeGCode(gcode);
			delete buffer.front();
			buffer.pop_front();
		}
	}
	
	
	void LayerPlanBuffer::addConnectingTravelMove(coord_tIrfan layer_thickness, int layernum, LayerPlan* prev_layer, const LayerPlan* newest_layer)
	{
		
		std::optional<std::pair<curaIrfan::PointIrfan, bool>> new_layer_destination_state = newest_layer->getFirstTravelDestinationState();

		if (!new_layer_destination_state)
		{
			
			return;
		}

		curaIrfan::PointIrfan first_location_new_layer = new_layer_destination_state->first;

		assert(newest_layer->extruder_plans.front().extruder_nr == prev_layer->extruder_plans.back().extruder_nr);
		assert(newest_layer->extruder_plans.front().paths.size() > 0);
		assert(newest_layer->extruder_plans.front().paths[0].points.size() == 1);
		assert(newest_layer->extruder_plans.front().paths[0].points[0] == first_location_new_layer);
		
		if (!prev_layer->last_planned_position || *prev_layer->last_planned_position != first_location_new_layer)
		{
			prev_layer->setIsInside(new_layer_destination_state->second);
			const bool retract_at_layer_change = false;
			bool travel_retract_before_outer_wall = true;
			bool outer_inset_first = false;
			size_t wall_line_count = 3;

			const bool force_retract = retract_at_layer_change||travel_retract_before_outer_wall && (outer_inset_first|| wall_line_count== 1); //Moving towards an outer wall.
			prev_layer->final_travel_z = newest_layer->z;
			prev_layer->addTravel(layer_thickness, layernum, first_location_new_layer, force_retract);
		}
	} 

	void LayerPlanBuffer::processFanSpeedLayerTime()
	{
		assert(buffer.size() > 0);
		auto newest_layer_it = --buffer.end();
		// Assume the print head is homed at the start of a mesh group.
		// This introduces small inaccuracies for the naive layer time estimates of the first layer of the second mesh group.
		// It's not that bad, though. They are naive estimates any way.
		curaIrfan::PointIrfan starting_position(0, 0);
		if (buffer.size() >= 2)
		{
			auto prev_layer_it = newest_layer_it;
			prev_layer_it--;
			const LayerPlan* prev_layer = *prev_layer_it;
			starting_position = prev_layer->getLastPlannedPositionOrStartingPosition();
		}
		LayerPlan* newest_layer = *newest_layer_it;
		newest_layer->processFanSpeedAndMinimalLayerTime(starting_position);
	}
	 	 
	void LayerPlan::processFanSpeedAndMinimalLayerTime(curaIrfan::PointIrfan starting_position)
	{
		for (unsigned int extr_plan_idx = 0; extr_plan_idx < extruder_plans.size(); extr_plan_idx++)
		{
			ExtruderPlan& extruder_plan = extruder_plans[extr_plan_idx];
			bool force_minimal_layer_time = extr_plan_idx == extruder_plans.size() - 1;
			extruder_plan.processFanSpeedAndMinimalLayerTime(force_minimal_layer_time, starting_position);
			if (!extruder_plan.paths.empty() && !extruder_plan.paths.back().points.empty())
			{
				starting_position = extruder_plan.paths.back().points.back();
			}
		}
	}

	void ExtruderPlan::forceMinimalLayerTime(double minTime, double minimalSpeed, double travelTime, double extrudeTime)
	{
		double totalTime = travelTime + extrudeTime;
		if (totalTime < minTime && extrudeTime > 0.0)
		{
			double minExtrudeTime = minTime - travelTime;
			if (minExtrudeTime < 1)
				minExtrudeTime = 1;
			double factor = extrudeTime / minExtrudeTime;
			for (GCodePath& path : paths)
			{
				if (path.isTravelPath())
					continue;
				double speed = path.config->getSpeed() * factor;
				if (speed < minimalSpeed)
					factor = minimalSpeed / path.config->getSpeed();
			}

			//Only slow down for the minimal time if that will be slower.
			assert(getExtrudeSpeedFactor() == 1.0); // The extrude speed factor is assumed not to be changed yet
			
			double inv_factor = 1.0 / factor; // cause multiplication is faster than division

			// Adjust stored naive time estimates
			estimates.extrude_time *= inv_factor;
			for (GCodePath& path : paths)
			{
				path.estimates.extrude_time *= inv_factor;
			}

			if (minTime - (extrudeTime * inv_factor) - travelTime > 0.1)
			{
				extraTime = minTime - (extrudeTime * inv_factor) - travelTime;
			}
			totalPrintTime = (extrudeTime * inv_factor) + travelTime;
		}
	}

	void ExtruderPlan::processFanSpeedAndMinimalLayerTime(bool force_minimal_layer_time, curaIrfan::PointIrfan starting_position)
	{
		//TimeMaterialEstimates estimates = computeNaiveTimeEstimates(starting_position);
		//totalPrintTime = estimates.getTotalTime();
		if (force_minimal_layer_time)
		{
			forceMinimalLayerTime(fan_speed_layer_time_settings.cool_min_layer_time, fan_speed_layer_time_settings.cool_min_speed, estimates.getTravelTime(), estimates.getExtrudeTime());
		}

		/*
					   min layer time
					   :
					   :  min layer time fan speed min
					|  :  :
		  ^    max..|__:  :
					|  \  :
		 fan        |   \ :
		speed  min..|... \:___________
					|________________
					  layer time >


		*/
		// interpolate fan speed (for cool_fan_full_layer and for cool_min_layer_time_fan_speed_max)
		fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
		double totalLayerTime = estimates.unretracted_travel_time + estimates.extrude_time;
		if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time)
		{
			fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max;
		}
		else if (fan_speed_layer_time_settings.cool_min_layer_time >= fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
		{
			fan_speed = fan_speed_layer_time_settings.cool_fan_speed_min;
		}
		else if (force_minimal_layer_time && totalLayerTime < fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max)
		{
			// when forceMinimalLayerTime didn't change the extrusionSpeedFactor, we adjust the fan speed
			double fan_speed_diff = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_layer_time_settings.cool_fan_speed_min;
			double layer_time_diff = fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max - fan_speed_layer_time_settings.cool_min_layer_time;
			double fraction_of_slope = (totalLayerTime - fan_speed_layer_time_settings.cool_min_layer_time) / layer_time_diff;
			fan_speed = fan_speed_layer_time_settings.cool_fan_speed_max - fan_speed_diff * fraction_of_slope;
		}
		/*
		Supposing no influence of minimal layer time;
		i.e. layer time > min layer time fan speed min:

				  max..   fan 'full' on layer
					   |  :
					   |  :
		  ^       min..|..:________________
		 fan           |  /
		speed          | /
			  speed_0..|/
					   |
					   |__________________
						 layer nr >

		*/
		if (layer_nr < fan_speed_layer_time_settings.cool_fan_full_layer
			&& fan_speed_layer_time_settings.cool_fan_full_layer > 0 // don't apply initial layer fan speed speedup if disabled.
			&& !is_raft_layer // don't apply initial layer fan speed speedup to raft, but to model layers
			)
		{
			//Slow down the fan on the layers below the [cool_fan_full_layer], where layer 0 is speed 0.
			fan_speed = fan_speed_layer_time_settings.cool_fan_speed_0 + (fan_speed - fan_speed_layer_time_settings.cool_fan_speed_0) * std::max(int(0), layer_nr) / fan_speed_layer_time_settings.cool_fan_full_layer;
		}
	}

	void LayerPlanBuffer::insertTempCommands()
	{
		//printf("the buffer.back()->extruder_plans.size() is %d \n ", buffer.back()->extruder_plans[1].paths.size());
		if (buffer.back()->extruder_plans.size() == 0 || (buffer.back()->extruder_plans.size() == 1 && buffer.back()->extruder_plans[0].paths.size() == 0))
		{ // disregard empty layer
			buffer.pop_back();
			return;
		}
		std::vector<ExtruderPlan*> extruder_plans; // sorted in print order
		extruder_plans.reserve(buffer.size() * 2);
		for (LayerPlan* layer_plan : buffer)
		{
			for (ExtruderPlan& extr_plan : layer_plan->extruder_plans)
			{
				extruder_plans.push_back(&extr_plan);
			}
		}

		// insert commands for all extruder plans on this layer
		//Scene& scene = Application::getInstance().current_slice->scene;
		LayerPlan& layer_plan = *buffer.back();
		for (size_t extruder_plan_idx = 0; extruder_plan_idx < layer_plan.extruder_plans.size(); extruder_plan_idx++)
		{
			const size_t overall_extruder_plan_idx = extruder_plans.size() - layer_plan.extruder_plans.size() + extruder_plan_idx;
			ExtruderPlan& extruder_plan = layer_plan.extruder_plans[extruder_plan_idx];
			size_t extruder = extruder_plan.extruder_nr;
		//	const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder].settings;
			Duration time = extruder_plan.estimates.getTotalUnretractedTime();
			double avg_flow;
			if (time > 0.0)
			{
				avg_flow = extruder_plan.estimates.getMaterial() / time;
			}
			else
			{
				assert(extruder_plan.estimates.getMaterial() == 0.0 && "No extrusion time should mean no material usage!");
				bool material_flow_dependent_temperature = false;
				if (material_flow_dependent_temperature) //Average flow is only used with flow dependent temperature.
				{
					printf("Empty extruder plans detected! Temperature control might suffer.\n");
				}
				avg_flow = 0.0;
			}

			const Temperature print_temp = preheat_config.getTemp(extruder, avg_flow, extruder_plan.is_initial_layer);
			const Temperature initial_print_temp =200;
			if (initial_print_temp == 0.0 // user doesn't want to use initial print temp feature
				|| !extruder_used_in_meshgroup[extruder] // prime blob uses print temp rather than initial print temp
				|| (overall_extruder_plan_idx > 0 && extruder_plans[overall_extruder_plan_idx - 1]->extruder_nr == extruder  // prev plan has same extruder ..
					&& extruder_plans[overall_extruder_plan_idx - 1]->estimates.getTotalUnretractedTime() > 0.0) // and prev extruder plan already heated to printing temperature
				)
			{
				extruder_plan.required_start_temperature = print_temp;
				extruder_used_in_meshgroup[extruder] = true;
			}
			else
			{
				extruder_plan.required_start_temperature = initial_print_temp;
				extruder_plan.extrusion_temperature = print_temp;
			}
			assert(extruder_plan.required_start_temperature != -1 && "extruder_plan.required_start_temperature should now have been set");

			if (buffer.size() == 1 && extruder_plan_idx == 0)
			{ // the very first extruder plan of the current meshgroup
				size_t extruder = 0;// extruder_plan.extruder_nr;
				for (size_t extruder_idx = 0; extruder_idx < 1; extruder_idx++)	//extruders.size()
				{ // set temperature of the first nozzle, turn other nozzles down
					//const Settings& other_extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_idx].settings;
					 //First mesh group.
					
						// override values from GCodeExport::setInitialTemps
						// the first used extruder should be set to the required temp in the start gcode
						// see  FffGcodeWriter::processStartingCode
						if (extruder_idx == extruder)
						{
							gcode.setInitialTemp(extruder_idx, extruder_plan.extrusion_temperature.value_or(extruder_plan.required_start_temperature));
						}
						else
						{
							Temperature material_standby_temperature = 100;
							gcode.setInitialTemp(extruder_idx, material_standby_temperature);
						}
					
					/*
					else
					{
						if (extruder_idx != extruder)
						{ // TODO: do we need to do this?
							Temperature material_standby_temperature = 100;
							gcode.setInitialTemp(extruder_idx, material_standby_temperature);
						}
					}
					*/
				}
				continue;
			}
			insertTempCommands(extruder_plans, overall_extruder_plan_idx);
		}
	}

	void LayerPlanBuffer::insertTempCommands(std::vector<ExtruderPlan*>& extruder_plans, unsigned int extruder_plan_idx)
	{
		ExtruderPlan& extruder_plan = *extruder_plans[extruder_plan_idx];
		const size_t extruder = 0;// extruder_plan.extruder_nr;

		ExtruderPlan* prev_extruder_plan = extruder_plans[extruder_plan_idx - 1];

		const size_t prev_extruder = 0;// prev_extruder_plan->extruder_nr;
		/*
		if (prev_extruder != extruder)
		{ // set previous extruder to standby temperature
			//const Settings& previous_extruder_settings = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings;
			

			Temperature material_standby_temperature = 100;
			gcode.setInitialTemp(extruder_idx, material_standby_temperature);
		}
		*/
		if (prev_extruder == extruder)
		{
			insertPreheatCommand_singleExtrusion(*prev_extruder_plan, extruder, extruder_plan.required_start_temperature);
			prev_extruder_plan->extrusion_temperature_command = --prev_extruder_plan->inserts.end();
		}
		/*
		else
		{
			insertPreheatCommand_multiExtrusion(extruder_plans, extruder_plan_idx);
			insertFinalPrintTempCommand(extruder_plans, extruder_plan_idx - 1);
			insertPrintTempCommand(extruder_plan);
		}
		*/
	}

	void LayerPlanBuffer::insertPreheatCommand_singleExtrusion(ExtruderPlan& prev_extruder_plan, const size_t extruder_nr, const Temperature required_temp)
	{
		bool machine_nozzle_temp_enabled = true;
        if (!machine_nozzle_temp_enabled)
		{
			return;
		}
		// time_before_extruder_plan_end is halved, so that at the layer change the temperature will be half way betewen the two requested temperatures
		constexpr bool during_printing = true;
		const double prev_extrusion_temp = prev_extruder_plan.extrusion_temperature.value_or(prev_extruder_plan.required_start_temperature);
		double time_before_extruder_plan_end = 0.5 * preheat_config.getTimeToGoFromTempToTemp(extruder_nr, prev_extrusion_temp, required_temp, during_printing);
		time_before_extruder_plan_end = std::min(prev_extruder_plan.estimates.getTotalTime(), time_before_extruder_plan_end);

		insertPreheatCommand(prev_extruder_plan, time_before_extruder_plan_end, extruder_nr, required_temp);
	}

	void LayerPlanBuffer::insertPreheatCommand(ExtruderPlan& extruder_plan_before, const Duration time_after_extruder_plan_start, const size_t extruder_nr, const Temperature temp)
	{
		Duration acc_time = 0.0;
		for (unsigned int path_idx = extruder_plan_before.paths.size() - 1; int(path_idx) != -1; path_idx--)
		{
			GCodePath& path = extruder_plan_before.paths[path_idx];
			const Duration time_this_path = path.estimates.getTotalTime();
			acc_time += time_this_path;
			{
				const Duration time_before_path_end = acc_time - time_after_extruder_plan_start;
				bool wait = false;
			if (acc_time > time_after_extruder_plan_start) false;
				extruder_plan_before.insertCommand(path_idx, extruder_nr, temp, wait, time_this_path - time_before_path_end);
				return;
			}
		}
		bool wait = false;
		constexpr size_t path_idx = 0;
		extruder_plan_before.insertCommand(path_idx, extruder_nr, temp, wait); // insert at start of extruder plan if time_after_extruder_plan_start > extruder_plan.time
	}

// namespace cura