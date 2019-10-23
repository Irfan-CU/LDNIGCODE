#pragma once
#ifndef WIPE_SCRIPT_CONFIG_H
#define WIPE_SCRIPT_CONFIG_H

#include "Coord_tIrfan.h"
#include "RetractionConfig.h"

	struct WipeScriptConfig
	{
		bool retraction_enable; // whether to retract the filament when the nozzle is moving over a non-printed area
		RetractionConfig retraction_config;

		double pause; // pause after the unretract, in seconds

		bool hop_enable; // whenever a retraction is done, the build plate is lowered to create clearance between the nozzle and the print
		coord_tIrfan hop_amount; // height difference when performing a Z Hop
		double hop_speed;

		coord_tIrfan brush_pos_x; // X coordinate - location where wipe script will start
		size_t repeat_count; // number of times to move the nozzle across the brush
		coord_tIrfan move_distance; // distance to move the head back and forth across the brush
		double move_speed;
		double max_extrusion_mm3; // maximum material that can be extruded before another nozzle wipe is initiated (in mm^3)
		bool clean_between_layers; // whether to include nozzle wipe g-code between layers
	};

//namespace cura

#endif // WIPE_SCRIPT_CONFIG_H

