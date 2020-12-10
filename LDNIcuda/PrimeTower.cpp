//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <algorithm>
#include <limits>


#include "ExtruderTrain.h"
#include "gcodeExport.h"
#include "infill.h"
#include "LayerPlan.h"
#include "PrimeTower.h"
#include "PrintFeature.h"
#include "raft.h"
#include "sliceDataStorage.h"

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.

	PrimeTower::PrimeTower()
		: wipe_from_middle(false)
	{
		
		coord_tIrfan prime_tower_min_volume = MM2INT(6.0);
		coord_tIrfan prime_tower_size = MM2INT(20);
		enabled = false
			&& prime_tower_min_volume > 10
			&& prime_tower_size > 10;

		extruder_count = 1;
		extruder_order.resize(extruder_count);
		for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
		{
			extruder_order[extruder_nr] = extruder_nr; //Start with default order, then sort.
		}
		//Sort from high adhesion to low adhesion.
		
		const Ratio adhesion_a = Ratio(10 / 100);// scene_pointer->extruders[extruder_nr_a].settings.get<Ratio>("material_adhesion_tendency");
		const Ratio adhesion_b = Ratio(10 / 100);// scene_pointer->extruders[extruder_nr_b].settings.get<Ratio>("material_adhesion_tendency");
		
		
	}

	void PrimeTower::generateGroundpoly()
	{
		if (!enabled)
		{
			return;
		}

		
		const coord_tIrfan tower_size = coord_tIrfan(20);// mesh_group_settings.get<coord_t>("prime_tower_size");
		size_t brim_line_count = 17;
		coord_tIrfan skirt_brim_line_width = MM2INT(0.35);
		Ratio initial_layer_line_width_factor = Ratio(120 / 100);
		const bool has_raft = false;// (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT);
		const bool has_prime_brim = false; // mesh_group_settings.get<bool>("prime_tower_brim_enable");
		const coord_tIrfan offset = (has_raft || !has_prime_brim) ? 0 :
			brim_line_count *
			skirt_brim_line_width*
			initial_layer_line_width_factor;

		PolygonRef p = outer_poly.newPoly();
		int tower_distance = 0;
		const coord_tIrfan x = MM2INT(175.70000000000002); // mesh_group_settings.get<coord_t>("prime_tower_position_x") - offset;
		const coord_tIrfan y = MM2INT(184.56);// mesh_group_settings.get<coord_t>("prime_tower_position_y") - offset;
		const coord_tIrfan tower_radius = tower_size / 2;
		for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
		{
			const double angle = (double)i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
			p.add(curaIrfan::PointIrfan(x - tower_radius + tower_distance + cos(angle) * tower_radius,
				y + tower_radius + tower_distance + sin(angle) * tower_radius));
		}
		middle = curaIrfan::PointIrfan(x - tower_size / 2, y + tower_size / 2);

		post_wipe_point = curaIrfan::PointIrfan(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);

		outer_poly_first_layer = outer_poly.offset(offset);
	}

	void PrimeTower::generatePaths(const SliceDataStorage& storage)
	{
		enabled &= storage.max_print_height_second_to_last_extruder >= 0; //Maybe it turns out that we don't need a prime tower after all because there are no layer switches.
		if (enabled)
		{
			generatePaths_denseInfill(storage);
			generateStartLocations();
		}
	}

	void PrimeTower::generatePaths_denseInfill(const SliceDataStorage& storage)
	{
		
		const coord_tIrfan layer_height = storage.layer_thickness;// mesh_group_settings.get<coord_tIrfan>("layer_height");
		pattern_per_extruder.resize(extruder_count);

		coord_tIrfan cumulative_inset = 0; //Each tower shape is going to be printed inside the other. This is the inset we're doing for each extruder.
		for (size_t extruder_nr : extruder_order)
		{
			const coord_tIrfan line_width = MM2INT(0.35);// scene.extruders[extruder_nr].settings.get<coord_tIrfan>("prime_tower_line_width");
			const coord_tIrfan required_volume = 6 * 1000000000; //To cubic microns.
			const Ratio flow = (100 / 100);// scene.extruders[extruder_nr].settings.get<Ratio>("prime_tower_flow");
			coord_tIrfan current_volume = 0;
			ExtrusionMoves& pattern = pattern_per_extruder[extruder_nr];

			//Create the walls of the prime tower.
			unsigned int wall_nr = 0;
			for (; current_volume < required_volume; wall_nr++)
			{
				//Create a new polygon with an offset from the outer polygon.
				Polygons polygons = outer_poly.offset(-cumulative_inset - wall_nr * line_width - line_width / 2);
				pattern.polygons.add(polygons);
				current_volume += polygons.polygonLength() * line_width * layer_height * flow;
				if (polygons.empty()) //Don't continue. We won't ever reach the required volume because it doesn't fit.
				{
					break;
				}
			}
			cumulative_inset += wall_nr * line_width;

			//Generate the pattern for the first layer.
			coord_tIrfan line_width_layer0 = line_width;
			line_width_layer0 *= Ratio(120 / 100);	
			pattern_per_extruder_layer0.emplace_back();

			ExtrusionMoves& pattern_layer0 = pattern_per_extruder_layer0.back();

			// Generate a concentric infill pattern in the form insets for the prime tower's first layer instead of using
			// the infill pattern because the infill pattern tries to connect polygons in different insets which causes the
			// first layer of the prime tower to not stick well.
			Polygons inset = outer_poly.offset(-line_width_layer0 / 2);
			while (!inset.empty())
			{
				pattern_layer0.polygons.add(inset);
				inset = inset.offset(-line_width_layer0);
			}
		}
	}

	void PrimeTower::generateStartLocations()
	{
		// Evenly spread out a number of dots along the prime tower's outline. This is done for the complete outline,
		// so use the same start and end segments for this.
		PolygonsPointIndex segment_start = PolygonsPointIndex(&outer_poly, 0, 0);
		PolygonsPointIndex segment_end = segment_start;

		PolygonUtils::spreadDots(segment_start, segment_end, number_of_prime_tower_start_locations, prime_tower_start_locations);
	}

	void PrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int prev_extruder, const int new_extruder) const
	{
		if (!enabled)
		{
			printf("primetower add to Gcode primer tower enabel is worrking fine\n");
			return;
		}
		/*
		if (gcode_layer.getPrimeTowerIsPlanned(new_extruder))
		{ // don't print the prime tower if it has been printed already with this extruder.
			return;
		}

		const LayerIndex layer_nr = gcode_layer.getLayerNr();
		if (layer_nr > storage.max_print_height_second_to_last_extruder + 1)
		{
			return;
		}

		bool post_wipe = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings.get<bool>("prime_tower_wipe_enabled");

		// Do not wipe on the first layer, we will generate non-hollow prime tower there for better bed adhesion.
		if (prev_extruder == new_extruder || layer_nr == 0)
		{
			post_wipe = false;
		}

		// Go to the start location if it's not the first layer
		if (layer_nr != 0)
		{
			gotoStartLocation(gcode_layer, new_extruder);
		}

		addToGcode_denseInfill(gcode_layer, new_extruder);

		// post-wipe:
		if (post_wipe)
		{
			//Make sure we wipe the old extruder on the prime tower.
			const Settings& previous_settings = Application::getInstance().current_slice->scene.extruders[prev_extruder].settings;
			const Point previous_nozzle_offset = Point(previous_settings.get<coord_tIrfan>("machine_nozzle_offset_x"), previous_settings.get<coord_tIrfan>("machine_nozzle_offset_y"));
			const Settings& new_settings = Application::getInstance().current_slice->scene.extruders[new_extruder].settings;
			const Point new_nozzle_offset = Point(new_settings.get<coord_tIrfan>("machine_nozzle_offset_x"), new_settings.get<coord_tIrfan>("machine_nozzle_offset_y"));
			gcode_layer.addTravel(post_wipe_point - previous_nozzle_offset + new_nozzle_offset);
		}

		gcode_layer.setPrimeTowerIsPlanned(new_extruder);
		*/
	}

	void PrimeTower::addToGcode_denseInfill(LayerPlan& gcode_layer, const size_t extruder_nr, SliceDataStorage& storage) const
	{
		const ExtrusionMoves& pattern = pattern_per_extruder[extruder_nr];

		const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];

		gcode_layer.addPolygonsByOptimizer(gcode_layer.getLayerNr(), storage.getlayer_thickness(), pattern.polygons, config);
		gcode_layer.addLinesByOptimizer(storage.getlayer_thickness(), config, pattern.lines, gcode_layer.getLayerNr(), SpaceFillType::Lines);
	}

	void PrimeTower::subtractFromSupport(SliceDataStorage& storage)
	{
		const Polygons outside_polygon = outer_poly.getOutsidePolygons();
		AABB outside_polygon_boundary_box(outside_polygon);
		for (size_t layer = 0; layer <= (size_t)storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
		{
			SupportLayer& support_layer = storage.support.supportLayers[layer];
			// take the differences of the support infill parts and the prime tower area
			support_layer.excludeAreasFromSupportInfillAreas(outside_polygon, outside_polygon_boundary_box);
		}
	}

	void PrimeTower::gotoStartLocation(LayerPlan& gcode_layer, const int extruder_nr,SliceDataStorage& storage) const
	{
		int current_start_location_idx = ((((extruder_nr + 1) * gcode_layer.getLayerNr()) % number_of_prime_tower_start_locations)
			+ number_of_prime_tower_start_locations) % number_of_prime_tower_start_locations;

		const ClosestPolygonPoint wipe_location = prime_tower_start_locations[current_start_location_idx];

		
		const coord_tIrfan inward_dist = MM2INT(0.4) * 3 / 2;
		const coord_tIrfan start_dist = MM2INT(0.4)*2;// train.settings.get<coord_tIrfan>("machine_nozzle_size") * 2;
		const curaIrfan::PointIrfan prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
		const curaIrfan::PointIrfan outward_dir = curaIrfan::operator-(wipe_location.location , prime_end);
		const curaIrfan::PointIrfan prime_start = curaIrfan::operator+(wipe_location.location , curaIrfan::normal(outward_dir, start_dist));

		gcode_layer.addTravel(storage.getlayer_thickness(), gcode_layer.getLayerNr(), prime_start);
	}

//namespace cura
