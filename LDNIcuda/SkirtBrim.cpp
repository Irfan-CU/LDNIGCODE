//Copyright (C) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.



#include "SkirtBrim.h"
#include "sliceDataStorage.h"


	void SkirtBrim::getFirstLayerOutline(SliceDataStorage& storage, const size_t primary_line_count, const bool is_skirt, Polygons& first_layer_outline)
	{
		//const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
		//const ExtruderTrain& support_infill_extruder = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
		bool brim_outside_only = true;
		const bool external_only = is_skirt || brim_outside_only; //Whether to include holes or not. Skirt doesn't have any holes.
		
		const int layer_nr = 0;
		if (is_skirt)
		{
			constexpr bool include_support = true;
			constexpr bool include_prime_tower = true;
			first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_only);
			first_layer_outline = first_layer_outline.approxConvexHull();
		}
		else
		{ // add brim underneath support by removing support where there's brim around the model
			constexpr bool include_support = false; //Include manually below.
			constexpr bool include_prime_tower = false; //Include manually below.
			constexpr bool external_outlines_only = false; //Remove manually below.
			first_layer_outline = storage.getLayerOutlines(layer_nr, include_support, include_prime_tower, external_outlines_only);
			printf("the first layer outline size is %d \n", first_layer_outline.size());
			first_layer_outline = first_layer_outline.unionPolygons(); //To guard against overlapping outlines, which would produce holes according to the even-odd rule.
			printf("the first layer outline size after union is %d \n", first_layer_outline.size());
			
			Polygons first_layer_empty_holes;
			
			if (external_only)
			{
				first_layer_empty_holes = first_layer_outline.getEmptyHoles();
				first_layer_outline = first_layer_outline.removeEmptyHoles();
			}
			
		}
		constexpr coord_tIrfan join_distance = 20;
		first_layer_outline = first_layer_outline.offset(join_distance).offset(-join_distance); // merge adjacent models into single polygon
		constexpr coord_tIrfan smallest_line_length = 200;
		constexpr coord_tIrfan largest_error_of_removed_point = 50;
		//first_layer_outline.simplify(smallest_line_length, largest_error_of_removed_point); // simplify for faster processing of the brim lines
		if (first_layer_outline.size() == 0)
		{
			printf("Couldn't generate skirt / brim! No polygons on first layer.\n");
		}
		/*
		Polygons& check = first_layer_outline;
		ConstPolygonRef check1 = check[0];	  
		for (int i = 0; i < check1.size(); i++)
		{
			curaIrfan::PointIrfan point_check = check1[i];
			Point3 check2 = Point3(point_check.X, point_check.Y, 0.0);
			printf("the point at the start of the polygon is inside the getlayeroutline %f and %f \n", INT2MM(check2.x), INT2MM(check2.y));
		}
		Positive till here
		*/
		


		printf("the first layer outline size is 2 %d \n", first_layer_outline.size());
	}

	int SkirtBrim::generatePrimarySkirtBrimLines(const coord_tIrfan start_distance, size_t primary_line_count, const coord_tIrfan primary_extruder_minimal_length, const Polygons& first_layer_outline, Polygons& skirt_brim_primary_extruder)
	{
		//const Settings& adhesion_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").settings;
		const coord_tIrfan primary_extruder_skirt_brim_line_width = MM2INT(0.35) * 120;// adhesion_settings.get<coord_tIrfan>("skirt_brim_line_width") * adhesion_settings.get<Ratio>("initial_layer_line_width_factor");
		coord_tIrfan offset_distance = start_distance - primary_extruder_skirt_brim_line_width / 2;
		for (unsigned int skirt_brim_number = 0; skirt_brim_number < primary_line_count; skirt_brim_number++)
		{
			offset_distance += primary_extruder_skirt_brim_line_width;

			Polygons outer_skirt_brim_line = first_layer_outline.offset(offset_distance, ClipperLib::jtRound);

			//Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then 100x extrusion "area"
			for (unsigned int n = 0; n < outer_skirt_brim_line.size(); n++)
			{
				double area = outer_skirt_brim_line[n].area();
				if (area < 0 && area > -primary_extruder_skirt_brim_line_width * primary_extruder_skirt_brim_line_width * 100)
				{
					outer_skirt_brim_line.remove(n--);
				}
			}

			skirt_brim_primary_extruder.add(outer_skirt_brim_line);

			int length = skirt_brim_primary_extruder.polygonLength();
			if (skirt_brim_number + 1 >= primary_line_count && length > 0 && length < primary_extruder_minimal_length) //Make brim or skirt have more lines when total length is too small.
			{
				primary_line_count++;
			}
		}
		return offset_distance;
	}

	void SkirtBrim::generate(SliceDataStorage& storage, Polygons first_layer_outline, int start_distance, unsigned int primary_line_count, bool allow_helpers /*= true*/)
	{
		printf("inside generate\n");
		const bool is_skirt = start_distance > 0;	//false

		//Scene& scene = Application::getInstance().current_slice->scene;
		const size_t adhesion_extruder_nr = 0;// scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr;
		//const Settings& adhesion_settings = scene.extruders[adhesion_extruder_nr].settings;
		const coord_tIrfan primary_extruder_skirt_brim_line_width = MM2INT(0.35) * 120; ;// adhesion_settings.get<coord_tIrfan>("skirt_brim_line_width") * adhesion_settings.get<Ratio>("initial_layer_line_width_factor");
		const coord_tIrfan primary_extruder_minimal_length = MM2INT(250);// adhesion_settings.get<coord_tIrfan>("skirt_brim_minimal_length");

		Polygons& skirt_brim_primary_extruder = storage.skirt_brim[adhesion_extruder_nr];

		const bool has_ooze_shield = false;// allow_helpers && storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0;
		const bool has_draft_shield = false;// allow_helpers && storage.draft_protection_shield.size() > 0;

		if (is_skirt && (has_ooze_shield || has_draft_shield))
		{ // make sure we don't generate skirt through draft / ooze shield
			//first_layer_outline = first_layer_outline.offset(start_distance - primary_extruder_skirt_brim_line_width / 2, ClipperLib::jtRound).unionPolygons(storage.draft_protection_shield);
			if (has_ooze_shield)
			{
			//	first_layer_outline = first_layer_outline.unionPolygons(storage.oozeShield[0]);
			}
			first_layer_outline = first_layer_outline.approxConvexHull();
			start_distance = primary_extruder_skirt_brim_line_width / 2;
		}

		int offset_distance = generatePrimarySkirtBrimLines(start_distance, primary_line_count, primary_extruder_minimal_length, first_layer_outline, skirt_brim_primary_extruder);
		printf("offset_distance is %d \n", offset_distance);
		// handle support-brim
	//	const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
		bool support_brim_enable = false;
		if (allow_helpers && support_brim_enable )
		{
			generateSupportBrim(storage);
		}

		// generate brim for ooze shield and draft shield
		if (!is_skirt && (has_ooze_shield || has_draft_shield))
		{
			// generate areas where to make extra brim for the shields
			// avoid gap in the middle
			//    V
			//  +---+     +----+
			//  |+-+|     |+--+|
			//  || ||     ||[]|| > expand to fit an extra brim line
			//  |+-+|     |+--+|
			//  +---+     +----+ 
			const int64_t primary_skirt_brim_width = (primary_line_count + primary_line_count % 2) * primary_extruder_skirt_brim_line_width; // always use an even number, because we will fil the area from both sides

			Polygons shield_brim;
			if (has_ooze_shield)
			{
			//	shield_brim = storage.oozeShield[0].difference(storage.oozeShield[0].offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width));
			}
			if (has_draft_shield)
			{
				//shield_brim = shield_brim.unionPolygons(storage.draft_protection_shield.difference(storage.draft_protection_shield.offset(-primary_skirt_brim_width - primary_extruder_skirt_brim_line_width)));
			}
			const Polygons outer_primary_brim = first_layer_outline.offset(offset_distance, ClipperLib::jtRound);
			shield_brim = shield_brim.difference(outer_primary_brim.offset(primary_extruder_skirt_brim_line_width));

			// generate brim within shield_brim
			skirt_brim_primary_extruder.add(shield_brim);
			while (shield_brim.size() > 0)
			{
				shield_brim = shield_brim.offset(-primary_extruder_skirt_brim_line_width);
				skirt_brim_primary_extruder.add(shield_brim);
			}

			// update parameters to generate secondary skirt around
			first_layer_outline = outer_primary_brim;
			if (has_draft_shield)
			{
				//first_layer_outline = first_layer_outline.unionPolygons(storage.draft_protection_shield);
			}
			if (has_ooze_shield)
			{
				//first_layer_outline = first_layer_outline.unionPolygons(storage.oozeShield[0]);
			}

			offset_distance = 0;
		}

		{ // process other extruders' brim/skirt (as one brim line around the old brim)
			printf("inside the loop \n");
			int last_width = primary_extruder_skirt_brim_line_width;
			std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
			for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
			{
				if (extruder_nr == adhesion_extruder_nr || !extruder_is_used[extruder_nr])
				{
					continue;
				}
				if (storage.skirt_brim[extruder_nr].polygonLength() == 0)
				{
					continue;
				}
				//const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
				const coord_tIrfan width = MM2INT(0.35) * 120;// train.settings.get<coord_tIrfan>("skirt_brim_line_width") * train.settings.get<Ratio>("initial_layer_line_width_factor");
				const coord_tIrfan minimal_length = MM2INT(250);// train.settings.get<coord_tIrfan>("skirt_brim_minimal_length");
				offset_distance += last_width / 2 + width / 2;
				last_width = width;
				while (storage.skirt_brim[extruder_nr].polygonLength() < minimal_length)
				{
					storage.skirt_brim[extruder_nr].add(first_layer_outline.offset(offset_distance, ClipperLib::jtRound));
					offset_distance += width;
				}
				
			}
			Polygons&check1 = storage.skirt_brim[0];
			ConstPolygonRef check2 = check1[0];
			for (int i = 0; i < check2.size(); i++)
			{
				const curaIrfan::PointIrfan point_check = check2[i];
				Point3 check = Point3(point_check.X, point_check.Y, 0.0);
				printf("the point at the start of the polygon is inside the platformadhesion %f and %f \n", INT2MM(check.x), INT2MM(check.y));

			}
			
		}
	}

	void SkirtBrim::generateSupportBrim(SliceDataStorage& storage)
	{
		constexpr coord_tIrfan brim_area_minimum_hole_size_multiplier = 100;

		//Scene& scene = Application::getInstance().current_slice->scene;
		//const ExtruderTrain& support_infill_extruder = scene.current_mesh_group->settings.get<ExtruderTrain&>("support_infill_extruder_nr");
		const coord_tIrfan brim_line_width = MM2INT(0.35) * 120;// support_infill_extruder.settings.get<coord_tIrfan>("skirt_brim_line_width") * support_infill_extruder.settings.get<Ratio>("initial_layer_line_width_factor");
		size_t line_count = 20;// support_infill_extruder.settings.get<size_t>("support_brim_line_count");
		const coord_tIrfan minimal_length = MM2INT(250);// support_infill_extruder.settings.get<coord_tIrfan>("skirt_brim_minimal_length");
		if (!storage.support.generated || line_count <= 0 || storage.support.supportLayers.empty())
		{
			return;
		}

		const coord_tIrfan brim_width = brim_line_width * line_count;
		Polygons& skirt_brim = storage.skirt_brim[0];

		SupportLayer& support_layer = storage.support.supportLayers[0];

		Polygons support_outline;
		for (SupportInfillPart& part : support_layer.support_infill_parts)
		{
			support_outline.add(part.outline);
		}
		const Polygons brim_area = support_outline.difference(support_outline.offset(-brim_width));
		support_layer.excludeAreasFromSupportInfillAreas(brim_area, AABB(brim_area));

		Polygons support_brim;

		coord_tIrfan offset_distance = brim_line_width / 2;
		for (size_t skirt_brim_number = 0; skirt_brim_number < line_count; skirt_brim_number++)
		{
			offset_distance -= brim_line_width;

			Polygons brim_line = support_outline.offset(offset_distance, ClipperLib::jtRound);

			//Remove small inner skirt and brim holes. Holes have a negative area, remove anything smaller then multiplier x extrusion "area"
			for (size_t n = 0; n < brim_line.size(); n++)
			{
				const double area = brim_line[n].area();
				if (area < 0 && area > -brim_line_width * brim_line_width * brim_area_minimum_hole_size_multiplier)
				{
					brim_line.remove(n--);
				}
			}

			support_brim.add(brim_line);

			const coord_tIrfan length = skirt_brim.polygonLength() + support_brim.polygonLength();
			if (skirt_brim_number + 1 >= line_count && length > 0 && length < minimal_length) //Make brim or skirt have more lines when total length is too small.
			{
				line_count++;
			}
			if (brim_line.empty())
			{ // the fist layer of support is fully filled with brim
				break;
			}
		}

		if (support_brim.size())
		{
			// to ensure that the skirt brim is printed from outside to inside, the support brim lines must
			// come before the skirt brim lines in the Polygon object so that the outermost skirt brim line
			// is at the back of the list
			support_brim.add(skirt_brim);
			skirt_brim = support_brim;
		}
	}
//namespace cura
