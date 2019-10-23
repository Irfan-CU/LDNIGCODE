//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <clipper.hpp>

 //To get settings.
#include <cmath>

#include "Raft.h"

#include "SliceDataStorage.h"

#include "EnumSettings.h" //For EPlatformAdhesion.


	void Raft::generate(SliceDataStorage& storage)
	{
		//assert(storage.raftOutline.size() == 0 && "Raft polygon isn't generated yet, so should be empty!");
		//const Settings& settings = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr").settings;
		const coord_tIrfan distance = 15;// settings.get<coord_t>("raft_margin");
		constexpr bool include_support = true;
		constexpr bool include_prime_tower = true;
		/*
		storage.raftOutline = storage.getLayerOutlines(0, include_support, include_prime_tower).offset(distance, ClipperLib::jtRound);
		const coord_tIrfan shield_line_width_layer0 = settings.get<coord_t>("skirt_brim_line_width");
		if (storage.draft_protection_shield.size() > 0)
		{
			Polygons draft_shield_raft = storage.draft_protection_shield.offset(shield_line_width_layer0) // start half a line width outside shield
				.difference(storage.draft_protection_shield.offset(-distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
			storage.raftOutline = storage.raftOutline.unionPolygons(draft_shield_raft);
		}
		if (storage.oozeShield.size() > 0 && storage.oozeShield[0].size() > 0)
		{
			const Polygons& ooze_shield = storage.oozeShield[0];
			Polygons ooze_shield_raft = ooze_shield.offset(shield_line_width_layer0) // start half a line width outside shield
				.difference(ooze_shield.offset(-distance - shield_line_width_layer0 / 2, ClipperLib::jtRound)); // end distance inside shield
			storage.raftOutline = storage.raftOutline.unionPolygons(ooze_shield_raft);
		}
		const coord_t smoothing = settings.get<coord_t>("raft_smoothing");
		storage.raftOutline = storage.raftOutline.offset(smoothing, ClipperLib::jtRound).offset(-smoothing, ClipperLib::jtRound); // remove small holes and smooth inward corners
	*/
	}

	coord_tIrfan Raft::getTotalThickness()
	{
		//const ExtruderTrain& train = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<ExtruderTrain&>("adhesion_extruder_nr");
		return 0.2412 + 0.3000 + (2 * 4000);
			//+ train.settings.get<coord_tIrfan>("raft_interface_thickness")+ train.settings.get<size_t>("raft_surface_layers") * train.settings.get<coord_t>("raft_surface_thickness");
	}

	coord_tIrfan Raft::getZdiffBetweenRaftAndLayer1()
	{
		//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
		//const ExtruderTrain& train = mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr");
		
		const coord_tIrfan airgap = 0.3;
		const coord_tIrfan layer_0_overlap =0.15;

		const coord_tIrfan layer_height_0 =0.201;

		const coord_tIrfan z_diff_raft_to_bottom_of_layer_1 = airgap + layer_height_0 - layer_0_overlap;
		return z_diff_raft_to_bottom_of_layer_1;
	}

	size_t Raft::getFillerLayerCount()
	{
		//const coord_tIrfan normal_layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<coord_t>("layer_height");
		return (getZdiffBetweenRaftAndLayer1() + 0.1) / 0.2;// round_divide(getZdiffBetweenRaftAndLayer1(), 0.2);
	}

	coord_tIrfan Raft::getFillerLayerHeight()
	{
		
		return (getZdiffBetweenRaftAndLayer1() + getFillerLayerCount()/2) / getFillerLayerCount();
		
	}


	size_t Raft::getTotalExtraLayers()
	{
		
		return 2 + 2 + getFillerLayerCount();
	}

//namespace cura
