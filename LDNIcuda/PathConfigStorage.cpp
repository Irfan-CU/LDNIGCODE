
#include "PathConfigStorage.h"
#include "Raft.h"
#include "SliceDataStorage.h" // SliceDataStorage
#include "EnumSettings.h" 
#include "Ratio.h"

std::vector<Ratio> PathConfigStorage::getLineWidthFactorPerExtruder(const int& layer_nr)
{
	std::vector<Ratio> ret;
	for (int i=0;i<1;i++)
	{
		if (layer_nr <= 0)
		{
			const Ratio factor = Ratio(120/100);
			ret.push_back(factor);
		}
		else
		{
			ret.push_back(1.0);
		}
	}
	return ret;
}

GCodePathConfig createPerimeterGapConfig(const SliceDataStorage& mesh, int layer_thickness, const int& layer_nr)
{
	// The perimeter gap config follows the skin config, but has a different line width:
	// wall_line_width_x divided by two because the gaps are between 0 and 1 times the wall line width
	const coord_tIrfan perimeter_gaps_line_width =MM2INT(0.175);
	double perimeter_gaps_speed = 20;
	const double materialflow=150;
	
	return GCodePathConfig(
		PrintFeatureType::Skin
		, perimeter_gaps_line_width
		, layer_thickness
		, (100/100) * ((layer_nr == 0) ? (100/100) : (1.0))
		, GCodePathConfig::SpeedDerivatives{ 500, 5 }
	);
}

PathConfigStorage::MeshPathConfigs::MeshPathConfigs(const SliceDataStorage& storage, const coord_tIrfan layer_thickness, const int& layer_nr, const std::vector<Ratio>& line_width_factor_per_extruder)
	: inset0_config(
		PrintFeatureType::OuterWall
		, MM2INT(0.35)* Ratio(120/100) //line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]
		, layer_thickness
		, Ratio (100 / 100) * ((layer_nr== 0 )? Ratio (100/100): Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 20, 500,5}
	)
	, insetX_config(
		PrintFeatureType::InnerWall
		, MM2INT(0.3) * Ratio(120 / 100) //line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]
		, layer_thickness
		, Ratio (100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 30, 1000,10 }
	)
	, bridge_inset0_config(
		PrintFeatureType::OuterWall
		, MM2INT(0.35) * Ratio(120 / 100) //line_width_factor_per_extruder[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr]
		, layer_thickness
		, Ratio (50.0/100)
		, GCodePathConfig::SpeedDerivatives{ 10, 500, 5 }
		, true // is_bridge_path
		, Ratio(0) * 100.0
	)
	, bridge_insetX_config(
		PrintFeatureType::InnerWall
		, MM2INT(0.35) *Ratio(120 / 100)
		, layer_thickness
		, Ratio(50.0 / 100)
		, GCodePathConfig::SpeedDerivatives{ 10,1000, 10 }
		, true // is_bridge_path
		, Ratio(0) * 100.0
	)
	, skin_config(
		PrintFeatureType::Skin
		, MM2INT(0.35) * Ratio(120 / 100)
		, layer_thickness
		, Ratio(100/100) * ((layer_nr == 0) ? Ratio(100/100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 20, 500, 5 }
	)
	, bridge_skin_config( // use bridge skin flow, speed and fan
		PrintFeatureType::Skin
		, MM2INT(0.35) * Ratio(120 / 100)
		, layer_thickness
		, Ratio(100 / 100)
		, GCodePathConfig::SpeedDerivatives{ 10.0, 500, 5 }
		, true // is_bridge_path
		, Ratio (0)* 100.0
	)
	, bridge_skin_config2( // use bridge skin 2 flow, speed and fan
		PrintFeatureType::Skin
		, MM2INT(0.35) * Ratio(120 / 100)
		, layer_thickness
		, Ratio(100 / 100)
		, GCodePathConfig::SpeedDerivatives{ 10,500,5 }
		, true // is_bridge_path
		, Ratio(0)* 100.0
	)
	, bridge_skin_config3( // use bridge skin 3 flow, speed and fan
		PrintFeatureType::Skin
		, MM2INT(0.35) *Ratio(120 / 100)
		, layer_thickness
		, Ratio(100 / 100)
		, GCodePathConfig::SpeedDerivatives{ 10,500,5 }
		, true // is_bridge_path
		, Ratio(0)* 100.0
	)
	, roofing_config(
		PrintFeatureType::Skin
		, MM2INT(0.3) 
		, layer_thickness
		, Ratio(100/100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{20, 500, 5 }
	)
	, ironing_config(
		PrintFeatureType::Skin
		, MM2INT(0.3)
		, layer_thickness
		, Ratio (10.0/100)
		, GCodePathConfig::SpeedDerivatives{ 13.33,500, 5 }
	)

	, perimeter_gap_config(createPerimeterGapConfig(storage, layer_thickness, layer_nr))
{
	infill_config.reserve(MAX_INFILL_COMBINE);

	for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
	{
		infill_config.emplace_back(
			PrintFeatureType::Infill
			, MM2INT(0.42) * (combine_idx + 1) * Ratio(120/100)
			, layer_thickness
			, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
			, GCodePathConfig::SpeedDerivatives{ 60, 3000, 25 }
		);
	
	}
}

PathConfigStorage::PathConfigStorage(const SliceDataStorage& storage, const int& layer_nr, const coord_tIrfan layer_thickness)
	: support_infill_extruder_nr(0)
	, support_roof_extruder_nr(0)
	, support_bottom_extruder_nr(0)
	, line_width_factor_per_extruder(PathConfigStorage::getLineWidthFactorPerExtruder(layer_nr))
	, raft_base_config(
		PrintFeatureType::SupportInterface
		, MM2INT(0.8)
		, MM2INT(0.24122)//adhesion_extruder_train.settings.get<coord_t>("raft_base_thickness")
		, ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 13.125, 4000, 25 }
	)
	, raft_interface_config(
		PrintFeatureType::Support
		, MM2INT (0.7)//adhesion_extruder_train.settings.get<coord_t>("raft_interface_line_width")
		, MM2INT (0.300)//adhesion_extruder_train.settings.get<coord_t>("raft_interface_thickness")
		, ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{13.125, 400, 25}
	)
	, raft_surface_config(
		PrintFeatureType::SupportInterface
		, MM2INT(0.35)
		, MM2INT(0.20)//adhesion_extruder_train.settings.get<coord_t>("raft_surface_thickness")
		, ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 17.5, 4000, 25 }
	)
	, support_roof_config(
		PrintFeatureType::SupportInterface
		, MM2INT(0.35) * Ratio(120/100)
		, layer_thickness
		, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 40, 3000, 20}
	)
	, support_bottom_config(
		PrintFeatureType::SupportInterface
		, MM2INT(0.35) * Ratio(120 / 100)
		, layer_thickness
		, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
		, GCodePathConfig::SpeedDerivatives{ 23, 500, 5 }
	)
{
	const size_t extruder_count = 1;
	travel_config_per_extruder.reserve(extruder_count);
	skirt_brim_config_per_extruder.reserve(extruder_count);
	prime_tower_config_per_extruder.reserve(extruder_count);
	//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
	{
		//const ExtruderTrain& train = Application::getInstance().current_slice->scene.extruders[extruder_nr];
		travel_config_per_extruder.emplace_back(
			PrintFeatureType::MoveCombing
			, 0
			, 0
			, 0.0
			, GCodePathConfig::SpeedDerivatives{ 250,5000, 30 }
		);
		skirt_brim_config_per_extruder.emplace_back(
			PrintFeatureType::SkirtBrim
			, MM2INT(0.35)* Ratio(120 / 100)//mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[extruder_nr]) // cause it's also used for the draft/ooze shield
			, layer_thickness
			, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
			, GCodePathConfig::SpeedDerivatives{20, 500, 5 }
		);
		prime_tower_config_per_extruder.emplace_back(
			PrintFeatureType::PrimeTower
			, MM2INT(0.35) * Ratio (120/100)//((mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0_r : line_width_factor_per_extruder[extruder_nr])
			, layer_thickness
			, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
			, GCodePathConfig::SpeedDerivatives{ 20, 500, 5 }//GCodePathConfig::SpeedDerivatives{ train.settings.get<Velocity>("speed_prime_tower"), train.settings.get<Acceleration>("acceleration_prime_tower"), train.settings.get<Velocity>("jerk_prime_tower") }
		);
	}

	mesh_configs.reserve(1);//  storage.meshes.size());
	mesh_configs.emplace_back(storage, layer_thickness, layer_nr, line_width_factor_per_extruder);
	

	support_infill_config.reserve(MAX_INFILL_COMBINE);
	const float support_infill_line_width_factor = Ratio(120 / 100);
	for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
	{
		support_infill_config.emplace_back(
			PrintFeatureType::Support
			, MM2INT(0.35) * (combine_idx + 1) * support_infill_line_width_factor
			, layer_thickness
			, Ratio(100 / 100) * ((layer_nr == 0) ? Ratio(100 / 100) : Ratio(1.0))
			, GCodePathConfig::SpeedDerivatives{ 60, 3000, 20 }
		);
	}

	const size_t initial_speedup_layer_count = 2;// mesh_group_settings.get<size_t>("speed_slowdown_layers");
	if (layer_nr >= 0 && static_cast<size_t>(layer_nr) < initial_speedup_layer_count)
	{
		handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
	}
}

void PathConfigStorage::MeshPathConfigs::smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, const int& layer_nr, const int& max_speed_layer)
{
	inset0_config.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	insetX_config.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	skin_config.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	ironing_config.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	perimeter_gap_config.smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	for (size_t idx = 0; idx < MAX_INFILL_COMBINE; idx++)
	{
		//Infill speed (per combine part per mesh).
		infill_config[idx].smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
	}
}

void PathConfigStorage::handleInitialLayerSpeedup(const SliceDataStorage& storage, const int& layer_nr, const size_t initial_speedup_layer_count)
{
	std::vector<GCodePathConfig::SpeedDerivatives> global_first_layer_config_per_extruder;
	global_first_layer_config_per_extruder.reserve(1);
	global_first_layer_config_per_extruder.emplace_back(
			GCodePathConfig::SpeedDerivatives{
				20
				, 500
				, 5
			});
	

	{ // support
		if (layer_nr < static_cast<int>(initial_speedup_layer_count))
		{
			//const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
			const size_t extruder_nr_support_infill = 0;
			GCodePathConfig::SpeedDerivatives& first_layer_config_infill = global_first_layer_config_per_extruder[extruder_nr_support_infill];
			for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
			{
				support_infill_config[idx].smoothSpeed(first_layer_config_infill, std::max(int(0), layer_nr), initial_speedup_layer_count);
			}

			const size_t extruder_nr_support_roof = 0;
			GCodePathConfig::SpeedDerivatives& first_layer_config_roof = global_first_layer_config_per_extruder[extruder_nr_support_roof];
			support_roof_config.smoothSpeed(first_layer_config_roof, std::max(int(0), layer_nr), initial_speedup_layer_count);
			const size_t extruder_nr_support_bottom =0;
			GCodePathConfig::SpeedDerivatives& first_layer_config_bottom = global_first_layer_config_per_extruder[extruder_nr_support_bottom];
			support_bottom_config.smoothSpeed(first_layer_config_bottom, std::max(int(0), layer_nr), initial_speedup_layer_count);
		}
	}

	{ // extruder configs: travel, skirt/brim (= shield)
		for (size_t extruder_nr = 0; extruder_nr < 1; extruder_nr++)
		{
			
			GCodePathConfig::SpeedDerivatives initial_layer_travel_speed_config{
					142.85714285714286
					, 625.0
					, 6.0
			};
			GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

			travel.smoothSpeed(initial_layer_travel_speed_config, std::max(int(0), layer_nr), initial_speedup_layer_count);

			// don't smooth speed for the skirt/brim!
			// NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

			const GCodePathConfig::SpeedDerivatives& initial_layer_print_speed_config = global_first_layer_config_per_extruder[extruder_nr];

			GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
			prime_tower.smoothSpeed(initial_layer_print_speed_config, std::max(int(0), layer_nr), initial_speedup_layer_count);
		}

	}

	{ // meshes
		for (size_t mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
		{
			const SliceMeshStorage& mesh = storage.meshes[mesh_idx];// storage.meshes[mesh_idx];

			GCodePathConfig::SpeedDerivatives initial_layer_speed_config{
					20
					,500
					, 5
			};

			mesh_configs[mesh_idx].smoothAllSpeeds(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
			mesh_configs[mesh_idx].roofing_config.smoothSpeed(initial_layer_speed_config, layer_nr, initial_speedup_layer_count);
		}
	}
}






