GetLatestPath 	   without flowconfig

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
	coord_tIrfan line_width = MM2INT(6.3);

	double flowconfig = 100 * ((layer_nr == 0) ? 100 : double(1.0));
	std::string current_mesh = "Mesh1";

	paths.emplace_back(speed, accelration, Jerk, feature, line_width, layer_thickness, flowconfig, current_mesh, space_fill_type, flow, spiralize, layernum, speed_factor);
	GCodePath* ret = &paths.back();
	ret->skip_agressive_merge_hint = mode_skip_agressive_merge;
	return ret;

}