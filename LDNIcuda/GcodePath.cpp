//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodePath.h"



GCodePath::GCodePath(double speed, double acceleration, double jerk, const PrintFeatureType type, const coord_tIrfan line_width1, const coord_tIrfan layer_thickness1, double flowconfig, std::string mesh_id, const SpaceFillType space_fill_type, double flow1, const bool spiralize, int layernum, const double speed_factor):
	speed(speed),
    acceleration(acceleration),
	jerk(jerk),
	type(type),
	line_width1(line_width1),
	layer_thickness1(layer_thickness1),
	flowconfig(flowconfig),
	mesh_id(mesh_id),
	space_fill_type(space_fill_type),
	flow1(flow1),
	spiralize(spiralize),
	layer_nr(layernum),
	speed_factor(speed_factor),
	retract(false),
	perform_z_hop(false),
	perform_prime(false),
	skip_agressive_merge_hint(false),
	points(std::vector<curaIrfan::PointIrfan>()),
	done(false),
	extrusion_mm3_per_mm1(calculateExtrusion()),
	is_bridge_path1(false),
	fan_speed1(50)
{

}
	bool GCodePath::isTravelPath() const
	{
		return isTravelPath1();
	}
	bool GCodePath::isTravelPath1() const
	{
		return line_width1 == 0;
	}


	double GCodePath::getExtrusionMM3perMM() const
	{
		return flow1 * calculateExtrusion();
	}

	coord_tIrfan GCodePath::getLineWidthForLayerView() const
	{
		return flow1 * getLineWidth1() * getFlowRatio();
	}
	double GCodePath::getFlowRatio() const
	{
		return flowconfig;
	}

	coord_tIrfan GCodePath::getLineWidth1() const
	{
		return line_width1;
	}

	coord_tIrfan GCodePath::getLayerthickness1() const
	{
		return layer_thickness1;
	}
	
	void GCodePath::setFanSpeed(double fan_speed)
	{
		this->fan_speed1 = fan_speed;
	}

	double GCodePath::getFanSpeed() const
	{
		return fan_speed1;
	}

	double GCodePath::calculateExtrusion() const
	{
		return INT2MM(line_width1) * INT2MM(layer_thickness1) * double(flowconfig);
	}
	double GCodePath::getSpeed()
	{
		return speed;
	}
	double GCodePath::getAccelration()
	{
		return acceleration;
	}
	double GCodePath::getJerk()
	{
		return jerk;
	}

	

