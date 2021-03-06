//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GCodePath.h"
#include "../GCodePathConfig.h"

	GCodePath::GCodePath(const GCodePathConfig& config, std::string mesh_id, const SpaceFillType space_fill_type, const Ratio flow, const bool spiralize, const Ratio speed_factor) :
		config(&config),
		mesh_id(mesh_id),
		space_fill_type(space_fill_type),
		flow(flow),
		speed_factor(speed_factor),
		retract(false),
		perform_z_hop(false),
		perform_prime(false),
		skip_agressive_merge_hint(false),
		points(std::vector<curaIrfan::PointIrfan>()),
		done(false),
		spiralize(spiralize),
		fan_speed(GCodePathConfig::FAN_SPEED_DEFAULT),
		estimates(TimeMaterialEstimates())
	{
	}

	bool GCodePath::isTravelPath() const
	{
		return config->isTravelPath();
	}

	double GCodePath::getExtrusionMM3perMM() const
	{
		return flow * config->getExtrusionMM3perMM();
		
	}

	coord_tIrfan GCodePath::getLineWidthForLayerView() const
	{
		return flow * config->getLineWidth() * config->getFlowRatio();
	}

	void GCodePath::setFanSpeed(double fan_speed)
	{
		this->fan_speed = fan_speed;
	}

	double GCodePath::getFanSpeed() const
	{
		return (fan_speed >= 0 && fan_speed <= 100) ? fan_speed : config->getFanSpeed();
	}

	void GCodePath::setPathMat(int mat)
	{
		this->path_mat = mat;
	}
	int GCodePath::getPathMat()
	{
		return path_mat;
	}
	void GCodePath::setextruder(int extruder)
	{
		this->path_ext = extruder;
	}
	int GCodePath::getextruder()
	{
		return path_ext;
	}

	void GCodePath::setLDMIextruder(int extruder)
	{
		this->path_LDMIext = extruder;
	}
	int GCodePath::getLDMIextruder()
	{
		return path_LDMIext;
	}





