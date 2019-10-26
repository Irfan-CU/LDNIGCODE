
#include <assert.h>
#include <cmath>
#include <iomanip>
#include <stdarg.h>


#include "GCodeExport.h"
#include "RetractionConfig.h"
#include "Date.h"
#include "String.h"
#include "WipeScriptConfig.h"



GCodeExport::GCodeExport()
	: output_stream(&std::cout)
	, currentPosition(0, 0, MM2INT(20))
	, layer_nr(0)
	, relative_extrusion(false)
{
	*output_stream << std::fixed;
	current_e_value = 0;
	current_extruder = 0;
	current_fan_speed = -1;

	total_print_times = std::vector<double>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

	currentSpeed = 1;
	current_print_acceleration = -1;
	current_travel_acceleration = -1;
	current_jerk = -1;

	is_z_hopped = 0;
	setFlavor(EGCodeFlavor::GRIFFIN);
	initial_bed_temp = 0;
	build_volume_temperature = 0;

	fan_number = 0;
	use_extruder_offset_to_offset_coords = false;
	machine_name = "";
	machine_buildplate_type = "";
	relative_extrusion = false;
	new_line = "\n";

	total_bounding_box = AABB3D();
}

GCodeExport::~GCodeExport()
{
}



std::string transliterate(const std::string& text)
{
	// For now, just replace all non-ascii characters with '?'.
	// This function can be expaned if we need more complex transliteration.
	std::ostringstream stream;
	for (const char& c : text)
	{
		stream << static_cast<char>((c >= 0) ? c : '?');
	}
	return stream.str();
}
int GCodeExport::getPositionZ() const
{
	return currentPosition.z;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
	output_stream = stream;
	*output_stream << std::fixed;
}

int GCodeExport::getExtruderNr() const
{
	return 0;
}

void GCodeExport::setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr)
{
	//const Scene& scene = Application::getInstance().current_slice->scene;
	const size_t extruder_count = 1;
	for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
	{
		const double print_temp_0 = 205;
		const double print_temp_here = 200;
		const double temp = 195; //"material_standby_temperature");
		setInitialTemp(extruder_nr, temp);
	}

	initial_bed_temp = 60; // "material_bed_temperature_layer_0");
	build_volume_temperature = 28;// ("build_volume_temperature");
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
	extruder_attr[extruder_nr].initial_temp = temp;
	extruder_attr[extruder_nr].currentTemperature = temp;

}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
	for (size_t i = 0; i < total_print_times.size(); i++)
	{
		total_print_times[i] = 0.0;
	}
	for (unsigned int e = 0; e < 2; e++)
	{
		extruder_attr[e].totalFilament = 0.0;
		extruder_attr[e].currentTemperature = 0;
		extruder_attr[e].waited_for_temperature = false;
	}
	current_e_value = 0.0;
	//estimateCalculator.reset();
}
void GCodeExport::preSetup(const size_t start_extruder)
{
	current_extruder = start_extruder;
	EGCodeFlavor flavor= EGCodeFlavor::GRIFFIN;
	setFlavor(flavor);
	use_extruder_offset_to_offset_coords = true;
	const size_t extruder_count = 1;  //for current slice
	
	
	for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
	{
		setFilamentDiameter(extruder_nr, MM2INT(2.85));
		extruder_attr[extruder_nr].last_retraction_prime_speed = 15.0;// train.settings.get<Velocity>("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
		extruder_attr[extruder_nr].fan_number = 1;// train.settings.get<size_t>("machine_extruder_cooling_fan_number");
	}
	machine_name = "Ultimaker 3";
	machine_buildplate_type = "glass";
	relative_extrusion = false;
	new_line = "\n";
	for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
	{
		current_max_z_feedrate = 300.0;
		
	}
		
	//estimateCalculator.setFirmwareDefaults(mesh_group->settings);	   velpcity and jerk settings
}


void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
	this->flavor = flavor;
	for (int n = 0; n < 2; n++)
	{
		extruder_attr[n].extruderCharacter = 'E';
		is_volumetric = false;
	}
	
}

void GCodeExport::setFilamentDiameter(const size_t extruder, const double diameter)
{
	const double r = INT2MM(diameter) / 2;
	const double area = M_PI * r * r;
	extruder_attr[extruder].filament_area = area;


}
void GCodeExport::writeCode(const char* str)
{
	
	*output_stream << str << new_line;
}

void GCodeExport::writeComment(const std::string& unsanitized_comment)
{
	const std::string comment = transliterate(unsanitized_comment);

	*output_stream << ";";
	for (unsigned int i = 0; i < comment.length(); i++)
	{
		if (comment[i] == '\n')
		{
			*output_stream << new_line << ";";
		}
		else
		{
			*output_stream << comment[i];
		}
	}
	*output_stream << new_line;
}


void GCodeExport::writeLine(const char* line)
{
	*output_stream << line << new_line;
}

void GCodeExport::writeExtrusionMode(bool set_relative_extrusion_mode)
{
	if (set_relative_extrusion_mode)
	{
		*output_stream << "M83 ;relative extrusion mode" << new_line;
	}
	else
	{
		*output_stream << "M82 ;absolute extrusion mode" << new_line;
	}
}

void GCodeExport::startExtruder(const size_t new_extruder)
{
	extruder_attr[new_extruder].is_used = true;
	if (new_extruder != current_extruder)
	{
		*output_stream << "T" << new_extruder << new_line;
	}
	current_extruder = new_extruder;
	resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

	//Change the Z position so it gets re-written again. We do not know if the switch code modified the Z position.

	currentPosition.z += 1;

	setExtruderFanNumber(new_extruder);
}

void GCodeExport::resetExtrusionValue()
{
	if (!relative_extrusion)
	{
		*output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0" << new_line;
	}

	double current_extruded_volume = getCurrentExtrudedVolume();
	extruder_attr[current_extruder].totalFilament += current_extruded_volume;
	for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
	{ // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
		extruded_volume_at_retraction -= current_extruded_volume;
	}
	current_e_value = 0.0;
	extruder_attr[current_extruder].retraction_e_amount_at_e_start = extruder_attr[current_extruder].retraction_e_amount_current;


}
double GCodeExport::getCurrentExtrudedVolume() const
{
	double extrusion_amount = current_e_value;

	return extrusion_amount;
}
void GCodeExport::setExtruderFanNumber(int extruder)
{
	if (extruder_attr[extruder].fan_number != fan_number)
	{
		fan_number = extruder_attr[extruder].fan_number;
		current_fan_speed = -1; // ensure fan speed gcode gets output for this fan
	}
}



//use by layerplan.h
void GCodeExport::setLayerNr(unsigned int layer_nr_) 
{
	layer_nr = layer_nr_;
}
void GCodeExport::setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor)
{
	this->max_extrusion_offset = max_extrusion_offset;
	this->extrusion_offset_factor = extrusion_offset_factor;
}
void GCodeExport::writeBedTemperatureCommand(const double& temperature, const bool wait)
{
	if (wait)
	{
		*output_stream << "M190 S";
	}
	else
		*output_stream << "M140 S";
	*output_stream << temperature << new_line;
}
void GCodeExport::writePrintAcceleration(const double& acceleration)
{
	
		if (current_print_acceleration != acceleration)
		{
			*output_stream << "M204 S" << acceleration << new_line;
		}
		
	
	current_print_acceleration = acceleration;
	//estimateCalculator.setAcceleration(acceleration);
}
void GCodeExport::writeTravelAcceleration(const double& acceleration)
{
	writePrintAcceleration(acceleration);
	current_travel_acceleration = acceleration;
	//estimateCalculator.setAcceleration(acceleration);
}
void GCodeExport::writeExtrusion(const int x, const int y, const int z, const double& speed, const double extrusion_mm3_per_mm, const PrintFeatureType& feature, coord_tIrfan layer_thickness, const bool update_extrusion_offset)
{
	if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
	{
		return;
	}
	
#ifdef ASSERT_INSANE_OUTPUT
	assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
	assert(currentPosition != no_point3);
	assert(Point3(x, y, z) != no_point3);
	assert((Point3(x, y, z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
	assert(extrusion_mm3_per_mm >= 0.0);
#endif //ASSERT_INSANE_OUTPUT

	if (std::isinf(extrusion_mm3_per_mm))
	{
		printf("Extrusion rate is infinite!");
		assert(false && "Infinite extrusion move!");
		std::exit(1);
	}

	if (std::isnan(extrusion_mm3_per_mm))
	{
		printf("Extrusion rate is not a number!");
		assert(false && "NaN extrusion move!");
		std::exit(1);
	}

	if (extrusion_mm3_per_mm < 0.0)
	{
		printf("Warning! Negative extrusion move!\n");
	}

	double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

	if (is_z_hopped > 0)
	{
		writeZhopEnd();
	}
	   
	Point3 diff = Point3(x, y, z) - currentPosition;
	//printf(" diff is %d %d %d and the current poisitions is %d %d %d and Point  is %d %d %d  \n", diff.x, diff.y, diff.z, x, y, z, currentPosition.x, currentPosition.y, currentPosition.z);

	writeUnretractionAndPrime();

	//flow rate compensation
	
	double extrusion_offset = 0;
	if (diff.vSizeMM())
	{
		extrusion_offset = speed * extrusion_mm3_per_mm * extrusion_offset_factor;
		if (extrusion_offset > max_extrusion_offset)
		{
			extrusion_offset = max_extrusion_offset;
		}
	}
	// write new value of extrusion_offset, which will be remembered.
	if (update_extrusion_offset && (extrusion_offset != current_e_offset))
	{
		current_e_offset = extrusion_offset;
		*output_stream << ";FLOW_RATE_COMPENSATED_OFFSET = " << current_e_offset << new_line;
	}
	
	extruder_attr[current_extruder].last_e_value_after_wipe += extrusion_per_mm * diff.vSizeMM();
	double new_e_value = current_e_value + extrusion_per_mm * diff.vSizeMM();
	//printf("the values of the extrusiona are %f %f %f \n", current_e_value, extrusion_per_mm, diff.vSizeMM());
	*output_stream << "G1";
	writeFXYZE(speed, x, y, z, new_e_value,feature);
}
void GCodeExport::writeUnretractionAndPrime()
{
	const double prime_volume = extruder_attr[current_extruder].prime_volume;
	const double prime_volume_e = mm3ToE(prime_volume);
	current_e_value += prime_volume_e;
	if (extruder_attr[current_extruder].retraction_e_amount_current)
	{
		
		bool machine_firmware_retract = false;
		if (machine_firmware_retract)
		{ // note that BFB is handled differently
			*output_stream << "G11" << new_line;
			//Assume default UM2 retraction settings.
			if (prime_volume != 0)
			{
				const double output_e = (relative_extrusion) ? prime_volume_e : current_e_value;
				//*output_stream << "G1 F" << PrecisionedDouble{ 1, extruder_attr[current_extruder].last_retraction_prime_speed * 60 } << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{ 5, output_e } << new_line;
				currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
			}
		//	estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0, PrintFeatureType::MoveRetraction);
		}
		else
		{
			current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
			const double output_e = (relative_extrusion) ? extruder_attr[current_extruder].retraction_e_amount_current + prime_volume_e : current_e_value;
			*output_stream << "G1 F" << PrecisionedDouble{ 1, extruder_attr[current_extruder].last_retraction_prime_speed * 60 } << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{ 5, output_e } << new_line;
			currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
			estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
		}
	}
	else if (prime_volume != 0.0)
	{
		const double output_e = (relative_extrusion) ? prime_volume_e : current_e_value;
		*output_stream << "G1 F" << PrecisionedDouble{ 1, extruder_attr[current_extruder].last_retraction_prime_speed * 60 } << " " << extruder_attr[current_extruder].extruderCharacter;
		*output_stream << PrecisionedDouble{ 5, output_e } << new_line;
		currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
		estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::NoneType);
	}
	extruder_attr[current_extruder].prime_volume = 0.0;

	if (getCurrentExtrudedVolume() > 10000.0 ) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
	{
		resetExtrusionValue();
	}
	if (extruder_attr[current_extruder].retraction_e_amount_current)
	{
		extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
	}
}

Point3 GCodeExport::getPosition() const
{
	return currentPosition;
}
curaIrfan::PointIrfan GCodeExport::getPositionXY() const
{
	return curaIrfan::PointIrfan(currentPosition.x, currentPosition.y);
}


curaIrfan::PointIrfan GCodeExport::getGcodePos(const coord_tIrfan x, const coord_tIrfan y, const int extruder_train) const
{
	
		return curaIrfan::PointIrfan(x, y);
	
}
void GCodeExport::writeFXYZE(const double& speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature)
{	

	if (currentSpeed != speed)
	{
		*output_stream << " F" << speed * 60;
		currentSpeed = speed;
	}
	
	curaIrfan::PointIrfan gcode_pos = getGcodePos(x, y, current_extruder);
	total_bounding_box.include(Point3(gcode_pos.X, gcode_pos.Y, z));
	
	*output_stream << " X" << MMtoStream{ gcode_pos.X } << " Y" << MMtoStream{ gcode_pos.Y };
	
	if (z != currentPosition.z)
	{
		*output_stream << " Z" << MMtoStream{ z };
	}
	//printf("the current speed is %f and the speed is %f \n", currentSpeed, speed);
	if (layer_nr == 0)
	{
		current_e_offset = 0.000;
	}
	if (e + current_e_offset != current_e_value)
	{
	
		const double output_e = (relative_extrusion) ? e + current_e_offset - current_e_value : e + current_e_offset;
		*output_stream << " E" << output_e;
		
	}
	
	*output_stream << new_line;

	currentPosition = Point3(x, y, z);
	current_e_value = e;
	//estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(e)), speed, feature);




}
void GCodeExport::writeExtrusion(const curaIrfan::PointIrfan& p, coord_tIrfan layer_thickness, const double& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset)
{
	writeExtrusion(Point3(p.X, p.Y, current_layer_z), speed, extrusion_mm3_per_mm, feature, layer_thickness, update_extrusion_offset);
}
void GCodeExport::writeExtrusion(const Point3& p, const double& speed, double extrusion_mm3_per_mm, const PrintFeatureType& feature, coord_tIrfan layer_thickness, bool update_extrusion_offset)
{
	
	writeExtrusion(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature, layer_thickness, update_extrusion_offset);
}

double GCodeExport::getExtrudedVolumeAfterLastWipe(size_t extruder)
{
	return eToMm3(extruder_attr[extruder].last_e_value_after_wipe, extruder);
}

double GCodeExport::getTotalFilamentUsed(size_t extruder_nr)
{
	if (extruder_nr == current_extruder)
		return extruder_attr[extruder_nr].totalFilament + getCurrentExtrudedVolume();
	return extruder_attr[extruder_nr].totalFilament;
}
void   GCodeExport::writeTemperatureCommand(const size_t extruder, const double& temperature, const bool wait)
{
	*output_stream << "M109";
	extruder_attr[extruder].waited_for_temperature = true;

	if (extruder != current_extruder)
	{
		*output_stream << " T" << extruder;
	}
#ifdef ASSERT_INSANE_OUTPUT
	assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
	*output_stream << " S200" << new_line;

	extruder_attr[extruder].currentTemperature = 200;
}

void GCodeExport::writePrimeTrain(const double& travel_speed, coord_tIrfan layer_thicknees)
{

	if (layer_nr==0)//extruder_settings.get<bool>("prime_blob_enable"))
	{ // only move to prime position if we do a blob/poop
		// ideally the prime position would be respected whether we do a blob or not,
		// but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
		// which is needed to automatically ignore the prime position for the printer when blob is disabled
		coord_tIrfan extruder_prime_pos_x = MM2INT (0.00000);
		coord_tIrfan extruder_prime_pos_y = MM2INT (0.00000);
		coord_tIrfan extruder_prime_pos_z = MM2INT (0.00000);

		//extruder_settings.get<coord_t>("extruder_prime_pos_x");
		Point3 prime_pos(extruder_prime_pos_x, extruder_prime_pos_y, extruder_prime_pos_z);
		writeTravel(prime_pos, travel_speed, layer_thicknees);
	}

	bool should_correct_z = false;
	std::string command = "G280";
	*output_stream << command << new_line;
	extruder_attr[current_extruder].is_primed = true;
}
void GCodeExport::writeTravel(const Point3& p, const double& speed, coord_tIrfan layer_thicnkess)
{
	writeTravel(p.x, p.y, p.z + is_z_hopped, speed, layer_thicnkess);
}
void GCodeExport::writeTravel(const curaIrfan::PointIrfan& p, const double& speed, coord_tIrfan layer_thicnkess)
{
	writeTravel(Point3(p.X, p.Y, current_layer_z), speed, layer_thicnkess);
}
void GCodeExport::writeTravel(const coord_tIrfan& x, const coord_tIrfan& y, const coord_tIrfan& z, const double& speed, coord_tIrfan layer_thicnkess)
{
	if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
	{
		return;

	}
#ifdef ASSERT_INSANE_OUTPUT
	assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
	assert(currentPosition != no_point3);
	assert(Point3(x, y, z) != no_point3);
	assert((Point3(x, y, z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT

	const PrintFeatureType travel_move_type = extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
	const int display_width = extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1);
	const double layer_height = INT2MM(layer_thicnkess);// Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("layer_height");
	*output_stream << "G0";
	writeFXYZE(speed, x, y, z, current_e_value, travel_move_type);
	//printf("done with fxfyfz \n");

}

void GCodeExport::writeRetraction(const RetractionConfig& config)
{
	ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];
	bool force = false;
	double old_retraction_e_amount = extr_attr.retraction_e_amount_current;
	double new_retraction_e_amount = mmToE(config.distance);
	double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
	
	if (std::abs(retraction_diff_e_amount) < 0.000001)
	{
		printf("returning from the write retraction function\n");
		return;
	}
	printf("the code is working till line 379 of gcode export.cpp \n");
	{ // handle retraction limitation
		double current_extruded_volume = getCurrentExtrudedVolume();
		printf("The current extruded volume is %f\n", current_extruded_volume);
		std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
		while (extruded_volume_at_previous_n_retractions.size() > config.retraction_count_max && !extruded_volume_at_previous_n_retractions.empty())
		{
			// extruder switch could have introduced data which falls outside the retraction window
			// also the retraction_count_max could have changed between the last retraction and this
			extruded_volume_at_previous_n_retractions.pop_back();
		}
		if (!force && config.retraction_count_max <= 0)
		{
			return;
		}
		if (!force && extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max
			&& current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area)
		{
			return;
		}
		extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
		if (extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max + 1)
		{
			extruded_volume_at_previous_n_retractions.pop_back();
		}
	}

	double speed = ((retraction_diff_e_amount < 0.0) ? config.speed : extr_attr.last_retraction_prime_speed) * 60;
	current_e_value += retraction_diff_e_amount;
	const double output_e = (relative_extrusion) ? retraction_diff_e_amount : current_e_value;
	*output_stream << "G1 F" << speed << " " << "E" << output_e << new_line;
	currentSpeed = speed;
	//estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
	extr_attr.last_retraction_prime_speed = config.primeSpeed;
	extr_attr.retraction_e_amount_current = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
	extr_attr.prime_volume += config.prime_volume;

}

double GCodeExport::mm3ToE(double mm3)
{
	{
		
		return mm3 / extruder_attr[0].filament_area;
	}
}
double GCodeExport::eToMm(double e)
{
	
	{
		return e;
	}
}

double GCodeExport::mmToE(double mm)
{
	
	{
		return mm;
	}
}

double GCodeExport::eToMm3(double e, size_t extruder)
{
	
	{
		return e * extruder_attr[extruder].filament_area;
	}
}

void GCodeExport::writeFanCommand(double speed)
{
	if (std::abs(current_fan_speed - speed) < 0.1)
	{
		return;
	}
	if (speed > 0)
	{
		*output_stream << "M106 S" << PrecisionedDouble{ 1, speed * 255 / 100 };
		if (fan_number)
		{
			*output_stream << " P" << fan_number;
		}
		*output_stream << new_line;

	}
	else
	{
		*output_stream << "M107";
		if (fan_number)
		{
			*output_stream << " P" << fan_number;
		}
		*output_stream << new_line;
	}



	current_fan_speed = speed;
}
std::string GCodeExport::getFileHeader(const std::vector<bool>& extruder_is_used)
{
	double print_time;
	std::vector<double> filament_used;
	std::ostringstream prefix;
	const size_t extruder_count = 1;

	prefix << ";START_OF_HEADER" << new_line;
	prefix << ";HEADER_VERSION:0.1" << new_line;
	prefix << ";FLAVOR:GRIFFIN:" << new_line;
	prefix << ";GENERATOR.NAME:Cura_SteamEngine" << new_line;
	prefix << ";GENERATOR.VERSION: 2.01 " << new_line;
	prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
	prefix << ";;TARGET_MACHINE.NAME:Ultimaker 3" << new_line;
	extruder_attr[1].initial_temp = 205;

	for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
	{
		if (!extruder_is_used[extr_nr])
		{
			continue;
		}
		prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:205" << new_line;
		if (filament_used.size() == extruder_count)
		{
			prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
		}
	    prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:0.4" << new_line;
		prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.NAME:AA 0.4" << new_line;

	}
	
	prefix << ";BUILD_PLATE.TYPE:glass" << new_line;
	prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:60" << new_line;
	prefix << ";BUILD_VOLUME.TEMPERATURE:28" << new_line;
	prefix << ";PRINT.GROUPS:1" << new_line;

	if (total_bounding_box.min.x > total_bounding_box.max.x) //We haven't encountered any movement (yet). This probably means we're command-line slicing.
	{
		//Put some small default in there.
		total_bounding_box.min = Point3(0, 0, 0);
		total_bounding_box.max = Point3(10, 10, 10);
	}
	prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box.min.x) << new_line;
	prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box.min.y) << new_line;
	prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box.min.z) << new_line;
	prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box.max.x) << new_line;
	prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box.max.y) << new_line;
	prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box.max.z) << new_line;
	prefix << ";END_OF_HEADER" << new_line;
	prefix << ";Generated with Cura_SteamEngine 4.2.1";
	//printf("the code is @ line 547 of gcode export .cpp \n");
	return prefix.str();
}



void GCodeExport::setZ(int z)
{
	current_layer_z = z;
}

void GCodeExport::writeLayerComment(const int layer_nr)
{
	*output_stream << ";LAYER:" << layer_nr << new_line;
}

void GCodeExport::writeextrusion()
{
	*output_stream << ";Extrusion:" << current_e_value << new_line;
}

void GCodeExport::writeLayerCountComment(const size_t layer_count)
{
	*output_stream << ";LAYER_COUNT:" << layer_count << new_line;
}

void GCodeExport::writeDelay(const Duration& time_amount)
{
	*output_stream << "G4 P" << int(time_amount * 1000) << new_line;
	estimateCalculator.addTime(time_amount);
}

void GCodeExport::ResetLastEValueAfterWipe(size_t extruder)
{
	extruder_attr[extruder].last_e_value_after_wipe = 0;
}

void GCodeExport::insertWipeScript(const WipeScriptConfig& wipe_config, coord_tIrfan layer_thicnkess)
{
	Point3 prev_position = currentPosition;
	writeComment("WIPE_SCRIPT_BEGIN");

	if (wipe_config.retraction_enable)
	{
		writeRetraction(wipe_config.retraction_config);
	}

	if (wipe_config.hop_enable)
	{
		writeZhopStart(wipe_config.hop_amount, wipe_config.hop_speed);
	}
	
	writeTravel(curaIrfan::PointIrfan(wipe_config.brush_pos_x, currentPosition.y), wipe_config.move_speed, layer_thicnkess);
	for (size_t i = 0; i < wipe_config.repeat_count; ++i)
	{
		coord_tIrfan x = currentPosition.x + (i % 2 ? -wipe_config.move_distance : wipe_config.move_distance, layer_thicnkess);
		writeTravel(curaIrfan::PointIrfan(x, currentPosition.y), wipe_config.move_speed, layer_thicnkess);
	}

	writeTravel(prev_position, wipe_config.move_speed, layer_thicnkess);

	if (wipe_config.hop_enable)
	{
		writeZhopEnd(wipe_config.hop_speed);
	}

	if (wipe_config.retraction_enable)
	{
		writeUnretractionAndPrime();
	}

	if (wipe_config.pause > 0)
	{
		writeDelay(wipe_config.pause);
	}

	writeComment("WIPE_SCRIPT_END");
}