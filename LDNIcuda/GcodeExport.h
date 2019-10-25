#pragma once

#ifndef GCODEEXPORT_H
#define GCODEEXPORT_H

#include <deque>
#include <sstream>
#include <stdio.h>

#include "AABB3D.h"
#include "timeEstimate.h"
#include "ENUMSettings.h"
#include "PrintFeature.h"
#include "IntpointIrfan.h"


class RetractionConfig;
struct WipeScriptConfig;
  
class  GCodeExport
{
public:
	struct ExtruderTrainAttributes
	{
		bool is_primed; //!< Whether this extruder has currently already been primed in this print

		bool is_used; //!< Whether this extruder train is actually used during the printing of all meshgroups
		char extruderCharacter;

		double filament_area; //!< in mm^2 for non-volumetric, cylindrical filament

		double totalFilament; //!< total filament used per extruder in mm^3
		double currentTemperature;
		bool waited_for_temperature; //!< Whether the most recent temperature command has been a heat-and-wait command (M109) or not (M104).
		double  initial_temp; //!< Temperature this nozzle needs to be at the start of the print.

		std::string machine_name;
		
		double retraction_e_amount_current; //!< The current retracted amount (in mm or mm^3), or zero(i.e. false) if it is not currently retracted (positive values mean retracted amount, so negative impact on E values)
		double retraction_e_amount_at_e_start; //!< The ExtruderTrainAttributes::retraction_amount_current value at E0, i.e. the offset (in mm or mm^3) from E0 to the situation where the filament is at the tip of the nozzle.

		double prime_volume; //!< Amount of material (in mm^3) to be primed after an unretration (due to oozing and/or coasting)
		double last_retraction_prime_speed; //!< The last prime speed (in mm/s) of the to-be-primed amount

		double last_e_value_after_wipe; //!< The current material amount extruded since last wipe

		unsigned fan_number; // nozzle print cooling fan number

		std::deque<double> extruded_volume_at_previous_n_retractions; // in mm^3

		ExtruderTrainAttributes()
			: is_primed(false)
			, is_used(false)
			, extruderCharacter(0)
			, filament_area(0)
			, totalFilament(0)
			, currentTemperature(0)
			, waited_for_temperature(false)
			, initial_temp(0)
			, retraction_e_amount_current(0.0)
			, retraction_e_amount_at_e_start(0.0)
			, prime_volume(0.0)
			, last_retraction_prime_speed(0.0)
			, fan_number(0)
		{ }
	};

	ExtruderTrainAttributes extruder_attr[16];
	//used by layerplan.h
	unsigned int layer_nr;
	void setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor);
	void writeBedTemperatureCommand(const double& temperature, const bool wait = false);
	void writePrintAcceleration(const double& acceleration);
	void writeJerk(const double& jerk);
	void writeTravelAcceleration(const double& acceleration);
	void writeExtrusion(const curaIrfan::PointIrfan& p,coord_tIrfan layer_thickness, const double& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset = false);
	void writeExtrusion(const Point3& p, const double& speed, double extrusion_mm3_per_mm, const PrintFeatureType& feature, coord_tIrfan layer_thickness, bool update_extrusion_offset = false);
	void writeExtrusion(const int x, const int y, const int z, const double& speed, const double extrusion_mm3_per_mm, const PrintFeatureType& feature, coord_tIrfan layer_thickness,const bool update_extrusion_offset = false);
	void writeFXYZE(const double& speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature);
	void writeUnretractionAndPrime();
	double getExtrudedVolumeAfterLastWipe(size_t extruder);
	curaIrfan::PointIrfan getGcodePos(const coord_tIrfan x, const coord_tIrfan y, const int extruder_train) const;

	double current_e_offset; //!< Offset to compensate for flow rate (mm or mm^3)
	double max_extrusion_offset; //!< 0 to turn it off, normally 4
	double extrusion_offset_factor; //!< default 1
	double current_print_acceleration; //!< The current acceleration (in mm/s^2) used for print moves (and also for travel moves if the gcode flavor doesn't have separate travel acceleration)
	double current_travel_acceleration;
	double current_jerk;
	TimeEstimateCalculator estimateCalculator;
	double current_max_z_feedrate;

	Point3 getPosition() const;

	curaIrfan::PointIrfan getPositionXY() const;

	int getPositionZ() const;

	void preSetup(const size_t start_extruder);

	void setFlavor(EGCodeFlavor flavor);

	void setFilamentDiameter(const size_t extruder, const double diameter);

	std::string getFileHeader(const std::vector<bool>& extruder_is_used);

	double getTotalFilamentUsed(size_t extruder_nr);

	bool use_extruder_offset_to_offset_coords;

	size_t current_extruder;

	std::string machine_name = "Ultimaker 3";

	std::string machine_buildplate_type;

	bool relative_extrusion;

	std::ostream* output_stream;
	std::string new_line;

	void writeComment(const std::string& comment);

	EGCodeFlavor flavor;

	void setLayerNr(unsigned int layer_nr);

	AABB3D total_bounding_box;

	

	void writeCode(const char*str);

	void writeLine(const char*line);

	void writeExtrusionMode(bool set_relative_extrusion_mode);

	void startExtruder(const size_t new_extruder);

	void resetTotalPrintTimeAndFilament();

	void setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr);

	void setInitialTemp(int extruder_nr, double temp);

	void resetExtrusionValue();

	double getCurrentExtrudedVolume() const;

	void setExtruderFanNumber(int extruder);

	void writeTemperatureCommand(const size_t extruder, const double& temperature, const bool wait);

	void writePrimeTrain(const double& travel_speed,coord_tIrfan layer_thickness);

	void setOutputStream(std::ostream* stream);

	void writeTravel(const curaIrfan::PointIrfan& p, const double& speed, coord_tIrfan layer_thicnkess);

	void writeTravel(const Point3& p, const double& speed, coord_tIrfan layer_thicnkess);

	void writeTravel(const coord_tIrfan& x, const coord_tIrfan& y, const coord_tIrfan& z, const double& speed, coord_tIrfan layer_thicnkess);

	void GCodeExport::writeDelay(const Duration& time_amount);
	
	void writeRetraction(const RetractionConfig& config);

	double mm3ToE(double mm3);
	double mmToE(double mm);
	double eToMm3(double e, size_t extruder);
	double eToMm(double e);
	void writeFanCommand(double speed);

	void writeLayerComment(const int layer_nr);

	void writeextrusion();

	void writeLayerCountComment(const size_t layer_count);

	int getExtruderNr() const;


	void setZ(int z);

	coord_tIrfan max_object_height;

	double current_e_value;

	double initial_bed_temp;

	double build_volume_temperature;

	Point3 currentPosition;

	double current_fan_speed;

	unsigned fan_number;

	bool is_volumetric;

	coord_tIrfan current_layer_z;
	

	coord_tIrfan is_z_hopped;

	std::vector<double> total_print_times;


	void writeTypeComment(const PrintFeatureType& type);

	void writeZhopStart(const coord_tIrfan hop_height, double speed = 0);

	/*!
	 * End a z hop: go back to the layer height
	 *
	 * \param speed The speed used for moving.
	 */
	void writeZhopEnd(double speed = 0);

	double currentSpeed;

	void ResetLastEValueAfterWipe(size_t extruder);

	void insertWipeScript(const WipeScriptConfig& wipe_config,coord_tIrfan layer_thicnkess);

	GCodeExport();
	~GCodeExport();


};
#endif
