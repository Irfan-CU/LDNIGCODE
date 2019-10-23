//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include "PrintFeature.h"
#include "SpaceFillType.h"
#include "IntpointIrfan.h"


#include "timeMaterialEstimate.h"


/*!
	 * A class for representing a planned path.
	 *
	 * A path consists of several segments of the same type of movement: retracted travel, infill extrusion, etc.
	 *
	 * This is a compact premature representation in which are line segments have the same config, i.e. the config of this path.
	 *
	 * In the final representation (gcode) each line segment may have different properties,
	 * which are added when the generated GCodePaths are processed.
	 */
	class GCodePath
	{
	public:
	
		double speed; //!< movement speed (mm/s)
		double acceleration; //!< acceleration of head movement (mm/s^2)
		double jerk;
		const PrintFeatureType type;
		static constexpr double FAN_SPEED_DEFAULT = -1;
		//SpeedDerivatives speed_derivatives; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
		const coord_tIrfan line_width1; //!< wiflowdth of the line extruded
		const coord_tIrfan layer_thickness1; //!< current layer height in micron
		double flow1; //!< extrusion flow modifier.
		double flowconfig;
		const double extrusion_mm3_per_mm1;//!< current mm^3 filament moved per mm line traversed
		const bool is_bridge_path1; //!< whether current config is used when bridging
		double fan_speed1;
		int layer_nr;
		std::string mesh_id; //!< Which mesh this path belongs to, if any. If it's not part of any mesh, the mesh ID should be 0.
		SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
		//double flow; //!< A type-independent flow configuration (used for wall overlap compensation)
		double speed_factor; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
		bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path. 
		bool perform_z_hop; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
		bool perform_prime; //!< Whether this path is preceded by a prime (blob)
		bool skip_agressive_merge_hint; //!< Wheter this path needs to skip merging if any travel paths are in between the extrusions.
		std::vector<curaIrfan::PointIrfan> points; //!< The points constituting this path.
		bool done; //!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

		bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

		//double fan_speed; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise

		TimeMaterialEstimates estimates;
		

		/*!
		 * \brief Creates a new g-code path.
		 *
		 * \param config The line configuration to use when printing this path.
		 * \param mesh_id The mesh that this path is part of.
		 * \param space_fill_type The type of space filling of which this path is a
		 * part.
		 * \param flow The flow rate to print this path with.
		 * \param spiralize Gradually increment the z-coordinate while traversing
		 * \param speed_factor The factor that the travel speed will be multiplied with
		 * this path.
		 */
		//GCodePath(const SpaceFillType space_fill_type, const double flow, const bool spiralize, const double speed_factor = 1.0);
		GCodePath(double speed, double acceleration, double jerk, const PrintFeatureType type, const coord_tIrfan line_width1, const coord_tIrfan layer_thickness1, double flowconfig, std::string mesh_id, const SpaceFillType space_fill_type, double flow1, const bool spiralize, int layernum, const double speed_factor = 1.0);
		
	
		/*!
		 * Whether this config is the config of a travel path.
		 *
		 * \return Whether this config is the config of a travel path.
		 */
		bool isTravelPath() const;

		/*!
		 * Get the material flow in mm^3 per mm traversed.
		 *
		 * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
		 *
		 * \return The flow
		 */
		double getExtrusionMM3perMM() const;

		/*!
		 * Get the actual line width (modulated by the flow)
		 * \return the actual line width as shown in layer view
		 */
		coord_tIrfan getLineWidthForLayerView() const;

		/*!
		 * Set fan_speed
		 *
		 * \param fan_speed the fan speed to use for this path
		 */
		void setFanSpeed(double fan_speed);

		/*!
		 * Get the fan speed for this path
		 * \return the value of fan_speed if it is in the range 0-100, otherwise the value from the config
		 */
		double getFanSpeed() const;

		double calculateExtrusion() const;

		coord_tIrfan getLineWidth1() const;

		coord_tIrfan getLayerthickness1() const;
		double getFlowRatio() const;
		bool isTravelPath1() const;
		double getSpeed();
		double getAccelration();
		double getJerk();


	};

//namespace cura

#endif//PATH_PLANNING_G_CODE_PATH_H
