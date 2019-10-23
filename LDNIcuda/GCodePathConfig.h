#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef G_CODE_PATH_CONFIG_H
#define G_CODE_PATH_CONFIG_H

#include "PrintFeature.h"
#include "Coord_tIrfan.h"


	class GCodePathConfig
	{
	public:
		/*!
		 * A simple wrapper class for all derivatives of position which are used when printing a line
		 */
		struct SpeedDerivatives
		{
			double speed; //!< movement speed (mm/s)
			double acceleration; //!< acceleration of head movement (mm/s^2)
			double jerk; //!< jerk of the head movement (around stand still) as instantaneous speed change (mm/s)
		};
		//!< name of the feature type
		const PrintFeatureType type;
		static constexpr double FAN_SPEED_DEFAULT = -1;
	private:
		SpeedDerivatives speed_derivatives; //!< The speed settings (and acceleration and jerk) of the extruded line. May be changed when smoothSpeed is called.
		const coord_tIrfan line_width; //!< width of the line extruded
		const coord_tIrfan layer_thickness; //!< current layer height in micron
		const double flow; //!< extrusion flow modifier.
		const double extrusion_mm3_per_mm;//!< current mm^3 filament moved per mm line traversed
		const bool is_bridge_path; //!< whether current config is used when bridging
		const double fan_speed; //!< fan speed override for this path, value should be within range 0-100 (inclusive) and ignored otherwise
	public:
		GCodePathConfig(const PrintFeatureType& type, const coord_tIrfan line_width, const coord_tIrfan layer_height, const double& flow, const SpeedDerivatives speed_derivatives, const bool is_bridge_path = false, const double fan_speed = FAN_SPEED_DEFAULT);

		/*!
		 * copy constructor
		 */
		GCodePathConfig(const GCodePathConfig& other);

		/*!
		 * Set the speed to somewhere between the speed of @p first_layer_config and the iconic speed.
		 *
		 * \warning This functions should not be called with @p layer_nr > @p max_speed_layer !
		 *
		 * \warning Calling this function twice will smooth the speed more toward \p first_layer_config
		 *
		 * \param first_layer_config The speed settings at layer zero
		 * \param layer_nr The layer number
		 * \param max_speed_layer The layer number for which the speed_iconic should be used.
		 */
		void smoothSpeed(SpeedDerivatives first_layer_config, const int& layer_nr, const int& max_speed_layer);

		/*!
		 * Can only be called after the layer height has been set (which is done while writing the gcode!)
		 */
		double getExtrusionMM3perMM() const;

		/*!
		 * Get the movement speed in mm/s
		 */
		double getSpeed() const;

		/*!
		 * Get the current acceleration of this config
		 */
		double getAcceleration() const;

		/*!
		 * Get the current jerk of this config
		 */
		double getJerk() const;

		coord_tIrfan getLineWidth() const;

		bool isTravelPath() const;

		bool isBridgePath() const;

		double getFanSpeed() const;

		double getFlowRatio() const;

		coord_tIrfan getLayerThickness() const;

		//const PrintFeatureType& getPrintFeatureType() const;

	private:
		double calculateExtrusion() const;
	};


#endif // G_CODE_PATH_CONFIG_H

