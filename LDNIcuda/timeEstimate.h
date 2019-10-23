#pragma once
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef TIME_ESTIMATE_H
#define TIME_ESTIMATE_H

#include <stdint.h>
#include <vector>
#include <unordered_map>

#include "PrintFeature.h"
#include "Duration.h" //Print time estimates.
 //Speeds and accelerations at which we print.


	/*!
	 *  The TimeEstimateCalculator class generates a estimate of printing time calculated with acceleration in mind.
	 *  Some of this code has been adapted from the Marlin sources.
	 */

	class TimeEstimateCalculator
	{
	public:
		constexpr static unsigned int NUM_AXIS = 4;
		constexpr static unsigned int X_AXIS = 0;
		constexpr static unsigned int Y_AXIS = 1;
		constexpr static unsigned int Z_AXIS = 2;
		constexpr static unsigned int E_AXIS = 3;


		class Position
		{
		public:
			Position() { for (unsigned int n = 0; n < NUM_AXIS; n++) axis[n] = 0; }
			Position(double x, double y, double z, double e) { axis[0] = x; axis[1] = y; axis[2] = z; axis[3] = e; }
			double axis[NUM_AXIS];

			double& operator[](const int n) { return axis[n]; }
		};

		class Block
		{
		public:
			bool recalculate_flag;

			double accelerate_until;
			double decelerate_after;
			double initial_feedrate;
			double final_feedrate;

			double entry_speed;
			double max_entry_speed;
			bool nominal_length_flag;

			double nominal_feedrate;
			double maxTravel;
			double distance;
			double acceleration;
			Position delta;
			Position absDelta;

			PrintFeatureType feature;
		};

	private:
		double max_feedrate[NUM_AXIS] = { 600, 600, 40, 25 }; // mm/s
		double minimumfeedrate = 0.01;
		double acceleration = 3000;
		double max_acceleration[NUM_AXIS] = { 9000, 9000, 100, 10000 };
		double max_xy_jerk = 20.0;
		double max_z_jerk = 0.4;
		double max_e_jerk = 5.0;
		Duration extra_time = 0.0;

		Position previous_feedrate;
		double previous_nominal_feedrate;

		Position currentPosition;

		std::vector<Block> blocks;
	public:
		/*!
		 * \brief Set the movement configuration of the firmware.
		 * \param settings_base Where to get the settings from.
		 */
		void setFirmwareDefaults();
		void setPosition(Position newPos);
		void plan(Position newPos, double feedRate, PrintFeatureType feature);
		void addTime(const Duration& time);
		void setAcceleration(const double& acc); //!< Set the default acceleration to \p acc
		void setMaxXyJerk(const double& jerk); //!< Set the max xy jerk to \p jerk

		void reset();

		std::vector<Duration> calculate();
	private:
		void reverse_pass();
		void forward_pass();
		void recalculate_trapezoids();

		void calculate_trapezoid_for_block(Block *block, const double entry_factor, const double exit_factor);
		void planner_reverse_pass_kernel(Block *previous, Block *current, Block *next);
		void planner_forward_pass_kernel(Block *previous, Block *current, Block *next);
	};

//namespace cura
#endif//TIME_ESTIMATE_H

