#pragma once
#ifndef PATH_PLANNING_NOZZLE_TEMP_INSERT_H
#define PATH_PLANNING_NOZZLE_TEMP_INSERT_H

class GCodeExport;

struct NozzleTempInsert
{
	const unsigned int path_idx; //!< The path before which to insert this command
	double time_after_path_start; //!< The time after the start of the path, before which to insert the command // TODO: use this to insert command in between moves in a path!
	int extruder; //!< The extruder for which to set the temp
	double temperature; //!< The temperature of the temperature command to insert
	bool wait; //!< Whether to wait for the temperature to be reached
	NozzleTempInsert(unsigned int path_idx, int extruder, double temperature, bool wait, double time_after_path_start = 0.0);

	/*!
	 * Write the temperature command at the current position in the gcode.
	 * \param gcode The actual gcode writer
	 */
	void write(GCodeExport& gcode);
};
#endif

