#pragma once

#ifndef RETRACTION_CONFIG_H
#define RETRACTION_CONFIG_H

#include "IntpointIrfan.h"

class RetractionConfig
{
public:
	double distance; //!< The distance retracted (in mm)
	double speed; //!< The speed with which to retract (in mm/s)
	double primeSpeed; //!< the speed with which to unretract (in mm/s)
	double prime_volume; //!< the amount of material primed after unretracting (in mm^3)
	coord_tIrfan zHop; //!< the amount with which to lift the head during a retraction-travel
	coord_tIrfan retraction_min_travel_distance; //!< Minimal distance traversed to even consider retracting (in micron)
	double retraction_extrusion_window; //!< Window of mm extruded filament in which to limit the amount of retractions
	size_t retraction_count_max=90; //!< The maximum amount of retractions allowed to occur in the RetractionConfig::retraction_extrusion_window
};
#endif