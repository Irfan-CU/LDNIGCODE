#pragma once

#include <cassert>
#include <vector>

#include "Temprature.h"

class FlowTempGraph
{
public:
	struct Datum
	{
		const double flow; //!< The flow in mm^3/s
		const Temperature temp; //!< The temperature in *C
		Datum(const double flow, const Temperature temp)
			: flow(flow)
			, temp(temp)
		{}
	};

	std::vector<Datum> data; //!< The points of the graph between which the graph is linearly interpolated

	/*!
	 * Get the temperature corresponding to a specific flow.
	 *
	 * For flows outside of the chart, the temperature at the minimal or maximal flow is returned.
	 * When the graph is empty, the @p material_print_temperature is returned.
	 *
	 * \param flow the flow in mm^3/s
	 * \param material_print_temperature The default printing temp (backward compatibility for when the graph fails)
	 * \return the corresponding temp
	 */
	double getTemp(const double flow, const Temperature material_print_temperature, const bool flow_dependent_temperature) const;
};
