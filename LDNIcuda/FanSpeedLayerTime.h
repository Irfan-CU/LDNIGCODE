#pragma once
#ifndef FanSettings
#define FanSettings


struct FanSpeedLayerTimeSettings

{
public:
	double cool_min_layer_time;
	double cool_min_layer_time_fan_speed_max;
	double cool_fan_speed_0;
	double cool_fan_speed_min;
	double cool_fan_speed_max;
	double cool_min_speed;	 //velocity
	int cool_fan_full_layer;
};
#endif // !FanSettings
