// This provides some plotting functionality on top of implot that scales to lots of plots
// with very large time-series data.
// The main idea is to separate the ui element that enables a specific plot (plot_history())
// from the actual drawing of the ui element with all the plots in it (plot_draw()).
// Using this, you can place lots of plot_history calls in your UI code and structure it
// however you like.
// The downside is that you need to keep track of a Plot pointer for each plot window,
// but usually you just have one plot window, so it's not really an issue.
// This also assumes that the provided data is sorted with respect to time.
// 
// Another feature is that plot_history() will call your get_value-lambda only for the
// parts of the plot that are visible. This way it will remain fast, even if you have lots of data,
// and you can still compute values on the fly.
//
// You can also supply different units to the plots in a very simple way (with Plot_Y_Axis).
// Different units can then by transformed independently by the user and a separate axis is shown
// for each visible unit.
// Only up to 3 units are supported at this time though (an implot limitation).

#pragma once
#include "../common/helper.h"
#include <functional>

struct Plot_Y_Axis
{
	const char* name;
	double y_min = 0, y_max = 1;
};

struct Plot;

Plot* plot_create();

void set_plot_history_elements(Plot* p, s64 history_num_elements,
		std::function<s64(s64)> get_history_time, double time_to_seconds_factor);

void plot_get_display_range(Plot* p, s64* start, s64* end);

void plot_set_x_limit_min(Plot* p, double x_min);
void plot_set_x_limit_max(Plot* p, double x_max);
void plot_set_x_limit_width(Plot* p, double width);
void plot_reset_display_range(Plot* p);
s64 plot_get_visual_selection_index(Plot* p);

void plot_set_visual_selection_index(Plot* p, s64 index);

void plot_history(Plot* p, const char* name, const std::function<float(s64)>& get_value, bool* show, Plot_Y_Axis* y_axis = nullptr);

void plot_draw(Plot* p, float height);
int plot_get_num_active_graphs(Plot* p);
