#include "plot.h"
#include <vector>
#include <string>
#include <algorithm>

#include "imgui.h"
#include "../3rdparty/implot/implot.h"
#include "../3rdparty/fonts/IconsFontAwesome6.h"

struct Plot_Graph
{
	std::string name;
	std::vector<double> x, y;
	ImVec4 color = {0, 0, 0, 1};
	Plot_Y_Axis* y_axis;
};

struct Plot
{
	s64 history_num_elements = 0;
	std::function<s64(s64)> get_history_time;
	double time_to_seconds_factor = 0;

	std::vector<Plot_Graph> graphs;
	int graph_pos = 0;
	std::vector<Plot_Y_Axis*> y_axes;

	double x_min = 0, x_max = 0;
	double x_selection = 0;
	double x_selection_relative = 0.9;
	bool auto_scroll = true;
	bool ui_dragging_limits = false;
	bool reset_display_range = true;
	bool reset_x_selection = true;
};

Plot* plot_create()
{
	return new Plot;
}


float get_history_value(Plot* p, s64 index, const std::function<float(s64)>& get_value)
{
	assert(index >= 0 && index < p->history_num_elements);
	return get_value(index);
}

s64 plot_get_history_time(Plot* p, s64 index)
{
	assert(index >= 0 && index < p->history_num_elements);
	return p->get_history_time(index);
}

void set_plot_history_elements(Plot* p, s64 history_num_elements,
		std::function<s64(s64)> get_history_time, double time_to_seconds_factor)
{
	p->history_num_elements = history_num_elements;
	p->get_history_time = std::move(get_history_time);
	p->time_to_seconds_factor = time_to_seconds_factor;

	p->graph_pos = 0;
	p->y_axes.clear();

	if (p->reset_display_range)
	{
		p->reset_display_range = false;
		if (p->history_num_elements)
		{
			p->x_min = plot_get_history_time(p, 0) * p->time_to_seconds_factor;
			p->x_max = plot_get_history_time(p, p->history_num_elements-1) * p->time_to_seconds_factor;
			if (p->x_max < p->x_min+10) p->x_max = p->x_min+10;
		}
		else
		{
			p->x_min = 0;
			p->x_max = 10;
		}
	}
	if (p->reset_x_selection)
	{
		p->reset_x_selection = false;
		p->x_selection = p->x_min + p->x_selection_relative*(p->x_max-p->x_min);
	}
}

s64 plot_time_to_history_index(Plot* p, s64 time, bool round)
{
	if (p->history_num_elements == 0)
		return 0;
	s64 start = 0;
	s64 end = p->history_num_elements-1;
	while (true)
	{
		if (start == end)
			break;
		s64 middle = (start+end)/2;
		if (plot_get_history_time(p, middle) < time)
			start = middle+1;
		else
			end = middle;
	}
	if (start == 0) return 0;
	assert(plot_get_history_time(p, start-1) <= time);
	assert(start == p->history_num_elements-1 || time <= plot_get_history_time(p, start));
	if (!round) return start;
	if (time-plot_get_history_time(p, start-1) < plot_get_history_time(p, start)-time)
		return start-1;
	else
		return start;
}

void plot_get_display_range(Plot* p, s64* start, s64* end)
{
	s64 time_start = (s64)(p->x_min / p->time_to_seconds_factor);
	s64 time_end   = (s64)(p->x_max / p->time_to_seconds_factor);

	*start = plot_time_to_history_index(p, time_start, false)-1;
	if (*start == -1) *start = 0;

	*end = plot_time_to_history_index(p, time_end+1, false)+1;
	if (*end > p->history_num_elements) *end = p->history_num_elements;
}

void plot_set_x_limit_min(Plot* p, double x_min)
{
	p->x_min = x_min;
	p->reset_display_range = false;
	p->reset_x_selection = true;
}

void plot_set_x_limit_max(Plot* p, double x_max)
{
	p->x_max = x_max;
	p->reset_display_range = false;
	p->reset_x_selection = true;
}

void plot_set_x_limit_width(Plot* p, double width)
{
	p->x_max = p->x_min + width;
	p->reset_display_range = false;
	p->reset_x_selection = true;
}

void plot_reset_display_range(Plot* p)
{
	// history_num_elements might not be set yet, delay this until rendering
	p->reset_display_range = true;
	p->reset_x_selection = true;

	// Prevent crash in case the elements are reduced
	p->graphs.clear();
	p->graph_pos = 0;
	p->history_num_elements = 0;
}

s64 plot_get_visual_selection_index(Plot* p)
{
	s64 selection_time = (s64)round(p->x_selection / p->time_to_seconds_factor);
	return plot_time_to_history_index(p, selection_time, true);
}

void plot_set_visual_selection_index(Plot* p, s64 index)
{
	if (p->history_num_elements == 0) return;
	if (index < 0) index = 0;
	if (index > p->history_num_elements) index = p->history_num_elements-1;
	p->x_selection = plot_get_history_time(p, index) * p->time_to_seconds_factor;

	double width = p->x_max-p->x_min;
	p->x_min = p->x_selection - p->x_selection_relative*width;
	p->x_max = p->x_min+width;
}

void plot_history(Plot* p, const char* name, const std::function<float(s64)>& get_value, bool* show, Plot_Y_Axis* y_axis)
{
	float value = p->history_num_elements ? get_history_value(p, plot_get_visual_selection_index(p), get_value) : 0;
	char buffer[128];
	sprintf_s(buffer, "%s %.03f%s###%s", name, value, y_axis && y_axis->name ? y_axis->name : "", name);

	bool do_colored = *show;
	Plot_Graph* g = nullptr;
	if (do_colored)
	{
		p->graph_pos++;
		if (p->graphs.size() < p->graph_pos)
			p->graphs.resize(p->graph_pos);

		g = &p->graphs[p->graph_pos-1];

		ImGui::PushStyleColor(ImGuiCol_Text, g->color);
		//ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(plot_colors[plot_color_index][0], plot_colors[plot_color_index][1], plot_colors[plot_color_index][2], 1));
	}
	ImGui::Text("%s", ICON_FA_WAVE_SQUARE);
	ImGui::SameLine();
	ImGui::Checkbox(buffer, show);
	if (do_colored) ImGui::PopStyleColor();

	if (!g && *show)
	{
		p->graph_pos++;
		if (p->graphs.size() < p->graph_pos)
			p->graphs.resize(p->graph_pos);

		g = &p->graphs[p->graph_pos-1];
	}
	if (g && !*show)
	{
		p->graph_pos--;
	}

	if (*show)
	{
		for (int i = 0; i < p->y_axes.size(); i++)
			if (p->y_axes[i] == y_axis)
				goto skip;
		p->y_axes.push_back(y_axis);
		skip:

		s64 start, end;
		plot_get_display_range(p, &start, &end);

		g->name = name;
		g->x.resize(end-start);
		g->y.resize(end-start);
		g->y_axis = y_axis;
		for (s64 i = start; i < end; ++i)
		{
			s64 time = plot_get_history_time(p, i);
			float value = get_history_value(p, i, get_value);
			g->x[i-start] = (double)time*p->time_to_seconds_factor;
			g->y[i-start] = (double)value;
		}
	}
}

bool ImGuiSliderInt64(const char* label, s64* v, s64 v_min, s64 v_max, const char* format, ImGuiSliderFlags flags)
{
    return ImGui::SliderScalar(label, ImGuiDataType_S64, v, &v_min, &v_max, format, flags);
}
bool ImGuiSliderDouble(const char* label, double* v, double v_min, double v_max, const char* format, ImGuiSliderFlags flags)
{
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
}
bool ImGuiDragInt64(const char* label, s64* v, float v_speed, s64 v_min, s64 v_max, const char* format, ImGuiSliderFlags flags)
{
    return ImGui::DragScalar(label, ImGuiDataType_S64, v, v_speed, &v_min, &v_max, format, flags);
}

static void HelpMarkerPlotUserGuide()
{
    ImGui::TextDisabled("%s", ICON_FA_CIRCLE_QUESTION);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImPlot::ShowUserGuide();
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void plot_draw(Plot* p, float height)
{
	p->graphs.resize(p->graph_pos);
	p->graph_pos = 0;

	if (p->history_num_elements == 0)
		return;

	//ImGui::TextDisabled("Axes:");
	//for (int i = 0; i < p->y_axes.size(); i++)
	//	ImGui::TextDisabled(p->y_axes[i] && p->y_axes[i]->name ? p->y_axes[i]->name : "null");

	double limit_min = plot_get_history_time(p, 0) * p->time_to_seconds_factor;
	double limit_max = plot_get_history_time(p, p->history_num_elements-1) * p->time_to_seconds_factor;
	double limit_extra = 10;

	bool dragging_x_selection = false;
	double old_x_min = p->x_min;
	double width = p->x_max-p->x_min;

	if (ImPlot::BeginPlot("Plot2", ImVec2(-1, height), ImPlotFlags_NoTitle))
	{
        ImPlot::SetupLegend(ImPlotLocation_SouthWest, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);

        ImPlot::SetupAxisLinks(ImAxis_X1, &p->x_min, &p->x_max);
        //ImPlot::SetupAxes(nullptr, nullptr, 0, ImPlotAxisFlags_AutoFit);

		ImPlot::SetupAxisLimitsConstraints(ImAxis_X1, limit_min-limit_extra, limit_max+limit_extra);


		Plot_Y_Axis* axes[3] = {(Plot_Y_Axis*)-1, (Plot_Y_Axis*)-1, (Plot_Y_Axis*)-1};
		for (int i = 0; i < p->y_axes.size(); i++)
		{
			const char* name = p->y_axes[i] && p->y_axes[i]->name ? p->y_axes[i]->name : nullptr;
            if (i == 0) ImPlot::SetupAxis(ImAxis_Y1+i, name, ImPlotAxisFlags_NoInitialFit);
            else        ImPlot::SetupAxis(ImAxis_Y1+i, name, ImPlotAxisFlags_NoInitialFit | ImPlotAxisFlags_NoGridLines);
			if (name) ImPlot::SetupAxisLimits(ImAxis_Y1+i, p->y_axes[i]->y_min, p->y_axes[i]->y_max, ImPlotCond_Once);
			axes[i] = p->y_axes[i];
			if (i == 2) break;
		}

		for (Plot_Graph& g : p->graphs)
		{
			if (g.y_axis == axes[0]) ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
			else if (g.y_axis == axes[1]) ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
			else if (g.y_axis == axes[2]) ImPlot::SetAxes(ImAxis_X1, ImAxis_Y3);
			else continue;
			ImPlot::SetNextMarkerStyle(ImPlotMarker_None);
			ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.25f);
			ImPlot::PlotStairs(g.name.c_str(), g.x.data(), g.y.data(), (int)g.x.size(), 0);
			g.color = ImPlot::GetLastItemColor();
		}

        ImPlot::DragLineX(0, &p->x_selection, ImVec4(0,0,0.7f,1), 1, ImPlotDragToolFlags_NoFit, 0, 0, &dragging_x_selection);
        ImPlot::TagX(p->x_selection, ImVec4(0,0,0.7f,1), " ");
        //ImPlot::TagX(p->x_min + p->x_selection_relative*width, ImVec4(0,1,0.7f,1), " ");

		ImPlot::EndPlot();
	}


	ImGuiSliderDouble("selection", &p->x_selection, limit_min, limit_max+limit_extra, "%.1f", 0);
	bool changing_x_selection = ImGui::IsItemActive();
	ImGui::SameLine();
    HelpMarkerPlotUserGuide();

	width = p->x_max-p->x_min;
	ImGuiSliderDouble("scale", &width, 0.1, std::max(limit_max-limit_min, limit_extra), "%.1f", ImGuiSliderFlags_Logarithmic);
	bool changing_width = ImGui::IsItemActive();

	if (ImGui::Checkbox("Autoscroll", &p->auto_scroll) && p->auto_scroll)
	{
		p->x_selection = limit_max;	
		p->x_selection_relative = (p->x_selection-p->x_min)/width;
		if (p->x_selection_relative < 0 || p->x_selection_relative > 1)
			p->x_selection_relative = 0.9;
	}

	if (!dragging_x_selection && !changing_x_selection && !changing_width && old_x_min != p->x_min)
	{
		p->ui_dragging_limits = true;
	}

	if (changing_x_selection || (p->ui_dragging_limits && p->auto_scroll))
		p->auto_scroll = p->x_selection >= limit_max;

	if (p->auto_scroll && !dragging_x_selection && !changing_x_selection)
		p->x_selection = limit_max;	

	if (dragging_x_selection)
	{
		p->x_selection_relative = (p->x_selection-p->x_min)/width;
		if (p->x_selection_relative < 0) p->x_selection_relative = 0;
		if (p->x_selection_relative > 1) p->x_selection_relative = 1;
	}

	if (p->ui_dragging_limits && !dragging_x_selection)
	{
		p->x_selection = p->x_min + p->x_selection_relative*width;
	}

	if ((changing_x_selection || changing_width || p->auto_scroll) && !dragging_x_selection)
	{
		p->x_min = p->x_selection - p->x_selection_relative*width;
		p->x_max = p->x_min+width;

		if (p->x_min < limit_min-limit_extra)
		{
			p->x_max += (limit_min-limit_extra) - p->x_min;
			p->x_min = (limit_min-limit_extra);
		}
		if (p->x_max > limit_max+limit_extra)
		{
			p->x_min -= p->x_max - (limit_max+limit_extra);
			p->x_max = (limit_max+limit_extra);
		}
	}

	if (!changing_x_selection && !changing_width && !p->ui_dragging_limits && !dragging_x_selection)
	{
		// snap selection to nearest data point
		s64 index = plot_get_visual_selection_index(p);
		s64 selection_time = plot_get_history_time(p, index);
		p->x_selection = (double)selection_time * p->time_to_seconds_factor;
	}

	//ImGui::TextDisabled(p->ui_dragging_limits ? "dragging_limits" : "");
	//ImGui::TextDisabled(changing_x_selection ? "changing_x_selection" : "");
	//ImGui::TextDisabled(dragging_x_selection ? "dragging_x_selection" : "");

	if (!ImGui::IsMouseDragging(0, 0.0f))
	{
		p->ui_dragging_limits = false;
	}
}

int plot_get_num_active_graphs(Plot* p)
{
	return p->graph_pos;
}
