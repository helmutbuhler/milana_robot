// control_ui is an application that connects to the robot via WIFI and gives you a lot of features:
// - Control the robot via keyboard (arrow keys to move, PLOK to move joints)
// - Monitor about 300 variables received by the robot. Displayed as plots.
// - Control lots of variables on the robot (tune pid controllers etc.)
// - Display usb webcam data and sync it to the data received from the robot.
// - Display sensor state in 2D and 3D models of the robot.
//
// When not connected to the robot, this can also be used to run simulations of the robot
// (mainly of the movement of the wheels) and to debug IK stuff.
// It can also be used to load capture data that was saved to a file earlier.
//
// Note that the robot can also work when control_ui is not connected to it.

#ifndef COMPILING_CONTROL_UI
#error COMPILING_CONTROL_UI must be defined for the control_ui project
#endif

#define _CRT_SECURE_NO_WARNINGS
#include "control_ui.h"
#include "control_ui_client.h"
#include "simulation_ui.h"
#include "webcam.h"
#include "video_recorder.h"
#include "pendulum_simulation.h"
#include "../common/leaning.h"
#include "../common/geometry.h"
#include "../robot/command.h"

#include <math.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <cstdio>
#include <cstdlib>

#ifdef _MSC_VER
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <GL/glu.h>
#else
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#endif
#include "GLFW/glfw3.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include "../3rdparty/implot/implot.h"
#include "plot.h"

#ifdef _MSC_VER
#define INCLUDE_ICON_FONTS _MSC_VER >= 1920
#else
#define INCLUDE_ICON_FONTS 1
#endif
#if INCLUDE_ICON_FONTS
#include "../3rdparty/fonts/IconsFontAwesome6.h"
#include "../3rdparty/fonts/DroidSans.h"
#include "../3rdparty/fonts/fa-solid-900.h"
#endif

using namespace std;

#ifdef _MSC_VER
// link to libraries
#pragma comment(lib, "../3rdparty/glfw/lib-vc2015/glfw3dll.lib")
#pragma comment (lib, "opengl32.lib") // link with Microsoft OpenGL lib
#pragma comment (lib, "Glu32.lib")
#else
#define MessageBeep(x)
#pragma GCC diagnostic ignored "-Wunused-result"
#endif


static void gl_draw_bitmap(u8* data, int data_cx, int data_cy, int pos_x, int pos_y, int d_cx, int d_cy);
void draw_robot_3d_views(const RobotJointState& s, const RobotIKTarget* target,
		float ui_x_pos, float ui_y_pos);

float x1_=-100, x2_=100, y1_=-100, y2_=100, z1_=-100, z2_=100;
bool show_big_3d_view = false;
float zoom_3d_view = 1.5f;

// window
GLFWwindow* window;
int window_size_x, window_size_y;
float dpi_scaling;
bool pressing_key[GLFW_KEY_LAST] = {0};
bool show_simulation = false;

ControlData cd;

static std::vector<MonitorDataEx> history;

Plot* main_plot = plot_create();
Plot* error_history_plot = plot_create();

Plot_Y_Axis unit_current{"A", -50, 50};
Plot_Y_Axis unit_vel    {"rev/s", 0, 10};


VideoRecorder* webcam_video = video_recorder_create(100);

// time difference between odrive time and webcam time
s64 webcam_time_delta = 0;

std::vector<MonitorDataEx>& get_controller_history()
{
	return history;
}

MonitorData& get_last_monitor_data()
{
	assert(history.size());
	return history.back();
}

void push_history(const MonitorData& md_)
{
	MonitorDataEx md;
	(MonitorData&)md = md_;
	const s64 odrive_delta_time = 1;
	if (history.size() == 0)
		md.display_time = 0;
	else
	{
		s64 delta = md.odrive_counter-history.back().odrive_counter;
		if (delta <= 0)
		{
			// If odrive is disabled, or we add this during simulation, the counter doesn't increase
			delta = cd.target_delta_time_ms * odrive_frequency / 1000;
		}
		if (delta > odrive_frequency)
		{
			delta = odrive_frequency;
		}
		md.display_time = history.back().display_time + delta;
	}
	history.push_back(md);
}

template<typename T>
void cd_change(T& old_value, T new_value)
{
	if (old_value != new_value)
	{
		old_value = new_value;
		cd.counter++;
	}
}

void update()
{
	MonitorData& md = get_last_monitor_data();
	if (!cd.leg_control_enable_motor[0])
		cd.leg_abs_target[0] = -md.leg_rel_angle[0] - md.leg_base_angle[0];
	if (!cd.leg_control_enable_motor[1])
		cd.leg_abs_target[1] = md.leg_rel_angle[1] + md.leg_base_angle[1];

	float rotation_vel = 0;
	if (pressing_key[GLFW_KEY_LEFT]) rotation_vel += cd.balance_control_rotation_vel_user_max;
	if (pressing_key[GLFW_KEY_RIGHT]) rotation_vel -= cd.balance_control_rotation_vel_user_max;
	cd_change(cd.balance_control_rotation_vel_user_target, rotation_vel);

	float vel_target = 0;
	if (pressing_key[GLFW_KEY_UP]) vel_target += cd.balance_control_vel_user_max;
	if (pressing_key[GLFW_KEY_DOWN]) vel_target += -cd.balance_control_vel_user_max;
	cd_change(cd.balance_control_vel_target, vel_target);
	
	bool force_still = false;
	if (pressing_key[GLFW_KEY_KP_0] || pressing_key[GLFW_KEY_0]) force_still = true;
	cd_change(cd.balance_control_force_still, force_still);
	
	float common_leg_vel_target = 0;
	if (pressing_key['P']) common_leg_vel_target += -cd.common_leg_vel_user_max;
	if (pressing_key['L']) common_leg_vel_target +=  cd.common_leg_vel_user_max;
	cd_change(cd.common_leg_vel_target, common_leg_vel_target);

	float hip_control = 0;
	if (pressing_key['O']) hip_control -= cd.common_servo_vel_user_max;
	if (pressing_key['K']) hip_control += cd.common_servo_vel_user_max;
	if (cd.servo_enable)
		cd_change(cd.common_servo_target_vel_user, hip_control);
}


int oscilloscope_history_start = -1;
void on_new_monitor_data(MonitorData& md)
{
	if (history.size() == 1 && history[0].counter == 0)
	{
		// clear first dummy element
		history.clear();
	}
	if (md.oscilloscope_state == 0)
		oscilloscope_history_start = -1;
	if (md.oscilloscope_state == 1 || md.oscilloscope_state == 2)
	{
		if (md.oscilloscope_start == md.odrive_counter)
		{
			oscilloscope_history_start = (int)history.size();
		}
		else
		{
			MonitorData last_md = history.back();
			int count = md.odrive_counter-last_md.odrive_counter-1;
			for (int i = 0; i < count; i++)
			{
				last_md.odrive_counter++;
				push_history(last_md);
			}
		}
	}
	if (md.oscilloscope_state == 3)
	{
		assert(oscilloscope_history_start != -1);
		for (int i = md.oscilloscope_start; i < md.oscilloscope_end; i++)
		{
			// #osci
			float value = md.oscilloscope_transmitting[i-md.oscilloscope_start];
			const int oscilloscope_values_per_step = 10;
			int h = oscilloscope_history_start+i/oscilloscope_values_per_step;
			switch (i%oscilloscope_values_per_step)
			{
			//case 0: history[h].side_balance_error           = value; break;
			//case 1: history[h].side_balance_error_d         = value; break;
			case 0: history[h].leg_rel_angle[0]             = value; break;
			case 1: history[h].leg_rel_angle[1]             = value; break;
			case 2: history[h].leg_current_target[0]        = value; break;
			case 3: history[h].leg_current_target[1]        = value; break;
			//case 4: history[h].leg_current[0]               = value; break;
			//case 5: history[h].leg_current[1]               = value; break;
			//case 4: history[h].leg_rel_angle_target_odrv[0] = value; break;
			//case 5: history[h].leg_rel_angle_target_odrv[1] = value; break;
			case 4: history[h].leg_vel[0]                   = value; break;
			case 5: history[h].leg_vel[1]                   = value; break;
			//case 8: history[h].leg_integrator[0]            = value; break;
			//case 9: history[h].leg_integrator[1]            = value; break;
			//case 6: history[h].side_angle_motor             = value; break;
			//case 7: history[h].side_angle_motor_d           = value; break;
			//case 6: history[h].side_angle_motor_target      = value; break;
			//case 7: history[h].side_angle_motor_target_d    = value; break;
			//case 8: history[h].side_angle_motor_force_odrv  = value; break;
			//case 9: history[h].average_leg_pos_force_odrv   = value; break;
			case 6: history[h].leg_vel_target_odrv[0]       = value; break;
			case 7: history[h].leg_vel_target_odrv[1]       = value; break;
			case 8: history[h].l_frame_counter[0]           = value; break;
			case 9: history[h].l_frame_counter[1]           = value; break;
			}
		}
	}
	push_history(md);
}

void save_history(const char* filename)
{
	char buffer[128];
	sprintf_s(buffer, "../logs/%s", filename);
	FILE* log = fopen(buffer, "wb");
	if (!log)
	{
		MessageBeep(0);
		return;
	}
	int version = 2;
	fwrite(&version, 4, 1, log);
	int len = sizeof(MonitorData);
	fwrite(&len, 4, 1, log);
	for (int i = 0; i < history.size(); i++)
	{
		fwrite(&history[i], sizeof(MonitorData), 1, log);
	}
	fclose(log);

	sprintf_s(buffer, "../logs/%s", filename);
	if (strlen(filename) > 4) buffer[strlen(buffer)-4] = 0;
	strcat_s(buffer, ".cam");
	if (video_recorder_get_num_frames(webcam_video))
	{
		log = fopen(buffer, "wb");
		if (!log)
		{
			printf("Cannot open %s for writing!\n", buffer);
			MessageBeep(0);
			return;
		}
		fwrite(&webcam_time_delta, sizeof(webcam_time_delta), 1, log);
		if (!video_recorder_save_to_stream(webcam_video, log))
		{
			printf("error writing video\n");
			MessageBeep(0);
		}
		fclose(log);
	}
	else
		std::remove(buffer);
}

s64 history_to_webcam_time(s64 index)
{
	return s64(double(history[index].odrive_counter) / odrive_frequency / webcam_time_to_seconds) - webcam_time_delta;
}

void clear_webcam_frames()
{
	video_recorder_reset(webcam_video);
}

void clear_webcam_non_jump_frames()
{
	vector<bool> webcam_keep;
	webcam_keep.resize(video_recorder_get_num_frames(webcam_video), false);
	{
		int jump_timer = 0;
		for (int i = 0; i < history.size(); i++)
		{
			MonitorData& md = history[i];
			if (md.jump_phase != JP_None)
				jump_timer = md.odrive_counter;
			if (md.odrive_counter-jump_timer < odrive_frequency*2)
			{
				s64 webcam_time = history_to_webcam_time(i);
				s64 index = video_recorder_get_frame_index_from_time(webcam_video, webcam_time);
				webcam_keep[index] = true;
			}
		}
	}
	{
		int jump_timer = history.back().odrive_counter;
		for (int i = (int)history.size()-1; i >= 0; i--)
		{
			MonitorData& md = history[i];
			if (md.jump_phase != JP_None)
				jump_timer = md.odrive_counter;
			if (jump_timer-md.odrive_counter < odrive_frequency*2)
			{
				s64 webcam_time = history_to_webcam_time(i);
				s64 index = video_recorder_get_frame_index_from_time(webcam_video, webcam_time);
				webcam_keep[index] = true;
			}
		}
	}

	video_recorder_clear_frames(webcam_video, webcam_keep);
}

void clear_history()
{
	history.clear();
	plot_reset_display_range(main_plot);

	MonitorData md;
	md.counter = 0;
	on_new_monitor_data(md);

	clear_webcam_frames();
	webcam_time_delta = 0;
}

void load_history(const char* filename, bool ignore_non_jumps = false)
{
	const bool load_odrive_extra_frames = true;
	clear_history();

	char buffer[128];
	sprintf_s(buffer, "../logs/%s", filename);
	FILE* log = fopen(buffer, "rb");
	if (!log)
	{
		printf("Cannot open %s for reading!\n", buffer);
		MessageBeep(0);
		return;
	}
	int version = 0;
	fread(&version, 4, 1, log);
	if (version != 1 && version != 2)
	{
		printf("error reading history: unknown version %i\n", version);
		fclose(log);
		MessageBeep(0);
		return;
	}
	int len = 0;
	fread(&len, 4, 1, log);
	if (len <= 0)
	{
		printf("error reading history\n");
		fclose(log);
		MessageBeep(0);
		return;
	}
	int extra = 0;
	if (len > sizeof(MonitorData))
		extra = len - sizeof(MonitorData);
	int jump_timer = 0;
	int last_counter = -1;
	MonitorData md;
	while (!feof(log))
	{
		size_t r = fread(&md, min(len, (int)sizeof(MonitorData)), 1, log);
		if (r == 0)
			break;
		assert(r == 1);
		if (version == 1 && len <= 1144)
		{
			// This is set to garbage data in older logs
			md.monitor_data_version = 0;
		}
		assert(md.monitor_data_version >= 0);
		if (md.jump_phase != JP_None)
			jump_timer = md.odrive_counter;
		//printf("frame %d %d %d %f\n", (int)(md.jump_phase != JP_None), md.odrive_counter, jump_timer, md.display_time / 8000.0f);
		if (!ignore_non_jumps || md.odrive_counter-jump_timer < odrive_frequency*2)
		{
			if (load_odrive_extra_frames || last_counter+1 == md.counter)
				push_history(md);
		}
		last_counter = md.counter;
		if (extra)
			fseek(log, extra, SEEK_CUR);
	}
	fclose(log);

	if (history.size() == 0)
	{
		printf("history empty!\n");
		fclose(log);
		MessageBeep(0);
		return;
	}
	plot_reset_display_range(main_plot);

	sprintf_s(buffer, "../logs/%s", filename);
	if (strlen(filename) > 4) buffer[strlen(buffer)-4] = 0;
	strcat_s(buffer, ".cam");

	log = fopen(buffer, "rb");
	if (!log)
		return;
	fread(&webcam_time_delta, sizeof(webcam_time_delta), 1, log);
	if (!video_recorder_load_from_stream(webcam_video, log))
		MessageBeep(0);
	fclose(log);
}

void do_sensor_fusion_gyro(MonitorData& md, bool constant_dt)
{
	// TODO code duplication
    //float gravity_length = sqrt(sq(md.imu_a_x) + 
    //                            sq(md.imu_a_y) + 
    //                            sq(md.imu_a_z));

	float delta_time = md.delta_time;
	if (constant_dt)
		delta_time = cd.target_delta_time_ms * 0.001f;

	switch (cd.imu_fifo_frequency)
	{
	case 0: delta_time *= 50.0f / 10; break;
	case 1: delta_time *= 50.0f / 25; break;
	case 2: delta_time *= 50.0f / 50; break;
	case 3: delta_time *= 50.0f / 100; break;
	case 4: delta_time *= 50.0f / 200; break;
	case 5: delta_time *= 50.0f / 400; break;
	case 6: delta_time *= 50.0f / 800; break;
	case 7: delta_time *= 50.0f / 1600; break;
	case 8: delta_time *= 50.0f / 3300; break;
	case 9: delta_time *= 50.0f / 6600; break;
	case 10:delta_time *= 50.0f / 13300; break;
	}

	float delta_x = ((md.imu_gyro_x + md.imu_gyro_bias_x) * cd.imu_gyro_scale) * delta_time;
    float delta_y = ((md.imu_gyro_y + md.imu_gyro_bias_y) * cd.imu_gyro_scale) * delta_time;
    float delta_z = ((md.imu_gyro_z + md.imu_gyro_bias_z) * cd.imu_gyro_scale) * delta_time;

	Quaternion gyro_rotation;
	gyro_rotation.w = 1 - (sq(delta_x) + sq(delta_y) + sq(delta_z)) * 0.125f;
	gyro_rotation.x = -delta_x * 0.5f;
	gyro_rotation.y = -delta_y * 0.5f;
	gyro_rotation.z = -delta_z * 0.5f;

	// The above is a linear approximation of this
	// and is not only faster, but also seems to be working better.
	//gyro_rotation.w = cos((delta_x + delta_y + delta_z) * 0.5);
	//gyro_rotation.x = sin(-delta_x * 0.5);
	//gyro_rotation.y = sin(-delta_y * 0.5);
	//gyro_rotation.z = sin(-delta_z * 0.5);

	quaternion_rotate_vector(
            md.imu_gravity_x,
            md.imu_gravity_y,
            md.imu_gravity_z,
            gyro_rotation);
}

/*
template<typename T>
void plot_history_2D(const char* name, T var, bool* show)
{
	char buffer[128];
	sprintf_s(buffer, "%s ###%s", name, name);

	bool do_colored = *show;
	if (do_colored) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(plot_colors[plot_color_index][0], plot_colors[plot_color_index][1], plot_colors[plot_color_index][2], 1));
	ImGui::Checkbox(buffer, show);
	if (do_colored) ImGui::PopStyleColor();
	if (*show)
	{
		float x0 = 360*dpi_scaling;
		float x1 = x0 + 800*dpi_scaling;
		float y0 = 200*dpi_scaling;
		float y1 = y0 + 300*dpi_scaling;
		
		glBegin(GL_LINE_STRIP);
		glColor3f(0, 0, 0);
		glVertex2f(x1, y1);
		glVertex2f(x0, y1);
		glVertex2f(x0, y0);
		//glVertex2f(x1, y0);
		glEnd();

		glBegin(GL_POINTS);
		glPointSize(1);
		glColor3f(plot_colors[plot_color_index][0], plot_colors[plot_color_index][1], plot_colors[plot_color_index][2]);
		plot_color_index = (plot_color_index+1) % num_plot_colors;

		s64 start, end;
		s64 time_start, time_size;
		plot_get_display_range(main_plot, &start, &end, &time_start, &time_size);
		for (s64 i = start; i < end; ++i)
		{
			for (int j = -500; j <= 500; j++)
			{
				float value = j * 0.01f * 10;
				if (!var(history[i], value)) continue;

				float x = x0+(x1-x0)*float(history[i].display_time-time_start)/time_size;
				float y = y1-(y1-y0)*value*main_plot.visual_scale_y;

				glVertex2f(x, y);
			}
		}
		glEnd();
	}
}*/

void plot_leg_x_angle_correlation()
{
	static bool show = false;
	ImGui::Checkbox("leg-x-angle correlation", &show);
	if (show)
	{
		float x0 = 550*dpi_scaling;
		float x1 = x0 + 300*dpi_scaling;
		float y0 = 300*dpi_scaling;
		float y1 = y0+200*dpi_scaling;
		
		glBegin(GL_LINE_LOOP);
		glColor3f(0, 0.5, 0);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();

		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);

		glColor3f(0, 0, 1);
		//glPointSize(2);
		glBegin(GL_LINE_STRIP);
		for (int k = 0; k < 2; k++)
		{
			for (s64 i = start; i < end; ++i)
			{
				if (!history[i].balance_control_is_quiet) continue;
				float x = history[i].common_leg_pos_sensor;
				float y = history[i].imu_x_angle - history[i].error_p; // this is target_angle

				y -= calculate_squat_angle(history[i].common_hip_pos_sensor, history[i].common_leg_pos_sensor);

				//y *= -main_plot.visual_scale_y;

				glVertex2f(x0+(x1-x0)*x, y1);
				glVertex2f(x0+(x1-x0)*x, y1-(y1-y0)*y);
			}
			glEnd();
			glColor3f(1, 0, 0);
			glPointSize(2);
			glBegin(GL_POINTS);
		}
		glEnd();
		glColor3f(0, 0, 0);
	}
}

void plot_hip_x_angle_correlation()
{
	static bool show = false;
	ImGui::Checkbox("hip-x-angle correlation", &show);
	if (show)
	{
		float x0 = 850*dpi_scaling;
		float x1 = x0 + 300*dpi_scaling;
		float y0 = 300*dpi_scaling;
		float y1 = y0+200*dpi_scaling;
		
		glBegin(GL_LINE_LOOP);
		glColor3f(0, 0.5, 0);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();

		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);

		glColor3f(0, 0, 1);
		glPointSize(2);
		//glBegin(GL_LINE_STRIP);
		glBegin(GL_POINTS);
		for (s64 i = start; i < end; ++i)
		{
			if (!history[i].balance_control_is_quiet) continue;
			float x = history[i].common_hip_pos_sensor;

			float y = history[i].imu_x_angle - history[i].error_p; // this is target_angle

			y -= calculate_squat_angle(history[i].common_hip_pos_sensor, history[i].common_leg_pos_sensor);
			//y *= main_plot.visual_scale_y;

			glVertex2f(x0+(x1-x0)*(x), y1-(y1-y0)*y);
		}
		glEnd();
	}
}

void draw_wheel(float r, int num_segments)
{
    glBegin(GL_LINE_STRIP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = tau * float(ii) / float(num_segments);//get the current angle

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x, y);//output vertex

    }
    glVertex2f(r, 0);
    glVertex2f(0, 0);
    glEnd();
}

void draw_robot_leg(float x_angle, float hip_angle, float leg_angle, float wheel_angle, float* leg_y)
{
	float joint_x, joint_y;
	{
		// upper leg
		float x0 = -32.28f*0.5f, x1 = 32.28f*0.5f, y0 = 0, y1 = upper_leg_length;

		glPushMatrix();
		glRotatef(to_degree(x_angle + hip_angle), 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();
		glPopMatrix();

		joint_x = -sin(x_angle + hip_angle)*y1;
		joint_y = cos(x_angle + hip_angle)*y1;
	}
		
	{
		// lower leg
		float x0 = -17.5f, x1 = 17.5f, y0 = 0, y1 = lower_leg_length;

		glPushMatrix();
		glTranslatef(joint_x, joint_y, 0);
		glRotatef(to_degree(x_angle + hip_angle + leg_angle), 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();
		glPopMatrix();

		joint_x += -sin(x_angle + hip_angle + leg_angle)*y1;
		joint_y +=  cos(x_angle + hip_angle + leg_angle)*y1;
	}

	{
		// wheel
		glPushMatrix();
		glTranslatef(joint_x, joint_y, 0);
		glRotatef(to_degree(x_angle + hip_angle + leg_angle + wheel_angle), 0, 0, 1);
		
		draw_wheel(wheel_radius, 8);
		glPopMatrix();
	}

	*leg_y = joint_y;
}

void draw_robot(float ui_x_pos, float ui_y_pos)
{
	MonitorDataEx& md = history[plot_get_visual_selection_index(main_plot)];
	float leg_left_y = 0, leg_right_y = 0;
	float leg_left_y2 = 0, leg_right_y2 = 0;
	
	// draw robot from the side
	{
		glPushMatrix();
		glTranslatef(ui_x_pos, ui_y_pos, 0);
		glScalef(-0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);

		{
			// robot body dimensions in mm
			float x0 = -31, x1 = 31, y0 = 12.5, y1 = -169;
			glPushMatrix();
			glRotatef(to_degree(md.imu_x_angle), 0, 0, 1);
		
			glBegin(GL_LINE_LOOP);
			glColor3f(0, 0, 0);
			glVertex2f(x0, y1);
			glVertex2f(x1, y1);
			glVertex2f(x1, y0);
			glVertex2f(x0, y0);
			glEnd();
			glPopMatrix();
		}

		// right
		{
			float hip_angle = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
			//float hip_angle = lerp(hip_angle_0, hip_angle_1, md.common_hip_pos_target);

			float leg_pos = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
			float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
			//float leg_angle = cd.fall_vertical_angle - md.x_angle - hip_angle;

			float wheel_angle = -md.foot_pos[1] * tau;

			glColor3f(0.7f, 0.7f, 0.7f);
			draw_robot_leg(md.imu_x_angle, hip_angle, leg_angle, wheel_angle, &leg_right_y);
			leg_right_y2 = leg_height_from_angles(md.imu_x_angle, hip_angle, leg_angle, md.imu_side_angle, -1);
		}

		// left
		{
			float hip_angle = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
			//float hip_angle = lerp(hip_angle_0, hip_angle_1, md.common_hip_pos_target);
			//if (rand()%2)
			/*{
				float best_error = -1;
				float steps[] = {0, 0.2f, -0.2f, 0.1f, -0.1f, 0.05f, -0.05f, 0.02f, -0.02f, 0.01f, -0.01f};
				for (int i = 0; i < 11; i++)
				{
					leg_left_y2 = leg_height_from_angles_falling(md.x_angle, hip_angle+steps[i], md.side_angle, 1);
					float error = sq(leg_left_y2-leg_right_y2);
					if (best_error < 0 || error < best_error)
					{
						best_error = error;
						hip_angle += steps[i];
					}
				}
			}*/
			float leg_pos = (md.leg_rel_angle[1]+md.leg_base_angle[1]);
			float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
			//float leg_angle = cd.fall_vertical_angle - md.x_angle - hip_angle;
			float wheel_angle = md.foot_pos[0] * tau;

			glColor3f(0, 0, 0);
			draw_robot_leg(md.imu_x_angle, hip_angle, leg_angle, wheel_angle, &leg_left_y);
			leg_left_y2 = leg_height_from_angles(md.imu_x_angle, hip_angle, leg_angle, md.imu_side_angle, 1);
		}

		glPopMatrix();
	}

	{
		// front view side_angle_motor
		float x0 = -165.0f/2, x1 = 165.0f/2, y0 = 12.5, y1 = -169;
		glPushMatrix();
		glTranslatef(ui_x_pos+150*dpi_scaling, ui_y_pos, 0);
		glScalef(0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);

		glBegin(GL_LINE_STRIP);
		glColor3f(0, 0, 0);
		glVertex2f(x1-40, leg_left_y2);
		glVertex2f(x1+40, leg_left_y2);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glVertex2f(x0-40, leg_right_y2);
		glVertex2f(x0+40, leg_right_y2);
		glEnd();

		glRotatef(md.side_angle_motor_calc*180/pi, 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glColor3f(0.8f, 0.8f, 0.8f);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();

		glPopMatrix();
	}
	{
		// front view
		float x0 = -165.0f/2, x1 = 165.0f/2, y0 = 12.5, y1 = -169;
		glPushMatrix();
		glTranslatef(ui_x_pos+150*dpi_scaling, ui_y_pos, 0);
		glScalef(0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);

		glBegin(GL_LINE_STRIP);
		glColor3f(0, 0, 0);
		glVertex2f(x1-40, leg_left_y2);
		glVertex2f(x1+40, leg_left_y2);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glVertex2f(x0-40, leg_right_y2);
		glVertex2f(x0+40, leg_right_y2);
		glEnd();

		glRotatef(md.imu_side_angle*180/pi, 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glColor3f(0, 0, 0);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();

		{
			float x0 = -7.5f, x1 = 7.5f, y0 = -wheel_radius, y1 = wheel_radius;

			// left wheel
			glPushMatrix();
			glTranslatef(cd.wheel_side_distance*0.5f, leg_left_y, 0);
		
			glBegin(GL_LINE_LOOP);
			glColor3f(0, 0, 0);
			glVertex2f(x0, y1);
			glVertex2f(x1, y1);
			glVertex2f(x1, y0);
			glVertex2f(x0, y0);
			glEnd();
			glPopMatrix();

			// right wheel
			glPushMatrix();
			glTranslatef(-cd.wheel_side_distance*0.5f, leg_right_y, 0);
		
			glBegin(GL_LINE_LOOP);
			glColor3f(0.7f, 0.7f, 0.7f);
			glVertex2f(x0, y1);
			glVertex2f(x1, y1);
			glVertex2f(x1, y0);
			glVertex2f(x0, y0);
			glEnd();
			glPopMatrix();
		}

		glPopMatrix();
	}

	// squat cog side view
	{
		glPushMatrix();
		glTranslatef(ui_x_pos, ui_y_pos, 0);
		glScalef(-0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);
		glRotatef(to_degree(md.imu_x_angle), 0, 0, 1);

		{
			float hip_angle = hip_angle_0+cd.hip_sensor_angle_range*md.common_hip_pos_sensor;
			float leg_pos = md.common_leg_pos_sensor;
			float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;

			float debug_render_points[14];
			calculate_squat_angle_from_angles(hip_angle, leg_angle, 0, debug_render_points);
			float px0   = debug_render_points[0];
			float pz0   = debug_render_points[1];
			float px1   = debug_render_points[2];
			float pz1   = debug_render_points[3];
			float px2   = debug_render_points[4];
			float pz2   = debug_render_points[5];
			float px3   = debug_render_points[6];
			float pz3   = debug_render_points[7];
			float px4   = debug_render_points[8];
			float pz4   = debug_render_points[9];
			float px5   = debug_render_points[10];
			float pz5   = debug_render_points[11];
			float com_x = debug_render_points[12];
			float com_z = debug_render_points[13];

			glBegin(GL_LINE_STRIP);
			glColor3f(1, 0, 0);
			glVertex2f(px0, pz0);
			glVertex2f(px1, pz1);
			glVertex2f(px2, pz2);
			glVertex2f(px3, pz3);
			glVertex2f(px4, pz4);
			glVertex2f(px5, pz5);
			glEnd();

			glPointSize(3);
			glBegin(GL_POINTS);
			glVertex2f(px0, pz0);
			glVertex2f(px2, pz2);
			glVertex2f(px4, pz4);
			glEnd();

			glColor3f(0, 0, 1);
			glPointSize(3);
			glBegin(GL_POINTS);
			glVertex2f(com_x, com_z);
			glEnd();
		}
		glPopMatrix();
	}
}

void draw_progress(float progress, const char* format, int format_int, const char* label, bool low_bad)
{
	char buffer[128];
	sprintf(buffer, format, format_int);
	if (low_bad)
	{
		ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor::HSV(min(progress, 0.3f), 0.6f, 0.6f));
		ImGui::PushStyleColor(ImGuiCol_PlotHistogram, (ImVec4)ImColor(255,255,255));
	}
	else
	{
		ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImColor(255,255,255));
		ImGui::PushStyleColor(ImGuiCol_PlotHistogram, (ImVec4)ImColor::HSV(min(1-progress, 0.3f), 0.6f, 0.6f));
	}
	ImGui::ProgressBar(progress, ImVec2(0.0f, 0.0f), buffer);
    ImGui::PopStyleColor(2);
	ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
	ImGui::Text("%s", label);
}
template<typename T>
void each_squat_var(T t)
{
	t(cd.squat_body_cog_x     );
	t(cd.squat_body_cog_y     );
	//t(cd.squat_body_mass );
	//t(cd.squat_upper_leg_cog_x);
	t(cd.squat_upper_leg_cog_y);
	//t(cd.squat_upper_leg_mass );
	//t(cd.squat_lower_leg_cog_x);
	t(cd.squat_lower_leg_cog_y);
	//t(cd.squat_lower_leg_mass );
	t(cd.squat_base_angle2);
	//t(cd.squat_hip_angle_bias);
	//t(cd.squat_leg_angle_bias);
	//t(cd.squat_bias_x);
	//t(cd.squat_bias_y);
	t(cd.squat_upper_leg_play_factor);
	t(cd.squat_lower_leg_play_factor);
}
float squat_variables_error()
{
	s64 start, end;
	plot_get_display_range(main_plot, &start, &end);
	float error = 0;
	int num = 0;
	for (s64 i = start; i < end; ++i)
	{
		if (!history[i].balance_control_is_quiet) continue;
		float y = history[i].imu_x_angle - history[i].error_p; // this is target_angle+t_correction_angle
		y -= calculate_squat_angle(history[i].common_hip_pos_sensor, history[i].common_leg_pos_sensor);
		error += sq(y);
		num++;
	}
	return num ? error * 100 / num : 0;
}

float l_initial_vel[2] = {0,0};

void do_offline_calculation()
{
	if (1)
	{
		//static int leg_delta = 0;
		//ImGui::DragInt("leg delta", &leg_delta, 1, 0, 0);
		// odrive variables
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		for (s64 i = start; i < end; i++)
		{
			MonitorDataEx& md = history[i];

			float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
			float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
			float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
			float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
			float hip_angle_l = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
			float hip_angle_r = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
			float leg_height_upper_l = cos(md.imu_x_angle + hip_angle_l) * upper_leg_length;
			float leg_height_upper_r = cos(md.imu_x_angle + hip_angle_r) * upper_leg_length;
			float leg_height_lower_l = cos(md.imu_x_angle + hip_angle_l + leg_angle_l) * lower_leg_length;
			float leg_height_lower_r = cos(md.imu_x_angle + hip_angle_r + leg_angle_r) * lower_leg_length;
			float leg_height_l = leg_height_upper_l + leg_height_lower_l;
			float leg_height_r = leg_height_upper_r + leg_height_lower_r;

			// Skip atan for performance reasons (argument should be small enough)
			//float side_angle_motor = atan((leg_height_r-leg_height_l)/cd.wheel_side_distance);
			float side_angle_motor = (leg_height_r-leg_height_l)/cd.wheel_side_distance;
			md.side_angle_motor_calc = side_angle_motor;

			float side_angle_motor_dl = -lower_leg_length*cd.wheel_side_distance*(tau / leg_gear_reduction)*sin(md.imu_x_angle + hip_angle_l + leg_angle_l) / (sq(leg_height_upper_l+leg_height_lower_l-leg_height_r)+sq(cd.wheel_side_distance));
			float side_angle_motor_dr = -lower_leg_length*cd.wheel_side_distance*(tau / leg_gear_reduction)*sin(md.imu_x_angle + hip_angle_r + leg_angle_r) / (sq(leg_height_upper_r+leg_height_lower_r-leg_height_l)+sq(cd.wheel_side_distance));
				
			float leg_vel_r = md.leg_vel[0];
			float leg_vel_l = md.leg_vel[1];

			float side_angle_motor_d = side_angle_motor_dr*leg_vel_r +
										side_angle_motor_dl*leg_vel_l;
			md.side_angle_motor_d_calc = side_angle_motor_d;

			float average_leg_pos = (leg_pos_l+leg_pos_r)*0.5f;
			md.average_leg_pos_calc = average_leg_pos;

			float average_leg_pos_dl = 0.5f;
			float average_leg_pos_dr = -0.5f;
			float average_leg_pos_d = average_leg_pos_dr*leg_vel_r + average_leg_pos_dl*leg_vel_l;
			md.average_leg_pos_d_calc = average_leg_pos_d;

			float side_angle_motor_force = (md.side_angle_target - md.side_balance_error) * cd.target_gain_p +
											(0-md.side_balance_error_d)                   * cd.target_gain_d +
											(0-md.side_angle_motor_d_calc)                * cd.side_gain_d;
				
			side_angle_motor_force = clamp(side_angle_motor_force,
					-cd.side_angle_motor_force_limit,
						cd.side_angle_motor_force_limit);

			md.side_angle_motor_force_calc = side_angle_motor_force;
			float average_leg_pos_force =
					md.average_leg_pos_integrator +
					(md.common_leg_pos_target-average_leg_pos  ) * cd.average_gain_p +
					(0                       -average_leg_pos_d) * cd.average_gain_d;
			md.average_leg_pos_force_calc = average_leg_pos_force;

			// multiplying by jacobian transpose to get the force in motor space
			float force_l = side_angle_motor_force * side_angle_motor_dl + average_leg_pos_force * average_leg_pos_dl;
			float force_r = side_angle_motor_force * side_angle_motor_dr + average_leg_pos_force * average_leg_pos_dr;

			if (force_r > 0)
			{
				float acc_current = cd.acc_current_gain * md.acc_r_calc;
				if (force_r+acc_current < 0 && leg_vel_r < 0)
					force_r = 0;
				else
					force_r += acc_current;

				if (-leg_vel_r < cd.side_balance_down_limit_threshold)
					force_r += (-leg_vel_r-cd.side_balance_down_limit_threshold) * cd.side_balance_down_limit_factor;
			}
			if (force_l < 0) {
				float acc_current = cd.acc_current_gain * md.acc_l_calc;
				if (force_l+acc_current > 0 && leg_vel_l > 0)
					force_l = 0;
				else
					force_l += acc_current;

				if (leg_vel_l < cd.side_balance_down_limit_threshold)
					force_l -= (leg_vel_l-cd.side_balance_down_limit_threshold) * cd.side_balance_down_limit_factor;
			}
			
			if (md.imu_x_angle+hip_angle_l+leg_angle_l > cd.side_balance_max_vertical_angle)
			    force_l += (md.imu_x_angle+hip_angle_l+leg_angle_l - cd.side_balance_max_vertical_angle) * cd.side_balance_max_vertical_angle_gain;
			if (md.imu_x_angle+hip_angle_r+leg_angle_r > cd.side_balance_max_vertical_angle)
			    force_r -= (md.imu_x_angle+hip_angle_r+leg_angle_r - cd.side_balance_max_vertical_angle) * cd.side_balance_max_vertical_angle_gain;

			md.force_l_calc = force_l;
			md.force_r_calc = force_r;

			float hip_pos_l = md.hip_sensor_pos[0];
			float hip_pos_r = md.hip_sensor_pos[1];

			float squat_length_l, squat_length_r;
			md.target_angle_l = calculate_squat_angle(hip_pos_l, leg_pos_l, &squat_length_l);
			md.target_angle_r = calculate_squat_angle(hip_pos_r, leg_pos_r, &squat_length_r);
			md.squat_length_sim = (squat_length_l + squat_length_r) * 0.5f;

#if 1
			if (md.l_frame_counter[0] == 0) l_initial_vel[0] = -md.leg_vel[0];
			if (md.l_frame_counter[1] == 0) l_initial_vel[1] =  md.leg_vel[1];
			// simulation of current calculation for landing mode
			float v_err_0 = -(md.leg_vel_target_odrv[0] - md.leg_vel[0]);
			float v_err_1 =   md.leg_vel_target_odrv[1] - md.leg_vel[1];
	        md.leg_current_landing_sim[0] = cd.l_vel_gain * v_err_0;
	        md.leg_current_landing_sim[1] = cd.l_vel_gain * v_err_1;

			/*float initial_vel_target0 = l_initial_vel[0] + md.l_frame_counter[0]*10*cd.l_initial_expected_accel_per_s / odrive_frequency;
		    float initial_leg_current0 = (initial_vel_target0-(-md.leg_vel[0])) * cd.l_initial_accel_gain;
		    md.leg_current_landing_sim[0] = std::min(md.leg_current_landing_sim[0], initial_leg_current0);

			float initial_vel_target1 = l_initial_vel[1] + md.l_frame_counter[1]*10*cd.l_initial_expected_accel_per_s / odrive_frequency;
		    float initial_leg_current1 = (initial_vel_target1-(md.leg_vel[1])) * cd.l_initial_accel_gain;
		    md.leg_current_landing_sim[1] = std::min(md.leg_current_landing_sim[1], initial_leg_current1);*/
		    md.leg_current_landing_sim[0] = -md.leg_current_landing_sim[0];
#else
			// simulation of current calculation
			float bandwidth = cd.input_filter_bandwidth;
			float input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
			float input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
			for (int j = 0; j < 2; j++)
			{
				md.leg_vel_target_odrv_sim[j] = history[max(i-1, 0)].leg_vel_target_odrv_sim[j];
				if (i == start) md.leg_vel_target_odrv_sim[j] = 0;
				md.leg_rel_angle_target_odrv_sim[j] = history[max(i-1, 0)].leg_rel_angle_target_odrv_sim[j];
				if (i == start) md.leg_rel_angle_target_odrv_sim[j] = md.leg_rel_angle_target_odrv[j];

				//float delta_pos = history[max(i+leg_delta, 0)].leg_rel_angle_target[j] - md.leg_rel_angle_target_odrv_sim[j]; // Pos error
				float delta_pos = md.leg_rel_angle_target[j] - md.leg_rel_angle_target_odrv_sim[j]; // Pos error
				float delta_vel = 0 - md.leg_vel_target_odrv_sim[j]; // Vel error
				float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
				//torque_setpoint_ = accel * config_.inertia; // Accel
				md.leg_vel_target_odrv_sim[j] += accel / odrive_frequency; // delta vel
				md.leg_rel_angle_target_odrv_sim[j] += md.leg_vel_target_odrv_sim[j] / odrive_frequency; // Delta pos
			}

			float pos_err_0 = md.leg_rel_angle_target_odrv_sim[0]-md.leg_rel_angle[0];
			float pos_err_1 = md.leg_rel_angle_target_odrv_sim[1]-md.leg_rel_angle[1];
			float vel_des_0 = md.leg_vel_target_odrv_sim[0] + pos_err_0*cd.pos_gain;
			float vel_des_1 = md.leg_vel_target_odrv_sim[1] + pos_err_1*cd.pos_gain;
			//vel_des_0 = clamp(vel_des_0, -cd.l_vel_limit, cd.l_vel_limit);
			//vel_des_1 = clamp(vel_des_1, -cd.l_vel_limit, cd.l_vel_limit);
			float v_err_0 = vel_des_0 - md.leg_vel[0];
			float v_err_1 = vel_des_1 - md.leg_vel[1];
	        md.leg_current_landing_sim[0] = cd.vel_gain * v_err_0 + md.leg_integrator[0];
	        md.leg_current_landing_sim[1] = cd.vel_gain * v_err_1 + md.leg_integrator[1];
#endif
		}
	}
		
	if (1)
	{
		// acc offline calculation
		float smooth = cd.acc_smooth_factor;
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		history[start].acc_r_calc = 0;
		history[start].acc_l_calc = 0;
		for (s64 i = max(start+1, (s64)1); i < end; i++)
		{
			MonitorDataEx& md = history[i];

			MonitorDataEx& pr = history[i-1];
			md.acc_r_calc = pr.acc_r_calc * smooth + (md.leg_vel[0]-pr.leg_vel[0]) * (1-smooth);
			md.acc_l_calc = pr.acc_l_calc * smooth + (md.leg_vel[1]-pr.leg_vel[1]) * (1-smooth);

			//md.foot_vel_coarse_sim[0] = (md.foot_shadow_count[0]-pr.foot_shadow_count[0]) / (foot_cpr*cd.max_delta_time_ms*0.001f);
			//md.foot_vel_coarse_sim[1] = (md.foot_shadow_count[1]-pr.foot_shadow_count[1]) / (foot_cpr*cd.max_delta_time_ms*0.001f);
		}
	}
	
	if (1)
	{
		// com position and derivatives offline calculation
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		for (s64 i = start+1; i < end; i++)
		{
			MonitorDataEx& md = history[i];
			
			{
				float hip_angle_l = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
				float hip_angle_r = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
				float foot_pos_l = -md.foot_pos[1];
				float foot_pos_r =  md.foot_pos[0];
				float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
				float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
				float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
				float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;

				float com_x = 0, com_z = 0;
				//float coi_x, coi_z;
				get_com_world_position(md.imu_x_angle, history[max((int)i-cd.coi_theta_past-1, 0)].error_p+history[max((int)i-cd.coi_theta_past-1, 0)].t_correction_angle,
						hip_angle_l, leg_angle_l, foot_pos_l,
						hip_angle_r, leg_angle_r, foot_pos_r,
						0, 0, //&coi_x, &coi_z,
						&com_x, &com_z,
						&md.coi_theta_acc_sim);

				float old_coi_world_pos = history[i-1].coi_world_pos_sim;
				//float old_com_world_pos_z = history[i-1].com_world_pos_z;
				md.coi_world_pos_sim = com_x;
				//md.com_world_pos_z = com_z;

				float delta_time = md.delta_time;
				float old_coi_world_vel = history[i-1].coi_world_vel_sim;
				//float old_com_world_vel_z = history[i-1].com_world_vel_z_sim;
				md.coi_world_vel_sim = update_smooth(
						old_coi_world_vel,
						(md.coi_world_pos_sim-old_coi_world_pos) / delta_time,
						cd.coi_world_vel_smooth_factor, cd.target_delta_time_ms);
				/*md.com_world_vel_z = update_smooth(
						old_com_world_vel_z,
						(md.com_world_pos_z-old_com_world_pos_z) / delta_time,
						cd.coi_world_vel_smooth_factor, cd.target_delta_time_ms);*/

				float old_coi_world_acc = history[i-1].coi_world_acc_sim;
				//float old_com_world_acc_z = history[i-1].com_world_acc_z;
				md.coi_world_acc_sim = update_smooth(
						old_coi_world_acc,
						(md.coi_world_vel_sim-old_coi_world_vel) / delta_time / 1000 / earth_acc,
						cd.coi_world_acc_smooth_factor, cd.target_delta_time_ms);
				/*md.com_world_acc_z = update_smooth(
						old_com_world_acc_z,
						(md.com_world_vel_z-old_com_world_vel_z) / delta_time / 1000 / earth_acc,
						cd.coi_world_acc_smooth_factor, cd.target_delta_time_ms);*/
			}
		}
	}


	if (1)
	{
		// imu sensor offline calculation
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		MonitorData md2;
		md2.imu_gravity_x = history[start].imu_gravity_x;
		md2.imu_gravity_y = history[start].imu_gravity_y;
		md2.imu_gravity_z = history[start].imu_gravity_z;
		for (s64 i = start+1; i < end; i++)
		{
			MonitorDataEx& md = history[i];
			
			/*{
				float hip_angle_l = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
				float hip_angle_r = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
				float foot_pos_l = -md.foot_pos[1];
				float foot_pos_r =  md.foot_pos[0];
				float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
				float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
				float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
				float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;

				float imu_x, imu_y;
				get_imu_sensor_position(md.imu_x_angle,
						hip_angle_l, leg_angle_l, foot_pos_l,
						hip_angle_r, leg_angle_r, foot_pos_r,
						imu_x, imu_y);

				float old_imu_sensor_to_floor_pos_x = history[i-1].imu_sensor_to_floor_pos_x;
				float old_imu_sensor_to_floor_pos_y = history[i-1].imu_sensor_to_floor_pos_y;
				md.imu_sensor_to_floor_pos_x = imu_x;
				md.imu_sensor_to_floor_pos_y = imu_y;

				float old_imu_sensor_to_floor_vel_x = history[i-1].imu_sensor_to_floor_vel_x;
				float old_imu_sensor_to_floor_vel_y = history[i-1].imu_sensor_to_floor_vel_y;
				md.imu_sensor_to_floor_vel_x = update_smooth(
						old_imu_sensor_to_floor_vel_x,
						(md.imu_sensor_to_floor_pos_x-old_imu_sensor_to_floor_pos_x) / md.delta_time,
						cd.imu_sensor_to_floor_vel_smooth_factor);
				md.imu_sensor_to_floor_vel_y = update_smooth(
						old_imu_sensor_to_floor_vel_y,
						(md.imu_sensor_to_floor_pos_y-old_imu_sensor_to_floor_pos_y) / md.delta_time,
						cd.imu_sensor_to_floor_vel_smooth_factor);

				float old_imu_sensor_to_floor_acc_x = history[i-1].imu_sensor_to_floor_acc_x;
				float old_imu_sensor_to_floor_acc_y = history[i-1].imu_sensor_to_floor_acc_y;
				md.imu_sensor_to_floor_acc_x = update_smooth(
						old_imu_sensor_to_floor_acc_x,
						(md.imu_sensor_to_floor_vel_x-old_imu_sensor_to_floor_vel_x) / md.delta_time / 1000 / earth_acc,
						cd.imu_sensor_to_floor_acc_smooth_factor);
				md.imu_sensor_to_floor_acc_y = update_smooth(
						old_imu_sensor_to_floor_acc_y,
						(md.imu_sensor_to_floor_vel_y-old_imu_sensor_to_floor_vel_y) / md.delta_time / 1000 / earth_acc,
						cd.imu_sensor_to_floor_acc_smooth_factor);

				md.imu_gravity_acc_x_comp = md.imu_gravity_acc_x;
				md.imu_gravity_acc_y_comp = md.imu_gravity_acc_y;
				md.imu_gravity_acc_z_comp = md.imu_gravity_acc_z;
				md.imu_gravity_acc_y_comp += sin(history[i-1].imu_x_angle) * md.imu_sensor_to_floor_acc_x;
				md.imu_gravity_acc_z_comp -= cos(history[i-1].imu_x_angle) * md.imu_sensor_to_floor_acc_x;
				md.imu_gravity_acc_y_comp -= cos(history[i-1].imu_x_angle) * md.imu_sensor_to_floor_acc_y;
				md.imu_gravity_acc_z_comp -= sin(history[i-1].imu_x_angle) * md.imu_sensor_to_floor_acc_y;
			}*/


			md2.imu_gyro_bias_x = md.imu_gyro_bias_x;
			md2.imu_gyro_bias_y = md.imu_gyro_bias_y;
			md2.imu_gyro_bias_z = md.imu_gyro_bias_z;
			
			for (int j = 0; j < md.imu_fifo_gyro_count; j++)
			{
				md2.imu_gyro_x = md.imu_fifo_gyro_x[j];
				md2.imu_gyro_y = md.imu_fifo_gyro_y[j];
				md2.imu_gyro_z = md.imu_fifo_gyro_z[j];
				do_sensor_fusion_gyro(md2, true);
			}

   			if (md.jump_phase == JP_Jumping || md.jump_phase == JP_LegUp || md.jump_phase == JP_Falling ||
				cd.imu_force_no_acc)
			{
				// During a jump, the accelerometer returns mostly garbage data because of the acceleration
				// and because you cannot measure gravity during free fall.
			}
			else
			{
				if (cd.imu_compensate_body_acc)
				{
					md2.imu_gravity_x = md2.imu_gravity_x * cd.imu_gyro_weight + md.imu_gravity_acc_x_comp;
					md2.imu_gravity_y = md2.imu_gravity_y * cd.imu_gyro_weight + md.imu_gravity_acc_y_comp;
					md2.imu_gravity_z = md2.imu_gravity_z * cd.imu_gyro_weight + md.imu_gravity_acc_z_comp;
				}
				else
				{
					md2.imu_gravity_x = md2.imu_gravity_x * cd.imu_gyro_weight + md.imu_gravity_acc_x;
					md2.imu_gravity_y = md2.imu_gravity_y * cd.imu_gyro_weight + md.imu_gravity_acc_y;
					md2.imu_gravity_z = md2.imu_gravity_z * cd.imu_gyro_weight + md.imu_gravity_acc_z;
				}
			}

			float gravity_length = sqrt(sq(md2.imu_gravity_x) + 
										sq(md2.imu_gravity_y) + 
										sq(md2.imu_gravity_z));
			float one_by_length = 1.0f / gravity_length;
			md2.imu_gravity_x *= one_by_length;
			md2.imu_gravity_y *= one_by_length;
			md2.imu_gravity_z *= one_by_length;
	
			md.imu_gravity_sim_x = md2.imu_gravity_x;
			md.imu_gravity_sim_y = md2.imu_gravity_y;
			md.imu_gravity_sim_z = md2.imu_gravity_z;
			md.imu_x_angle_sim = (float)-atan2(md2.imu_gravity_z, -md2.imu_gravity_y);
		}
	}
	/*
	if (1)
	{
		// imu sensor offline calculation
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		float old_x_angle = history[max(start-1, 0)].imu2_x_bias_acc;
		float old_y_angle = history[max(start-1, 0)].imu2_y_bias_acc;
		float old_z_angle = history[max(start-1, 0)].imu2_z_bias_acc;
		for (int i = max(start, 1); i < end; i++)
		{
			MonitorData& md = history[i];
			if (history[i-1].counter != md.counter)
			{
				float delta_x = (-md.imu2_g_x * cd.x_gyro_scale + history.back().imu2_x_bias) * md.delta_time;
				float delta_y = (-md.imu2_g_z * cd.y_gyro_scale + history.back().imu2_y_bias) * md.delta_time;
				float delta_z = ( md.imu2_g_y * cd.z_gyro_scale + history.back().imu2_z_bias) * md.delta_time;
				old_x_angle += delta_x;
				old_y_angle += delta_y;
				old_z_angle += delta_z;
			}

			md.imu2_x_bias_gyro = old_x_angle;
			md.imu2_y_bias_gyro = old_y_angle;
			md.imu2_z_bias_gyro = old_z_angle;
		}
	}*/
}

void draw_ui_monitoring(float monitor_height, float sidebar_width)
{
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, monitor_height));
	ImGui::Begin("Monitoring", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	/*ImGui::DragFloat("x1", &x1_, 1.0f, 0, 0, "%.2f");
	ImGui::DragFloat("x2", &x2_, 0.01f, 0, 0, "%.2f");
	ImGui::DragFloat("y1", &y1_, 1.0f, 0, 0, "%.2f");
	ImGui::DragFloat("y2", &y2_, 0.01f, 0, 0, "%.2f");
	ImGui::DragFloat("z1", &z1_, 0.01f, 0, 0, "%.2f");
	ImGui::DragFloat("z2", &z2_, 0.01f, 0, 0, "%.2f");*/
	
	MonitorData& md = get_last_monitor_data();
	
    ImGui::BeginDisabled();
	ImGui::Checkbox("ASR", &md.asr_client_connected);
	ImGui::SameLine();
	ImGui::Checkbox("TTS", &md.tts_client_connected);
    ImGui::EndDisabled();

	ImGui::SameLine();
	switch (client_get_connection_state())
	{
	case 0: ImGui::TextDisabled(" "); break;
	case 1: ImGui::TextDisabled("Connecting..."); break;
	case 2: ImGui::TextDisabled("Connected"); break;
	}

	if (md.oscilloscope_state)
	{
		ImGui::SameLine();
		ImGui::TextDisabled("OSC");
	}

	{
		draw_progress(md.battery_status_1, "%d%%", (int)(md.battery_status_1*100), "Battery1", true);
		draw_progress(md.battery_status_2, "%d%%", (int)(md.battery_status_2*100), "Battery2", true);
		draw_progress(md.odrive_temperature / 120, u8"%d°C", (int)max(md.odrive_temperature, md.odrv_foot_temperature), "ODrive Temp", false);
		draw_progress(md.jetson_temperature / 95, u8"%d°C", (int)md.jetson_temperature, "Jetson Temp", false);
		//draw_progress(md.imu_temperature / 85, u8"%d°C", (int)md.imu_temperature, "IMU Temp", false);
	}
	ImGui::End();
}

void draw_ui_sidebar(float monitor_height, float sidebar_width,
		RobotJointState& robot_state, RobotIKTarget& robot_target)
{
	s64 plot_start_history = plot_get_visual_selection_index(main_plot);

	ImGui::SetNextWindowPos(ImVec2(0, monitor_height));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, (float)window_size_y-monitor_height));
	ImGui::Begin("UI", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	//static float width = 0.35f;
	//ImGui::DragFloat("width", &width, 0.01f, 0, 0, "%.3f");
	ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.35f);
	
#define PLOT_HISTORY(name, var) \
	{ \
		static bool show = false; \
		plot_history(main_plot, name, [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show); \
	}
#define PLOT_HISTORY_U(name, var, unit) \
	{ \
		static bool show = false; \
		plot_history(main_plot, name, [&](s64 i){const MonitorDataEx& md = history[i]; return var;}, &show, unit); \
	}
	
	/*if (ImGui::Button("Crash"))
	{
		*(int*)0 = 0;
	}*/

	if (ImGui::CollapsingHeader("Startup"))
	{
		if (ImGui::Button("trigger startup sequence"))
		{
			cd.startup_sequence_trigger++;
			cd.servo_enable = true;
			cd.enable_stand_control = true;
			cd.balance_control_enable = true;
			cd.counter++;
		}

		if (ImGui::DragFloat("leg_vel", &cd.startup_leg_vel, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("leg_current_threshold", &cd.startup_leg_current_threshold, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip1_servo_vel", &cd.startup_hip1_servo_vel, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip1_servo_threshold", &cd.startup_hip1_servo_vel_threshold, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip1_servo_min", &cd.startup_hip1_servo_min, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip2_servo_vel", &cd.startup_hip2_servo_vel, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip3_servo_vel", &cd.startup_hip3_servo_vel, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("balance_tolerance", &cd.startup_balance_control_standing_tolerance, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("leg_final_pos", &cd.startup_leg_final_pos, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("hip_final_pos", &cd.startup_hip_final_pos, 0.01f)) cd.counter++;
		PLOT_HISTORY("startup_sequence", md.startup_sequence*0.1f);
		PLOT_HISTORY("startup_sequence_timer", md.startup_sequence_timer);
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Sensors"))
	{
		PLOT_HISTORY("battery_voltage 1:", md.battery_1_voltage);
		PLOT_HISTORY("battery_voltage 1+2:", md.battery_total_voltage);
		PLOT_HISTORY("battery_1_sensor_voltage", md.battery_1_sensor_voltage);
		PLOT_HISTORY("battery_sensor_counter", md.battery_sensor_counter*0.1f);
		PLOT_HISTORY("battery_status_1", md.battery_status_1);
		PLOT_HISTORY("battery_status_2", md.battery_status_2);
		PLOT_HISTORY("jetson_temperature", md.jetson_temperature);
		PLOT_HISTORY("odrive_temperature", md.odrive_temperature);
		//PLOT_HISTORY("odrv_foot_temperature", md.odrv_foot_temperature);
		PLOT_HISTORY("imu_temperature", md.imu_temperature);
		if (ImGui::DragFloat("bus_current_smooth", &cd.bus_current_smooth, 0.01f, 0, 1)) { cd.counter++; cd.odrive_init_counter++; }
		PLOT_HISTORY("leg_control_bus_current .1x", md.leg_control_bus_current*0.1f);
		PLOT_HISTORY("foot_control_bus_current .1x", md.foot_control_bus_current*0.1f);
		PLOT_HISTORY("both_bus_currents .1x", (md.leg_control_bus_current+md.foot_control_bus_current)*0.1f);
		PLOT_HISTORY("leg_control_endpoint_count .1x", md.leg_control_endpoint_request_count*0.1f);
		PLOT_HISTORY("foot_control_endpoint_count .1x", md.foot_control_endpoint_request_count*0.1f);
		PLOT_HISTORY("cpu_usage 0", md.cpu_usage[0]);
		PLOT_HISTORY("cpu_usage 1", md.cpu_usage[1]);
		PLOT_HISTORY("cpu_usage 2", md.cpu_usage[2]);
		PLOT_HISTORY("cpu_usage 3", md.cpu_usage[3]);
		PLOT_HISTORY("cpu_usage", (md.cpu_usage[0]+md.cpu_usage[1]+md.cpu_usage[2]+md.cpu_usage[3])/4);
		PLOT_HISTORY("ram available GB", md.ram_avail_kb / (1024.0f*1024));
		PLOT_HISTORY("swap available GB", md.swap_avail_kb / (1024.0f*1024));
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Timing"))
	{
		if (ImGui::SliderInt("target_delta_time_ms", &cd.target_delta_time_ms, 0, 100)) cd.counter++;
		if (ImGui::SliderInt("max_delta_time_ms", &cd.max_delta_time_ms, 0, 100)) cd.counter++;
		if (ImGui::SliderInt("pause_delta_time_ms", &cd.pause_delta_time_ms, 0, 500)) cd.counter++;
		if (ImGui::Button("trigger pause"))
		{
			cd.pause_trigger++;
			cd.counter++;
		}
		if (ImGui::Checkbox("do_random_pauses", &cd.do_random_pauses)) cd.counter++;
		if (ImGui::SliderInt("random_pauses_ms", &cd.random_pauses_max_ms, 1, 200)) cd.counter++;

		PLOT_HISTORY("delta_time", md.delta_time * 1000.f);
		PLOT_HISTORY("delta_time_total_ms", md.delta_time_micros * 0.001f);
		PLOT_HISTORY("delta_time_battery", md.delta_time_battery * 0.001f);
		PLOT_HISTORY("delta_time_leg_control", md.delta_time_leg_control * 0.001f);
		PLOT_HISTORY("delta_time_foot_control_1", md.delta_time_foot_control_1 * 0.001f);
		PLOT_HISTORY("delta_time_foot_control_2", md.delta_time_foot_control_2 * 0.001f);
		PLOT_HISTORY("delta_time_sleep", md.delta_time_sleep * 0.001f);
		PLOT_HISTORY("delta_time_network", md.delta_time_network * 0.001f);
		PLOT_HISTORY("delta_time_imu", md.delta_time_imu * 0.001f);
		PLOT_HISTORY("delta_time_joystick", md.delta_time_joystick * 0.001f);
		PLOT_HISTORY("delta_time_servo", md.delta_time_servo * 0.001f);
		PLOT_HISTORY("delta_time_tts", md.delta_time_tts * 0.001f);
		PLOT_HISTORY("delta_time_asr", md.delta_time_asr * 0.001f);
		PLOT_HISTORY("delta_time_ik", md.delta_time_ik * 0.001f);
		PLOT_HISTORY("frame counter", (float)md.counter);

		ImGui::TextDisabled("Uptime: %llus\n", history[plot_start_history].uptime_jetson_micros/1000000);
		//ImGui::TextDisabled("Uptime: %fs\n", history[plot_start_history].uptime_jetson_micros/1000000.0);
		ImGui::TextDisabled("Time: %s\n", ctime(&history[plot_start_history].local_time));
	}
	ImGui::NewLine();
		
	if (ImGui::CollapsingHeader("Leg Control"))
	{
		ImGui::PushID("leg_control");
		if (ImGui::Checkbox("enable odrive", &cd.enable_leg_control)) cd.counter++;
		if (ImGui::Button("reset calibration"))
		{
			cd.reset_leg_sensors_trigger++;
			cd.counter++;
		}

		if (ImGui::Checkbox("enable_stand_control", &cd.enable_stand_control)) cd.counter++;
		if (ImGui::DragFloat("vel", &cd.common_leg_vel_target, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("vel_user_max", &cd.common_leg_vel_user_max, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("min", &cd.common_leg_pos_min, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("max", &cd.common_leg_pos_max, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("min_side_balance", &cd.common_leg_pos_min_side_balance, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("wheel_side_distance", &cd.wheel_side_distance, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		PLOT_HISTORY("common_leg_pos_target ", md.common_leg_pos_target);
		PLOT_HISTORY("common_leg_pos_sensor", md.common_leg_pos_sensor);
		ImGui::NewLine();
				
		if (ImGui::Checkbox("enable_side_balance", &cd.enable_side_balance)) cd.counter++;
		if (ImGui::DragFloat("side_angle_target", &cd.side_angle_target, 0.001f)) { cd.counter++; }
		if (ImGui::DragFloat("side_gain_p",    &cd.side_gain_p   , 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("side_gain_d",    &cd.side_gain_d   , 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("average_gain_p", &cd.average_gain_p, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("average_gain_i", &cd.average_gain_i, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("average_gain_d", &cd.average_gain_d, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("target_gain_p", &cd.target_gain_p, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("target_gain_d", &cd.target_gain_d, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("side_motor_force_limit", &cd.side_angle_motor_force_limit, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::SliderInt("side_balance_mode", &cd.side_balance_mode, 0, 1)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("hip_correction_factor", &cd.side_angle_hip_correction_factor, 0.01f)) { cd.counter++; }
		if (ImGui::DragFloat("down_limit_factor",    &cd.side_balance_down_limit_factor, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("down_limit_threshold", &cd.side_balance_down_limit_threshold, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("max_vertical_angle", &cd.side_balance_max_vertical_angle, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("max_vertical_gain", &cd.side_balance_max_vertical_angle_gain, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::Checkbox("start_oscilloscope_on_ramp", &cd.start_oscilloscope_on_ramp)) cd.counter++;
		if (ImGui::DragFloat("acc_smooth_factor", &cd.acc_smooth_factor, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("acc_current_gain", &cd.acc_current_gain, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("error_d_smooth_factor", &cd.side_balance_error_d_smooth_factor, 0.001f)) { cd.counter++; cd.odrive_init_counter++; }

		PLOT_HISTORY("side_angle_hip_correction", md.side_angle_hip_correction);

		PLOT_HISTORY("side_angle_target", md.side_angle_target);
		PLOT_HISTORY("side_angle_motor_calc", md.side_angle_motor_calc);
		PLOT_HISTORY("side_angle_motor_d_calc", md.side_angle_motor_d_calc);

		PLOT_HISTORY("average_leg_pos_calc", md.average_leg_pos_calc);
		PLOT_HISTORY("average_leg_pos_d_calc", md.average_leg_pos_d_calc);
		PLOT_HISTORY("average_leg_pos_integrator", md.average_leg_pos_integrator);

		PLOT_HISTORY("average_leg_pos_force_calc", md.average_leg_pos_force_calc*0.1f);
		//PLOT_HISTORY("average_leg_pos_force_odrv", md.average_leg_pos_force_odrv*0.1f);
		PLOT_HISTORY("side_angle_motor_force_calc", md.side_angle_motor_force_calc*0.1f);
		//PLOT_HISTORY("side_angle_motor_force_odrv", md.side_angle_motor_force_odrv*0.1f);

		PLOT_HISTORY("side_balance_error", md.side_balance_error);
		PLOT_HISTORY("side_balance_error_d", md.side_balance_error_d);
		ImGui::NewLine();

		if (ImGui::DragFloat("recover_threshold_on", &cd.side_balance_recover_mode_side_angle_threshold_on, 0.01f)) { cd.counter++; }
		if (ImGui::DragFloat("recover_threshold_off", &cd.side_balance_recover_mode_side_angle_threshold_off, 0.01f)) { cd.counter++; }
		if (ImGui::DragFloat("recover_time_on", &cd.side_balance_recover_mode_side_angle_time_on, 0.01f)) { cd.counter++; }
		if (ImGui::DragFloat("recover_time_off", &cd.side_balance_recover_mode_side_angle_time_off, 0.01f)) { cd.counter++; }
		if (ImGui::DragFloat("recover_delta", &cd.side_balance_recover_mode_side_angle_delta, 0.01f)) { cd.counter++; }
		PLOT_HISTORY("recover_mode", (float)md.side_balance_recover_mode);
		PLOT_HISTORY("recover_mode_timer", md.side_balance_recover_mode_timer);
		ImGui::NewLine();

		PLOT_HISTORY("jump_phase", (float)md.jump_phase*0.2f);
		ImGui::NewLine();

		ImGui::TextDisabled("right data (0)");
		PLOT_HISTORY("pos-", -(md.leg_rel_angle[0]+md.leg_base_angle[0]));
		PLOT_HISTORY("pos_target-", -(md.leg_rel_angle_target[0]+md.leg_base_angle[0]));
		//PLOT_HISTORY("pos_target_ik", md.leg_pos_target_ik[0]);
		PLOT_HISTORY("pos_target_odrv-", -(md.leg_rel_angle_target_odrv[0]+md.leg_base_angle[0]));
		PLOT_HISTORY("pos_target_odrv_sim-", -(md.leg_rel_angle_target_odrv_sim[0]+md.leg_base_angle[0]));
		PLOT_HISTORY("pos_error-",  -(md.leg_rel_angle[0]-md.leg_rel_angle_target[0]));
		//PLOT_HISTORY("base", md.leg_base_angle[0]);
		PLOT_HISTORY_U("vel-", -md.leg_vel[0], &unit_vel);
		PLOT_HISTORY_U("vel_target-", -md.leg_vel_target_odrv[0], &unit_vel);
		PLOT_HISTORY_U("vel_target_sim-", -md.leg_vel_target_odrv_sim[0], &unit_vel);
		PLOT_HISTORY_U("l_initial_vel_target ", l_initial_vel[0]+md.l_frame_counter[0]*10*cd.l_initial_expected_accel_per_s/odrive_frequency, &unit_vel);
		PLOT_HISTORY_U("current-", -md.leg_current[0], &unit_current);
		PLOT_HISTORY_U("current_target-", -md.leg_current_target[0], &unit_current);
		PLOT_HISTORY_U("current_landing_sim-", -md.leg_current_landing_sim[0], &unit_current);
		PLOT_HISTORY("l_frame_counter  .01x", md.l_frame_counter[0]*0.1f);
		PLOT_HISTORY("integrator- .1x", -md.leg_integrator[0]*0.1f);
		//PLOT_HISTORY("encoder_error_rate", md.leg_encoder_error_rate[0]);
		//PLOT_HISTORY("encoder_error_flag", (float)md.leg_error_flag[0]);
		PLOT_HISTORY("force_r_calc-", -md.force_r_calc*0.1f);
		PLOT_HISTORY("acc_r_calc-", -md.acc_r_calc*50);
		//PLOT_HISTORY("pos_gain_check ", md.leg_control_pos_gain_check[0]);

		ImGui::TextDisabled("left data (1)");
		PLOT_HISTORY("pos ", md.leg_rel_angle[1]+md.leg_base_angle[1]);
		PLOT_HISTORY("pos_target ", md.leg_rel_angle_target[1]+md.leg_base_angle[1]);
		//PLOT_HISTORY("pos_target_ik ", md.leg_pos_target_ik[1]);
		PLOT_HISTORY("pos_target_odrv ", md.leg_rel_angle_target_odrv[1]+md.leg_base_angle[1]);
		PLOT_HISTORY("pos_target_odrv_sim ", md.leg_rel_angle_target_odrv_sim[1]+md.leg_base_angle[1]);
		PLOT_HISTORY("pos_error ",  md.leg_rel_angle[1]-md.leg_rel_angle_target[1]);
		//PLOT_HISTORY("base ", md.leg_base_angle[1]);
		PLOT_HISTORY_U("vel ", md.leg_vel[1], &unit_vel);
		PLOT_HISTORY_U("vel_target ", md.leg_vel_target_odrv[1], &unit_vel);
		PLOT_HISTORY_U("vel_target_sim ", md.leg_vel_target_odrv_sim[1], &unit_vel);
		PLOT_HISTORY_U("l_initial_vel_target", l_initial_vel[1]+md.l_frame_counter[1]*10*cd.l_initial_expected_accel_per_s/odrive_frequency, &unit_vel);
		PLOT_HISTORY_U("current", md.leg_current[1], &unit_current);
		PLOT_HISTORY_U("current_target", md.leg_current_target[1], &unit_current);
		PLOT_HISTORY_U("current_landing_sim", md.leg_current_landing_sim[1], &unit_current);
		PLOT_HISTORY("l_frame_counter .01x", md.l_frame_counter[1]*0.1f);
		PLOT_HISTORY("integrator .1x", md.leg_integrator[1]*0.1f);
		//PLOT_HISTORY("encoder_error_rate ", md.leg_encoder_error_rate[1]);
		//PLOT_HISTORY("encoder_error_flag ", (float)md.leg_error_flag[1]);
		PLOT_HISTORY("force_l_calc", md.force_l_calc*0.1f);
		PLOT_HISTORY("acc_l_calc", md.acc_l_calc*50);
		//PLOT_HISTORY("pos_gain_check", md.leg_control_pos_gain_check[1]);
		ImGui::NewLine();

		PLOT_HISTORY("common_vel", (-md.leg_vel[0]+md.leg_vel[1])*0.5f);
		{
			static bool show = false;
			plot_history(main_plot, "vel_target_straight", [&](s64 i)
			{
				const MonitorData& md = history[i];
				float leg_pos_l =   md.leg_rel_angle[1]+md.leg_base_angle[1];
				float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
				float leg_pos = (leg_pos_l+leg_pos_r)*0.5f;
				float leg_angle = (1 - leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
			    float vel_target_straight = -md.leg_vel_target_odrv[0] / sin(md.l_base_angle + leg_angle) / (tau / leg_gear_reduction);
				return vel_target_straight;
			}, &show);
		}
		ImGui::NewLine();

		{
			MonitorData& md = get_last_monitor_data();

			if (md.oscilloscope_state == 0 &&
				(ImGui::Button("trigger oscilloscope (R)") || pressing_key['R']))
			{
				pressing_key['R'] = false;
				cd.oscilloscope_force_trigger++;
				cd.counter++;
			}
			ImGui::TextDisabled("oscilloscope_state: %d", md.oscilloscope_state);
		}
		PLOT_HISTORY("oscilloscope_state", md.oscilloscope_state * 0.2f);

		PLOT_HISTORY("odrive counter .01x", (float)(md.odrive_counter-history[plot_start_history].odrive_counter)*0.01f);
		//PLOT_HISTORY("odrive counter abs", (float)md.odrive_counter);
		ImGui::NewLine();

		const char* items[] = { "0", "PASSTHROUGH", "2", "POS_FILTER", "4", "TRAP_TRAJ" };
		if (ImGui::Combo("input mode", &cd.input_mode, items, IM_ARRAYSIZE(items))) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("current_lim"        , &cd.current_lim, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("current_lim_margin" , &cd.current_lim_margin, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_gain"           , &cd.vel_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("pos_gain"           , &cd.pos_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_integrator_gain", &cd.vel_integrator_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_limit"          , &cd.vel_limit, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("accel_limit"        , &cd.accel_limit, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("input_filter_bandwidth", &cd.input_filter_bandwidth, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("encoder_bandwidth"  , &cd.encoder_bandwidth, 1.0f)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();
		
		ImGui::TextDisabled("positioning:");
		if (ImGui::Checkbox("do positioning", &cd.do_positioning)) cd.counter++;
		if (ImGui::DragFloat("avg_smooth", &cd.stepper_avg_smooth, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("error_p_threshold", &cd.positioning_error_p_threshold, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("vel_delta", &cd.positioning_vel_delta, 0.001f, 0, 0, "%.3f")) cd.counter++;
		PLOT_HISTORY("error_p_avg", md.avg_error_p);
		ImGui::NewLine();

		ImGui::TextDisabled("jumping:");
		if (ImGui::Button("trigger jump phase (F12)"))
		{
			cd.jump_phase_trigger++;
			cd.counter++;
		}
		if (ImGui::Checkbox("oscilloscope on jump", &cd.start_oscilloscope_on_jump)) cd.counter++;
		if (ImGui::DragFloat("jump_pos_target"      , &cd.jump_pos_target, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_vel_gain"           , &cd.j_vel_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_pos_gain"           , &cd.j_pos_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_vel_integrator_gain", &cd.j_vel_integrator_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_vel_limit"          , &cd.j_vel_limit, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_current_lim"        , &cd.j_current_lim, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("j_current_lim_margin" , &cd.j_current_lim_margin, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("current_side_deviation", &cd.jump_current_side_deviation, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragInt("left_right_timeout"     , &cd.jump_left_right_timeout)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();

		ImGui::TextDisabled("leg up:");
		if (ImGui::Checkbox("do_leg_up"               , &cd.do_jump_leg_up)) cd.counter++;
		if (ImGui::DragFloat("pos_target"             , &cd.leg_up_pos_target, 0.1f)) { cd.counter++; }
		if (ImGui::DragFloat("lu_min_time"            , &cd.leg_up_min_time, 0.1f)) { cd.counter++; }
		//if (ImGui::DragFloat("lu_input_filter_bandwi.", &cd.leg_up_input_filter_bandwidth, 0.1f)) { cd.counter++; }
		ImGui::NewLine();

		ImGui::TextDisabled("falling:");
		if (ImGui::Button("trigger fall phase (F)") || pressing_key['F'])
		{
			pressing_key['F'] = false;
			cd.fall_phase_trigger++;
			cd.counter++;
		}
		if (ImGui::DragFloat("fall_min_leg_angle", &cd.fall_min_leg_angle, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("fall_max_leg_angle", &cd.fall_max_leg_angle, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("fall_vertical_angle", &cd.fall_vertical_angle, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("landing detec. threshold", &cd.landing_detector_current_threshold, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("landing detec. delay", &cd.landing_detector_delay, 0.01f)) { cd.counter++; }
		if (ImGui::Combo    ("f_input mode"         , &cd.f_input_mode, items, IM_ARRAYSIZE(items)))  { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_vel_gain"           , &cd.f_vel_gain, 0.1f))                          { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_pos_gain"           , &cd.f_pos_gain, 0.1f))                          { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_vel_integrator_gain", &cd.f_vel_integrator_gain, 0.1f))               { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_vel_limit"          , &cd.f_vel_limit, 0.1f, 0.1f, 100))              { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_bandwidth"          , &cd.f_input_filter_bandwidth, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("f_current_lim"        , &cd.f_current_lim, 0.1f, 0.1f, 100))            { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("fall_servo_correction", &cd.fall_servo_correction, 0.1f)) { cd.counter++; }
		if (ImGui::DragFloat("fall_servo_target", &cd.fall_servo_target, 0.1f)) { cd.counter++; }
		if (ImGui::DragFloat("fall_servo_x_stop", &cd.fall_servo_x_angle_stop, 0.1f)) { cd.counter++; }
		PLOT_HISTORY("landing detector enabled", (float)md.enabled_landing_detector*0.75f);
		ImGui::NewLine();

		ImGui::TextDisabled("landing:");
		if (ImGui::DragFloat("l_leg_pos_target"           , &cd.l_leg_pos_target, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_base_angle_delta"         , &cd.l_base_angle_delta, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_max_time"                 , &cd.l_max_time, 0.01f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_vel_gain"                 , &cd.l_vel_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_vel_integrator_gain"      , &cd.l_vel_integrator_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_sync_gain_pos"            , &cd.l_sync_gain_pos, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_sync_gain_vel"            , &cd.l_sync_gain_vel, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_foot_vel_delta_time"      , &cd.landing_foot_vel_delta_time, 0.1f)) { cd.counter++; }
		if (ImGui::DragFloat("l_foot_vel_delta"           , &cd.landing_foot_vel_delta, 0.1f)) { cd.counter++; }
		if (ImGui::DragFloat("integrator_after_landing"   , &cd.initial_integrator_after_landing, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_measure_time"             , &cd.l_initial_measure_time, 0.001f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_measure_current"          , &cd.l_initial_measure_current, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::Checkbox("l_do_max_vel"                , &cd.l_do_max_vel)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::Checkbox("l_do_straightening"          , &cd.l_do_straightening)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_initial_expected_accel"   , &cd.l_initial_expected_accel_per_s, 1)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_initial_accel_gain"       , &cd.l_initial_accel_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("l_pos_target_delta"         , &cd.l_pos_target_delta_per_s, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		PLOT_HISTORY("post_landing_timer", md.post_landing_timer);
		ImGui::NewLine();
		
		//PLOT_HISTORY("debug1", (float)md.leg_control_debug1);
		//PLOT_HISTORY("debug2", (float)md.leg_control_debug2);
		//PLOT_HISTORY("debug3", (float)md.leg_control_debug3);
		//ImGui::NewLine();

		if (ImGui::Checkbox("enable right", &cd.leg_control_enable_motor[0])) { cd.counter++; }
		if (ImGui::DragFloat("right target", &cd.leg_abs_target[0], 0.001f)) { cd.counter++; }
		ImGui::NewLine();
		if (ImGui::Checkbox("enable left", &cd.leg_control_enable_motor[1])) { cd.counter++; }
		if (ImGui::DragFloat("left target", &cd.leg_abs_target[1], 0.001f)) { cd.counter++; }

		ImGui::PopID();
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Foot Control"))
	{
		ImGui::PushID("foot_control");
		if (ImGui::Checkbox("enable", &cd.enable_foot_control)) cd.counter++;

		ImGui::TextDisabled("ready: %d %d",
				(int)history[plot_start_history].foot_motor_ready[0], 
				(int)history[plot_start_history].foot_motor_ready[1]);
		if (ImGui::Button("trigger encoder z search"))
		{
			cd.encoder_z_search_trigger++;
			cd.counter++;
		}
		ImGui::NewLine();

		if (ImGui::Checkbox("enable left", &cd.foot_control_enable_motor[0])) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("left target vel", &cd.foot_control_vel_target[0], 0.001f)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();

		if (ImGui::Checkbox("enable right", &cd.foot_control_enable_motor[1])) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("right target vel", &cd.foot_control_vel_target[1], 0.001f)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();

		ImGui::TextDisabled("left data (0) %f", history[plot_start_history].foot_pos[0]);
		PLOT_HISTORY("pos", md.foot_pos[0]);
		PLOT_HISTORY("vel", md.foot_vel[0]);
		PLOT_HISTORY("vel_coarse", md.foot_vel_coarse[0]);
		//PLOT_HISTORY("vel_coarse_sim", md.foot_vel_coarse_sim[0]);
		PLOT_HISTORY("vel_target", md.foot_vel_target[0]);
		//PLOT_HISTORY("vel_target_unclamped", md.balance_control_vel_output - md.balance_control_rotation_vel + md.balance_control_delta_vel_l - md.balance_control_rotation_error * cd.balance_control_rotation_error_factor);
		PLOT_HISTORY("current .1x", md.foot_current[0]*0.1f);
		PLOT_HISTORY("current_target .1x", md.foot_current_target[0]*0.1f);
		PLOT_HISTORY("set_current_limit .1x", md.foot_set_current_lim[0]*0.1f);
		//PLOT_HISTORY("shadow_count", (md.foot_shadow_count[0]-history[plot_start_history].foot_shadow_count[0])/8192.0f);
		//PLOT_HISTORY("count_in_cpr", md.foot_count_in_cpr[0]*0.1f);
		//PLOT_HISTORY("ready", (float)md.foot_motor_ready[0]);
		PLOT_HISTORY("sensor_index_error", md.foot_sensor_index_error[0]*0.0001f);
		PLOT_HISTORY("sensor_index_count", md.foot_sensor_index_count[0]*0.01f);

		ImGui::TextDisabled("right data (1) %f", history[plot_start_history].foot_pos[1]);
		PLOT_HISTORY("pos-", -md.foot_pos[1]);
		PLOT_HISTORY("vel-", -md.foot_vel[1]);
		PLOT_HISTORY("vel_coarse-", -md.foot_vel_coarse[1]);
		//PLOT_HISTORY("vel_coarse_sim-", -md.foot_vel_coarse_sim[1]);
		PLOT_HISTORY("vel_target-", -md.foot_vel_target[1]);
		//PLOT_HISTORY("vel_target_unclamped-", md.balance_control_vel_output + md.balance_control_rotation_vel + md.balance_control_delta_vel_r + md.balance_control_rotation_error * cd.balance_control_rotation_error_factor);
		PLOT_HISTORY("current- .1x", -md.foot_current[1]*0.1f);
		PLOT_HISTORY("current_target- .1x", -md.foot_current_target[1]*0.1f);
		PLOT_HISTORY("set_current_limit- .1x", -md.foot_set_current_lim[1]*0.1f);
		//PLOT_HISTORY("shadow_count-", -(md.foot_shadow_count[1]-history[plot_start_history].foot_shadow_count[1])/float(foot_cpr));
		//PLOT_HISTORY("count_in_cpr-", -md.foot_count_in_cpr[1]*0.1f);
		//PLOT_HISTORY("ready ", (float)md.foot_motor_ready[1]);
		PLOT_HISTORY("sensor_index_error ", md.foot_sensor_index_error[1]*0.0001f);
		PLOT_HISTORY("sensor_index_count ", md.foot_sensor_index_count[1]*0.01f);

		ImGui::NewLine();
		
		PLOT_HISTORY("foot_acc", md.foot_acc);
		PLOT_HISTORY("do limit accel", (float)md.foot_control_limit_accel);
		ImGui::NewLine();

		if (ImGui::DragFloat("current_lim"        , &cd.foot_control_current_lim, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("current_lim_landing", &cd.foot_control_current_lim_landing, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("current_lim_margin" , &cd.foot_control_current_lim_margin, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_gain"           , &cd.foot_control_vel_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_integrator_gain", &cd.foot_control_vel_integrator_gain, 0.1f)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("vel_limit"          , &cd.foot_control_vel_limit, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("accel_limit"        , &cd.foot_control_accel_limit, 0.1f, 0.1f, 100)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::Checkbox("enable anticogging"  , &cd.foot_control_enable_anticogging)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();

		if (ImGui::Checkbox("force torque mode"   , &cd.foot_control_force_torque_mode)) { cd.counter++; }
		if (ImGui::DragFloat("torque target left" , &cd.foot_control_torque_target[0], 0.1f, -10, 10)) { cd.counter++; cd.odrive_init_counter++; }
		if (ImGui::DragFloat("torque target right", &cd.foot_control_torque_target[1], 0.1f, -10, 10)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::PopID();
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Servo"))
	{
		const u16 servo_min = 90;
		const u16 servo_max = 660;
		if (ImGui::Checkbox("enable servos", &cd.servo_enable)) cd.counter++;
		if (ImGui::Checkbox("enable manual control", &cd.servo_manual_enable)) cd.counter++;
		ImGui::NewLine();

		if (ImGui::SliderFloat("left hip" , &cd.servo_manual_target[0], 0, 1)) cd.counter++;
		if (ImGui::SliderInt("channel left" , &cd.servo_channel[0], 0, 32)) cd.counter++;
		if (ImGui::SliderScalar("left min" , ImGuiDataType_U16, &cd.servo_min[0], &servo_min, &servo_max)) cd.counter++;
		if (ImGui::SliderScalar("left max" , ImGuiDataType_U16, &cd.servo_max[0], &servo_min, &servo_max)) cd.counter++;
		PLOT_HISTORY("left pos", md.servo_pos[0]);
		PLOT_HISTORY("left pos sensor", md.hip_sensor_pos[0]);
		PLOT_HISTORY("left target", md.hip_pos_target[0]);
		PLOT_HISTORY("left target ik", md.hip_pos_target_ik[0]);
		PLOT_HISTORY("left target+correction", md.hip_pos_target[0]+md.hip_sensor_correction_delta[0]);
		PLOT_HISTORY("left correction", md.hip_sensor_correction_delta[0]);
		PLOT_HISTORY("left sensor base", md.hip_sensor_pos_base[0]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("right hip", &cd.servo_manual_target[1], 0, 1)) cd.counter++;
		if (ImGui::SliderInt("channel right", &cd.servo_channel[1], 0, 32)) cd.counter++;
		if (ImGui::SliderScalar("right min", ImGuiDataType_U16, &cd.servo_min[1], &servo_min, &servo_max)) cd.counter++;
		if (ImGui::SliderScalar("right max", ImGuiDataType_U16, &cd.servo_max[1], &servo_min, &servo_max)) cd.counter++;
		PLOT_HISTORY("right pos", md.servo_pos[1]);
		PLOT_HISTORY("right pos sensor", md.hip_sensor_pos[1]);
		PLOT_HISTORY("right target", md.hip_pos_target[1]);
		PLOT_HISTORY("right target ik", md.hip_pos_target_ik[1]);
		PLOT_HISTORY("right target+correction", md.hip_pos_target[1]+md.hip_sensor_correction_delta[1]);
		PLOT_HISTORY("right correction", md.hip_sensor_correction_delta[1]);
		PLOT_HISTORY("right sensor base", md.hip_sensor_pos_base[1]);
		ImGui::NewLine();

		if (ImGui::DragFloat("vel user_max", &cd.common_servo_vel_user_max, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("vel max", &cd.servo_vel_max, 0.01f)) cd.counter++;
		PLOT_HISTORY("common_hip_pos_target", md.common_hip_pos_target);
		PLOT_HISTORY("common_hip_pos_sensor", md.common_hip_pos_sensor);
		PLOT_HISTORY("common_servo_vel", md.common_servo_vel);
		ImGui::NewLine();

		ImGui::TextDisabled("hip balance");
		if (ImGui::DragFloat("theta_factor", &cd.hip_balance_theta_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("backcalc_factor", &cd.hip_balance_backcalc_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("action_factor", &cd.hip_balance_action_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("x_factor", &cd.hip_balance_x_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("deadzone", &cd.hip_balance_deadzone, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("factor", &cd.hip_balance_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("integrator", &cd.hip_balance_integrator, 0.01f)) cd.counter++;
		PLOT_HISTORY("hip_balance_current", md.hip_balance_current);
		PLOT_HISTORY("hip_balance_target", md.hip_balance_target);
		ImGui::NewLine();

		ImGui::TextDisabled("Hip Sensor");
		if (ImGui::Button("reset hip_sensor calibration"))
		{
			cd.reset_hip_sensors_trigger++;
			cd.counter++;
		}
			
		if (ImGui::Checkbox("enable correction", &cd.hip_sensor_enable_correction)) cd.counter++;
		if (ImGui::DragFloat("correction_factor", &cd.hip_sensor_correction_factor, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("correction_dead_zone", &cd.hip_sensor_correction_dead_zone, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("max_deviation", &cd.hip_sensor_max_deviation, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("side_angle_factor", &cd.hip_sensor_correction_side_angle_factor, 0.01f)) cd.counter++;
		if (ImGui::Checkbox("do_hip_sensor_index", &cd.do_hip_sensor_index)) { cd.counter++; cd.odrive_init_counter++; }
		ImGui::NewLine();

		PLOT_HISTORY("left hip_sensor_raw1", md.hip_sensor_raw1[0]*0.0001f);
		PLOT_HISTORY("left hip_sensor_raw2", md.hip_sensor_raw2[0]*0.0001f);
		PLOT_HISTORY("left hip_sensor_index_error", md.hip_sensor_index_error[0]*0.0001f);
		PLOT_HISTORY("left hip_sensor_index_count", md.hip_sensor_index_count[0]*0.01f);
		PLOT_HISTORY("left  hip_sensor_pos", md.hip_sensor_pos[0]);
		PLOT_HISTORY("left  hip_sensor_vel", md.hip_sensor_vel[0]);
		ImGui::NewLine();
			
		PLOT_HISTORY("right hip_sensor_raw1", md.hip_sensor_raw1[1]*0.0001f);
		PLOT_HISTORY("right hip_sensor_raw2", md.hip_sensor_raw2[1]*0.0001f);
		PLOT_HISTORY("right hip_sensor_index_error", md.hip_sensor_index_error[1]*0.0001f);
		PLOT_HISTORY("right hip_sensor_index_count", md.hip_sensor_index_count[1]*0.01f);
		PLOT_HISTORY("right hip_sensor_pos", md.hip_sensor_pos[1]);
		PLOT_HISTORY("right hip_sensor_vel", md.hip_sensor_vel[1]);
		ImGui::NewLine();
		
		PLOT_HISTORY("hip_sensor_index_enabled", (float)md.hip_sensor_index_enabled);
			
		if (ImGui::Checkbox("do_pos_base_manual", &cd.do_hip_sensor_pos_base_manual)) { cd.counter++; }
		if (ImGui::Button("copy current -> manual"))
		{
			MonitorData& md = get_last_monitor_data();
			cd.hip_sensor_pos_base_manual[0] = md.hip_sensor_pos_base[0];
			cd.hip_sensor_pos_base_manual[1] = md.hip_sensor_pos_base[1];
			cd.counter++;
		}
		if (ImGui::DragFloat("left pos_base_manual", &cd.hip_sensor_pos_base_manual[0], 0.001f)) cd.counter++;
		if (ImGui::DragFloat("right pos_base_manual", &cd.hip_sensor_pos_base_manual[1], 0.001f)) cd.counter++;
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Arm"))
	{
		const u16 servo_min = 90;
		const u16 servo_max = 660;
		if (ImGui::Checkbox("enable manual control", &cd.servo_manual_enable)) cd.counter++;
		if (ImGui::SliderScalar("servo min" , ImGuiDataType_U16, &cd.arm_servo_min, &servo_min, &servo_max)) cd.counter++;
		if (ImGui::SliderScalar("servo max" , ImGuiDataType_U16, &cd.arm_servo_max, &servo_min, &servo_max)) cd.counter++;
		if (ImGui::SliderScalar("servo min v2" , ImGuiDataType_U16, &cd.arm_servo_min_v2, &servo_min, &servo_max)) cd.counter++;
		if (ImGui::SliderScalar("servo max v2" , ImGuiDataType_U16, &cd.arm_servo_max_v2, &servo_min, &servo_max)) cd.counter++;
		ImGui::NewLine();

		if (ImGui::SliderFloat("l_arm_a target"       , &cd.arm_manual_target[0], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("l_arm_a channel"      , &cd.arm_servo_channel[0], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("l_arm_a v2"           , &cd.arm_servo_v2     [0])) cd.counter++;
		PLOT_HISTORY(          "l_arm_a pos target"       , md.arm_pos_target[0]);
		PLOT_HISTORY(          "l_arm_a pos target smooth", md.arm_pos_target_smooth[0]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("l_arm_b target"       , &cd.arm_manual_target[1], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("l_arm_b channel"      , &cd.arm_servo_channel[1], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("l_arm_b v2"           , &cd.arm_servo_v2     [1])) cd.counter++;
		PLOT_HISTORY(          "l_arm_b pos target"       , md.arm_pos_target[1]);
		PLOT_HISTORY(          "l_arm_b pos target smooth", md.arm_pos_target_smooth[1]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("l_arm_c target"       , &cd.arm_manual_target[2], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("l_arm_c channel"      , &cd.arm_servo_channel[2], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("l_arm_c v2"           , &cd.arm_servo_v2     [2])) cd.counter++;
		PLOT_HISTORY(          "l_arm_c pos target"       , md.arm_pos_target[2]);
		PLOT_HISTORY(          "l_arm_c pos target smooth", md.arm_pos_target_smooth[2]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("r_arm_a target"       , &cd.arm_manual_target[3], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("r_arm_a channel"      , &cd.arm_servo_channel[3], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("r_arm_a v2"           , &cd.arm_servo_v2     [3])) cd.counter++;
		PLOT_HISTORY(          "r_arm_a pos target"       , md.arm_pos_target[3]);
		PLOT_HISTORY(          "r_arm_a pos target smooth", md.arm_pos_target_smooth[3]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("r_arm_b target"       , &cd.arm_manual_target[4], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("r_arm_b channel"      , &cd.arm_servo_channel[4], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("r_arm_b v2"           , &cd.arm_servo_v2     [4])) cd.counter++;
		PLOT_HISTORY(          "r_arm_b pos target"       , md.arm_pos_target[4]);
		PLOT_HISTORY(          "r_arm_b pos target smooth", md.arm_pos_target_smooth[4]);
		ImGui::NewLine();

		if (ImGui::SliderFloat("r_arm_c target"       , &cd.arm_manual_target[5], 0, 1)) cd.counter++;
		if (ImGui::SliderInt  ("r_arm_c channel"      , &cd.arm_servo_channel[5], -1, 16)) cd.counter++;
		if (ImGui::Checkbox   ("r_arm_c v2"           , &cd.arm_servo_v2     [5])) cd.counter++;
		PLOT_HISTORY(          "r_arm_c pos target"       , md.arm_pos_target[5]);
		PLOT_HISTORY(          "r_arm_c pos target smooth", md.arm_pos_target_smooth[5]);
		ImGui::NewLine();

		if (ImGui::Checkbox("enable arm angle control", &cd.arm_enable_angle_control)) cd.counter++;
		if (ImGui::SliderAngle("l_arm_a angle target" , &cd.arm_angle_target[0], 0, 180)) cd.counter++;
		if (ImGui::SliderFloat("l_arm_a pos 0"  , &cd.l_arm_a_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("l_arm_a pos 90" , &cd.l_arm_a_pos_90, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
		if (ImGui::SliderAngle("l_arm_b angle target" , &cd.arm_angle_target[1], -40, 40)) cd.counter++;
		if (ImGui::SliderFloat("l_arm_b pos 0"  , &cd.l_arm_b_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("l_arm_b pos 40 (inner)" , &cd.l_arm_b_pos_40, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
		if (ImGui::SliderAngle("l_arm_c angle target" , &cd.arm_angle_target[2], -40, 40)) cd.counter++;
		if (ImGui::SliderFloat("l_arm_c pos 0"  , &cd.l_arm_c_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("l_arm_c pos 40 (fwrd)" , &cd.l_arm_c_pos_40, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
		if (ImGui::SliderAngle("r_arm_a angle target" , &cd.arm_angle_target[3], 0, 180)) cd.counter++;
		if (ImGui::SliderFloat("r_arm_a pos 0"  , &cd.r_arm_a_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("r_arm_a pos 90" , &cd.r_arm_a_pos_90, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
		if (ImGui::SliderAngle("r_arm_b angle target" , &cd.arm_angle_target[4], -40, 40)) cd.counter++;
		if (ImGui::SliderFloat("r_arm_b pos 0"  , &cd.r_arm_b_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("r_arm_b pos 40 (inner)" , &cd.r_arm_b_pos_40, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
		if (ImGui::SliderAngle("r_arm_c angle target" , &cd.arm_angle_target[5], -40, 40)) cd.counter++;
		if (ImGui::SliderFloat("r_arm_c pos 0"  , &cd.r_arm_c_pos_0 , 0, 1, "%.2f")) cd.counter++;
		if (ImGui::SliderFloat("r_arm_c pos 40 (fwrd)" , &cd.r_arm_c_pos_40, 0, 1, "%.2f")) cd.counter++;
		ImGui::NewLine();
	
		if (ImGui::DragFloat("arm_a_servo_vel_max", &cd.arm_a_servo_vel_max, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("arm_b_servo_vel_max", &cd.arm_b_servo_vel_max, 0.01f)) cd.counter++;
		if (ImGui::DragFloat("arm_c_servo_vel_max", &cd.arm_c_servo_vel_max, 0.01f)) cd.counter++;
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("IMU"))
	{
		if (ImGui::DragFloat("gyro_weight", &cd.imu_gyro_weight, 0.1f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("gyro_scale", &cd.imu_gyro_scale, 0.00001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("imu_bias_x", &cd.imu_bias_x, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("imu_bias_y", &cd.imu_bias_y, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("imu_bias_z", &cd.imu_bias_z, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::Checkbox("force_no_acc", &cd.imu_force_no_acc)) cd.counter++;
		if (ImGui::Checkbox("compensate_body_acc", &cd.imu_compensate_body_acc)) cd.counter++;
		if (ImGui::SliderInt("gyro_frequency", &cd.imu_gyro_frequency, 0, 7)) cd.counter++;
		if (ImGui::SliderInt("fifo_frequency", &cd.imu_fifo_frequency, 0, 10)) cd.counter++;
		if (ImGui::Checkbox("use_gyro_thread", &cd.imu_use_gyro_thread)) cd.counter++;
		ImGui::NewLine();

		PLOT_HISTORY("x_angle", md.imu_x_angle);
		PLOT_HISTORY("side_angle", md.imu_side_angle);
		ImGui::NewLine();

		PLOT_HISTORY("gravity_acc_x", md.imu_gravity_acc_x);
		PLOT_HISTORY("gravity_acc_x_comp", md.imu_gravity_acc_x_comp);
		PLOT_HISTORY("gravity_x", md.imu_gravity_x);
		PLOT_HISTORY("gravity_sim_x", md.imu_gravity_sim_x);
		ImGui::NewLine();

		PLOT_HISTORY("gravity_acc_y", md.imu_gravity_acc_y);
		PLOT_HISTORY("gravity_acc_y_comp", md.imu_gravity_acc_y_comp);
		PLOT_HISTORY("gravity_y", md.imu_gravity_y);
		PLOT_HISTORY("gravity_sim_y", md.imu_gravity_sim_y);
		ImGui::NewLine();

		PLOT_HISTORY("gravity_acc_z", md.imu_gravity_acc_z);
		PLOT_HISTORY("gravity_acc_z_comp", md.imu_gravity_acc_z_comp);
		PLOT_HISTORY("gravity_z", md.imu_gravity_z);
		PLOT_HISTORY("gravity_sim_z", md.imu_gravity_sim_z);
		ImGui::NewLine();

		PLOT_HISTORY("gravity magnitude", sqrt(sq(md.imu_gravity_acc_x) + sq(md.imu_gravity_acc_y) + sq(md.imu_gravity_acc_z)));
		PLOT_HISTORY("gyro_x", md.imu_gyro_x);
		PLOT_HISTORY("gyro_y", md.imu_gyro_y);
		PLOT_HISTORY("gyro_z", md.imu_gyro_z);
		PLOT_HISTORY("fifo gyro count", (float)md.imu_fifo_gyro_count);
		ImGui::NewLine();

		if (ImGui::Button("do bias estimation"))
		{
			cd.imu_bias_estimation_trigger++;
			cd.counter++;
		}
		if (ImGui::DragFloat("wait_time", &cd.imu_bias_estimation_wait_time, 0.1f, 0, 0, "%.5f")) cd.counter++;
		PLOT_HISTORY("bias_estimation_timer", md.imu_bias_estimation_timer);
		PLOT_HISTORY("gyro_bias_x", md.imu_gyro_bias_x);
		PLOT_HISTORY("gyro_bias_y", md.imu_gyro_bias_y);
		PLOT_HISTORY("gyro_bias_z", md.imu_gyro_bias_z);
		
		ImGui::NewLine();
		
		if (ImGui::DragFloat("sensor vel smooth fac", &cd.imu_sensor_to_floor_vel_smooth_factor, 0.01f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("sensor acc smooth fac", &cd.imu_sensor_to_floor_acc_smooth_factor, 0.01f, 0, 0, "%.5f")) cd.counter++;
		PLOT_HISTORY("sensor_pos_x", md.imu_sensor_to_floor_pos_x);
		PLOT_HISTORY("sensor_pos_y", md.imu_sensor_to_floor_pos_y);
		PLOT_HISTORY("sensor_vel_x", md.imu_sensor_to_floor_vel_x/(tau*wheel_radius));
		PLOT_HISTORY("sensor_vel_y", md.imu_sensor_to_floor_vel_y/(tau*wheel_radius));
		PLOT_HISTORY("sensor_acc_x", md.imu_sensor_to_floor_acc_x);
		PLOT_HISTORY("sensor_acc_y", md.imu_sensor_to_floor_acc_y);
		ImGui::NewLine();
		
		if (ImGui::DragFloat("temp_to_gyro_x_a", &cd.imu_temp_to_gyro_x_a, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("temp_to_gyro_x_b", &cd.imu_temp_to_gyro_x_b, 0.01f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("temp_to_gyro_y_a", &cd.imu_temp_to_gyro_y_a, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("temp_to_gyro_y_b", &cd.imu_temp_to_gyro_y_b, 0.01f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("temp_to_gyro_z_a", &cd.imu_temp_to_gyro_z_a, 0.001f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("temp_to_gyro_z_b", &cd.imu_temp_to_gyro_z_b, 0.01f, 0, 0, "%.5f")) cd.counter++;
		PLOT_HISTORY("imu_temp_gyro_x", md.imu_temp_gyro_x);
		PLOT_HISTORY("imu_temp_gyro_y", md.imu_temp_gyro_y);
		PLOT_HISTORY("imu_temp_gyro_z", md.imu_temp_gyro_z);

		/*s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		for (int i = start; i < end; ++i)
		{
			MonitorData& md = history[i];
			md.imu_temp_gyro_x = md.imu_temperature   *cd.imu_temp_to_gyro_x_a+cd.imu_temp_to_gyro_x_b;
			md.imu_temp_gyro_y = md.imu_temperature   *cd.imu_temp_to_gyro_y_a+cd.imu_temp_to_gyro_y_b;
			md.imu_temp_gyro_z = md.odrive_temperature*cd.imu_temp_to_gyro_z_a+cd.imu_temp_to_gyro_z_b;
		}*/
		
		/*ImGui::NewLine();
		ImGui::DragFloat("x_from_hip_l_bias"  , &cd.x_angle_from_hip_l_bias  , 0.001f, 0, 0, "%.5f");
		ImGui::DragFloat("x_from_hip_l_factor", &cd.x_angle_from_hip_l_factor, 0.001f, 0, 0, "%.5f");
		ImGui::DragFloat("x_from_hip_r_bias"  , &cd.x_angle_from_hip_r_bias  , 0.001f, 0, 0, "%.5f");
		ImGui::DragFloat("x_from_hip_r_factor", &cd.x_angle_from_hip_r_factor, 0.001f, 0, 0, "%.5f");
		PLOT_HISTORY("x_angle_from_hip_l", md.x_angle_from_hip_l);
		PLOT_HISTORY("x_angle_from_hip_r", md.x_angle_from_hip_r);
		PLOT_HISTORY("x_angle_from_hip", md.x_angle_from_hip);
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		for (int i = start; i < end; ++i)
		{
			MonitorData& md = history[i];
			md.x_angle_from_hip_l = md.hip_sensor_pos[0]*cd.x_angle_from_hip_l_factor + cd.x_angle_from_hip_l_bias;
			md.x_angle_from_hip_r = md.hip_sensor_pos[1]*cd.x_angle_from_hip_r_factor + cd.x_angle_from_hip_r_bias;
			md.x_angle_from_hip = (md.x_angle_from_hip_l+md.x_angle_from_hip_r) * 0.5f;
		}*/
	}

	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Balance Control"))
	{
		if (ImGui::Checkbox("enable control", &cd.balance_control_enable)) cd.counter++;
		if (ImGui::DragFloat("gain_p", &cd.balance_control_gain_p, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("gain_i", &cd.balance_control_gain_i, 0.01f, 0, 0, "%.2f")) cd.counter++;
		//PLOT_HISTORY("gain_i", md.balance_control_gain_i);
		if (ImGui::DragFloat("gain_d", &cd.balance_control_gain_d, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("gain_vel_high",    &cd.balance_control_gain_vel_high, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("gain_vel_low",     &cd.balance_control_gain_vel_low , 0.001f, 0, 0, "%.3f")) cd.counter++;
		ImGui::Checkbox("use_coi_vel", &cd.balance_control_use_coi_vel);
		if (ImGui::DragFloat("gain_backcalculation", &cd.balance_control_gain_backcalculation, 0.01f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::Checkbox("integrator_rotation_decay", &cd.balance_control_integrator_rotation_decay)) cd.counter++;
		if (ImGui::DragFloat("max_theta_integrator", &cd.balance_control_max_theta_integrator, 0.01f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("target_angle_vel_gain", &cd.balance_control_target_angle_vel_gain, 0.01f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("t_correction_simple_fac"    , &cd.t_correction_simple_factor, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::Checkbox("do_t_correction_acc", &cd.do_t_correction_acc)) cd.counter++;
		if (ImGui::DragFloat("t_correction_acc_p"   , &cd.t_correction_acc_p, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("t_correction_acc_smooth", &cd.t_correction_acc_smooth, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("t_correction_angle_max", &cd.t_correction_angle_max, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("t_correction_max_rot_vel", &cd.t_correction_max_rotation_vel, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("standing theta tol.", &cd.balance_control_standing_theta_tolerance, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("standing action tol.", &cd.balance_control_standing_action_tolerance, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("action tol. time", &cd.balance_control_standing_action_tolerance_time, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("landing_warmup_time", &cd.balance_control_gain_vel_landing_warmup_time, 0.01f, 0, 0, "%.2f")) cd.counter++;

		if (ImGui::DragFloat("rotation_vel_user_max", &cd.balance_control_rotation_vel_user_max, 0.001f, -0.5f, 0.5f, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("rotation_vel_acc", &cd.balance_control_rotation_vel_acc, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("rotation_error_factor", &cd.balance_control_rotation_error_factor, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("control_vel_accel", &cd.balance_control_vel_user_acceleration, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_accel_limit", &cd.balance_control_vel_acceleration_limit, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_user_max", &cd.balance_control_vel_user_max, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_extra_factor_speed", &cd.balance_control_vel_extra_factor_speed, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_extra_factor_max", &cd.balance_control_vel_extra_factor_max, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("x_delta_vel_max", &cd.balance_control_x_delta_vel_max, 0.01f, 0, 0, "%.2f")) cd.counter++;
		ImGui::NewLine();

		if (ImGui::Checkbox("enable_position_control", &cd.balance_control_enable_position_control)) cd.counter++;
		ImGui::DragFloat("pos_con_gain_p_high", &cd.balance_control_position_control_gain_p_high, 0.01f);
		ImGui::DragFloat("pos_con_gain_d_high", &cd.balance_control_position_control_gain_d_high, 0.01f);
		ImGui::DragFloat("pos_con_gain_p_low" , &cd.balance_control_position_control_gain_p_low, 0.01f);
		ImGui::DragFloat("pos_con_gain_d_low" , &cd.balance_control_position_control_gain_d_low, 0.01f);
		ImGui::NewLine();
	
		if (ImGui::DragFloat("error_d_smooth", &cd.balance_control_error_d_smooth, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_quasi_smooth_fac", &cd.balance_control_vel_quasi_smooth_factor, 0.01f, 0, 0, "%.2f")) cd.counter++;
		if (ImGui::DragFloat("vel_quasi_sm_max_acc", &cd.balance_control_vel_quasi_smooth_max_acc, 0.01f, 0, 0, "%.2f")) cd.counter++;

		PLOT_HISTORY("vel_action", md.balance_control_vel_output);
		PLOT_HISTORY("vel_action_smooth", md.balance_control_vel_output_smooth);
		PLOT_HISTORY("theta", md.error_p);
		PLOT_HISTORY("theta_d", md.error_d);
		PLOT_HISTORY("error_p", md.error_p);

		//PLOT_HISTORY("error_d", md.error_d);
		/*PLOT_HISTORY("error_vel", md.error_vel);
		PLOT_HISTORY("error_vel_i", md.error_vel_i);
		PLOT_HISTORY("error_square", md.error_square);*/
		PLOT_HISTORY("vel_p", md.vel_p);
		//PLOT_HISTORY("vel_vel", md.vel_vel);
		//PLOT_HISTORY("vel_vel_i", md.vel_vel_i);
		//PLOT_HISTORY("vel_square", md.vel_square);
		PLOT_HISTORY("vel_pos", md.vel_pos);
		PLOT_HISTORY("integrator", md.balance_control_integrator);
		PLOT_HISTORY("vel_backcalculation", md.vel_backcalculation);
		//PLOT_HISTORY("vel_pos_d", md.vel_pos_d);
		//PLOT_HISTORY("vel_pos*theta", md.vel_pos*md.error_p);
		//PLOT_HISTORY("vel_pos+balance_control_integrator", md.vel_pos+md.balance_control_integrator);
		PLOT_HISTORY("t_correction_angle", md.t_correction_angle);
		PLOT_HISTORY("t_correction_angle_raw", md.t_correction_angle_raw);
		PLOT_HISTORY("is_standing", (float)md.is_standing);
		PLOT_HISTORY("action_tolerance_timer", md.balance_control_standing_action_tolerance_timer);

		PLOT_HISTORY("rotation_vel", md.balance_control_rotation_vel);
		PLOT_HISTORY("rotation_error", md.balance_control_rotation_error);
		PLOT_HISTORY("vel_user", md.balance_control_vel_user);
		PLOT_HISTORY("vel_quasi", md.balance_control_vel_quasi);
		PLOT_HISTORY("vel_quasi_smooth", md.balance_control_vel_quasi_smooth);
		PLOT_HISTORY("vel_extra_factor", md.balance_control_vel_extra_factor);

		PLOT_HISTORY("position_control_error", md.balance_control_position_control_target + md.ik_x_delta/(tau*wheel_radius) - md.coi_world_pos/(tau*wheel_radius));
		PLOT_HISTORY("position_control_target", md.balance_control_position_control_target);

		PLOT_HISTORY("x_delta", md.ik_x_delta/(tau*wheel_radius));
				
		PLOT_HISTORY("delta_pos_l", md.balance_control_delta_pos_l);
		PLOT_HISTORY("delta_pos_r", md.balance_control_delta_pos_r);

		PLOT_HISTORY("world_angle", md.balance_control_world_angle);
		PLOT_HISTORY("world_angle sin", sin(md.balance_control_world_angle));
		PLOT_HISTORY("world_angle cos", cos(md.balance_control_world_angle));
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Squat Stabilization"))
	{
		/*static int angle_correlation_quiet_timer = 5;
		ImGui::SliderInt("quiet_timer", &angle_correlation_quiet_timer, 0, 500);
		int timer = 0;
		s64 start, end;
		plot_get_display_range(main_plot, &start, &end);
		for (int i = start+1; i < end; ++i)
		{
			if (!history[i].is_standing)
				timer = angle_correlation_quiet_timer;
			if (history[i].common_servo != history[i-1].common_servo)
				timer = angle_correlation_quiet_timer;
			if (history[i].common_leg_pos_target != history[i-1].stand_control_angle)
				timer = angle_correlation_quiet_timer;
			history[i].balance_control_is_quiet = (timer == 0);
			if (timer) timer--;
		}
		*/

		if (ImGui::DragFloat("body_cog_x"           , &cd.squat_body_cog_x        , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("body_cog_y"           , &cd.squat_body_cog_y        , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("body_mass"            , &cd.squat_body_mass         , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("whole_body_inertia"   , &cd.squat_whole_body_inertia, 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("upper_leg_cog_x"      , &cd.squat_upper_leg_cog_x   , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("upper_leg_cog_y"      , &cd.squat_upper_leg_cog_y   , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("upper_leg_mass"       , &cd.squat_upper_leg_mass    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("lower_leg_cog_x"      , &cd.squat_lower_leg_cog_x   , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("lower_leg_cog_y"      , &cd.squat_lower_leg_cog_y   , 1, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("lower_leg_mass"       , &cd.squat_lower_leg_mass    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("wheel_mass"           , &cd.squat_wheel_mass        , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("wheel_inertia"        , &cd.squat_wheel_inertia     , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("base_angle2"          , &cd.squat_base_angle2       , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("hip_angle_bias"       , &cd.squat_hip_angle_bias    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("leg_angle_bias"       , &cd.squat_leg_angle_bias    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("bias_x"               , &cd.squat_bias_x    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("bias_y"               , &cd.squat_bias_y    , 0.001f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("upper_leg_play_fac"   , &cd.squat_upper_leg_play_factor, 0.1f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("lower_leg_play_fac"   , &cd.squat_lower_leg_play_factor, 0.1f, 0, 0, "%.3f")) cd.counter++;
		if (ImGui::DragFloat("target_angle_vel_smth", &cd.balance_control_target_angle_vel_smooth, 0.001f, 0, 0, "%.3f")) cd.counter++;
		
		PLOT_HISTORY("target_angle", md.balance_control_target_angle);
		PLOT_HISTORY("target_angle_vel", md.balance_control_target_angle_vel);
		PLOT_HISTORY("target_angle_l sim", md.target_angle_l);
		PLOT_HISTORY("target_angle_r sim", md.target_angle_r);
		PLOT_HISTORY("squat_length (dm)", md.balance_control_squat_length*0.01f);
		PLOT_HISTORY("squat_length_sim (dm)", md.squat_length_sim*0.01f);
		ImGui::NewLine();
		
		if (ImGui::DragFloat("coi vel smooth fac", &cd.coi_world_vel_smooth_factor, 0.01f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragFloat("coi acc smooth fac", &cd.coi_world_acc_smooth_factor, 0.01f, 0, 0, "%.5f")) cd.counter++;
		if (ImGui::DragInt("coi_theta_past", &cd.coi_theta_past)) cd.counter++;

		PLOT_HISTORY("coi_pos", md.coi_world_pos);
		//PLOT_HISTORY("com_pos_z", md.com_world_pos_z);
		PLOT_HISTORY("coi_vel", md.coi_world_vel/(tau*wheel_radius));
		//PLOT_HISTORY("com_vel_z", md.com_world_vel_z/(tau*wheel_radius));
		PLOT_HISTORY("coi_acc", md.coi_world_acc);
		//PLOT_HISTORY("com_acc_z", md.com_world_acc_z);
		PLOT_HISTORY("coi_theta_acc", md.coi_theta_acc);
		ImGui::NewLine();

		PLOT_HISTORY("coi_pos_sim", md.coi_world_pos_sim);
		PLOT_HISTORY("coi_vel_sim", md.coi_world_vel_sim/(tau*wheel_radius));
		PLOT_HISTORY("coi_acc_sim", md.coi_world_acc_sim);
		PLOT_HISTORY("coi_theta_acc_sim", md.coi_theta_acc_sim);
		ImGui::NewLine();

		//PLOT_HISTORY("delta_pos_l", md.balance_control_delta_pos_l);
		plot_leg_x_angle_correlation();
		plot_hip_x_angle_correlation();

		if (ImGui::TreeNode("Optimize squat variables"))
		{
			static int optimizer_iterations = 100;
			ImGui::DragInt("optimizer_iterations", &optimizer_iterations, 1, 1, 1000);

			float error = squat_variables_error();
			ImGui::TextDisabled("Error: %f", error);

			static float epsilon = 0.0001f;
			ImGui::DragFloat("epsilon", &epsilon, 0.0001f, 0, 0, "%.6f");
			static float lr = 0.01f;
			ImGui::DragFloat("lr", &lr, 0.0001f, 0, 0, "%.6f");
			static float alpha = 0.999f;
			ImGui::DragFloat("alpha", &alpha, 0.0001f, 0, 0, "%.6f");

			static bool optimize = false;
			ImGui::Checkbox("Optimize", &optimize);
			if (optimize)
			{
				static vector<float> momentum;
				for (int i = 0; i < optimizer_iterations; i++)
				{
					error = squat_variables_error();
					vector<float> gradients;
					each_squat_var([&](float& var)
						{
							float old_var = var;
							var += epsilon;
							float new_error = squat_variables_error();
							var = old_var;
							gradients.push_back((new_error-error) / epsilon);
						});

					if (momentum.size() == 0)
					{
						momentum.resize(gradients.size());
						memset(momentum.data(), 0, momentum.size()*sizeof(float));
					}

					int index = 0;
					each_squat_var([&](float& var)
						{
							float delta = momentum[index]*alpha - gradients[index]*lr;
							momentum[index] = delta;
							var += delta;
							index++;
						});
				}
			}
			ImGui::TreePop();
		}

	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Leaning"))
	{
		if (ImGui::DragFloat("target_angle", &cd.leaning_target_angle_user, 0.01f, 0, 0, "%.3f")) { cd.counter++; }
		if (ImGui::DragFloat("smooth_factor", &cd.leaning_target_angle_smooth_factor, 0.01f, 0, 0, "%.3f")) { cd.counter++; }
		if (ImGui::DragFloat("max_vel_high", &cd.leaning_target_angle_max_vel_high, 0.01f, 0, 0, "%.3f")) { cd.counter++; }
		if (ImGui::DragFloat("max_vel_low", &cd.leaning_target_angle_max_vel_low, 0.01f, 0, 0, "%.3f")) { cd.counter++; }

		PLOT_HISTORY("target_angle_joystick", md.leaning_target_angle_joystick);
		PLOT_HISTORY("target_angle", md.leaning_target_angle);
		PLOT_HISTORY("optimized_target_angle", md.leaning_optimized_target_angle);
		PLOT_HISTORY("target_angle_smooth", md.leaning_target_angle_smooth);
		PLOT_HISTORY("rotation_radius_m", (cd.wheel_side_distance*(md.balance_control_vel_output+md.balance_control_rotation_vel)/(-2*md.balance_control_rotation_vel)+cd.wheel_side_distance/2)*0.001f);
		
		float com_length;
		float angle_tolerance;
		{
			MonitorData& md = history[plot_start_history];

			calculate_squat_angle(md.common_hip_pos_target, md.common_leg_pos_target, &com_length);
			//float angle_tolerance = cd.leaning_angle_rot_tolerance / com_length;
			angle_tolerance = atan(0.5f*cd.wheel_side_distance/com_length) * cd.leaning_angle_rot_tolerance;
			ImGui::TextDisabled("com_length: %f", com_length);
		}
		static float vel_delta = 0;
		ImGui::DragFloat("vel_delta", &vel_delta, 0.01f, -cd.leaning_vel_tolerance, cd.leaning_vel_tolerance, "%.3f");
		{
			/*static bool show = false;
			plot_history_2D("range 2D", [&](MonitorData& md, float value){
				float h = com_length+wheel_radius;
				float c = value*tau*wheel_radius;
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				//float v = md.balance_control_vel_quasi_smooth*tau*wheel_radius;
				float g = earth_acc*1000;
				float d = cd.wheel_side_distance;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				float alpha = tan(side_angle);
				float x0 = (4*h*c*v)/(d*d*g);
				float x1 = (8*h*h*c*c*alpha)/(d*d*d*g);
				float x2 = (2*h*alpha)/(d);
				float x = x0-x1-x2;
				return -cd.leaning_angle_rot_tolerance < x && x < cd.leaning_angle_rot_tolerance;
				}, &show);*/
		}
		{
			static bool show = false;
			plot_history(main_plot, "range 1", [&](s64 i){
				const MonitorData& md = history[i];
				float h = com_length+wheel_radius;
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float g = earth_acc*1000;
				float d = cd.wheel_side_distance;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				float alpha = tan(side_angle);
				float x2 = -(8*h*h*alpha)/(d*d*d*g);
				float x1 = (4*h*v)/(d*d*g);
				float x0 = -(2*h*alpha)/(d)-cd.leaning_angle_rot_tolerance;
				x1 /= x2;
				x0 /= x2;
				return (-x1/2 + sqrt(sq(x1/2)-x0))/(tau*wheel_radius);
				}, &show);
		}
		{
			static bool show = false;
			plot_history(main_plot, "range 2", [&](s64 i){
				const MonitorData& md = history[i];
				float h = com_length+wheel_radius;
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float g = earth_acc*1000;
				float d = cd.wheel_side_distance;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				float alpha = tan(side_angle);
				float x2 = -(8*h*h*alpha)/(d*d*d*g);
				float x1 = (4*h*v)/(d*d*g);
				float x0 = -(2*h*alpha)/(d)-cd.leaning_angle_rot_tolerance;
				x1 /= x2;
				x0 /= x2;
				return (-x1/2 - sqrt(sq(x1/2)-x0))/(tau*wheel_radius);
				}, &show);
		}
		{
			static bool show = false;
			plot_history(main_plot, "range 3", [&](s64 i){
				const MonitorData& md = history[i];
				float h = com_length+wheel_radius;
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float g = earth_acc*1000;
				float d = cd.wheel_side_distance;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				float alpha = tan(side_angle);
				float x2 = -(8*h*h*alpha)/(d*d*d*g);
				float x1 = (4*h*v)/(d*d*g);
				float x0 = -(2*h*alpha)/(d)+cd.leaning_angle_rot_tolerance;
				x1 /= x2;
				x0 /= x2;
				return (-x1/2 + sqrt(sq(x1/2)-x0))/(tau*wheel_radius);
				}, &show);
		}
		{
			static bool show = false;
			plot_history(main_plot, "range 4", [&](s64 i){
				const MonitorData& md = history[i];
				float h = com_length+wheel_radius;
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float g = earth_acc*1000;
				float d = cd.wheel_side_distance;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				float alpha = tan(side_angle);
				float x2 = -(8*h*h*alpha)/(d*d*d*g);
				float x1 = (4*h*v)/(d*d*g);
				float x0 = -(2*h*alpha)/(d)+cd.leaning_angle_rot_tolerance;
				x1 /= x2;
				x0 /= x2;
				return (-x1/2 - sqrt(sq(x1/2)-x0))/(tau*wheel_radius);
				}, &show);
		}

		{
			static bool show = false;
			plot_history(main_plot, "range min", [&](s64 i){
				const MonitorData& md = history[i];
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float r_min, r_max;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				calculate_rotation_vel_range(com_length+wheel_radius, v, side_angle, r_min, r_max);
				return r_min;
				}, &show);
		}
		{
			static bool show = false;
			plot_history(main_plot, "range max", [&](s64 i){
				const MonitorData& md = history[i];
				float v = (md.balance_control_vel_output+vel_delta)*tau*wheel_radius;
				float r_min, r_max;
				float side_angle = md.imu_side_angle+abs(md.error_p)*sign(md.imu_side_angle);
				calculate_rotation_vel_range(com_length+wheel_radius, v, side_angle, r_min, r_max);
				return r_max;
				}, &show);
		}
		if (ImGui::DragFloat("angle_rot_tolerance", &cd.leaning_angle_rot_tolerance, 0.001f)) { cd.counter++; }
		if (ImGui::DragFloat("vel_tolerance", &cd.leaning_vel_tolerance, 0.001f)) { cd.counter++; }
		if (ImGui::Checkbox("clamp_rotation_vel", &cd.leaning_clamp_rotation_vel)) { cd.counter++; }
		PLOT_HISTORY("min_rotation_vel", md.leaning_min_rotation_vel);
		PLOT_HISTORY("max_rotation_vel", md.leaning_max_rotation_vel);
		ImGui::NewLine();

		ImGui::TextDisabled("Sim rotation vel range:");
		PLOT_HISTORY("maxrotvel",             tan(md.leaning_optimized_target_angle+angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_output*sq(tau*wheel_radius)));
		PLOT_HISTORY("minrotvel",             tan(md.leaning_optimized_target_angle-angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_output*sq(tau*wheel_radius)));
		PLOT_HISTORY("maxrotvel_smooth",      tan(md.leaning_optimized_target_angle+angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_quasi_smooth*sq(tau*wheel_radius)));
		PLOT_HISTORY("minrotvel_smooth",      tan(md.leaning_optimized_target_angle-angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_quasi_smooth*sq(tau*wheel_radius)));
		PLOT_HISTORY("maxrotvel_side",        tan(md.imu_side_angle+angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_output*sq(tau*wheel_radius)));
		PLOT_HISTORY("minrotvel_side",        tan(md.imu_side_angle-angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_output*sq(tau*wheel_radius)));
		PLOT_HISTORY("maxrotvel_smooth_side", tan(md.imu_side_angle+angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_quasi_smooth*sq(tau*wheel_radius)));
		PLOT_HISTORY("minrotvel_smooth_side", tan(md.imu_side_angle-angle_tolerance)*earth_acc*1000*0.5f*cd.wheel_side_distance / (md.balance_control_vel_quasi_smooth*sq(tau*wheel_radius)));

		PLOT_HISTORY("rotation_vel2", md.balance_control_rotation_vel);
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("IK"))
	{
		ImGui::Checkbox("show_big_3d_view", &show_big_3d_view);
		ImGui::DragFloat("zoom_3d_view", &zoom_3d_view, 0.01f, 1, 3, "%.1f");
		ImGui::NewLine();
		if (ImGui::Checkbox("ik_enable", &cd.ik_enable)) { cd.counter++; }
		if (ImGui::Checkbox("enable_arm_mode", &cd.ik_enable_arm_mode)) { cd.ik_target_counter++; cd.counter++; }
		PLOT_HISTORY("is arm mode", md.ik_enable_arm_mode);

		if (ImGui::DragFloat("side target_angle", &cd.leaning_target_angle_user, 0.001f, -cd.leaning_max_angle, cd.leaning_max_angle, "%.3f")) { cd.counter++; }
		
		if (ImGui::DragFloat("x_delta_target (mm)", &cd.ik_x_delta_target, 1, -100, 100, "%.1f")) { cd.ik_target_counter++; cd.counter++; }
		if (ImGui::DragFloat("arm_target_x"  , &cd.ik_arm_target_x  , 1, 50, 300  , "%.1f")) { cd.ik_target_counter++; cd.counter++; }
		if (ImGui::DragFloat("arm_target_y"  , &cd.ik_arm_target_y  , 1, 40, 150  , "%.1f")) { cd.ik_target_counter++; cd.counter++; }
		if (ImGui::DragFloat("arm_target_z"  , &cd.ik_arm_target_z  , 1, -500, -15, "%.1f")) { cd.ik_target_counter++; cd.counter++; }

		if (ImGui::DragInt  ("cont_iterations"           , &cd.ik_optimizer_iterations, 1.0f, 0, 5000)) { cd.counter++; }
		if (ImGui::DragFloat("epsilon"                   , &cd.ik_optimizer_epsilon, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("lr"                        , &cd.ik_optimizer_lr, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("x_delta_lr"                , &cd.ik_optimizer_x_delta_lr, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("alpha"                     , &cd.ik_optimizer_alpha, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_angle"               , &cd.ik_optimizer_error_angle_factor, 0.01f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_angle_scale"         , &cd.ik_optimizer_error_angle_scale, 0.01f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_wheel_x_var"         , &cd.ik_optimizer_error_wheel_x_var_factor, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_leaning_angle"       , &cd.ik_optimizer_error_leaning_angle_factor, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_arm"                 , &cd.ik_optimizer_error_arm_factor, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_arm_scale"           , &cd.ik_optimizer_error_arm_scale, 0.01f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_x_delta"             , &cd.ik_optimizer_error_x_delta_factor, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("error_collision"           , &cd.ik_optimizer_error_collision_factor, 0.0001f, 0, 0, "%.6f")) { cd.counter++; }
		ImGui::NewLine();
		
		if (ImGui::DragFloat("test_delta", &test_delta, 0.01f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("test_tresh", &test_tresh, 0.01f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("test_x", &test_x, 0.1f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("test_y", &test_y, 0.1f, 0, 0, "%.6f")) { cd.counter++; }
		if (ImGui::DragFloat("test_z", &test_z, 0.1f, 0, 0, "%.6f")) { cd.counter++; }
		{
			MonitorData& md = history[plot_start_history];
			ImGui::SliderAngle("r_arm_a", &md.ik_r_arm_a, 0, 180 , u8"%.1f°");
			ImGui::SliderAngle("r_arm_b", &md.ik_r_arm_b, -40, 40, u8"%.1f°");
			ImGui::SliderAngle("r_arm_c", &md.ik_r_arm_c, -40, 40, u8"%.1f°");
		}
		ImGui::NewLine();
		
		if (ImGui::Checkbox("use_thread", &cd.ik_use_thread)) { cd.counter++; }
		ImGui::NewLine();

		static bool show_simulation = false;
		ImGui::Checkbox("show ik simulation", &show_simulation);
		if (show_simulation)
		{
			ImGui::PushID("sim");
			static int sim_iterations = 500;
			ImGui::DragInt("sim_iterations", &sim_iterations, 1.0f, 0, 5000);

			static RobotJointState robot_state_cont;

			static RobotIKTarget target_sim;
			static bool keep_current = false;
			ImGui::Checkbox("keep_current", &keep_current);
			if (keep_current || ImGui::Button("set sim target to current"))
			{
				target_sim = robot_target;
			}

			static bool do_continuous = false;
			ImGui::Checkbox("do_continuous", &do_continuous);
			if (do_continuous)
				robot_state = robot_state_cont;

			static float arm_target_y_delta = 80;
			ImGui::Checkbox("enable_arm_mode", &target_sim.enable_arm_mode);
			ImGui::DragFloat("target_side_angle", &target_sim.side_angle, 0.01f, 0, 0, "%.3f");
			ImGui::DragFloat("x_delta_target"   , &target_sim.x_delta, 1, -100, 100, "%.1f");
			ImGui::DragFloat("arm_target_x"     , &target_sim.r_arm_target_x, 1, 50, 300, "%.1f");
			ImGui::DragFloat("arm_target_y"     , &arm_target_y_delta, 1, 40, 150, "%.1f");
			ImGui::DragFloat("arm_target_z"     , &target_sim.r_arm_target_z, 1, -500, -15, "%.1f");
			ImGui::DragFloat("common_hip_angle" , &target_sim.common_hip_angle, 0.01f, min_hip_angle, max_hip_angle, "%.3f");
			ImGui::DragFloat("common_leg_angle" , &target_sim.common_leg_angle, 0.01f, min_leg_angle, max_leg_angle, "%.3f");
			ImGui::NewLine();

			target_sim.r_arm_target_y = arm_target_y_delta;
			target_sim.l_arm_target_y = -arm_target_y_delta;
			target_sim.l_arm_target_x = target_sim.r_arm_target_x;
			target_sim.l_arm_target_z = target_sim.r_arm_target_z;

			robot_target = target_sim;

			float error_parts[7] = {0};
			std::vector<float> error_history[num_ik_opt_vars*2+1];
			float error = calculate_ik(robot_state, robot_target,
					do_continuous ? cd.ik_optimizer_iterations : sim_iterations,
					error_history, error_parts);
			ImGui::TextDisabled("Error: %f", error);
			ImGui::TextDisabled("hip-leg: %f", error_parts[0]);
			ImGui::TextDisabled("wheel x var: %f", error_parts[1]);
			ImGui::TextDisabled("leaning: %f", error_parts[2]);
			ImGui::TextDisabled("arm: %f", error_parts[3]);
			ImGui::TextDisabled("x_delta: %f", error_parts[4]);
			ImGui::TextDisabled("collision: %f", error_parts[5]);
			ImGui::TextDisabled("side_angle_sim: %f", robot_state.side_angle);
			ImGui::NewLine();
			//ImGui::TextDisabled("slider_b: %f", error_parts[5]);
			//ImGui::TextDisabled("slider_c: %f", error_parts[6]);

			if (do_continuous)
				robot_state_cont = robot_state;


			/*ImGui::TextDisabled("leg_pos_target_leaning_sim0: %f", leg_pos_target_leaning_sim[0]);
			ImGui::TextDisabled("leg_pos_target_leaning_sim1: %f", leg_pos_target_leaning_sim[1]);
			ImGui::TextDisabled("hip_pos_target_leaning_sim0: %f", hip_pos_target_leaning_sim[0]);
			ImGui::TextDisabled("hip_pos_target_leaning_sim1: %f", hip_pos_target_leaning_sim[1]);
			ImGui::TextDisabled("x_angle_leaning_sim: %f"        , x_angle_leaning_sim);*/
			
			set_plot_history_elements(error_history_plot, error_history[0].size(),
					[](s64 i){ return i; }, 1.0);

			for (int j = 0; j < num_ik_opt_vars*2+1; j++)
			{
				char buffer[128] = "error history";
				if (j >= 1 && j < num_ik_opt_vars+1) sprintf(buffer, "var %d", j-1);
				if (j >= num_ik_opt_vars+1) sprintf(buffer, "mom %d", j-1-num_ik_opt_vars);
				static bool show[num_ik_opt_vars*2+1];
				plot_history(error_history_plot, buffer, [&](s64 i){return error_history[j][i];}, &show[j]);
			}
			ImGui::PopID();
		}
		
		/*{
			static bool show = false;
			plot_history("optimized_target_angle_sim", [&](MonitorData& md)
				{
					float opt_vars[num_leaning_opt_vars];
					float leaning_angle;
					calculate_ik(md, md.leaning_target_angle_smooth, opt_vars, &leaning_angle, nullptr, nullptr, nullptr);
					return leaning_angle;
				}, &show);
		}*/
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("TTS"))
	{
		if (ImGui::SliderFloat("volume", &cd.tts_volume, 0, 1, "%.2f")) { cd.counter++; }

		ImGui::Text("Say:");
        if (ImGui::BeginListBox("##listboxsay", ImVec2(-FLT_MIN, tts_say_things_num * ImGui::GetTextLineHeightWithSpacing())))
		{
			for (int i = 0; i < tts_say_things_num; i++)
			{
				if (ImGui::Selectable(tts_say_things[i], false))
				{
					cd.tts_say_thing_index = i;
					cd.tts_say_thing_trigger++;
					cd.counter++;
				}
			}
            ImGui::EndListBox();
		}
		ImGui::Text("Commands:");
        if (ImGui::BeginListBox("##listboxcommands", ImVec2(-FLT_MIN, num_commands * ImGui::GetTextLineHeightWithSpacing())))
		{
			for (int i = 0; i < num_commands; i++)
			{
				if (ImGui::Selectable(command_names[i], false))
				{
					cd.tts_say_thing_index = -i-1;
					cd.tts_say_thing_trigger++;
					cd.counter++;
				}
			}
            ImGui::EndListBox();
		}
		PLOT_HISTORY("asr_client_connected", md.asr_client_connected*.4f);
		PLOT_HISTORY("asr_recv_text", md.asr_recv_text*.45f);
		PLOT_HISTORY("tts_client_connected", md.tts_client_connected*.3f);
		PLOT_HISTORY("tts_calculating", md.tts_calculating*.1f);
		PLOT_HISTORY("tts_playback", md.tts_playback*.2f);
		PLOT_HISTORY("tts_recv_audio", md.tts_recv_audio*.25f);
		PLOT_HISTORY("tts_queue_size", md.tts_queue_size/24000.0f);
		PLOT_HISTORY("command_running", (md.command_running != -1)*0.5f);
		PLOT_HISTORY("command_queue_size", md.command_queue_size*0.1f);
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Webcam Capture"))
	{
		WebcamInfo wi = webcam_get_info();
		ImGui::TextDisabled("%s", wi.title);
		ImGui::TextDisabled("frames: %lld measured framerate: %.02f", video_recorder_get_num_frames(webcam_video), wi.framerate_estimate);

		float delta_time = float(history.back().odrive_counter) / odrive_frequency - (wi.last_capture_time+webcam_time_delta)*webcam_time_to_seconds;
		static float delta_time_avg;
		delta_time_avg = delta_time*0.01f + delta_time_avg*0.99f;
		ImGui::TextDisabled("delta time avg: %.02f  delta time: %.02f", delta_time_avg, delta_time);

		if (wi.capturing)
		{
			if (ImGui::Button("stop capturing"))
				webcam_stop_capture();
		}
		else
		{
			if (ImGui::Button("start capturing"))
				webcam_start_capture("Stereo Vision 1");
			if (client_get_connection_state() != 2 && ImGui::IsItemHovered())
				ImGui::SetTooltip("The robot needs to be connected for this to work");
		}

		ImGui::DragScalar("user time delta", ImGuiDataType_S64, &webcam_time_delta, 0.001f/webcam_time_to_seconds);

		std::vector<WebcamFrame> new_frames = webcam_get_new_frames();

		// Automatically synchronize webcam time and robot time when capture is started
		// The time of the first webcam frames can be inprecise, so we wait for a few frames to arrive.
		bool init_time_delta = false;
		if (webcam_time_delta == 0 && 
			wi.capturing &&
			video_recorder_get_num_frames(webcam_video) >= 60 &&
			new_frames.size() != 0)
		{
			init_time_delta = true;
		}
		if (ImGui::Button("reset time delta") || init_time_delta)
		{
			webcam_time_delta = s64(float(history.back().odrive_counter) / odrive_frequency / webcam_time_to_seconds) - wi.last_capture_time;
			delta_time_avg = 0;
		}

		if (ImGui::Button("clear all frames"))
			clear_webcam_frames();
		if (ImGui::Button("clear non jump frames"))
			clear_webcam_non_jump_frames();
				
		for (int i = 0; i < new_frames.size(); i++)
		{
			u8* data = webcam_get_frame(new_frames[i]);
			video_recorder_add_frame(webcam_video, data, new_frames[i].cx, new_frames[i].cy, new_frames[i].time);
			webcam_free_frame(data);
		}
		webcam_free_frames(new_frames);
	}
	ImGui::NewLine();


	if (ImGui::Button("clear history"))
		clear_history();
	ImGui::NewLine();

	{
		ImGui::TextDisabled("Save/Load all plotting data");
		ImGui::TextDisabled("in the folder ../logs/");
		static char filename[128] = "monitor_log.bin";
		ImGui::InputText("filename", filename, IM_ARRAYSIZE(filename));
		
		ImGui::BeginDisabled(history.size() <= 1);
		if (ImGui::Button("save history"))
			save_history(filename);
		ImGui::EndDisabled();
		ImGui::BeginDisabled(client_get_connection_state() == 2);
		if (ImGui::Button("load history"))
			load_history(filename);
		ImGui::EndDisabled();
	}

	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::PopItemWidth();
	ImGui::End();
}

void draw_ui_main_1(float monitor_height, float sidebar_width,
		RobotJointState& robot_state, RobotIKTarget& robot_target)
{
	ImGui::SetNextWindowPos(ImVec2(sidebar_width, 0));
	ImGui::SetNextWindowSize(ImVec2((float)window_size_x-sidebar_width+5, (float)window_size_y));
	ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

	if (ImGui::TreeNodeEx("Robot visualization", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImVec2 draw_robot_ui_pos = ImGui::GetCursorScreenPos();
		draw_robot(draw_robot_ui_pos.x+80*dpi_scaling, draw_robot_ui_pos.y+80*dpi_scaling);
		draw_robot_3d_views(robot_state, &robot_target, draw_robot_ui_pos.x, draw_robot_ui_pos.y);
		ImGui::Dummy(ImVec2(0, (show_big_3d_view ? 570 : 200)*dpi_scaling));
		ImGui::TreePop();
	}
	ImGui::End();
}

void draw_ui_main_2()
{
	s64 plot_start_history = plot_get_visual_selection_index(main_plot);

	ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

	if (ImGui::TreeNodeEx("Plot", ImGuiTreeNodeFlags_DefaultOpen))
	{
		plot_draw(main_plot, 400*dpi_scaling);
		ImGui::TreePop();
	}

	if (plot_get_num_active_graphs(error_history_plot))
		plot_draw(error_history_plot, 400*dpi_scaling);

	if (ImGui::TreeNodeEx("Webcam Display"))
	{
		double start = glfwGetTime();
		bool fast = false;
		s64 video_frames = video_recorder_get_num_frames(webcam_video);
		if (video_frames)
		{
			WebcamInfo wi = webcam_get_info();
			s64 webcam_time = history_to_webcam_time(plot_start_history);
			s64 index = video_recorder_get_frame_index_from_time(webcam_video, webcam_time);
					
			VideoRecorderFrameInfo frame;
			// If the latest captured webcam image is close to what we want to display
			// we snap to the latest image. This improves performance of video_recorder_get_frame
			video_recorder_get_frame_info(webcam_video, video_frames-1, &frame);
			if (wi.capturing && abs(frame.time-webcam_time) < s64(1/webcam_time_to_seconds))
			{
				index = video_frames-1;
			}

			video_recorder_get_frame_info(webcam_video, index, &frame);

			if (abs(webcam_time-frame.time) < s64(1/webcam_time_to_seconds))
			{
				ImVec2 ui_pos = ImGui::GetCursorScreenPos();

				u8* frame_data = video_recorder_get_frame(webcam_video, index, &fast);
				if (frame.cy > 700)
				{
					ImGui::Dummy(ImVec2(0, (float)frame.cy/2));
					gl_draw_bitmap(frame_data, frame.cx, frame.cy, (int)ui_pos.x, (int)ui_pos.y, frame.cx/2, frame.cy/2);
				}
				else
				{
					ImGui::Dummy(ImVec2(0, (float)frame.cy));
					gl_draw_bitmap(frame_data, frame.cx, frame.cy, (int)ui_pos.x, (int)ui_pos.y, frame.cx, frame.cy);
				}
			}
		}
		double end = glfwGetTime();
		ImGui::TextDisabled("Time: %dms", (int)((end-start)*1000.0));
		ImGui::TextDisabled("fast: %d", fast ? 1 : 0);
		ImGui::TreePop();
	}
		
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();
	ImGui::NewLine();

	ImGui::End();
}

void draw_ui()
{
	if (show_simulation)
	{
		control_draw_ui(pressing_key);
		return;
	}

	set_plot_history_elements(main_plot, history.size(),
		[](s64 i){ return history[i].display_time; },
		1.0 / odrive_frequency);


	float monitor_height = 160*dpi_scaling;
	float sidebar_width = 350*dpi_scaling;
	
	draw_ui_monitoring(monitor_height, sidebar_width);

	s64 plot_start_history = plot_get_visual_selection_index(main_plot);
	RobotJointState robot_state = get_robot_joint_state(history[plot_start_history], true);
	RobotIKTarget robot_target = get_robot_ik_target(history[plot_start_history]);

	draw_ui_sidebar(monitor_height, sidebar_width, robot_state, robot_target);

	draw_ui_main_1(monitor_height, sidebar_width, robot_state, robot_target);
	draw_ui_main_2();

	do_offline_calculation();

	//ImGui::ShowDemoWindow();
	//ImPlot::ShowDemoWindow();
}


void Display()
{
	glfwGetWindowSize(window, &window_size_x, &window_size_y);
    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    glClearColor(0.95f, 0.98f, 0.99f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, window_size_x, window_size_y);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, window_size_x, window_size_y, 0, -1.0f, +1.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	draw_ui();

	ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
}

void OnKeyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	//if (ImGui::GetIO().WantCaptureKeyboard)
	// space isn't used by imgui, so we handle that for emergency exit
	if (ImGui::GetIO().WantTextInput && key != GLFW_KEY_SPACE)
	{
		ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
		return;
	}

	if (key >= 0 && key < GLFW_KEY_LAST)
		pressing_key[key] = (action != GLFW_RELEASE);

	ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);

	if (action == GLFW_RELEASE)
		return;

	if (key >= '0' && key <= '9')
	{
		static s64 save[10];
		if (mods & GLFW_MOD_CONTROL)
			save[key-'0'] = plot_get_visual_selection_index(main_plot);
		else if (save[key-'0'] != 0)
			plot_set_visual_selection_index(main_plot, save[key-'0']);
	}

	switch (key)
    {
	case GLFW_KEY_SPACE:
		// emergency stop
		// todo: duplicated in joystick do_emergency_stop()
		cd.leg_control_enable_motor[0] = false;
		cd.leg_control_enable_motor[1] = false;
		cd.foot_control_enable_motor[0] = false;
		cd.foot_control_enable_motor[1] = false;
		cd.enable_stand_control = false;
		cd.balance_control_enable = false;
		cd.servo_enable = false;
		cd.servo_manual_enable = false;
		cd.arm_enable_angle_control = false;
		cd.stop_startup_sequence_trigger++;
		cd.counter++;
		cd.odrive_init_counter++;
		break;
	case GLFW_KEY_Q:
		history.back().balance_control_is_quiet = true;
		break;
    case GLFW_KEY_ESCAPE:
	case GLFW_KEY_F11:
	{
		static int xPosWindowed, yPosWindowed;
		static int xSizeWindowed, ySizeWindowed;
		if (glfwGetWindowMonitor(window))
		{
			glfwSetWindowMonitor(window, nullptr,
					xPosWindowed, yPosWindowed,
					xSizeWindowed, ySizeWindowed, GLFW_DONT_CARE);
		}
		else
		{
			if (key == GLFW_KEY_ESCAPE)
			{
				glfwSetWindowShouldClose(window, 1);
				break;
			}
			glfwGetWindowPos(window, &xPosWindowed, &yPosWindowed);
			glfwGetWindowSize(window, &xSizeWindowed, &ySizeWindowed);

			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
			const GLFWvidmode* v = glfwGetVideoMode(monitor);
			int xpos, ypos;
			glfwGetMonitorPos(monitor, &xpos, &ypos);
			glfwSetWindowMonitor(window, monitor, xpos, ypos, v->width, v->height, GLFW_DONT_CARE);
		}
		break;
	}
	case GLFW_KEY_S:
		show_simulation = !show_simulation;
		break;
	case GLFW_KEY_F12:
		cd.jump_phase_trigger++;
		cd.counter++;
		break;
	}
}

void on_mouse_button(GLFWwindow* window, int button, int action, int mods)
{
	//cout << button << endl;
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
}

void on_mouse_wheel(GLFWwindow* window, double xOffset, double yOffset)
{
	ImGui_ImplGlfw_ScrollCallback(window, xOffset, yOffset);
}

void on_window_sized(GLFWwindow* window, int w, int h)
{
	Display();
}

void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
	assert(0);
}

static void gl_draw_bitmap(u8* data, int data_cx, int data_cy, int pos_x, int pos_y, int d_cx, int d_cy)
{
	static GLuint tex;
	static bool init = false;
	if (!init)
	{
		init = true;
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT); 
	}

	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data_cx, data_cy, 0, GL_RGB, GL_UNSIGNED_BYTE, (GLvoid*)data);

	glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex2i(pos_x     , pos_y     );
    glTexCoord2f(1, 0); glVertex2i(pos_x+d_cx, pos_y     );
    glTexCoord2f(1, 1); glVertex2i(pos_x+d_cx, pos_y+d_cy);
    glTexCoord2f(0, 1); glVertex2i(pos_x     , pos_y+d_cy);
    glEnd();
    glDisable(GL_TEXTURE_2D);

	// leak...
	//glDeleteTextures(1, &tex);
}

#ifdef _MSC_VER
// Linker -> System -> SubSystem
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, char* CmdLine, int CmdShow)
#else
int main(int argc, char** argv)
#endif
{
	time_init();
	{
		MonitorData md;
		md.counter = 0;
		on_new_monitor_data(md);
	}
	if (!client_init()) return EXIT_FAILURE;

	if (0)
	{
		load_history("monitor_log.bin");
		//plot_set_x_limit_min(main_plot, 599);
		//plot_set_x_limit_width(main_plot, 2);
	}


	control_init();
	webcam_init();

	// Setup window
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        return EXIT_FAILURE;

	glfwWindowHint(GLFW_VISIBLE, 0);
	glfwWindowHint(GLFW_MAXIMIZED, 1);
    window = glfwCreateWindow(1280, 720, "Milana Robot Control UI", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Disable vsync

    // register callbacks
    glfwSetMouseButtonCallback(window, on_mouse_button);
    glfwSetScrollCallback     (window, on_mouse_wheel);
    glfwSetKeyCallback        (window, OnKeyboard);
    glfwSetCharCallback       (window, ImGui_ImplGlfw_CharCallback);
	glfwSetWindowSizeCallback (window, on_window_sized);

	// get windows dpi scaling factor
#ifdef _MSC_VER
	HDC hdc = GetDC(0);
	dpi_scaling = (float)GetDeviceCaps(hdc, LOGPIXELSX) / 96.f;
	ReleaseDC(0, hdc);
#else
	dpi_scaling = 1;
#endif

	// setup ImGui
    ImGui::CreateContext();
    ImPlot::CreateContext();
	ImGui::StyleColorsLight();
	ImGui::GetStyle().ScaleAllSizes(dpi_scaling);
	ImGui::GetStyle().WindowRounding = 0;
    ImGuiIO& io = ImGui::GetIO();
	io.IniFilename = nullptr;
#if INCLUDE_ICON_FONTS
	io.Fonts->AddFontFromMemoryCompressedBase85TTF(DroidSans_compressed_data_base85, 18.0f*dpi_scaling);
	ImFontConfig config;
	config.MergeMode = true;
	//config.GlyphMinAdvanceX = 18.0f*dpi_scaling; // Use if you want to make the icon monospaced
	static const ImWchar icon_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
	io.Fonts->AddFontFromMemoryCompressedBase85TTF(fa_solid_900_compressed_data_base85, 18.0f*dpi_scaling, &config, icon_ranges);
#else
	io.Fonts->AddFontDefault();
#endif

	ImGui_ImplGlfw_InitForOpenGL(window, false);
	ImGui_ImplOpenGL2_Init();

	glfwShowWindow(window);
	
	double last_time = glfwGetTime();

	// main loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();
		Display();

		if (!client_update())
			break;

		update();
		
		double current_time = glfwGetTime();

		if (!glfwGetWindowAttrib(window, GLFW_FOCUSED) || glfwGetWindowAttrib(window, GLFW_ICONIFIED))
			imprecise_sleep(0.1);
		else
		{
			const int target_delta_ms = 18;
			double remaining = target_delta_ms/1000.0-(current_time-last_time);
			precise_sleep(remaining);
			current_time = glfwGetTime();
		}
		last_time = current_time;
    }

    // cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwTerminate();
	client_close();
	webcam_close();

	return EXIT_SUCCESS;
}

void draw_robot_3d_views(const RobotJointState& s, const RobotIKTarget* target,
		float ui_x_pos, float ui_y_pos)
{
	glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

	//glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	// good old-fashioned fixed function lighting
    float black[]    = { 0.0f, 0.0f, 0.0f, 1.0f };
    float white[]    = { 1.0f, 1.0f, 1.0f, 1.0f };
    float ambient[]  = { 0.1f, 0.1f, 0.1f, 1.0f };
    float diffuse[]  = { 0.9f, 0.9f, 0.9f, 1.0f };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);

    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	float zoom = show_big_3d_view ? zoom_3d_view : 2;

	for (int i = 0; i < 4; i++)
	{
		int view_port_x = (int)(ui_x_pos + (show_big_3d_view ? -100+250*i : 200+120*i)*dpi_scaling);
		int view_port_y = window_size_y - (int)(ui_y_pos + (show_big_3d_view ? 560 : 310)*dpi_scaling);
		int view_port_cx = (int)(400*dpi_scaling);
		int view_port_cy = (int)(400*dpi_scaling);
		glViewport(view_port_x, view_port_y, view_port_cx, view_port_cy);

		/*glEnable(GL_SCISSOR_TEST);
		glScissor(view_port_x, view_port_y, view_port_cx, view_port_cy);
		glClearColor(0.5f, 0.98f, 0.99f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);*/

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		if (i == 3)
			glOrtho(-200*zoom, 200*zoom, -200*zoom, 200*zoom, -2000, 2000);
		else
		{
#ifdef _MSC_VER
			gluPerspective(60, 1, 0.1, 2000);
#else
			glm::mat4 m = glm::perspective(to_rad(60), 1.0f, 0.1f, 2000.0f);
			glLoadMatrixf(&m[0][0]);
#endif
		}

		float x_pos = 200, y_pos = 0, z_pos = -220+50;
		if (i == 0) y_pos = 300;
		if (i == 1 || i == 3) x_pos = 400;
		if (i == 2) y_pos = -300;
		
#ifdef _MSC_VER
		gluLookAt(x_pos*zoom, y_pos*zoom, z_pos,  0, 0, z_pos,  0, 0, -1);
#else
		glm::mat4 m = glm::lookAt(glm::vec3(x_pos*zoom, y_pos*zoom, z_pos), glm::vec3(0, 0, z_pos), glm::vec3(0, 0, -1));
		glMultMatrixf(&m[0][0]);
#endif

		float light_pos[] = { x_pos, y_pos+80, -400, 0.0f };
		glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		draw_robot_3d(s, target, true);
	}

	// Restore GL state

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glViewport(0, 0, window_size_x, window_size_y);

	glDisable(GL_CULL_FACE);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
}
