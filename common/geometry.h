// In this file, we have all the nasty math involved in converting from one coordinate system into
// others. We also define here a bunch of constants that are directly from the CAD file or manual measurements.
// This file also includes an IK algorithm that optimizes a joint state for a given target specification.
// This IK algo also has collision detection included that will try to prevent the arms from touching the body
// or the floor (the servos controlling the arms are not backdrivable, so we want to prevent damage to them).
// And finally, this file includes a function to visualize the robot in 3D (all the geometry for collision
// detection can be rendered).
#pragma once
#include "common.h"
#include <vector>

// Number of variables IK optimizes
const int num_ik_opt_vars = 1+4+3+3;

const float min_leg_angle = (1-0.97f) * (tau / leg_gear_reduction) + leg_angle_1; // see cd.common_leg_pos_max
const float max_leg_angle = (1-(-0.65f)) * (tau / leg_gear_reduction) + leg_angle_1; // see cd.common_leg_pos_min

const float min_hip_angle = hip_angle_0;
const float max_hip_angle = hip_angle_0+1.847749f; // see cd.hip_sensor_angle_range

// Each arm is controlled by 3 motors (A, B, C). B and C control sliders which have a variable distance
// between the point where the gear touches the slider and the point the slider connects with the
// lower arm. This distance depends on the angle the arm has in the plane that is controlled by the slider.
// These variables define that length depending on the angle (0° and 40°).
// Note that this is not a linear relationship
const float slider_b_length_0  = 83.0f;
const float slider_b_length_40 = 93.36f;
const float slider_c_length_0 = 63.5f;
const float slider_c_length_40 = 53.35f;

// Global variables for testing stuff, unused at the moment.
extern float test_delta, test_tresh;
extern float test_x, test_y, test_z;

// The following 2 structs define a "joint state" of the robot. It basically defines the
// angle of each joint and could be computed from all the sensor data.
// The units are a bit simpler here than the ones in md. Here everything
// is in radians and left and right side are symmetric. (This makes all the math, collision
// and visualization a bit simpler)
// The IK algo modifies basically all the members here with Gradient Descent to optimize
// an error function.
struct RobotJointStateSide
{
	float arm_a, arm_b, arm_c;
	float hip_angle, leg_angle;
	float wheel_angle; // only for visualization
};

struct RobotJointState
{
	float x_angle, side_angle; // These angles are determined by the hip and leg angles and are here only for visualization.
	float x_delta;
	RobotJointStateSide l, r; // left and right side
	float theta_for_ground_collision;
};

// This specifies what IK will optimize. It's basically filled out with how the user
// wants the joint space to look like. The error function compares these values with the
// joint space and the IK Algo will then optimize the joint space.
// side_angle for example specifies how far the robot should be leaning to the left or right.
// But this angle isn't controlled by a single motor. It depends on all leg and hip motors,
// so calculating those joints directly isn't that simple. But given a specific joint space,
// we can calculate the resulting leaning angle and then optimize based on an error function.
struct RobotIKTarget
{
	float t_correction_angle;
	float side_angle;
	float x_delta;
	float common_hip_angle, common_leg_angle;
	float l_arm_target_x, l_arm_target_y, l_arm_target_z;
	float r_arm_target_x = 180, r_arm_target_y = 80, r_arm_target_z = -140;
	bool enable_arm_mode; // If false, the arm target is ignored and the arm joints will only move to prevent an collision
	float theta_for_ground_collision;
};

void hip_wheel_vector_from_angles(float x_angle, float hip_angle, float leg_angle,
		float& x, float& z);
float leg_height_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor);
float leg_height_from_angles_falling(float x_angle, float hip_angle,
		float side_angle, float side_factor);
float leg_delta_from_angles(float x_angle, float hip_angle, float leg_angle);

// The "squat angle" is the angle the robot has to be in so that it won't fall forward or backwards.
// It depends on the joint state of the legs and hips.
// We simplify and ignore the arms for that because they are so light.
float calculate_squat_angle_from_angles(float hip_angle, float leg_angle, float* length_out = nullptr, float* debug_render_points_out = nullptr);
float calculate_squat_angle(float hip_pos, float leg_pos, float* length_out = nullptr);

// Calculate the imu sensor world position. This is needed to calculate the acceleration
// the robot impacts on the sensor and to compensate for that when calculating gravity acceleration direction.
void get_imu_sensor_position(float x_angle,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float& x, float& z);

// Center of Mass calculation.
// This also calculates the "Center of Inertia". This is a word I made up and represents the point
// around which the robot will rotate while balancing.
void get_com_world_position(float x_angle, float theta,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float* com_x_out, float* com_z_out,
		float* coi_x_out, float* coi_z_out,
		float* coi_theta_acc_out);

// This is the IK algo and will change s such that the spec in target is reached as best as possible.
// If multiple states can reach the same target, it will try to keep the state closer to the original state.
// In error_history, the error in each iteration can optionally be returned.
// If error_parts is supplied the individual parts of the final error can be returned.
float calculate_ik(RobotJointState& s, const RobotIKTarget& target, int iterations,
		std::vector<float>* error_history, float* error_parts);

// md contains for every joint a sensor value and the current target the motor is trying to each.
// This function allows you to convert either of those into a Joint State.
RobotJointState get_robot_joint_state(const MonitorData& md, bool from_sensor);

RobotIKTarget get_robot_ik_target(const MonitorData& md);

void draw_robot_3d(const RobotJointState& s, const RobotIKTarget* target, bool render);

void get_slider_lengths_from_arm_angles(float l_arm_b, float l_arm_c, float r_arm_b, float r_arm_c,
		float* l_slider_b, float* l_slider_c, float* r_slider_b, float* r_slider_c);

// This is called when changing the arm mode on the robot
void switch_arm_mode_smooth(MonitorData& md);

