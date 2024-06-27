#ifndef COMPILING_CONTROL_UI
// This somehow speeds Eigen on Jetson up. Not much, but significant.
#define EIGEN_DONT_VECTORIZE
#endif
#ifdef COMPILING_CONTROL_UI
#define INCLUDE_RENDER
#endif

#include "geometry.h"
#include "helper.h"
#include "../../3rdparty/eigen/Eigen/Geometry"
#include <vector>
#include <algorithm>
#ifdef INCLUDE_RENDER
#ifdef _MSC_VER
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif
#include <GL/gl.h>
#endif

#define DO_COLLISION_OPTIMIZATION

using Eigen::Vector3f;
using Eigen::Affine3f;
using Eigen::AngleAxisf;
using Eigen::Translation3f;

struct CollisionGeometry;

void draw_robot_3d(const RobotJointState& s, const RobotIKTarget* target, bool render, CollisionGeometry* collisions);

float test_delta = 10;
float test_tresh = 0;
float test_x, test_y, test_z;

extern ControlData cd;

// Oriented Bounding Box
struct OBB
{
	float pos[3]; // center of box
	float h_dim[3]; // half dimension
	Affine3f t_inv;

#ifdef DO_COLLISION_OPTIMIZATION
	Vector3f bounding_sphere_center;
	float bounding_sphere_radius_sq;
#endif
};

OBB make_obb(float x1, float x2, float y1, float y2, float z1, float z2, const Affine3f& t)
{
	OBB obb;
	obb.pos[0] = (x1+x2)*0.5f;
	obb.pos[1] = (y1+y2)*0.5f;
	obb.pos[2] = (z1+z2)*0.5f;
	obb.h_dim[0] = (x2-x1)*0.5f;
	obb.h_dim[1] = (y2-y1)*0.5f;
	obb.h_dim[2] = (z2-z1)*0.5f;

	// We only apply rotations and translations, nothing that changes distances.
	// That means we can optimize the matrix inverse by a lot by using the transpose
	// on the rotation part.
	// In Eigen we can simply supply Eigen::Isometry to do this.
	obb.t_inv = t.inverse(Eigen::Isometry);

#ifdef DO_COLLISION_OPTIMIZATION
	obb.bounding_sphere_center = t*Vector3f(obb.pos[0], obb.pos[1], obb.pos[2]);
	obb.bounding_sphere_radius_sq = sq(obb.h_dim[0]) + sq(obb.h_dim[1]) + sq(obb.h_dim[2]);
#endif
	return obb;
}

// Checks whether a point and an oriented box collide.
// If no, returns 0.
// If yes, returns distance of the point to the closest surface of the box.
float point_box_intrusion(const Vector3f& point, const OBB& obb)
{
#ifdef DO_COLLISION_OPTIMIZATION
	float dist_sq = (point - obb.bounding_sphere_center).squaredNorm();
	if (dist_sq >= obb.bounding_sphere_radius_sq)
		return 0;
#endif
	Vector3f p = obb.t_inv*point;
	float dist[3];
	for (int i = 0; i < 3; i++)
	{
		if (p[i] < obb.pos[i]) dist[i] = p[i] - (obb.pos[i]-obb.h_dim[i]);
		else                   dist[i] = (obb.pos[i]+obb.h_dim[i]) - p[i];
	}
	return std::max(std::min(dist[0], std::min(dist[1], dist[2])), 0.0f);
}

// Oriented Cylinder
// Center before transformation is assumed to be in origin
// The axis is along the y-axis.
struct OCylinder
{
	float radius;
	float h_dim; // half dimension along y-axis
	Affine3f t_inv;

#ifdef DO_COLLISION_OPTIMIZATION
	Vector3f bounding_sphere_center;
	float bounding_sphere_radius_sq;
#endif
};

OCylinder make_o_cylinder(float radius, float h_dim, const Affine3f& t)
{
	OCylinder o;
	o.radius = radius;
	o.h_dim = h_dim;
	o.t_inv = t.inverse(Eigen::Isometry);
#ifdef DO_COLLISION_OPTIMIZATION
	o.bounding_sphere_center = t*Vector3f(0, 0, 0);
	o.bounding_sphere_radius_sq = sq(h_dim) + sq(radius);
#endif
	return o;
}

// Checks whether a point and an oriented cylinder collide.
// If no, returns 0.
// If yes, returns distance of the point to the closest surface of the cylinder.
float point_cylinder_intrusion(const Vector3f& point, const OCylinder& c)
{
#ifdef DO_COLLISION_OPTIMIZATION
	float dist_sq = (point - c.bounding_sphere_center).squaredNorm();
	if (dist_sq >= c.bounding_sphere_radius_sq)
		return 0;
#endif
	Vector3f p = c.t_inv*point;
	float axial_dist = c.h_dim - abs(p[1]);
	if (axial_dist <= 0) return 0;
	float radial_dist = c.radius - sqrt(sq(p[0])+sq(p[2]));
	return std::max(std::min(radial_dist, axial_dist), 0.0f);
}

struct CollisionGeometry
{
	std::vector<OBB> boxes;
	std::vector<OCylinder> cylinders;
	std::vector<Vector3f> arm_points;
	
	// 0: right, 1: left
	Vector3f arm_end_pos[2];
	float slider_b[2]; // slider length
	float slider_c[2];
};

// Calculates the vector between the hip and wheel in 2D from the side and gravity in z-direction
void hip_wheel_vector_from_angles(float x_angle, float hip_angle, float leg_angle,
		float& x, float& z)
{
	x = 0;
	z = 0;

	x += sin(x_angle + hip_angle) * upper_leg_length;
	z += cos(x_angle + hip_angle) * upper_leg_length;

	x += sin(x_angle + hip_angle + leg_angle) * lower_leg_length;
	z += cos(x_angle + hip_angle + leg_angle) * lower_leg_length;
}

static float leg_height_plus_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor)
{
	float joint_z;
	joint_z =  cos(x_angle + hip_angle) * upper_leg_length;
	joint_z += cos(x_angle + hip_angle + leg_angle) * lower_leg_length;
	joint_z += wheel_radius;
	joint_z = cos(side_angle) * joint_z + sin(side_angle) * cd.wheel_side_distance* 0.5f * side_factor;
	return joint_z;
}

float leg_height_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor)
{
	float x;
	x =  cos(x_angle + hip_angle) * upper_leg_length;
	x += cos(x_angle + hip_angle + leg_angle) * lower_leg_length;
	x = cos(side_angle) * x + sin(side_angle) * cd.wheel_side_distance * 0.5f * side_factor;
	return x;
}

float leg_height_from_angles_falling(float x_angle, float hip_angle,
		float side_angle, float side_factor)
{
	float x;
	x =  cos(x_angle + hip_angle) * upper_leg_length;
	x += cos(cd.fall_vertical_angle) * lower_leg_length;
	x = cos(side_angle) * x + sin(side_angle) * cd.wheel_side_distance * 0.5f * side_factor;
	return x;
}

static float leg_y_plus_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor)
{
	float joint_y;
	joint_y =  cos(x_angle + hip_angle) * upper_leg_length;
	joint_y += cos(x_angle + hip_angle + leg_angle) * lower_leg_length;
	joint_y += wheel_radius;
	joint_y = -sin(side_angle) * joint_y + cos(side_angle) * cd.wheel_side_distance* 0.5f * side_factor;
	return joint_y;
}

/*static float leg_y_from_angles(float x_angle, float hip_angle, float leg_angle,
		float side_angle, float side_factor)
{
	float joint_y;
	joint_y =  cos(x_angle + hip_angle) * upper_leg_length;
	joint_y += cos(x_angle + hip_angle + leg_angle) * lower_leg_length;
	joint_y = sin(side_angle) * joint_y + cos(side_angle) * cd.wheel_side_distance* 0.5f * side_factor;
	return joint_y;
}*/

// only used in balance control
float leg_delta_from_angles(float x_angle, float hip_angle, float leg_angle)
{
	float joint_y;
	joint_y =  cos(x_angle + hip_angle)*upper_leg_length;
	joint_y += cos(x_angle + hip_angle + leg_angle)*lower_leg_length;
	return -joint_y;
}

// Calculates vector of com relative to wheels in 2D from the side view, assuming x_angle is 0.
static void calculate_com_from_angles(float hip_angle, float leg_angle,
		float* com_x_out, float* com_z_out, float* i_out, float* debug_render_points_out,
		float upper_leg_play, float lower_leg_play)
{
	float px0 = cd.squat_body_cog_x;
	float pz0 = cd.squat_body_cog_y;

	float px1 = 0;
	float pz1 = 0;

	float s = sin(hip_angle+cd.squat_hip_angle_bias), c = cos(hip_angle+cd.squat_hip_angle_bias);

	float px2 = px1 + c*(cd.squat_upper_leg_cog_x+upper_leg_play) - s*cd.squat_upper_leg_cog_y;
	float pz2 = pz1 + s*(cd.squat_upper_leg_cog_x+upper_leg_play) + c*cd.squat_upper_leg_cog_y;

	float px3 = px1 - s*(upper_leg_length);
	float pz3 = pz1 + c*(upper_leg_length);

	s = sin(hip_angle+leg_angle+cd.squat_leg_angle_bias), c = cos(hip_angle+leg_angle+cd.squat_leg_angle_bias);

	float px4 = px3 + c*(cd.squat_lower_leg_cog_x+lower_leg_play) - s*cd.squat_lower_leg_cog_y;
	float pz4 = pz3 + s*(cd.squat_lower_leg_cog_x+lower_leg_play) + c*cd.squat_lower_leg_cog_y;

	float px5 = px3 - s*(lower_leg_length);
	float pz5 = pz3 + c*(lower_leg_length);

	float com_x = (px0 * cd.squat_body_mass + px2 * cd.squat_upper_leg_mass + px4 * cd.squat_lower_leg_mass) / (cd.squat_body_mass + cd.squat_upper_leg_mass + cd.squat_lower_leg_mass);
	float com_z = (pz0 * cd.squat_body_mass + pz2 * cd.squat_upper_leg_mass + pz4 * cd.squat_lower_leg_mass) / (cd.squat_body_mass + cd.squat_upper_leg_mass + cd.squat_lower_leg_mass);

	if (i_out)
	{
		// Using Parallel axis theorem to calculate inertia of body consisting of 3 parts:
		float i = 0;
		i += cd.squat_body_mass      * (sq(px0-com_x)+sq(pz0-com_z));
		i += cd.squat_upper_leg_mass * (sq(px2-com_x)+sq(pz2-com_z));
		i += cd.squat_lower_leg_mass * (sq(px4-com_x)+sq(pz4-com_z));
		*i_out = i;
	}

	if (debug_render_points_out)
	{
		debug_render_points_out[0]  = px0  ;
		debug_render_points_out[1]  = pz0  ;
		debug_render_points_out[2]  = px1  ;
		debug_render_points_out[3]  = pz1  ;
		debug_render_points_out[4]  = px2  ;
		debug_render_points_out[5]  = pz2  ;
		debug_render_points_out[6]  = px3  ;
		debug_render_points_out[7]  = pz3  ;
		debug_render_points_out[8]  = px4  ;
		debug_render_points_out[9]  = pz4  ;
		debug_render_points_out[10] = px5  ;
		debug_render_points_out[11] = pz5  ;
		debug_render_points_out[12] = com_x;
		debug_render_points_out[13] = com_z;
	}

	if (com_x_out)
	{
		*com_x_out = com_x-px5;
		*com_z_out = com_z-pz5;
	}
}


float calculate_squat_angle_from_angles(float hip_angle, float leg_angle, float* length_out, float* debug_render_points_out)
{
	float com_x, com_z;
	calculate_com_from_angles(hip_angle, leg_angle, &com_x, &com_z, nullptr, nullptr, 0, 0);
	float target_angle = atan2(-com_x+cd.squat_bias_x, -com_z+cd.squat_bias_y)-cd.squat_base_angle2;

	// First we calculate com without play, then we calculate gravitational influence
	// on the legs and estimate the play in the joints.
	// Then we calculate com again, including the estimated play.
	// Including this calculation significantly improves the target angle estimation.
	float upper_leg_play = sin(target_angle+hip_angle          )*cd.squat_upper_leg_play_factor;
	float lower_leg_play = sin(target_angle+hip_angle+leg_angle)*cd.squat_lower_leg_play_factor;
	calculate_com_from_angles(hip_angle, leg_angle, &com_x, &com_z, nullptr, debug_render_points_out, upper_leg_play, lower_leg_play);

	if (length_out) *length_out = sqrt(sq(com_x)+sq(com_z));
	return atan2(-com_x+cd.squat_bias_x, -com_z+cd.squat_bias_y)-cd.squat_base_angle2;
}

float calculate_squat_angle(float hip_pos, float leg_pos, float* length_out)
{
	float hip_angle = hip_angle_0+cd.hip_sensor_angle_range*hip_pos;
	float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	return calculate_squat_angle_from_angles(hip_angle, leg_angle, length_out);
}

// Get x position of wheel relative to fixed point in world
float get_wheel_world_pos(float x_angle, float hip_angle, float leg_angle, float foot_pos)
{
	float x = foot_pos * tau * wheel_radius;
	x += (x_angle + hip_angle + leg_angle)*wheel_radius;
	return x;
}

// Get the imu sensor position relative to a fixed point on the ground
void get_imu_sensor_position(float x_angle,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float& x, float& z)
{
	float hip_angle = (hip_angle_l+hip_angle_r)*0.5f;
	float leg_angle = (leg_angle_l+leg_angle_r)*0.5f;
	float foot_pos = (foot_pos_l+foot_pos_r)*0.5f;

	float x_ = 0, z_ = -140; // imu position in body relative to hip axis
	x = x_*cos(x_angle) - z_*sin(x_angle);
	z = x_*sin(x_angle) + z_*cos(x_angle);

	float wheel_x, wheel_z;
	hip_wheel_vector_from_angles(x_angle, hip_angle, leg_angle, wheel_x, wheel_z);

	x += wheel_x;
	z -= wheel_z;

	x += get_wheel_world_pos(x_angle, hip_angle, leg_angle, foot_pos);
}

// Get com position relative to a fixed point on the ground
void get_com_world_position(float x_angle, float theta,
		float hip_angle_l, float leg_angle_l, float foot_pos_l,
		float hip_angle_r, float leg_angle_r, float foot_pos_r,
		float* com_x_out, float* com_z_out,
		float* coi_x_out, float* coi_z_out,
		float* coi_theta_acc_out)
{
	float hip_angle = (hip_angle_l+hip_angle_r)*0.5f;
	float leg_angle = (leg_angle_l+leg_angle_r)*0.5f;
	float foot_pos = (foot_pos_l+foot_pos_r)*0.5f;

	float x_, z_; // com position relative to wheel, without x_angle
	float I_body;
	calculate_com_from_angles(hip_angle, leg_angle, &x_, &z_, &I_body, nullptr, 0, 0);
	float com_x = x_*cos(x_angle) - z_*sin(x_angle);
	float com_z = x_*sin(x_angle) + z_*cos(x_angle);

	float l = sqrt(sq(com_x)+sq(com_z));

	float r = wheel_radius;
	//float g = earth_acc*1000;
	float m = cd.squat_body_mass + cd.squat_upper_leg_mass + cd.squat_lower_leg_mass;
	float I_wheel = cd.squat_wheel_inertia*1000000 / (l*m*r);
	I_body += cd.squat_whole_body_inertia*1000000;
	I_body /= l*m*r;
	float m_wheel_factor = cd.squat_wheel_mass/m + 1;

	float coi_factor = (I_body*r+l+r) / (l*I_wheel+m_wheel_factor*r+l);

	float coi_x = com_x * coi_factor;
	float coi_z = com_z * coi_factor;

	float x_delta = get_wheel_world_pos(x_angle, hip_angle, leg_angle, foot_pos);
	com_x += x_delta;
	coi_x += x_delta;

	if (com_x_out) *com_x_out = com_x;
	if (com_z_out) *com_z_out = com_z;
	if (coi_x_out) *coi_x_out = coi_x;
	if (coi_z_out) *coi_z_out = coi_z;

	if (coi_theta_acc_out)
		*coi_theta_acc_out = sin(theta) / (cos(theta) + I_wheel + m_wheel_factor*r/l);

}

float my_norm(float x, float scale)
{
	float abs_x = abs(x);
	if (abs_x < scale)
		return x*x;
	else
		return (2*abs_x-scale)*scale;
}

float ik_variables_error(const float opt_vars[num_ik_opt_vars], const RobotIKTarget& target,
		float* error_parts, RobotJointState* state_out)
{
	RobotJointState s = {0};
	s.x_delta 	  = opt_vars[0];
	s.l.hip_angle = opt_vars[1];
	s.r.hip_angle = opt_vars[2];
	s.l.leg_angle = opt_vars[3];
	s.r.leg_angle = opt_vars[4];
	s.r.arm_a 	  = opt_vars[5];
	s.r.arm_b 	  = opt_vars[6];
	s.r.arm_c 	  = opt_vars[7];
	s.l.arm_a 	  = opt_vars[8];
	s.l.arm_b 	  = opt_vars[9];
	s.l.arm_c 	  = opt_vars[10];

	float error = 0;

	{
		float target_angle_l = calculate_squat_angle_from_angles(s.l.hip_angle, s.l.leg_angle);
		float target_angle_r = calculate_squat_angle_from_angles(s.r.hip_angle, s.r.leg_angle);
		float target_angle = (target_angle_l + target_angle_r) * 0.5f;
		s.x_angle = target_angle + target.t_correction_angle;
	}

	float right_x, right_y;
	{
		hip_wheel_vector_from_angles(s.x_angle, s.r.hip_angle, s.r.leg_angle, right_x, right_y);
		error += my_norm(s.r.hip_angle - target.common_hip_angle, cd.ik_optimizer_error_angle_scale)*cd.ik_optimizer_error_angle_factor;
		error += my_norm(s.r.leg_angle - target.common_leg_angle, cd.ik_optimizer_error_angle_scale)*cd.ik_optimizer_error_angle_factor;
	}
	float left_x, left_y;
	{
		hip_wheel_vector_from_angles(s.x_angle, s.l.hip_angle, s.l.leg_angle, left_x, left_y);
		error += my_norm(s.l.hip_angle - target.common_hip_angle, cd.ik_optimizer_error_angle_scale)*cd.ik_optimizer_error_angle_factor;
		error += my_norm(s.l.leg_angle - target.common_leg_angle, cd.ik_optimizer_error_angle_scale)*cd.ik_optimizer_error_angle_factor;
	}
	if (error_parts) error_parts[0] = error;

	error += sq(right_x-left_x)*cd.ik_optimizer_error_wheel_x_var_factor;
	if (error_parts) error_parts[1] = sq(right_x-left_x)*cd.ik_optimizer_error_wheel_x_var_factor;

	s.side_angle = atan((right_y-left_y)/cd.wheel_side_distance);
	float leaning_error = sq(s.side_angle - target.side_angle)*cd.ik_optimizer_error_leaning_angle_factor;
	leaning_error += sq(std::max(0.0f, abs(s.side_angle) - cd.leaning_max_angle))*cd.ik_optimizer_error_leaning_angle_factor*50;
	error += leaning_error;	
	if (error_parts) error_parts[2] = leaning_error;

	s.theta_for_ground_collision = target.theta_for_ground_collision;

	CollisionGeometry cg;
	draw_robot_3d(s, nullptr, false, &cg);
	float arm_error = 0;
	if (target.enable_arm_mode)
	{
		// We include the arm error only in arm mode.
		// That means the arms will stay still in non-arm mode, unless they would collide
		// with the robot body.
		arm_error += my_norm(cg.arm_end_pos[0][0]-target.r_arm_target_x, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
		arm_error += my_norm(cg.arm_end_pos[0][1]-target.r_arm_target_y, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
		arm_error += my_norm(cg.arm_end_pos[0][2]-target.r_arm_target_z, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
		arm_error += my_norm(cg.arm_end_pos[1][0]-target.l_arm_target_x, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
		arm_error += my_norm(cg.arm_end_pos[1][1]-target.l_arm_target_y, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
		arm_error += my_norm(cg.arm_end_pos[1][2]-target.l_arm_target_z, cd.ik_optimizer_error_arm_scale) * cd.ik_optimizer_error_arm_factor;
	}
	if (error_parts) error_parts[3] = arm_error;
	error += arm_error;

	error                          += sq(s.x_delta-target.x_delta) * cd.ik_optimizer_error_x_delta_factor;
	if (error_parts) error_parts[4] = sq(s.x_delta-target.x_delta) * cd.ik_optimizer_error_x_delta_factor;

	float collision_error = 0;
	for (const Vector3f& p : cg.arm_points)
	{
		for (int i = 0; i < cg.boxes.size(); i++)
			collision_error = std::max(collision_error, point_box_intrusion(p, cg.boxes[i]));

		for (int i = 0; i < cg.cylinders.size(); i++)
			collision_error = std::max(collision_error, point_cylinder_intrusion(p, cg.cylinders[i]));
	}
	collision_error = sq(collision_error);
	error += collision_error*cd.ik_optimizer_error_collision_factor;
	if (error_parts) error_parts[5] = collision_error;

	if (state_out) *state_out = s;

	return error;
}

RobotJointState get_robot_joint_state(const MonitorData& md, bool from_sensor)
{
	RobotJointState s;
	s.side_angle = from_sensor ? md.imu_side_angle : md.leaning_optimized_target_angle;
	s.x_angle = md.imu_x_angle;
	s.x_delta = md.ik_x_delta;

	s.l.hip_angle = hip_angle_0+cd.hip_sensor_angle_range * (from_sensor ? md.hip_sensor_pos[0] : md.hip_pos_target_ik[0]);
	float leg_pos = from_sensor ? md.leg_rel_angle[1]+md.leg_base_angle[1] : md.leg_pos_target_ik[1];
	s.l.leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;

	s.r.hip_angle = hip_angle_0+cd.hip_sensor_angle_range * (from_sensor ? md.hip_sensor_pos[1] : md.hip_pos_target_ik[1]);
	leg_pos = from_sensor ? -(md.leg_rel_angle[0]+md.leg_base_angle[0]) : md.leg_pos_target_ik[0];
	s.r.leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	
	s.r.wheel_angle = -md.foot_pos[1] * tau;
	s.l.wheel_angle =  md.foot_pos[0] * tau;

	s.l.arm_a = md.ik_l_arm_a;
	s.l.arm_b = md.ik_l_arm_b;
	s.l.arm_c = md.ik_l_arm_c;
	s.r.arm_a = md.ik_r_arm_a;
	s.r.arm_b = md.ik_r_arm_b;
	s.r.arm_c = md.ik_r_arm_c;

	s.theta_for_ground_collision = 0;

	return s;
}

RobotIKTarget get_robot_ik_target(const MonitorData& md)
{
	RobotIKTarget target;
	memset(&target, 0, sizeof(target));
	target.t_correction_angle = md.t_correction_angle;

	float hip_angle = hip_angle_0+cd.hip_sensor_angle_range*md.common_hip_pos_target;
	float leg_pos = md.common_leg_pos_target;
	float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	target.common_hip_angle = hip_angle;
	target.common_leg_angle = leg_angle;

	target.side_angle = md.leaning_target_angle_smooth;
	target.x_delta = md.ik_x_delta_target;
	
	target.r_arm_target_x = md.ik_arm_target_x;
	target.l_arm_target_x = md.ik_arm_target_x;
	target.r_arm_target_y = md.ik_arm_target_y;
	target.l_arm_target_y = -md.ik_arm_target_y;
	target.r_arm_target_z = md.ik_arm_target_z;
	target.l_arm_target_z = md.ik_arm_target_z;

	target.enable_arm_mode = md.ik_enable_arm_mode;

	target.theta_for_ground_collision = md.error_p;

	return target;
}

float calculate_ik(RobotJointState& s, const RobotIKTarget& target, int iterations,
		std::vector<float>* error_history, float* error_parts)
{
	float opt_vars[num_ik_opt_vars];
	opt_vars[0] = s.x_delta;      
	opt_vars[1] = s.l.hip_angle;
	opt_vars[2] = s.r.hip_angle;
	opt_vars[3] = s.l.leg_angle;
	opt_vars[4] = s.r.leg_angle;
	opt_vars[5] = s.r.arm_a;
	opt_vars[6] = s.r.arm_b;
	opt_vars[7] = s.r.arm_c;
	opt_vars[8] = s.l.arm_a;
	opt_vars[9] = s.l.arm_b;
	opt_vars[10] = s.l.arm_c;

	float epsilon = cd.ik_optimizer_epsilon;
	float alpha = cd.ik_optimizer_alpha;
	float lr = cd.ik_optimizer_lr;

	std::vector<float> momentum(num_ik_opt_vars, 0);
	std::vector<float> gradients(num_ik_opt_vars, 0);

	for (int i = 0; i < iterations; i++)
	{
		float error = ik_variables_error(opt_vars, target, nullptr, nullptr);
		if (error_history) error_history[0].push_back(error);
		for (int j = 0; j < num_ik_opt_vars; j++)
		{
			float old_var = opt_vars[j];
			opt_vars[j] += epsilon;
			float new_error = ik_variables_error(opt_vars, target, nullptr, nullptr);
			opt_vars[j] = old_var;
			gradients[j] = (new_error-error) / epsilon;

			if (error_history) error_history[j+1].push_back(old_var);
			//if (error_history) error_history[j+num_ik_opt_vars+1].push_back(gradients[j]/100);
		}


		for (int j = 0; j < num_ik_opt_vars; j++)
		{
			float delta = momentum[j]*alpha - gradients[j]*(j == 0 ? cd.ik_optimizer_x_delta_lr : lr);
			momentum[j] = delta;
			opt_vars[j] += delta;
			if (error_history) error_history[j+num_ik_opt_vars+1].push_back(momentum[j]);
		}

		opt_vars[1] = clamp(opt_vars[1], min_hip_angle, max_hip_angle);
		opt_vars[2] = clamp(opt_vars[2], min_hip_angle, max_hip_angle);
		opt_vars[3] = clamp(opt_vars[3], min_leg_angle, max_leg_angle);
		opt_vars[4] = clamp(opt_vars[4], min_leg_angle, max_leg_angle);
		opt_vars[5] = clamp(opt_vars[5], 0, to_rad(180));
		opt_vars[6] = clamp(opt_vars[6], to_rad(-40), to_rad(40));
		opt_vars[7] = clamp(opt_vars[7], to_rad(-40), to_rad(40));
		opt_vars[8] = clamp(opt_vars[8], 0, to_rad(180));
		opt_vars[9] = clamp(opt_vars[9], to_rad(-40), to_rad(40));
		opt_vars[10]= clamp(opt_vars[10],to_rad(-40), to_rad(40));
	}

	RobotJointState new_state;
	float error = ik_variables_error(opt_vars, target, error_parts, &new_state);

	new_state.l.wheel_angle = s.l.wheel_angle;
	new_state.r.wheel_angle = s.r.wheel_angle;
	s = new_state;

	return error;
}

#ifdef INCLUDE_RENDER
void draw_box(float x1, float x2, float y1, float y2, float z1, float z2)
{
	// Vertices information
	GLfloat vertices[] = {x2,y2,z2,  x1,y2,z2,  x1,y1,z2,  x2,y1,z2,   // (front)
					      x2,y2,z2,  x2,y1,z2,  x2,y1,z1,  x2,y2,z1,   // (right)
					      x2,y2,z2,  x2,y2,z1,  x1,y2,z1,  x1,y2,z2,   // (top)
					      x1,y2,z2,  x1,y2,z1,  x1,y1,z1,  x1,y1,z2,   // (left)
					      x1,y1,z1,  x2,y1,z1,  x2,y1,z2,  x1,y1,z2,   // (bottom)
					      x2,y1,z1,  x1,y1,z1,  x1,y2,z1,  x2,y2,z1 }; // (back)
	
	// Normal information
	GLfloat normals[72] = { 0, 0, 1,   0, 0, 1,   0, 0, 1,   0, 0, 1,   // (front)
					        1, 0, 0,   1, 0, 0,   1, 0, 0,   1, 0, 0,   // (right)
					        0, 1, 0,   0, 1, 0,   0, 1, 0,   0, 1, 0,   // (top)
					       -1, 0, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0, 0,   // (left)
					        0,-1, 0,   0,-1, 0,   0,-1, 0,   0,-1, 0,   // (bottom)
					        0, 0,-1,   0, 0,-1,   0, 0,-1,   0, 0,-1 }; // (back)

	// drawing cube
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glNormalPointer(GL_FLOAT, 0, normals);
	glVertexPointer(3, GL_FLOAT, 0, vertices);
	glDrawArrays(GL_QUADS, 0, 24);
	glDisableClientState(GL_VERTEX_ARRAY); 
	glDisableClientState(GL_NORMAL_ARRAY);
}

void draw_point_3d(const Vector3f& p, const Vector3f& color)
{
	float color2[] = { color[0], color[1], color[2], 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, color2);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	float size = 3;
	draw_box(p[0]-size, p[0]+size, p[1]-size, p[1]+size, p[2]-size, p[2]+size);

	glEnable(GL_DEPTH_TEST);
	draw_box(p[0]-size, p[0]+size, p[1]-size, p[1]+size, p[2]-size, p[2]+size);

	float diffuse[]  = { 0.9f, 0.9f, 0.9f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	glDisable(GL_CULL_FACE);
}
#endif

void draw_robot_arm_3d(const RobotJointStateSide& s, int side, Affine3f t,
		bool render, CollisionGeometry* collisions)
{
	// side == 0: right
	// side == 1: left

	if (side == 1)
	{
		t *= Eigen::Scaling(1.0f, -1.0f, 1.0f);
	}

	{
		// Upper arm
		t *= Translation3f(-2.84f, 98.33f, -94.2f);
		t *= AngleAxisf(to_rad(-3), Vector3f(1, 0, 0));
		t *= AngleAxisf(s.arm_a, Vector3f(0, 1, 0));
		float x1=-8.5, x2=8.5, y1=-3, y2=3, z1=0, z2=73;
#ifdef INCLUDE_RENDER
		if (render)
		{
			glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
		}
#endif
	}

	//Vector3f p             = t*Vector3f(test_x, test_y, test_z);
	Vector3f slider_b_gear = t*Vector3f(0, 16, -10);
	Vector3f slider_c_gear = t*Vector3f(16, 0, 9.5f);
#ifdef INCLUDE_RENDER
	if (render)
	{
		glLoadIdentity();
		draw_point_3d(slider_b_gear, Vector3f(0.9f, 0.0f, 0.0f));
		draw_point_3d(slider_c_gear, Vector3f(0.0f, 0.9f, 0.9f));
	}
#endif
	{
		// Lower arm
		t *= Translation3f(0, 0, 73);
		t *= AngleAxisf(s.arm_b, Vector3f(1, 0, 0));
		t *= AngleAxisf(s.arm_c, Vector3f(0, 1, 0));

		float x1=-8.5, x2=8.5, y1=-3, y2=3, z1=0, z2=85;
#ifdef INCLUDE_RENDER
		if (render)
		{
			glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
		}
#endif
	}

	//Vector3f p(test_x, test_y, test_z);
	Vector3f arm_end_pos    = t*Vector3f(0, 0, 85);
	Vector3f slider_b_joint = t*Vector3f(0, 16, 0);
	Vector3f slider_c_joint = t*Vector3f(16, 0, 0);

	if (collisions)
	{
		static const Vector3f points[] =
		{
			Vector3f(18.741989f, -7.210282f, 76.722290f-73),
			Vector3f(13.269242f, -7.127266f, 72.002884f-73),
			Vector3f(18.671135f, -5.081724f, 73.353241f-73),
			Vector3f(18.637650f, -7.208289f, 82.135414f-73),
			Vector3f(-8.749283f, -5.363447f, 85.271202f-73),

			Vector3f(8.7497420f, -2.5f     , 86.619133f-73),
			Vector3f(8.7497420f, -2.5f     , 90-73),
			Vector3f(8.7497420f, -2.5f     , 100-73),
			Vector3f(8.7497420f, -2.5f     , 110-73),
			Vector3f(8.7497420f, -2.5f     , 120-73),
			Vector3f(8.7497420f, -2.5f     , 130-73),
			Vector3f(8.7497420f, -2.5f     , 140-73),
			Vector3f(8.7497420f, -2.5f     , 150-73),

			Vector3f(8.7497420f,  3.5f     , 86.619133f-73),
			Vector3f(8.7497420f,  3.5f     , 90-73),
			Vector3f(8.7497420f,  3.5f     , 100-73),
			Vector3f(8.7497420f,  3.5f     , 110-73),
			Vector3f(8.7497420f,  3.5f     , 120-73),
			Vector3f(8.7497420f,  3.5f     , 130-73),
			Vector3f(8.7497420f,  3.5f     , 140-73),
			Vector3f(8.7497420f,  3.5f     , 150-73),

			Vector3f(-8.749353f, -2.5f     , 85.272797f-73),
			Vector3f(-8.749353f, -2.5f     , 90-73),
			Vector3f(-8.749353f, -2.5f     , 100-73),
			Vector3f(-8.749353f, -2.5f     , 110-73),
			Vector3f(-8.749353f, -2.5f     , 120-73),
			Vector3f(-8.749353f, -2.5f     , 130-73),
			Vector3f(-8.749353f, -2.5f     , 140-73),
			Vector3f(-8.749353f, -2.5f     , 150-73),

			Vector3f(-8.749353f,  3.5f     , 85.272797f-73),
			Vector3f(-8.749353f,  3.5f     , 90-73),
			Vector3f(-8.749353f,  3.5f     , 100-73),
			Vector3f(-8.749353f,  3.5f     , 110-73),
			Vector3f(-8.749353f,  3.5f     , 120-73),
			Vector3f(-8.749353f,  3.5f     , 130-73),
			Vector3f(-8.749353f,  3.5f     , 140-73),
			Vector3f(-8.749353f,  3.5f     , 150-73),

			Vector3f(-8        , 0         , 85),
			Vector3f( 8        , 0         , 85),
			Vector3f(0         ,  8        , 85),
			Vector3f(0         , -8        , 85),
			Vector3f(0         , 0         , 85+8),
		};
		
		for (int i = 0; i < sizeof(points) / sizeof(points[0]); i++)
			collisions->arm_points.push_back(t*points[i]);
	}

#ifdef INCLUDE_RENDER
	if (render)
	{
		glLoadIdentity();
		draw_point_3d(slider_b_joint, Vector3f(0.9f, 0.0f, 0.0f));
		draw_point_3d(slider_c_joint, Vector3f(0.0f, 0.9f, 0.9f));
		draw_point_3d(arm_end_pos, Vector3f(0.5f, 0.5f, 1.0f));
		//for (Vector3f p : collisions->arm_points)
		//	draw_point_3d(p, Vector3f(0.5f, 0.5f, 1.0f));
	}
#endif

	if (collisions) collisions->arm_end_pos[side] = arm_end_pos;
	if (collisions) collisions->slider_b[side] = (slider_b_joint-slider_b_gear).norm();
	if (collisions) collisions->slider_c[side] = (slider_c_joint-slider_c_gear).norm();
}

void draw_robot_leg_3d(const RobotJointStateSide& s, int leg, Affine3f t, bool render, CollisionGeometry* collisions)
{
	// leg == 0: right
	// leg == 1: left

	if (leg == 1) t *= Eigen::Scaling(1.0f, -1.0f, 1.0f);

	{
		// upper leg
		float x1 = -31, x2 = 31, y1=165.0f/2, y2=165.0f/2+10, z1 = -25, z2 = upper_leg_length;
		t *= AngleAxisf(-s.hip_angle, Vector3f(0, 1, 0));
		if (collisions) collisions->boxes.push_back(make_obb(x1, x2, y1, y2, z1, z2, t));
#ifdef INCLUDE_RENDER
		if (render)
		{
			glLoadMatrixf(t.data());
			{
				//float x1 = -32.28f*0.5f, x2 = 32.28f*0.5f, y1=165.0f/2, y2=165.0f/2+10, z1 = 0, z2 = upper_leg_length;
				draw_box(x1, x2, y1, y2, z1, z2);
			}
		}
#endif
	}

	{
		// lower leg
		t *= Translation3f(0, 0, upper_leg_length);
		t *= AngleAxisf(-s.leg_angle, Vector3f(0, 1, 0));
#ifdef INCLUDE_RENDER
		if (render)
		{
			glLoadMatrixf(t.data());
			float x1 = -17.5f, x2 = 17.5f, y1 = 62.5, y2 = 72.5, z1 = 0, z2 = lower_leg_length;
			draw_box(x1, x2, y1, y2, z1, z2);
		}
#endif
	}

	{
		// wheel
		t *= Translation3f(0, cd.wheel_side_distance/2, lower_leg_length);
		t *= AngleAxisf(-s.wheel_angle, Vector3f(0, 1, 0));

		if (collisions) collisions->cylinders.push_back(make_o_cylinder(wheel_radius, 8.2f, t));
		
#ifdef INCLUDE_RENDER
		if (render)
		{
			float r = sqrt(sq(wheel_radius)/2);
			float x1 = -r, x2 = r, y1 = -8.2f, y2 = 8.2f, z1 = -r, z2 = r;
			glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);

			t *= AngleAxisf(to_rad(15), Vector3f(0, 1, 0)); glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
			t *= AngleAxisf(to_rad(15), Vector3f(0, 1, 0)); glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
			t *= AngleAxisf(to_rad(15), Vector3f(0, 1, 0)); glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
			t *= AngleAxisf(to_rad(15), Vector3f(0, 1, 0)); glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
			t *= AngleAxisf(to_rad(15), Vector3f(0, 1, 0)); glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
			draw_point_3d({0, 0, 0}, {0.0f, 1.0f, 0.0f});
		}
#endif
	}
}

void draw_robot_3d(const RobotJointState& s, const RobotIKTarget* target, bool render, CollisionGeometry* collisions)
{
#ifdef INCLUDE_RENDER
	if (render) glPushMatrix();
#else
	// This path isn't compiled for the robot, rendering is just used in the Controller application.
	assert(!render);
#endif
	
	Affine3f t = Affine3f::Identity();

	{
		// draw floor
		float x1=-100, x2=100, y1=-100, y2=100, z1=0, z2=10;
		if (collisions) collisions->boxes.push_back(make_obb(x1, x2, y1, y2, z1, z2, t));
#ifdef INCLUDE_RENDER
		if (render) draw_box(x1, x2, y1, y2, z1, z2);
#endif
	}

	{
		// During IK calculation, x_angle is set to the target angle for the given joint state
		// (so the robot doesn't fall over in the final state).
		// However, the real robot can have a very different value for x_angle, especially during fast movement
		// and it's possible that the arms touch the floor due to this.
		// To prevent this, we supply theta_for_ground_collision so we have access to the real angle
		// here and add a rotated "fake floor" here that has the same relationship to the robot
		// as the real floor has to the real robot.
		// This way we detect collisions and the ik solver will move the arms away from the floor
		// to prevent damage.
		Affine3f t2 = t;
		t2 *= Translation3f(s.x_delta, 0, -wheel_radius);
		t2 *= AngleAxisf(s.theta_for_ground_collision, Vector3f(0, 1, 0));
		t2 *= Translation3f(-s.x_delta, 0, wheel_radius);
		float x1=-200, x2=300, y1=-200, y2=200, z1=-30, z2=50;
		if (collisions) collisions->boxes.push_back(make_obb(x1, x2, y1, y2, z1, z2, t2));
#ifdef INCLUDE_RENDER
		if (render)
		{
			float x1=-100, x2=100, y1=-100, y2=100, z1=0, z2=10;
			glLoadMatrixf(t2.data());
			draw_box(x1, x2, y1, y2, z1, z2);
		}
#endif
	}

	// Determine x and z position of body relative to ground
	{
		float l_wheel_x, l_wheel_y;
		float r_wheel_x, r_wheel_y;
		hip_wheel_vector_from_angles(s.x_angle, s.l.hip_angle, s.l.leg_angle, l_wheel_x, l_wheel_y);
		hip_wheel_vector_from_angles(s.x_angle, s.r.hip_angle, s.r.leg_angle, r_wheel_x, r_wheel_y);

		float delta_x = (l_wheel_x + r_wheel_x)*0.5f;
		//float delta_y = -std::max(l_wheel_y, r_wheel_y);

		delta_x += s.x_delta;

		float leg_left_z  = leg_height_plus_from_angles(s.x_angle, s.l.hip_angle, s.l.leg_angle, s.side_angle, 1);
		float leg_right_z = leg_height_plus_from_angles(s.x_angle, s.r.hip_angle, s.r.leg_angle, s.side_angle, -1);
		float delta_z = -std::max(leg_left_z, leg_right_z);

		float leg_left_y  = leg_y_plus_from_angles(s.x_angle, s.l.hip_angle, s.l.leg_angle, s.side_angle, 1);
		float leg_right_y = leg_y_plus_from_angles(s.x_angle, s.r.hip_angle, s.r.leg_angle, s.side_angle, -1);
		float delta_y = (leg_left_y + leg_right_y)*0.5f;

		t *= Translation3f(delta_x, delta_y, delta_z);
	}

	t *= AngleAxisf(-s.side_angle, Vector3f(1, 0, 0));
	t *= AngleAxisf(-s.x_angle, Vector3f(0, 1, 0));
	

	{
		// Draw body
		float x1=-31, x2=31, y1=-165.0f/2, y2=165.0f/2, z1=-169, z2=12.5;
		if (collisions) collisions->boxes.push_back(make_obb(x1, x2, y1, y2, z1, z2, t));
#ifdef INCLUDE_RENDER
		if (render)
		{
			glLoadMatrixf(t.data());
			draw_box(x1, x2, y1, y2, z1, z2);
		
			// Point in center of body
			draw_point_3d({0, 0, 0}, {1.0f, 1.0f, 0.0f});
		}
#endif
	}

	draw_robot_arm_3d(s.r, 0, t, render, collisions);
	draw_robot_arm_3d(s.l, 1, t, render, collisions);

	draw_robot_leg_3d(s.r, 0, t, render, collisions);
	draw_robot_leg_3d(s.l, 1, t, render, collisions);

#ifdef INCLUDE_RENDER
	if (render && target)
	{
		glLoadIdentity();
		draw_point_3d({target->r_arm_target_x, target->r_arm_target_y, target->r_arm_target_z}, {0.0f, 0.0f, 0.9f});
		draw_point_3d({target->l_arm_target_x, target->l_arm_target_y, target->l_arm_target_z}, {0.0f, 0.0f, 0.9f});
	}

	if (render && collisions && 0)
	{
		for (float x = -100; x < 300; x += test_delta)
		for (float y = -200; y < 200; y += test_delta)
		for (float z = -500; z < 15; z += test_delta)
		{
			Vector3f p(x, y, z);
			float intrusion = 0;
			for (int i = 0; i < collisions->boxes.size(); i++)
				intrusion = std::max(intrusion, point_box_intrusion(p, collisions->boxes[i]));
			if (intrusion > test_tresh)
			{
				draw_point_3d(p, Vector3f(0.9f, 0.0f, 0.9f));
			}

			intrusion = 0;
			for (int i = 0; i < collisions->cylinders.size(); i++)
				intrusion = std::max(intrusion, point_cylinder_intrusion(p, collisions->cylinders[i]));
			if (intrusion > test_tresh)
			{
				draw_point_3d(p, Vector3f(0.5f, 0.0f, 0.0f));
			}
		}
	}
#endif

#ifdef INCLUDE_RENDER
	if (render)
		glPopMatrix();
#endif
}

void draw_robot_3d(const RobotJointState& s, const RobotIKTarget* target, bool render)
{
	CollisionGeometry cg;
	draw_robot_3d(s, target, render, &cg);
}

void get_slider_lengths_from_arm_angles(float l_arm_b, float l_arm_c, float r_arm_b, float r_arm_c,
		float* l_slider_b, float* l_slider_c, float* r_slider_b, float* r_slider_c)
{
	RobotJointState s = {0};
	s.l.arm_b = l_arm_b;
	s.l.arm_c = l_arm_c;
	s.r.arm_b = r_arm_b;
	s.r.arm_c = r_arm_c;
	CollisionGeometry cg;
	draw_robot_3d(s, nullptr, false, &cg);
	*l_slider_b = cg.slider_b[1];
	*l_slider_c = cg.slider_c[1];
	*r_slider_b = cg.slider_b[0];
	*r_slider_c = cg.slider_c[0];
}

void switch_arm_mode_smooth(MonitorData& md)
{
	md.ik_enable_arm_mode = !md.ik_enable_arm_mode;
	if (md.ik_enable_arm_mode)
	{
		// Set arm target to where the arms are before switching to arm mode
		// to prevent sudden movement
		RobotJointState s = get_robot_joint_state(md, false);
		CollisionGeometry cg;
		draw_robot_3d(s, nullptr, false, &cg);
		md.ik_arm_target_x = (cg.arm_end_pos[0][0]+cg.arm_end_pos[1][0])*0.5f;
		md.ik_arm_target_y = (cg.arm_end_pos[0][1]-cg.arm_end_pos[1][1])*0.5f;
		md.ik_arm_target_z = (cg.arm_end_pos[0][2]+cg.arm_end_pos[1][2])*0.5f;
	}
	else
	{
		// hip and leg positions can be very different from target in arm mode.
		// We set the target to previous ik positions to prevent sudden movement.
		md.common_hip_pos_target = (md.hip_pos_target_ik[0]+md.hip_pos_target_ik[1])*0.5f;
		md.common_leg_pos_target = (md.leg_pos_target_ik[0]+md.leg_pos_target_ik[1])*0.5f;
	}
}

/*
Links:

Coordinate system:
https://en.wikipedia.org/wiki/Axes_conventions
https://en.wikipedia.org/wiki/Aircraft_principal_axes

Eigen geometry: https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html

Collision matrix links: https://www.realtimerendering.com/intersections.html

Explanation OBB-OBB collision algo: https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat

Simple collision algo in C++: https://stackoverflow.com/a/52010428

Complicated C++ Algo: https://github.com/juj/MathGeoLib/blob/master/src/Geometry/OBB.cpp#L2445

Older implementation + desc: https://web.archive.org/web/19991129035017/http://www.gamasutra.com/features/19991018/Gomez_5.htm

Paper: https://www.geometrictools.com/Documentation/DynamicCollisionDetection.pdf
*/
