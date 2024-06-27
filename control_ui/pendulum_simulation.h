// control_ui has a simulation mode where basic robot movement can be simulated and compared to real data.
// This code is used for the simulation. It is basically the same as a double pendulum,
// but extended to 3 handles, a wheel at the bottom and also takes into account the 
// elasticity of the rubber on the wheels.
// Pretty much everything can be simulated, except jumping and 3D stuff like leaning or arms.
// (Extending this to jumping shouldn't be too hard. But extending the simulation to 3 dimensions
// might be computationally infeasable, not sure)
// I derived the code for the update function using the euler lagrangian method and did so
// using jupyter notebook with sympy for automatic differentiation. (Sympy offers similar features
// as MatLab and is free). I basically just had to provide functions for kinetic and potential energy
// and the rest is calculated by simpy.
// The resulting simulation matches the real robot very well and works with relatively large timesteps.
// It's also pretty fast (much faster than realtime).
#pragma once

// Triple pendulum on a wheel simulation
struct PendulumSimulation
{
	float gravity;
	float x1, x2, x3; // distance of com of each handle
	float l1, l2, l3; // handle length
	float m1, m2, m3; // mass of each handle
	float I1, I2, I3; // inertia
	float wheel_inertia;
	float wheel_radius;
	float wheel_mass;
	float rubber_inertia, rubber_factor;
 
	float wheel_pos, wheel_vel;
	float rubber_pos, rubber_vel;
	float theta1, theta2, theta3;
	float theta1_vel, theta2_vel, theta3_vel;
	
	void update(float dt, float wheel_torque, float joint_1_2_torque, float joint_2_3_torque);
};
