#include "pendulum_simulation.h"
#include <math.h>
#include "../3rdparty/eigen/Eigen/Geometry"

// Multiple versions of the simulation code are available
// The cart pole versions are much simpler and simplify the robot to a single rod on a wheel.

//#define MODEL_VERSION 0 // cart pole
//#define MODEL_VERSION 1 // cart pole with rubber
//#define MODEL_VERSION 2 // 3 handles
#define MODEL_VERSION 3 // 3 handles with rubber

static float sq(float v) { return v*v; }

#if 1
// https://stackoverflow.com/a/35442885
void getsub(double* sub, double* mat, double* vec)
{
	*(sub++) = *(mat  ) * *(mat+5) - *(mat+1) * *(mat+4);
	*(sub++) = *(mat  ) * *(mat+6) - *(mat+2) * *(mat+4);
	*(sub++) = *(mat  ) * *(mat+7) - *(mat+3) * *(mat+4);
	*(sub++) = *(mat  ) * *(vec+1) - *(vec  ) * *(mat+4);
	*(sub++) = *(mat+1) * *(mat+6) - *(mat+2) * *(mat+5);
	*(sub++) = *(mat+1) * *(mat+7) - *(mat+3) * *(mat+5);
	*(sub++) = *(mat+1) * *(vec+1) - *(vec  ) * *(mat+5);
	*(sub++) = *(mat+2) * *(mat+7) - *(mat+3) * *(mat+6);
	*(sub++) = *(mat+2) * *(vec+1) - *(vec  ) * *(mat+6);
	*(sub  ) = *(mat+3) * *(vec+1) - *(vec  ) * *(mat+7);
}
void solver_4D(double* mat, double* vec)
{
    double a[10], b[10]; // values of 20 specific 2D subdeterminants

    getsub(a, mat, vec);
    getsub(b, mat+8, vec+2);

    *(vec++) = a[5]*b[8] + a[8]*b[5] - a[6]*b[7] - a[7]*b[6] - a[4]*b[9] - a[9]*b[4];
    *(vec++) = a[1]*b[9] + a[9]*b[1] + a[3]*b[7] + a[7]*b[3] - a[2]*b[8] - a[8]*b[2];
    *(vec++) = a[2]*b[6] + a[6]*b[2] - a[0]*b[9] - a[9]*b[0] - a[3]*b[5] - a[5]*b[3];
    *(vec  ) = a[0]*b[8] + a[8]*b[0] + a[3]*b[4] + a[4]*b[3] - a[6]*b[1] - a[1]*b[6];

    double idet = 1./(a[0]*b[7] + a[7]*b[0] + a[2]*b[4] + a[4]*b[2] - a[5]*b[1] - a[1]*b[5]);

    *(vec--) *= idet;
    *(vec--) *= idet;
    *(vec--) *= idet;
    *(vec  ) *= idet;
}
#else

Eigen::Vector4d solveDirect(const Eigen::Matrix4d& A, const Eigen::Vector4d& b)
{
    Eigen::Matrix4d inv = A.inverse();
    Eigen::Vector4d x = inv * b;
    x += inv*(b-A*x);
    return x;
}

void solver_4D(double* mat, double* vec)
{
	Eigen::Matrix4d A(mat);
	Eigen::Vector4d b(vec);
	b = solveDirect(A, b);
	vec[0] = b[0];
	vec[1] = b[1];
	vec[2] = b[2];
	vec[3] = b[3];
}
#endif

template<int N>
Eigen::Matrix<double, N, 1> solveDirect(const Eigen::Matrix<double, N, N>& A, const Eigen::Matrix<double, N, 1>& b)
{
    Eigen::Matrix<double, N, N> inv = A.inverse();
    Eigen::Matrix<double, N, 1> x = inv * b;
    x += inv*(b-A*x);
    return x;
}

void PendulumSimulation::update(float dt, float wheel_torque, float joint_1_2_torque, float joint_2_3_torque)
{
	float g = gravity;
	float r = wheel_radius;

#if MODEL_VERSION == 0 || MODEL_VERSION == 1
	float m = m1+m2+m3;

	float px0 = 0;
	float py0 = 0;

	float px1 = px0 + sin(theta1)*x1;
	float py1 = py0 + cos(theta1)*x1;

	float px2 = px0 + sin(theta1)*l1;
	float py2 = py0 + cos(theta1)*l1;

	float px3 = px2 + sin(theta2)*x2;
	float py3 = py2 + cos(theta2)*x2;

	float px4 = px2 + sin(theta2)*l2;
	float py4 = py2 + cos(theta2)*l2;

	float px5 = px4 + sin(theta3)*x3;
	float py5 = py4 + cos(theta3)*x3;

	float com_x = (px5 * m3 + px3 * m2 + px1 * m1) / m;
	float com_y = (py5 * m3 + py3 * m2 + py1 * m1) / m;
	float theta = atan2(com_x, com_y);
	float l = sqrt(sq(com_x)+sq(com_y));

#if MODEL_VERSION == 0
	double M[4] =
	{
		wheel_inertia+(wheel_mass+m)*r*r, l*m*r*cos(theta),
		l*m*r*cos(theta)                , I1+l*l*m
	};

	double f[2] =
	{
		sq(theta1_vel)*l*m*r*sin(theta),
		g*l*m*sin(theta)
	};
	f[0] += wheel_torque;
	f[1] -= wheel_torque;

	Eigen::Vector2d b(f);
	b = solveDirect<2>(Eigen::Matrix2d(M), b);

	wheel_pos  += wheel_vel*dt;
	theta1     += theta1_vel*dt;
	theta2     += theta1_vel*dt;
	theta3     += theta1_vel*dt;
	wheel_vel  += (float)b[0]*dt;
	theta1_vel += (float)b[1]*dt;
	theta2_vel = theta1_vel;
	theta3_vel = theta1_vel;

	rubber_pos = wheel_pos;
	rubber_vel = wheel_vel;
#else
	double M[9] =
	{
		rubber_inertia+(wheel_mass+m)*r*r, 0            , l*m*r*cos(theta),
		0                                , wheel_inertia, 0               ,
		l*m*r*cos(theta)                 , 0            , I1+l*l*m
	};

	double f[3] =
	{
		sq(theta1_vel)*l*m*r*sin(theta)-rubber_factor*(rubber_pos-wheel_pos),
		rubber_factor*(rubber_pos-wheel_pos),
		g*l*m*sin(theta)
	};
	f[1] += wheel_torque;
	f[2] -= wheel_torque;

	Eigen::Vector3d b(f);
	b = solveDirect<3>(Eigen::Matrix3d(M), b);

	rubber_pos += rubber_vel*dt;
	wheel_pos  += wheel_vel*dt;
	theta1     += theta1_vel*dt;
	theta2     += theta1_vel*dt;
	theta3     += theta1_vel*dt;
	rubber_vel += (float)b[0]*dt;
	wheel_vel  += (float)b[1]*dt;
	theta1_vel += (float)b[2]*dt;
	theta2_vel = theta1_vel;
	theta3_vel = theta1_vel;
#endif

#elif MODEL_VERSION == 2

	double M[16] =
	{
		wheel_inertia+(wheel_mass+m1+m2+m3)*r*r, r*(l1*m2+l1*m3+m1*x1)*cos(theta1)      , r*(l2*m3 + m2*x2)*cos(theta2)          , m3*r*x3*cos(theta3)          ,
		r*(l1*m2+l1*m3+m1*x1)*cos(theta1)      , I1 + l1*l1*m2 + l1*l1*m3 + m1*x1*x1    , l1*(l2*m3 + m2*x2)*cos(theta1 - theta2), l1*m3*x3*cos(theta1 - theta3),
		r*(l2*m3 + m2*x2)*cos(theta2)          , l1*(l2*m3 + m2*x2)*cos(theta1 - theta2), I2 + l2*l2*m3 + m2*x2*x2               , l2*m3*x3*cos(theta2 - theta3),
		m3*r*x3*cos(theta3)                    , l1*m3*x3*cos(theta1 - theta3)          , l2*m3*x3*cos(theta2 - theta3)          , I3 + m3*x3*x3                
	};

	double f[4] =
	{
		r*(sq(theta1_vel)*m1*x1*sin(theta1) + m2*(sq(theta1_vel)*l1*sin(theta1)+sq(theta2_vel)*x2*sin(theta2)) + m3*(sq(theta1_vel)*l1*sin(theta1)+sq(theta2_vel)*l2*sin(theta2)+sq(theta3_vel)*x3*sin(theta3))),
		-sq(theta2_vel)*l1*l2*m3*sin(theta1 - theta2) - sq(theta2_vel)*l1*m2*x2*sin(theta1 - theta2) - sq(theta3_vel)*l1*m3*x3*sin(theta1 - theta3) + gravity*l1*m2*sin(theta1) + gravity*l1*m3*sin(theta1) + g*m1*x1*sin(theta1),
        sq(theta1_vel)*l1*l2*m3*sin(theta1 - theta2) + sq(theta1_vel)*l1*m2*x2*sin(theta1 - theta2) - sq(theta3_vel)*l2*m3*x3*sin(theta2 - theta3) + gravity*l2*m3*sin(theta2) + gravity*m2*x2*sin(theta2),
        m3*x3*(sq(theta1_vel)*l1*sin(theta1 - theta3) + sq(theta2_vel)*l2*sin(theta2 - theta3) + g*sin(theta3))
	};
	f[0] += wheel_torque;
	f[1] -= wheel_torque;
	f[1] += joint_1_2_torque;
	f[2] -= joint_1_2_torque;
	f[2] += joint_2_3_torque;
	f[3] -= joint_2_3_torque;

	solver_4D(M, f);
		
	wheel_pos  += wheel_vel*dt;
	theta1     += theta1_vel*dt;
	theta2     += theta2_vel*dt;
	theta3     += theta3_vel*dt;
	wheel_vel  += (float)f[0]*dt;
	theta1_vel += (float)f[1]*dt;
	theta2_vel += (float)f[2]*dt;
	theta3_vel += (float)f[3]*dt;

	rubber_pos = wheel_pos;
	rubber_vel = wheel_vel;

#elif MODEL_VERSION == 3

	double M[25] =
	{
		rubber_inertia+(wheel_mass+m1+m2+m3)*r*r, 0             , r*(l1*m2+l1*m3+m1*x1)*cos(theta1)      , r*(l2*m3 + m2*x2)*cos(theta2)          , m3*r*x3*cos(theta3)          ,
		0                                       ,  wheel_inertia, 0                                      , 0                                      , 0                            ,
		r*(l1*m2+l1*m3+m1*x1)*cos(theta1)       , 0             , I1 + l1*l1*m2 + l1*l1*m3 + m1*x1*x1    , l1*(l2*m3 + m2*x2)*cos(theta1 - theta2), l1*m3*x3*cos(theta1 - theta3),
		r*(l2*m3 + m2*x2)*cos(theta2)           , 0             , l1*(l2*m3 + m2*x2)*cos(theta1 - theta2), I2 + l2*l2*m3 + m2*x2*x2               , l2*m3*x3*cos(theta2 - theta3),
		m3*r*x3*cos(theta3)                     , 0             , l1*m3*x3*cos(theta1 - theta3)          , l2*m3*x3*cos(theta2 - theta3)          , I3 + m3*x3*x3                
	};

	double f[5] =
	{
		sq(theta1_vel)*m1*r*x1*sin(theta1) - rubber_factor*(rubber_pos-wheel_pos) + m2*r*(sq(theta1_vel)*l1*sin(theta1)+sq(theta2_vel)*x2*sin(theta2)) + m3*r*(sq(theta1_vel)*l1*sin(theta1)+sq(theta2_vel)*l2*sin(theta2)+sq(theta3_vel)*x3*sin(theta3)),
		rubber_factor*(rubber_pos-wheel_pos),
		-sq(theta2_vel)*l1*l2*m3*sin(theta1 - theta2) - sq(theta2_vel)*l1*m2*x2*sin(theta1 - theta2) - sq(theta3_vel)*l1*m3*x3*sin(theta1 - theta3) + gravity*l1*m2*sin(theta1) + gravity*l1*m3*sin(theta1) + g*m1*x1*sin(theta1),
        sq(theta1_vel)*l1*l2*m3*sin(theta1 - theta2) + sq(theta1_vel)*l1*m2*x2*sin(theta1 - theta2) - sq(theta3_vel)*l2*m3*x3*sin(theta2 - theta3) + gravity*l2*m3*sin(theta2) + gravity*m2*x2*sin(theta2),
        m3*x3*(sq(theta1_vel)*l1*sin(theta1 - theta3) + sq(theta2_vel)*l2*sin(theta2 - theta3) + g*sin(theta3))
	};
	f[1] += wheel_torque;
	f[2] -= wheel_torque;
	f[2] += joint_1_2_torque;
	f[3] -= joint_1_2_torque;
	f[3] += joint_2_3_torque;
	f[4] -= joint_2_3_torque;

	Eigen::Matrix<double, 5, 1> b(f);
	b = solveDirect<5>(Eigen::Matrix<double, 5, 5>(M), b);

	rubber_pos += rubber_vel*dt;
	wheel_pos  += wheel_vel*dt;
	theta1     += theta1_vel*dt;
	theta2     += theta2_vel*dt;
	theta3     += theta3_vel*dt;
	rubber_vel += (float)b[0]*dt;
	wheel_vel  += (float)b[1]*dt;
	theta1_vel += (float)b[2]*dt;
	theta2_vel += (float)b[3]*dt;
	theta3_vel += (float)b[4]*dt;
#else
#error
#endif
}

