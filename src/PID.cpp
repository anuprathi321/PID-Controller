#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double i_Kp, double i_Ki, double i_Kd, double i_a_p, double i_a_i, double i_a_d, double i_v, double i_frac) {
	PID::UpdateCoefficients(i_Kp, i_Ki, i_Kd, i_a_p, i_a_i, i_a_d, i_v);
	frac = i_frac;
	p_error = 0;
	i_error = 0;
	d_error = 0;

	cte = 0;
	cte_prev = 0;
	cte_mem = 0;
}

void PID::UpdateError(double i_cte) {
	cte_prev = cte;
	cte = i_cte;
	cte_mem = frac*cte_mem + cte;
	
	//std::cout << ki << " " << cte_prev  << " " << frac << " " <<cte_mem << std::endl;

	p_error = kp*cte;
	i_error = ki*cte_mem;
	std::cout << "  d_E " << kd << " " << cte << " " << cte_prev << std::endl; 
	d_error = kd*(cte - cte_prev);
	return;
}

double PID::TotalError() {
	return   p_error + d_error + i_error;
}

void PID::UpdateCoefficients(double i_Kp, double i_Ki, double i_Kd, double i_a_p, double i_a_i, double i_a_d, double i_v)
{
	kp0 = i_Kp;
	ki0 = i_Ki;
	kd0 = i_Kd;

	a_p = i_a_p;
	a_i = i_a_i;
	a_d = i_a_d;

	v_ = i_v;

	kp = kp0 + a_p*v_;
	ki = ki0 + a_i*v_;
	kd = kd0 + a_d*v_;
}

