#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double kp;
  double ki;
  double kd;

  double v_;
  double kp0;
  double ki0;
  double kd0;

  double a_p;
  double a_i;
  double a_d;
  double frac;

  double cte, cte_prev, cte_mem;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double kp0, double ki0, double kd0, double a_p, double a_d, double a_i, double v, double frac);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void UpdateCoefficients(double kp0, double ki0, double kd0, double a_p, double a_i, double a_d, double v);

};

#endif /* PID_H */
