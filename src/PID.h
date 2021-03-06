#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Cache
  */
  double prev_cte;
  double mse;
  int n; // number of updates

  /*
  * Constructor
  */
  PID(double Kp, double Ki, double Kd);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Reset PID to initial state
  */
  void Reset();

  /*
  * Optimize - used for twiddling
  */
  virtual bool Optimize();

  /*
  * Compute control value
  */
  virtual double ComputeControl(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
