#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd):Kp(Kp), Kd(Kd), Ki(Ki), d_error(0), i_error(0), mse(0), n(0), prev_cte(0) {}

PID::~PID() {}

void PID::UpdateError(double cte) {
  i_error += cte;
  mse += cte*cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  n++;
}

double PID::TotalError() {
  return mse/n;
}

double PID::ComputeControl(double cte) {
  return -Kp * cte - Kd * d_error - Ki * i_error;
}

// Optimize - used only for twiddling
bool PID::Optimize()
{
  return false;
}

// Reset PID to initial state
void PID::Reset()
{
  mse = 0;
  n = 0;
  prev_cte = i_error = d_error = 0;
}