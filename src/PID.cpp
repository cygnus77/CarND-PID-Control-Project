#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double Kp, double Ki, double Kd):Kp(Kp), Kd(Kd), Ki(Ki), d_error(0), i_error(0) {}

PID::~PID() {}

void PID::UpdateError(double cte) {
  i_error += cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
  n++;
}

double PID::TotalError() {
  return i_error/n;
}

