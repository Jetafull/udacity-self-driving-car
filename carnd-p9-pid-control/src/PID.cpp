#include "PID.h"

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  p_error = -Kp * cte;

  if (!initialized) {
    i_error = 0.0;
    cte_prev = cte;
    initialized = true;
  } else {
    i_error = -Ki * (cte - cte_prev);
  }

  cte_sum += cte;
  d_error = -Kd * cte_sum;
}

double PID::TotalError() { return p_error + i_error + d_error; }
