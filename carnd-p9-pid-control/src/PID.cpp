#include "PID.h"
#include <math.h>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
  cte_prev = 0.0;
  initialized = false;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  // p_error
  p_error = cte;

  // d_error update
  if (!initialized) {
    d_error = 0.0;
    cte_prev = cte;
    initialized = true;
  } else {
    d_error = cte - cte_prev;
    cte_prev = cte;
  }

  // i_error
  i_error += cte;
}

double PID::CalculateSteerValue() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
