#include "PID.h"
#include <math.h>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID(double tau_p, double tau_i, double tau_d)
    : tau_p(tau_p), tau_i(tau_i), tau_d(tau_d) {
  cte_prev = 0.0;
  initialized = false;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

PID::~PID() {}

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
  return -tau_p * p_error - tau_i * i_error - tau_d * d_error;
}
