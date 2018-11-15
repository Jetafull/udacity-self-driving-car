#ifndef pid_twiddle_H
#define pid_twiddle_H

#include "controller.h"

struct TauDeltas {
  double tau_delta_p;
  double tau_delta_i;
  double tau_delta_d;
};

class PIDTwiddle : public Controller {
 public:
  // Errors
  double p_error;
  double i_error;
  double d_error;

  // tau
  Taus taus;

  // Constructor
  PIDTwiddle(Taus, TauDeltas);

  // Destructor
  virtual ~PIDTwiddle();

  // Update the PID error variables given cross track error.
  void UpdateError(double cte);
  double CalculateSteerValue();

 private:
  double cte_prev;
  bool initialized;
};

#endif /* pid_twiddle_H */