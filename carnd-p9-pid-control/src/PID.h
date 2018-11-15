#ifndef pid_H
#define pid_H

#include "controller.h"

class PID : public Controller {
 public:
  // Errors
  double p_error;
  double i_error;
  double d_error;

  PID(Taus taus);

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);
  double CalculateSteerValue();

 private:
  Taus taus;
  double cte_prev;
  bool initialized;
};

#endif /* pid_H */
