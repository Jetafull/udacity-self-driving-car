#ifndef controller_H
#define controller_H

struct Taus {
  double tau_p;
  double tau_i;
  double tau_d;
};

struct Errors {
  double p_error;
  double i_error;
  double d_error;
};

class Controller {
 public:
  virtual void UpdateError(double) = 0;
  virtual double CalculateSteerValue() = 0;
};

#endif /* controller_H */
