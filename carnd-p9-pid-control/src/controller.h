#ifndef controller_H
#define controller_H

struct Coefficients {
  double tau_p;
  double tau_i;
  double tau_d;
};

struct Errors {
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;
};

class Controller {
 public:
  Controller(Coefficients coefficients) : coefficients(coefficients){};
  virtual ~Controller(){};
  virtual void UpdateError(double) = 0;
  virtual double CalculateSteerValue() = 0;

 protected:
  Coefficients coefficients;
  Errors errors;
};

#endif /* controller_H */
