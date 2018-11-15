#ifndef controller_H
#define controller_H

class Controller {
 public:
  virtual void UpdateError(double) = 0;
  virtual double CalculateSteerValue() = 0;
};

#endif /* controller_H */
