#ifndef PID_H
#define PID_H

class PID {
 public:
  // Errors
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */
  double tau_p;
  double tau_i;
  double tau_d;

  /*
   * Constructor
   */
  PID(double tau_p, double tau_i, double tau_d);

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
  /*
   * CTE records
   */
  double cte_prev;
  bool initialized;
};

#endif /* PID_H */
