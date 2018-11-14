#ifndef PID_H
#define PID_H

class PID {
 public:
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /*
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /*
   * Constructor
   */
  PID() {
    cte_sum = 0.0;
    initialized = false;
  };

  /*
   * Destructor.
   */
  virtual ~PID();

  /*
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   */
  double TotalError();

 private:
  /*
   * CTE records
   */
  double cte_prev;
  double cte_sum;
  bool initialized;
};

#endif /* PID_H */
