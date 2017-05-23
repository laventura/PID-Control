#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors for P, I, D params
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients for P, I, D params
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
    Cross Track Error (CTE)
  */
  double  cte_;
  double  cte_prev_;
  double  cte_sum_;

  /*
  * Constructor
  */
  PID();

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
};

#endif /* PID_H */
