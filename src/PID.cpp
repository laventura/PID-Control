#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    p_error_    = 0.0;
    i_error_    = 0.0;
    d_error_    = 0.0;

    cte_        = 0;
    cte_prev_   = 0;
    cte_sum_    = 0;
}

void PID::UpdateError(double cte) {
    // 1 - update errors
    cte_prev_   = cte_;
    cte_        = cte;
    cte_sum_    = cte_sum_ + cte_;

    // 2 - update coefficients
    p_error_    = Kp_ * cte;
    i_error_    = Ki_ * cte_sum_;
    d_error_    = Kd_ * (cte_ - cte_prev_);
}

double PID::TotalError() {
    // calc total CTE
    // Total CTE: -Kp * cte - Ki * cte_sum - Kd * cte_diff
    // NOTE: the caller MUST negate the value of TotalError!

    return -(p_error_ + i_error_ + d_error_);  
}

