#ifndef PID_HELPER_HPP
#define PID_HELPER_HPP

// sigmoid to clip value between Upper and Lower
double sigmoid(double value, double upper, double lower) {
    double diff = upper - lower;

    return diff / (1. + exp(-value)) + lower;
}


#endif  // PID_HELPER_HPP