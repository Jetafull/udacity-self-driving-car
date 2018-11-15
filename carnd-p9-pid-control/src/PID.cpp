#include "pid.h"
#include <math.h>

using namespace std;

PID::PID(Coefficients coefficients) : Controller(coefficients) {
    initialized = false;
    cte_prev = 0.0;
}

void PID::UpdateError(double cte) {
    // p_error
    errors.p_error = cte;

    // d_error update
    if (!initialized) {
        errors.d_error = 0.0;
        cte_prev = cte;
        initialized = true;
    } else {
        errors.d_error = cte - cte_prev;
        cte_prev = cte;
    }

    // i_error
    errors.i_error += cte;
}

double PID::CalculateSteerValue() {
    return -coefficients.tau_p * errors.p_error -
           coefficients.tau_i * errors.i_error -
           coefficients.tau_d * errors.d_error;
}
