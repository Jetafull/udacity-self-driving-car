#ifndef PID_PID_BASE_H
#define PID_PID_BASE_H

#include "controller.h"

class PID : public Controller {
public:
    PID(Coefficients coefficients);

    // Update the PID error variables given cross track error.
    void UpdateError(double cte);
    double CalculateSteerValue();

private:
    double cte_prev;
    bool initialized;
};


#endif //PID_PID_BASE_H
