#ifndef PID_H
#define PID_H

#include "pros.h"

class PID_Controller {
    public:
        PID_Controller();
    private:
        float pidSensorCurrentValue;

        float pidError;
        float pidLastError;
        float pidDerivative;
        float pidIntegral;
        float pidDrive;

};
#endif