#ifndef PID_H
#define PID_H

#include "pros.h"
#include "drivetrain.h"

class PID_Controller {
    public:
        PID_Controller();
        void DriveTo(double heading);

        Drivetrain drivetrain;
    private:
        float pidSensorCurrentValue;

        float pidError;
        float pidLastError;
        float pidDerivative;
        float pidIntegral;
        float pidDrive;

};
#endif