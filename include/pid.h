#ifndef PID_H
#define PID_H

#include "pros.h"
#include "drivetrain.h"

class PID {
    public:
        PID();
        void DriveTo(double heading);

        Drivetrain drivetrain;
    private:
        float pidSensorCurrentValue = 0.f;

        float pidError = 0.f;
        float pidLastError = 0.f;
        float pidDerivative = 0.f;
        float pidIntegral = 0.f;
        float pidDrive = 0.f;

};
#endif