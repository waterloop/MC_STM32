#include "PID.h"

void PIDController_Init(PIDController *pid, float Kp, float Ki, float LimMin, float LimMax, float LimMinInt, float LimMaxInt, float T) {
	/* Clear controller variables */
	pid->Integrator = 0.0f;
	pid->PrevError  = 0.0f;
	pid->Out = 0.0f;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->LimMin = LimMin;
	pid->LimMax = LimMax;
	pid->LimMinInt = LimMinInt;
	pid->LimMaxInt = LimMaxInt;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;

	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;

	/*
	* Integral
	*/
    pid->Integrator = pid->Integrator + 0.5f * pid->Ki * pid->T * (error + pid->PrevError);

	/* Anti-wind-up via Integrator clamping */
    if (pid->Integrator > pid->LimMaxInt) {
        pid->Integrator = pid->LimMaxInt;
    }

    else if (pid->Integrator < pid->LimMinInt) {
        pid->Integrator = pid->LimMinInt;
    }

    /*
	* Compute output and apply limits
	*/
    pid->Out = proportional + pid->Integrator;

    if (pid->Out > pid->LimMax) {
        pid->Out = pid->LimMax;
    }

    else if (pid->Out < pid->LimMin) {
        pid->Out = pid->LimMin;
    }

	/* Store error and measurement for later use */
    pid->PrevError = error;

	/* Return controller output */
    return pid->Out;
}
