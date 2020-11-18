#ifndef PID_CONTROLLER
#define PID_CONTROLLER

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;

	/* Output limits */
	float LimMin;
	float LimMax;
	
	/* Integrator limits */
	float LimMinInt;
	float LimMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float Integrator;
	float PrevError;			/* Required for integrator */

	/* Controller output */
	float Out;

} PIDController;

void PIDController_Init(PIDController *pid, float Kp, float Ki, float LimMin, float LimMax, float LimMinInt, float LimMaxInt, float T);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
