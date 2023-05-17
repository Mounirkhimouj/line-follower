#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID{
public:
    /* Controller gains */
	float kp;
	float ki;
	float kd;

    /* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

    /* Sample time (in seconds) */
	float T;

    /*Controller "memory"*/
    float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

    /*constructor*/
    PID(float , float , float , float , float, int , int );
    /*updating the contoller*/
    float Up_date(float ,float);

};


#endif