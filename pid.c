/*
 * pid.c
 *
 *  Created on: May 19, 2014
 *      Author: Tim Toliver
 *
 *      This is from Tim Wescott's PID without a PhD
 */

#include "pid.h"

float UpdatePID(SPid * pid, float error, float position) {

	float pTerm, dTerm, iTerm;
	pid->iMax = 65535;             // these correspond to the max and min of the integrator state limited to the max output of the drive
	pid->iMin = 0;

	pTerm = pid->pGain * error;     // calculate the proportional term

	pid->iState += error; // calculate the integral state with appropriate limiting

	if (pid->iState > pid->iMax)
		pid->iState = pid->iMax;

	else if (pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState;    // calculate the integral term

	dTerm = pid->dGain * (position - pid->dState); // calculate the derivative term
	pid->dState = position;

	return pTerm + iTerm - dTerm;

}
/*
signed int UpdatePID(SPid * pid, signed int error, signed int position) {
	signed int pTerm, dTerm, iTerm;
	pid->iMax = 3250;
	pid->iMin = 0;

	pTerm = pid->pGain * error;   // calculate the proportional term

	pid->iState += error;         // calculate the integral state with appropriate limiting
	if (pid->iState > pid->iMax)
		pid->iState = pid->iMax;

	else if (pid->iState < pid->iMin)
		pid->iState = pid->iMin;

	iTerm = pid->iGain * pid->iState;  // calculate the integral term

	dTerm = pid->dGain * (pid->dState - position);
	pid->dState = position;

	return pTerm + dTerm + iTerm;
}
*/
