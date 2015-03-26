/*
 * pid.h
 *
 *  Created on: May 19, 2014
 *      Author: Tim Toliver
 *
 *      This is from Tim Wescott's PID without a PhD
 */

#ifndef PID_H_
#define PID_H_

/*
typedef struct{
	signed int dState;     // Last position input
	signed int iState;     // Integrator state
	signed int iMax, iMin; // Maximum and minimum allowable integrator state
	signed int iGain;      // integral gain
	signed int pGain;      // proportional gain
	signed int dGain;      // derivative gain
} SPid;

signed int UpdatePID(SPid * pid, signed int error, signed int position);
*/

typedef struct{
	float dState;     // Last position input
	float iState;     // Integrator state
	float iMax, iMin; // Maximum and minimum allowable integrator state
	float iGain;      // integral gain
	float pGain;      // proportional gain
	float dGain;      // derivative gain
} SPid;

float UpdatePID(SPid * pid, float error, float position);

#endif /* PID_H_ */
