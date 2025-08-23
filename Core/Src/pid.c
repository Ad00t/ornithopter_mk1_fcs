/*
 * pid.c
 *
 *  Created on: Aug 22, 2025
 *      Author: adhit
 */

#include "pid.h"
#include "cmsis_os.h"
#include <stdlib.h>

void PID_init(PID *pid, float Kp, float Ki, float Kd) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

void PID_reset(PID *pid, float sp, float pv) {
	pid->sp = sp;
	pid->last_e = sp - pv;
	pid->integrator = 0;
	pid->derivator = 0;
	pid->last_update_time = osKernelGetTickCount();
}

float PID_update(PID *pid, float pv) {
	uint32_t t_ms = osKernelGetTickCount();
	float err = pid->sp - pv;
	if (abs(err) < 1) return 0;
	uint32_t dt = t_ms - pid->last_update_time;
	pid->integrator += err * dt;
	if (dt != 0) pid->derivator = (err - pid->last_e) / dt;
	float u = pid->Kp*err + pid->Ki*pid->integrator + pid->Kd*pid->derivator;
	pid->last_e = err;
	pid->last_update_time = t_ms;
	return u;
}

