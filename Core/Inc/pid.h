/*
 * pid.h
 *
 *  Created on: Aug 22, 2025
 *      Author: adhit
 */

#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float sp;
    float last_e;
	float integrator;
	float derivator;
    uint32_t last_update_time;
} PID;

void PID_init(PID *pid, float Kp, float Ki, float Kd);
void PID_reset(PID *pid, float sp, float pv);
float PID_update(PID *pid, float pv);

#endif /* PID_H */
