/*
  *utils.c
 *
  * Created on: Aug 17, 2025
  *     Author: adhit
 */

#include "utils.h"
#include <math.h>

uint8_t hex_to_bcd(uint8_t hex) {
	return (((hex / 10) << 4) | (hex % 10));
}

float clampf(float v, float lower, float upper) {
	return fmaxf(fminf(v, upper), lower);
}


