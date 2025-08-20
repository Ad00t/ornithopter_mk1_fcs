/*
 * utils.c
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#include "utils.h"
#include <math.h>

uint8_t hex_to_bcd(uint8_t hex) {
	return (((hex / 10) << 4) | (hex % 10));
}

float clampf(float v, float lower, float upper) {
	return fmaxf(fminf(v, upper), lower);
}

char* string_copy(char* s) {
    if (!s) return NULL;
    size_t len = strlen(s) + 1;
    char* copy = malloc(len);
    if (copy) memcpy(copy, s, len);
    return copy;
}

void string_free(char* s) {
    free(s);
}


