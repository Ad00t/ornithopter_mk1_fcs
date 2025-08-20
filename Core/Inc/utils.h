/*
 * utils.h
 *
 *  Created on: Aug 17, 2025
 *      Author: adhit
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

uint8_t hex_to_bcd(uint8_t hex);
float clampf(float v, float min, float max);
char* string_copy(char* s);
void string_free(char* s);

#endif /* UTILS_H */
