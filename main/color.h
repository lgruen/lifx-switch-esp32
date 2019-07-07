/*
**  LIFX C Library
**  Copyright 2016 Linard Arquint
*/

#ifndef COLOR_H
#define COLOR_H

#include <stdint.h>


typedef struct {
	/** 0 - 65535 */
	uint16_t hue;
	/** 0 - 65535 */
	uint16_t saturation;
	/** 0 - 65535 */
	uint16_t brightness;
	/** 2500° - 9000° */
	uint16_t kelvin;
} color_t;

#endif
