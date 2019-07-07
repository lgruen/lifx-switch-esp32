/*
**  LIFX C Library
**  Copyright 2016 Linard Arquint
*/

#ifndef LIFX_H
#define LIFX_H

#include <stdbool.h>
#include "bulb.h"
#include "color.h"

#define LIFX_LABEL_LENGTH (32) // does not include NULL char at the end


/** opens a UDP socket and initializes it. In addition, a random source_id gets generated */
int init_lifx_lib(void);

/** closes the UDP socket, which was opened in `init_lifx_lib` */
int close_lifx_lib(void);


/** 
 * Discovers LIFX bulbs in the local network
 * @param ppp_bulbs pointer to a NULL terminated array of pointers to bulb structs, `freeBulbs` has to be called when the array is no longer needed 
 */
int discoverBulbs(bulb_service_t ***ppp_bulbs);

/** Frees the NULL terminated array as well as the bulbs in the array */
int freeBulbs(bulb_service_t **pp_bulbs);


/** Retrieves the on/off state of a bulb */
int getPower(bulb_service_t *p_bulb, bool *p_on);

/** Sets the on/off state of a bulb with a duration in milliseconds to transition to the new state */
int setPower(bulb_service_t *p_bulb, bool on, uint32_t duration);


/** 
 * Retrives the currently set color, the on/off state as well as the bulb's label
 * @param p_label 32 byte string and 1 null character as terminator
 */
int getColor(bulb_service_t *p_bulb, bool *p_on, color_t *p_color, char p_label[LIFX_LABEL_LENGTH + 1]);

/** Sets the color of a bulb with a duration in milliseconds to transition to the new color */
int setColor(bulb_service_t *p_bulb, color_t color, uint32_t duration);

#endif
