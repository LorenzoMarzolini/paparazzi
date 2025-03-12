
/**
 * @file "modules/Floor_Detection_TCT/Floor_Detect.c"
 * @author Lorenzo Gaeta
 *
 * This module is based on orange avoider guided but uses the floor to estimate close objects instead of an orange color
 * 
 * 
 */

#include <stdio.h>

#ifndef GREEN_FLOOR_GUIDED_H
#define GREEN_FLOOR_GUIDED_H


extern short green_threshold;
extern short red_threshold;
extern short blue_threshold;
extern float oag_max_speed; 
extern float oag_heading_rate;

extern void floor_detect_init(void);
extern void floor_detect_periodic(void);

#endif
