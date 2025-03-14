#include <stdio.h>

#ifndef OPTICAL_FLOW_OBSTACLE_H
#define OPTICAL_FLOW_OBSTACLE_H


extern float oag_max_speed; 
extern float oag_heading_rate;

extern void optical_flow_obs_init(void);
extern void optical_flow_obs_periodic(void);
void optical_flow_obs_reset(void);

#endif // OPTICAL_FLOW_OBSTACLE_H
