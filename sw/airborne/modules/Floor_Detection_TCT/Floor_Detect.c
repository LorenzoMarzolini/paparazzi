#include "modules/Floor_Detection_TCT/Floor_Detect.h" 
#include <stdio.h>
#include <time.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

float oag_max_speed = 1.0; 
float oag_heading_rate = 5.0;

enum navigation_state_t{
    HOLD,
    SAFE,
    OBSTACLE_FOUND_LEFT,
    OBSTACLE_FOUND_RIGHT,
    OUT_OF_BOUNDS,
    OUT_OF_BOUNDS_DELAY
  };
enum navigation_state_t navigation_state = HOLD;

void floor_detect_reset(void)
{
  guidance_h_set_body_vel(0, 0);
  guidance_h_set_heading_rate(RadOfDeg(0));
}

#ifndef TCT_FLOOR_DETECTION_ID
#error This module requires a camera, as such you have to define TCT_FLOOR_DETECTION_ID to the orange filter
#error Please define TCT_FLOOR_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_listener(uint8_t __attribute__((unused)) sender_id, int8_t setting, int16_t __attribute__((unused)) extra)
{
  switch (setting)
  {
  case 0:
    navigation_state = HOLD;
    break;
  case 1:
    navigation_state = SAFE;
    break;
  case 2:
    navigation_state = OBSTACLE_FOUND_LEFT;
    break;
  case 3:
    navigation_state = OBSTACLE_FOUND_RIGHT;
    break;
  case 4:
    navigation_state = OUT_OF_BOUNDS;
    break;
  case 5:
    navigation_state = OUT_OF_BOUNDS_DELAY;
    break;
  default:
    navigation_state = HOLD;
    break;
  }
}

void floor_detect_init(void)
{
  printf("Listener_setup");
  AbiBindMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, &floor_detection_ev, floor_detection_listener);
  printf("Listener_ready");
}

void floor_detect_periodic(void)
{
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    floor_detect_reset();
    return;
  }
  printf("%d", (int)navigation_state);
  switch (navigation_state)
  {
  case HOLD:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(RadOfDeg(0));
    break;
  
  case SAFE:
    guidance_h_set_body_vel(oag_max_speed, 0);
    guidance_h_set_heading_rate(RadOfDeg(0));
    break;
  
  case OBSTACLE_FOUND_LEFT:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(-RadOfDeg(15));
    
    break;
  case OBSTACLE_FOUND_RIGHT:
  guidance_h_set_body_vel(0, 0);
  guidance_h_set_heading_rate(RadOfDeg(15));
    break;
  case OUT_OF_BOUNDS:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(-RadOfDeg(15));
    break;
  case OUT_OF_BOUNDS_DELAY:
    guidance_h_set_body_vel(0, 0);
    break;
  }

}