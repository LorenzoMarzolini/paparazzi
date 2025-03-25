#include "modules/Floor_Detection_TCT/Floor_Detect.h" 
#include <stdio.h>
#include <time.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

float oag_max_speed = 0.2; 
int increase_speed = 0;
int increase_speed_counter = 10;
int speed_counter_reset = 50;
char prev_spin = 1;
char extra_spin_counter =0;
float oag_heading_rate = 15.0;
int8_t prev_setting = 0; 
int counter = 0;
int counter_reset = 20;
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
  if(prev_setting != setting){
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
      if(prev_spin == 1){
        if(extra_spin_counter == 0){prev_spin *= -1;} else {extra_spin_counter -=1;}
      }
      else if (prev_spin == -1){prev_spin *= -1; extra_spin_counter = 1;}
      break;
    case 5:
      navigation_state = OUT_OF_BOUNDS_DELAY;
      break;
    default:
      navigation_state = HOLD;
      break;
    }
    printf("%d", (int)navigation_state);
    prev_setting = setting;
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
  switch (navigation_state)
  {
  case HOLD:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(RadOfDeg(0));
    increase_speed = 0;
    increase_speed_counter = speed_counter_reset;
    counter = counter_reset;
    break;
  
  case SAFE:
    if(counter < (counter_reset*3)/4){ guidance_h_set_heading_rate(RadOfDeg(0));};
    if(counter == 0){
      guidance_h_set_body_vel(oag_max_speed + increase_speed*0.1, 0);
      if (increase_speed_counter < 1 && increase_speed < 5){
        increase_speed_counter = speed_counter_reset;
        increase_speed += 1;
      }
      else {increase_speed_counter -= 1;}
    }
    else {counter = counter-1; printf("Waiting%d-%d", counter, counter_reset);}
    break;
  
  case OBSTACLE_FOUND_LEFT:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(-RadOfDeg(oag_heading_rate));
    increase_speed = 0;
    increase_speed_counter = speed_counter_reset;
    counter = counter_reset;
    
    break;
  case OBSTACLE_FOUND_RIGHT:
  guidance_h_set_body_vel(0, 0);
  guidance_h_set_heading_rate(RadOfDeg(oag_heading_rate));
  increase_speed = 0;
  increase_speed_counter = speed_counter_reset;
  counter = counter_reset;
    break;
  case OUT_OF_BOUNDS:
    guidance_h_set_body_vel(0, 0);
    guidance_h_set_heading_rate(RadOfDeg(prev_spin*oag_heading_rate));
    increase_speed = 0;
    increase_speed_counter = speed_counter_reset;
    counter = counter_reset;
    break;
  case OUT_OF_BOUNDS_DELAY:
    guidance_h_set_body_vel(0, 0);
    break;
  }

}