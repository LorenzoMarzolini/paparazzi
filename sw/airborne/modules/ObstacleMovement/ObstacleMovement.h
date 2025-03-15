/**
 * @file "modules/ObstacleMovement/ObstacleMovement.h"
 * @author 
 * Lorenzo Marzolini
 *
 * @brief Module for obstacle avoidance movement.
 *
 * This module receives obstacle detection commands via ABI and sets the drone's guidance
 * to avoid obstacles by turning left or right based on free space.
 */
#include <stdio.h>

#ifndef OBSTACLE_MOVEMENT_H
#define OBSTACLE_MOVEMENT_H


extern float oag_max_speed; 
extern float oag_heading_rate;

extern void obstacle_move_init(void);
extern void obstacle_move_periodic(void);

#endif // OBSTACLE_MOVEMENT_H
