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

#ifndef OBSTACLE_MOVEMENT_H
#define OBSTACLE_MOVEMENT_H

// Declare global variables to be modified via settings
#define oag_max_speed 10.0f
#define oag_heading_rate 5.0f

// Initialize the obstacle movement module
extern void obstacle_move_init(void);
// Periodically update the drone's movement based on navigation state
extern void obstacle_move_periodic(void);

#endif // OBSTACLE_MOVEMENT_H
