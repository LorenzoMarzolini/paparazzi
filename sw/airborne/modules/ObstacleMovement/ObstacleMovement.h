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

// Initialize the obstacle movement module
extern void obstacle_move_init(void);

// Periodically update the drone's movement based on the current navigation state
extern void obstacle_move_periodic(void);

#endif // OBSTACLE_MOVEMENT_H
