/**
 * @file "modules/OpticalFlow_Obstacle/OpticalFlow_Obstacle.h"
 * @author 
 * Lorenzo Marzolini
 *
 * @brief Module for optical flow based obstacle detection.
 *
 * This module computes optical flow from consecutive camera frames to detect obstacles.
 * The decision is based on free space estimation: the drone will turn towards the side with more free space.
 */

#ifndef OPTICAL_FLOW_OBSTACLE_H
#define OPTICAL_FLOW_OBSTACLE_H

// Initialize the optical flow obstacle detection module
extern void optical_flow_obs_init(void);

// Periodic function (if needed) to process frames; the optical flow is mainly computed in the camera callback.
extern void optical_flow_obs_periodic(void);

#endif // OPTICAL_FLOW_OBSTACLE_H
