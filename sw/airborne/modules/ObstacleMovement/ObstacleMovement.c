#include "ObstacleMovement.h"
#include <stdio.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

/* Guidance parameters: maximum speed and yaw rate (in degrees) */
float oag_max_speed = 0.7;
float oag_heading_rate = 5.0;

/* Definition of navigation states.
 * These states are updated by the ABI listener based on the commands sent by the optical flow module.
 */
enum navigation_state_t {
    HOLD,
    SAFE,
    AVOID_LEFT,   // Turn left (for obstacle detected on the right side)
    AVOID_RIGHT,  // Turn right (for obstacle detected on the left side)
    SPIN          // Spin in place (obstacles detected on both sides)
};

enum navigation_state_t navigation_state = HOLD;

#ifndef TCT_FLOOR_DETECTION_ID
#error Define TCT_FLOOR_DETECTION_ID (for example, COLOR_OBJECT_DETECTION1_ID) in the airframe configuration.
#endif

static abi_event obstacle_movement_ev;

/* 
 * Function: obstacle_movement_listener
 * --------------------------------------
 * ABI listener that maps the received command (via the optical flow module) to the navigation state.
 * Command mapping:
 *   1 -> SAFE
 *   2 -> AVOID_LEFT   (obstacle detected on the right side: turn left)
 *   3 -> AVOID_RIGHT  (obstacle detected on the left side: turn right)
 *   4 -> SPIN         (obstacles detected on both sides: spin)
 */
static void obstacle_movement_listener(uint8_t __attribute__((unused)) sender_id,
                                         int8_t setting,
                                         int16_t __attribute__((unused)) extra) {
    switch (setting) {
        case 1:
            navigation_state = SAFE;
            break;
        case 2:
            navigation_state = AVOID_LEFT;
            break;
        case 3:
            navigation_state = AVOID_RIGHT;
            break;
        case 4:
            navigation_state = SPIN;
            break;
        default:
            navigation_state = HOLD;
            break;
    }
}

/* 
 * Function: obstacle_move_init
 * ----------------------------
 * Initializes the obstacle movement management module by binding the ABI listener.
 * This function should be called during system initialization.
 */
void obstacle_move_init(void) {
    printf("Obstacle Movement Listener Setup\n");
    AbiBindMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, &obstacle_movement_ev, obstacle_movement_listener);
    printf("Obstacle Movement Listener Ready\n");
}

/* 
 * Function: obstacle_move_periodic
 * ----------------------------------
 * Periodic function that updates the drone's guidance based on the current navigation state.
 * It is called at regular intervals (e.g., by a scheduler) to apply the commands.
 */
void obstacle_move_periodic(void) {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        /* If the drone is not in guided mode, stop movement */
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        return;
    }
    
    /* Set the speed and yaw rate based on the received state */
    switch (navigation_state) {
        case SAFE:
            guidance_h_set_body_vel(oag_max_speed, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
        case AVOID_LEFT:
            /* Turn left (positive heading rate) */
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(20));
            break;
        case AVOID_RIGHT:
            /* Turn right (negative heading rate) */
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(-RadOfDeg(20));
            break;
        case SPIN:
            /* Execute a spin in place */
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(180));
            break;
        default:
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
    }
}
