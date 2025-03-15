/* obstacle_movement.c
 *
 * This module receives obstacle detection commands via ABI and updates the drone's guidance.
 * Commands are mapped as follows:
 *   1: SAFE (no obstacles)
 *   2: Turn left (obstacle detected on right)
 *   3: Turn right (obstacle detected on left)
 *   4: Spin in place (obstacles detected on both sides)
 */

#include "ObstacleMovement.h"
#include <stdio.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

float oag_max_speed = 1.0;
float oag_heading_rate = 5.0;

// Navigation state definitions.
enum navigation_state_t {
    HOLD,
    SAFE,
    AVOID_LEFT,   // Turn left
    AVOID_RIGHT,  // Turn right
    SPIN          // Spin in place
};

enum navigation_state_t navigation_state = HOLD;

#ifndef TCT_FLOOR_DETECTION_ID
#error Define TCT_FLOOR_DETECTION_ID (e.g., COLOR_OBJECT_DETECTION1_ID) in your airframe configuration.
#endif

static abi_event obstacle_movement_ev;

/* 
 * Function: obstacle_movement_listener
 * --------------------------------------
 * ABI listener that maps received commands to navigation states.
 * Commands:
 *   1 -> SAFE
 *   2 -> AVOID_LEFT
 *   3 -> AVOID_RIGHT
 *   4 -> SPIN
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
 * Initializes the obstacle movement module by binding the ABI listener.
 */
void obstacle_move_init(void) {
    printf("Obstacle Movement Listener Setup\n");
    AbiBindMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, &obstacle_movement_ev, obstacle_movement_listener);
    printf("Obstacle Movement Listener Ready\n");
}

/* 
 * Function: obstacle_move_periodic
 * ----------------------------------
 * Periodically updates the drone's guidance based on the current navigation state.
 */
void obstacle_move_periodic(void) {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        return;
    }
    
    switch (navigation_state) {
        case HOLD:
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
        case SAFE:
            guidance_h_set_body_vel(oag_max_speed, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
        case AVOID_LEFT:
            // Turn left: positive heading rate
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(15));
            break;
        case AVOID_RIGHT:
            // Turn right: negative heading rate
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(-RadOfDeg(15));
            break;
        case SPIN:
            // Spin in place: higher heading rate
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(30));
            break;
        default:
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
    }
}
