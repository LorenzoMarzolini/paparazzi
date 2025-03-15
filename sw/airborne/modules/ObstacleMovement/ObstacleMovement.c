#include "modules/ObstacleMovement/ObstacleMovement.h"
#include <stdio.h>
#include <time.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"


float oag_max_speed = 1.0; 
float oag_heading_rate = 5.0;

// Navigation state enum definitions
enum navigation_state_t {
    HOLD,
    SAFE,
    AVOID_LEFT,   // Turn left (drone will steer left)
    AVOID_RIGHT,  // Turn right (drone will steer right)
    SPIN          // Spin in place
};

enum navigation_state_t navigation_state = HOLD;


#ifndef TCT_FLOOR_DETECTION_ID
#error This module requires a camera, as such you have to define TCT_FLOOR_DETECTION_ID to the orange filter
#error Please define TCT_FLOOR_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event obstacle_movement_ev;

/**
 * @brief Listener for obstacle detection commands.
 *
 * Maps received ABI commands to navigation states.
 * Received commands:
 *  1 -> SAFE,
 *  2 -> AVOID_LEFT,
 *  3 -> AVOID_RIGHT,
 *  4 -> SPIN.
 */
static void obstacle_movement_listener(uint8_t __attribute__((unused)) sender_id, int8_t setting, int16_t __attribute__((unused)) extra) {
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

/**
 * @brief Initialize the obstacle movement module.
 *
 * Sets up the ABI listener for receiving obstacle detection commands.
 */
void obstacle_move_init(void) {
    printf("Obstacle Movement Listener Setup\n");
    // AbiBindMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, &obstacle_movement_ev, obstacle_movement_listener);
    printf("Obstacle Movement Listener Ready\n");
}

/**
 * @brief Periodic function to update drone guidance based on navigation state.
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
            guidance_h_set_body_vel(oag_heading_rate, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
        case AVOID_LEFT:
            // Turn left: set positive heading rate
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(15));
            break;
        case AVOID_RIGHT:
            // Turn right: set negative heading rate
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(-RadOfDeg(15));
            break;
        case SPIN:
            // Spin in place: adjust heading rate accordingly
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(30));
            break;
        default:
            guidance_h_set_body_vel(0, 0);
            guidance_h_set_heading_rate(RadOfDeg(0));
            break;
    }
}
