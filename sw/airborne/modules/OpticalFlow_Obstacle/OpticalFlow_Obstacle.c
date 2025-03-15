/* optical_flow_obs.c
 *
 * This module computes optical flow using a simplified Lucas-Kanade method and
 * detects obstacles with a sliding window approach. The top and bottom 1/5 of the
 * image are excluded (assumed not of interest due to drone altitude). For efficiency,
 * the code avoids computing square roots by comparing squared magnitudes.
 */

#include "OpticalFlow_Obstacle.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>    // Only used for constant definition
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include "modules/computer_vision/lib/vision/image.h"

#define WINDOW_SIZE 6
#define STEP_SIZE 20
#define OBSTACLE_THRESHOLD 4.0f
#define OBSTACLE_THRESHOLD_SQ (OBSTACLE_THRESHOLD * OBSTACLE_THRESHOLD)
#define MAX_FLOW_VECTORS 1000

#ifndef WIDTH
#define WIDTH 520
#endif
#ifndef HEIGHT
#define HEIGHT 240
#endif

// Structure for storing optical flow vectors (position and flow components)
typedef struct {
    int x;
    int y;
    float u;
    float v;
} flow_vector_t;

// Global variables for optical flow processing
static uint8_t prev_gray[HEIGHT * WIDTH];
static int first_frame = 1;
static flow_vector_t flow_vectors[MAX_FLOW_VECTORS];
static int flow_count = 0;

// Structure for passing the computed command
typedef struct {
    uint8_t cmd;
    bool updated;
} flow_msg_t;

static flow_msg_t flow_move = {0, false};
static pthread_mutex_t flow_mutex = PTHREAD_MUTEX_INITIALIZER;

/* 
 * Function: convert_to_gray
 * -------------------------
 * Converts a YUV422 image into grayscale by extracting the Y channel.
 */
static void convert_to_gray(uint8_t *src, uint8_t *gray, int w, int h) {
    int num_pixels = w * h;
    int src_index = 0;
    int gray_index = 0;
    while (gray_index < num_pixels) {
        gray[gray_index++] = src[src_index + 1]; // Y component of first pixel
        if (gray_index < num_pixels)
            gray[gray_index++] = src[src_index + 3]; // Y component of second pixel
        src_index += 4;
    }
}

/* 
 * Function: compute_gradients
 * ---------------------------
 * Computes horizontal (Ix) and vertical (Iy) gradients using central differences.
 */
static void compute_gradients(uint8_t *gray, int w, int h, float *Ix, float *Iy) {
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int idx = y * w + x;
            // Multiply difference by 0.5 instead of dividing by 2.0
            Ix[idx] = ((float)gray[y * w + (x + 1)] - (float)gray[y * w + (x - 1)]) * 0.5f;
            Iy[idx] = ((float)gray[(y + 1) * w + x] - (float)gray[(y - 1) * w + x]) * 0.5f;
        }
    }
}

/* 
 * Function: compute_optical_flow
 * ------------------------------
 * Computes optical flow using a grid-based, simplified Lucas-Kanade method.
 */
static void compute_optical_flow(uint8_t *prev, uint8_t *curr, int w, int h) {
    float Ix[HEIGHT * WIDTH] = {0};
    float Iy[HEIGHT * WIDTH] = {0};
    memset(Ix, 0, sizeof(Ix));
    memset(Iy, 0, sizeof(Iy));
    
    compute_gradients(prev, w, h, Ix, Iy);
    
    flow_count = 0;
    int margin = WINDOW_SIZE / 2;
    for (int y = margin; y < h - margin; y += STEP_SIZE) {
        for (int x = margin; x < w - margin; x += STEP_SIZE) {
            float sum_Ix2 = 0.0f, sum_Iy2 = 0.0f, sum_IxIy = 0.0f;
            float sum_IxIt = 0.0f, sum_IyIt = 0.0f;
            for (int j = -margin; j <= margin; j++) {
                for (int i = -margin; i <= margin; i++) {
                    int idx = (y + j) * w + (x + i);
                    float ix = Ix[idx];
                    float iy = Iy[idx];
                    float it = (float)curr[idx] - (float)prev[idx];
                    sum_Ix2 += ix * ix;
                    sum_Iy2 += iy * iy;
                    sum_IxIy += ix * iy;
                    sum_IxIt += ix * it;
                    sum_IyIt += iy * it;
                }
            }
            float det = sum_Ix2 * sum_Iy2 - sum_IxIy * sum_IxIy;
            float u = 0.0f, v = 0.0f;
            if (det != 0.0f) {
                u = (-sum_Iy2 * sum_IxIt + sum_IxIy * sum_IyIt) / det;
                v = (-sum_IxIy * sum_IxIt + sum_Ix2 * sum_IyIt) / det;
            }
            if (flow_count < MAX_FLOW_VECTORS) {
                flow_vectors[flow_count].x = x;
                flow_vectors[flow_count].y = y;
                flow_vectors[flow_count].u = u;
                flow_vectors[flow_count].v = v;
                flow_count++;
            }
        }
    }
}

/* 
 * Function: detect_obstacles_sliding_window
 * -----------------------------------------
 * Uses a sliding window (horizontally) to detect obstacles based on the average
 * squared magnitude of optical flow vectors. Only considers vectors within the
 * middle 3/5 of the image vertically.
 *
 * The function avoids using square roots (for efficiency) by comparing squared
 * magnitudes against OBSTACLE_THRESHOLD_SQ.
 */
static void detect_obstacles_sliding_window(int w, int h, int *obstacle_left, int *obstacle_right) {
    // Exclude the top and bottom 1/5 of the image.
    int min_y = h / 5;
    int max_y = (4 * h) / 5;
    
    // Define sliding window parameters.
    int window_width = w / 6;  // Divide the width into 6 regions.
    int step_size = 4;         // Slide window every 4 pixels.

    int left_obstacle_count = 0, right_obstacle_count = 0;
    
    // Loop over horizontal positions using the sliding window.
    for (int x_start = 0; x_start <= w - window_width; x_start += step_size) {
        int count = 0;
        float sum_sq = 0.0f;
        // Consider flow vectors within the current window and vertical range.
        for (int i = 0; i < flow_count; i++) {
            int x = flow_vectors[i].x;
            int y = flow_vectors[i].y;
            if (x >= x_start && x < x_start + window_width && y >= min_y && y < max_y) {
                // Compute squared magnitude (u^2 + v^2) to avoid sqrt.
                float sq_mag = flow_vectors[i].u * flow_vectors[i].u + flow_vectors[i].v * flow_vectors[i].v;
                sum_sq += sq_mag;
                count++;
            }
        }
        // Calculate average squared magnitude for the window.
        float avg_sq = (count > 0) ? (sum_sq / count) : 0.0f;
        // If average exceeds threshold, mark the window as having an obstacle.
        if (avg_sq > OBSTACLE_THRESHOLD_SQ) {
            int window_center = x_start + window_width / 2;
            if (window_center < w / 2)
                left_obstacle_count++;
            else
                right_obstacle_count++;
        }
    }
    
    *obstacle_left = (left_obstacle_count > 0) ? 1 : 0;
    *obstacle_right = (right_obstacle_count > 0) ? 1 : 0;
}

/* 
 * Function: optical_flow_obs_func
 * --------------------------------
 * Main function called by the camera to process the current frame.
 * It computes optical flow, detects obstacles using the sliding window method,
 * and sets the appropriate command based on the results.
 */
static struct image_t *optical_flow_obs_func(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
    uint8_t curr_gray[HEIGHT * WIDTH];
    convert_to_gray(img->buf, curr_gray, WIDTH, HEIGHT);
    
    if (first_frame) {
        memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
        first_frame = 0;
        return img;
    }
    
    // Compute optical flow between previous and current frames.
    compute_optical_flow(prev_gray, curr_gray, WIDTH, HEIGHT);
    
    // Detect obstacles with the sliding window method.
    int obs_left = 0, obs_right = 0;
    detect_obstacles_sliding_window(WIDTH, HEIGHT, &obs_left, &obs_right);
    
    // Decide command:
    // 1 -> SAFE (no obstacles)
    // 2 -> Turn left (obstacle on right)
    // 3 -> Turn right (obstacle on left)
    // 4 -> Spin (obstacles on both sides)
    uint8_t cmd = 1;
    if (obs_left && !obs_right)
        cmd = 3;
    else if (obs_right && !obs_left)
        cmd = 2;
    else if (obs_left && obs_right)
        cmd = 4;
    
    // Update global command (protected by mutex)
    pthread_mutex_lock(&flow_mutex);
    flow_move.cmd = cmd;
    flow_move.updated = true;
    pthread_mutex_unlock(&flow_mutex);
    
    // Update the previous frame for the next iteration.
    memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
    
    return img;
}

/* 
 * Function: optical_flow_obs_periodic
 * -------------------------------------
 * Periodically sends the computed command via ABI.
 */
void optical_flow_obs_periodic(void) {
    flow_msg_t local_move;
    pthread_mutex_lock(&flow_mutex);
    local_move = flow_move;
    if (flow_move.updated) {
        flow_move.updated = false;
    }
    pthread_mutex_unlock(&flow_mutex);
    
    if (local_move.updated) {
        AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, local_move.cmd, (int16_t)0);
    }
}

/* 
 * Function: optical_flow_obs_init
 * ---------------------------------
 * Initializes the optical flow obstacle detection module by registering the
 * processing callback with the camera device.
 */
void optical_flow_obs_init(void) {
    first_frame = 1;
    cv_add_to_device(&FLOOR_DET_CAMERA, optical_flow_obs_func, 0, 0);
    printf("Optical Flow Obstacle Detection Initialized\n");
}
