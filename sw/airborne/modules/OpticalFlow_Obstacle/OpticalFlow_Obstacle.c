#include "OpticalFlow_Obstacle.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include "modules/computer_vision/lib/vision/image.h"

// Define parameters (adjust as needed)
#define WINDOW_SIZE 6
#define STEP_SIZE 20
#define OBSTACLE_THRESHOLD 4.0f
#define MAX_FLOW_VECTORS 1000

// Use predefined image dimensions (from airframe settings)
#ifndef WIDTH
#define WIDTH 520
#endif
#ifndef HEIGHT
#define HEIGHT 240
#endif

// Structure for flow vector (position and flow components)
typedef struct {
  int x;
  int y;
  float u;
  float v;
} flow_vector_t;

// Global static variables for optical flow processing
static uint8_t prev_gray[HEIGHT * WIDTH];
static int first_frame = 1;
static flow_vector_t flow_vectors[MAX_FLOW_VECTORS];
static int flow_count = 0;

// Structure for message passing with flow command
typedef struct {
  uint8_t cmd;
  bool updated;
} flow_msg_t;

// Global variable for the command and mutex for synchronization
static flow_msg_t flow_move = {0, false};
static pthread_mutex_t flow_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Converts the image to grayscale.
 * For a YUV422 image, extracts the Y channel.
 */
static void convert_to_gray(uint8_t *src, uint8_t *gray, int w, int h) {
  int num_pixels = w * h;
  int src_index = 0;
  int gray_index = 0;
  while (gray_index < num_pixels) {
    // First pixel: Y component at index 1
    gray[gray_index++] = src[src_index + 1];
    // Second pixel: Y component at index 3
    if (gray_index < num_pixels)
      gray[gray_index++] = src[src_index + 3];
    src_index += 4;
  }
}

/**
 * @brief Computes horizontal and vertical gradients using central differences.
 */
static void compute_gradients(uint8_t *gray, int w, int h, float *Ix, float *Iy) {
  int x, y;
  for (y = 1; y < h - 1; y++) {
    for (x = 1; x < w - 1; x++) {
      int idx = y * w + x;
      Ix[idx] = ((float)gray[y * w + (x+1)] - (float)gray[y * w + (x-1)]) / 2.0f;
      Iy[idx] = ((float)gray[(y+1) * w + x] - (float)gray[(y-1) * w + x]) / 2.0f;
    }
  }
}

/**
 * @brief Computes optical flow using a simplified Lucas-Kanade method on a grid.
 */
static void compute_optical_flow(uint8_t *prev, uint8_t *curr, int w, int h) {
  float Ix[HEIGHT * WIDTH];
  float Iy[HEIGHT * WIDTH];
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

/**
 * @brief Detects obstacles by comparing the average optical flow magnitude
 * on the left and right halves of the image.
 *
 * @param w Image width.
 * @param h Image height.
 * @param obstacle_left Pointer to flag indicating an obstacle on the left (1 if yes).
 * @param obstacle_right Pointer to flag indicating an obstacle on the right (1 if yes).
 */
static void detect_obstacles(int w, int h, int *obstacle_left, int *obstacle_right) {
  float left_sum = 0.0f, right_sum = 0.0f;
  int left_count = 0, right_count = 0;
  for (int i = 0; i < flow_count; i++) {
    float mag = sqrtf(flow_vectors[i].u * flow_vectors[i].u + flow_vectors[i].v * flow_vectors[i].v);
    if (flow_vectors[i].x < w / 2) {
      left_sum += mag;
      left_count++;
    } else {
      right_sum += mag;
      right_count++;
    }
  }
  float left_avg = (left_count > 0) ? (left_sum / left_count) : 0.0f;
  float right_avg = (right_count > 0) ? (right_sum / right_count) : 0.0f;
  
  *obstacle_left = (left_avg > OBSTACLE_THRESHOLD) ? 1 : 0;
  *obstacle_right = (right_avg > OBSTACLE_THRESHOLD) ? 1 : 0;
}

/**
 * @brief Main optical flow processing function called by the camera.
 *
 * Processes the current frame, computes optical flow relative to the previous frame,
 * detects obstacles, and updates the command to be sent.
 *
 * @param img Pointer to the current image.
 * @param camera_id Unused camera identifier.
 * @return Pointer to the processed image.
 */
static struct image_t *optical_flow_obs_func(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  uint8_t curr_gray[HEIGHT * WIDTH];
  convert_to_gray(img->buf, curr_gray, WIDTH, HEIGHT);
  
  if (first_frame) {
    memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
    first_frame = 0;
    return img;
  }
  
  // Compute optical flow between previous and current grayscale images
  compute_optical_flow(prev_gray, curr_gray, WIDTH, HEIGHT);
  
  // Detect obstacles based on the computed flow vectors
  int obs_left = 0, obs_right = 0;
  detect_obstacles(WIDTH, HEIGHT, &obs_left, &obs_right);
  
  // Decide command based on free space:
  // - SAFE (command 1) if both sides are clear
  // - If only the left side has obstacles -> turn right (command 3)
  // - If only the right side has obstacles -> turn left (command 2)
  // - If both sides have obstacles -> spin in place (command 4)
  uint8_t cmd = 1; // SAFE by default
  if (obs_left && !obs_right) {
    cmd = 3; // Turn right
  } else if (obs_right && !obs_left) {
    cmd = 2; // Turn left
  } else if (obs_left && obs_right) {
    cmd = 4; // Spin in place
  }
  
  // Instead of sending the command directly, store it in a global variable
  // protected by a mutex, to be processed by the periodic function.
  pthread_mutex_lock(&flow_mutex);
  flow_move.cmd = cmd;
  flow_move.updated = true;
  pthread_mutex_unlock(&flow_mutex);
  
  // Update the previous frame for the next iteration
  memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
  
  return img;
}

/**
 * @brief Periodic function for sending the command via ABI.
 *
 * This function, called periodically (e.g., by a task scheduler), checks if a new
 * command is available, sends it via ABI, and resets the update flag.
 */
void optical_flow_obs_periodic(void) {
  flow_msg_t local_move;
  pthread_mutex_lock(&flow_mutex);
  local_move = flow_move; // Local copy to minimize time in critical section
  if (flow_move.updated) {
    flow_move.updated = false; // Reset update flag after reading
  }
  pthread_mutex_unlock(&flow_mutex);

  if (local_move.updated) {
    AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, local_move.cmd,  (int16_t)0);
  }
}

/**
 * @brief Initializes the optical flow obstacle detection module.
 */
void optical_flow_obs_init(void) {
  first_frame = 1;
  // Register the optical flow callback with the camera device
  cv_add_to_device(&FLOOR_DET_CAMERA, optical_flow_obs_func, 0, 0);
  printf("Optical Flow Obstacle Detection Initialized\n");
}
