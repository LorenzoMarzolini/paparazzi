#include "OpticalFlow_Obstacle.h"  // If necessary, for specific declarations
#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"
#include "modules/core/abi.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>

// Parameters for optical flow (modify if necessary)
#define WINDOW_SIZE 5
#define STEP_SIZE 20
#define OBSTACLE_THRESHOLD 1.5f
#define MAX_FLOW_VECTORS 1000

// Structure for the flow vector
typedef struct {
  int x;
  int y;
  float u;
  float v;
} flow_vector_t;

// Buffer and global variables for processing
static uint8_t prev_gray[HEIGHT * WIDTH];
static int first_frame = 1;
static flow_vector_t flow_vectors[MAX_FLOW_VECTORS];
static int flow_count = 0;

// Structure for the command message and corresponding global variable
typedef struct {
  uint8_t cmd;
  bool updated;
} flow_msg_t;

static flow_msg_t flow_move = {0, false};
static pthread_mutex_t flow_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief Converts a YUV422 image to grayscale by extracting the Y channel.
 *
 * @param src Pointer to the image buffer in YUV422 format.
 * @param gray Destination buffer for the grayscale image.
 * @param w Image width.
 * @param h Image height.
 */
static void convert_to_gray(uint8_t *src, uint8_t *gray, int w, int h) {
  int num_pixels = w * h;
  int src_index = 0;
  int gray_index = 0;
  while (gray_index < num_pixels) {
    // For a YUV422 image, the Y channel is located at indices 1 and 3
    gray[gray_index++] = src[src_index + 1];
    if (gray_index < num_pixels)
      gray[gray_index++] = src[src_index + 3];
    src_index += 4;
  }
}

/**
 * @brief Computes the horizontal and vertical gradients using central differences.
 */
static void compute_gradients(uint8_t *gray, int w, int h, float *Ix, float *Iy) {
  for (int y = 1; y < h - 1; y++) {
    for (int x = 1; x < w - 1; x++) {
      int idx = y * w + x;
      Ix[idx] = ((float)gray[y * w + (x + 1)] - (float)gray[y * w + (x - 1)]) / 2.0f;
      Iy[idx] = ((float)gray[(y + 1) * w + x] - (float)gray[(y - 1) * w + x]) / 2.0f;
    }
  }
}

/**
 * @brief Computes the optical flow using a simplified Lucas-Kanade method on a grid.
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
 * @brief Detects obstacles by comparing the average magnitude of the optical flow on the left and right halves of the image.
 *
 * @param w Image width.
 * @param h Image height.
 * @param obstacle_left Flag (1 if an obstacle is present on the left side).
 * @param obstacle_right Flag (1 if an obstacle is present on the right side).
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
 * @brief Main function for optical flow processing.
 *
 * Called by the camera device; processes the current frame, computes the optical flow with respect to the previous frame,
 * detects obstacles, and determines the command to send.
 *
 * @param img Pointer to the current image.
 * @param camera_id Camera identifier (unused).
 * @return Pointer to the processed image.
 */
static struct image_t *optical_flow_obs_func(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
  uint8_t curr_gray[HEIGHT * WIDTH];
  // Converts the image to grayscale by extracting the Y channel (same method as the floor detector)
  convert_to_gray(img->buf, curr_gray, WIDTH, HEIGHT);
  
  if (first_frame) {
    memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
    first_frame = 0;
    return img;
  }
  
  // Computes the optical flow between the previous and current frame
  compute_optical_flow(prev_gray, curr_gray, WIDTH, HEIGHT);
  
  // Detects obstacles by comparing the average flow vectors for the left and right halves
  int obs_left = 0, obs_right = 0;
  detect_obstacles(WIDTH, HEIGHT, &obs_left, &obs_right);
  
  // Decide the command to send based on the detected obstacles
  // Mapping chosen:
  // - If no obstacle: cmd = 1 (forward)
  // - If obstacle only on the left: turn right (cmd = 3)
  // - If obstacle only on the right: turn left (cmd = 2)
  // - If obstacles on both sides: spin in place (cmd = 4)
  uint8_t cmd = 1;  // SAFE: forward
  if (obs_left && !obs_right) {
    cmd = 3; // Obstacle on the left → turn right
  } else if (obs_right && !obs_left) {
    cmd = 2; // Obstacle on the right → turn left
  } else if (obs_left && obs_right) {
    cmd = 4; // Obstacles on both sides → spin
  }
  
  // Update the message to be sent in a thread-safe manner
  pthread_mutex_lock(&flow_mutex);
  flow_move.cmd = cmd;
  flow_move.updated = true;
  pthread_mutex_unlock(&flow_mutex);
  
  // Update the previous frame for the next iteration
  memcpy(prev_gray, curr_gray, WIDTH * HEIGHT);
  
  return img;
}

/**
 * @brief Periodic function that sends the command via ABI if updated.
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
    // Sends the command via ABI using the same ID used by the floor detector
    AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, local_move.cmd, (int16_t)0);
  }
}

/**
 * @brief Initializes the Optical Flow module for obstacle detection.
 */
void optical_flow_obs_init(void) {
  first_frame = 1;
  // Registers the callback for image processing with the camera device (using FLOOR_DET_CAMERA)
  cv_add_to_device(&FLOOR_DET_CAMERA, optical_flow_obs_func, 0, 0);
  printf("Optical Flow Obstacle Detection Initialized\n");
}
