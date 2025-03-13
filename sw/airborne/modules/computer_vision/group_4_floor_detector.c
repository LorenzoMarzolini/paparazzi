
#include <stdbool.h>
#include <stdio.h>
#include <string.h>


#include "modules/computer_vision/lib/vision/image.h"
#include "group_4_floor_detector.h"
#include "modules/core/abi.h"


uint8_t R_lim = 100;
uint8_t G_lim = 100;
uint8_t B_lim = 70;

bool det_draw1 = FLOOR_DET_DRAW;
bool det_draw2 = FLOOR_DET_DRAW2;


//copilot
bool visited[WIDTH][HEIGHT];

// Function for flood-fill to find a cluster
static int floodFill(uint8_t *buffer, uint8_t *new_buffer, int x, int y, short w, short h, bool write_to_new_buffer) {
  int cluster_size = 0;
  int queue[HEIGHT * WIDTH][2];  // Queue for BFS
  int front = 0, rear = 0;

  // Start flood-fill at (x, y)
  queue[rear][0] = x;
  queue[rear][1] = y;
  rear++;
  visited[x][y] = true;

  while (front < rear) {
      int cx = queue[front][0];
      int cy = queue[front][1];
      front++;

      cluster_size++;

      if (write_to_new_buffer) {
          new_buffer[cx * w + cy] = 255;  // Write to new buffer
          
      }

      // Check 8 neighbors (including diagonals)
      int neighbors[8][2] = {
          {cx - 1, cy}, {cx + 1, cy}, {cx, cy - 1}, {cx, cy + 1},      // Up, Down, Left, Right
          {cx - 1, cy - 1}, {cx - 1, cy + 1}, {cx + 1, cy - 1}, {cx + 1, cy + 1} // Diagonals
      };

      for (int i = 0; i < 8; i++) {
          int nx = neighbors[i][0];
          int ny = neighbors[i][1];

          if (nx >= 0 && nx < h && ny >= 0 && ny < w && !visited[nx][ny] && buffer[nx * w + ny] == 255) {
              visited[nx][ny] = true;
              queue[rear][0] = nx;
              queue[rear][1] = ny;
              rear++;
          }
      }
  }
  printf("cluster size: %d\n", cluster_size);
  return cluster_size;
}


// Main function to find and isolate the largest cluster
static void isolateLargestCluster(uint8_t *buffer, uint8_t *new_buffer, short w, short h) {
    memset(visited, 0, sizeof(visited));  // Reset the visited array
    printf("Start\n");
    int max_cluster_size = 0;
    int max_cluster_x = -1, max_cluster_y = -1;

    // First pass: find the largest cluster
    for (short i = 0; i < h; i++) {
        for (short j = 0; j < w; j++) {
            if (!visited[i][j] && buffer[i * w + j] == 255) {
                int cluster_size = floodFill(buffer, NULL ,i, j, w, h, false);  // Find cluster size
                if (cluster_size > max_cluster_size) {
                    max_cluster_size = cluster_size;
                    max_cluster_x = i;
                    max_cluster_y = j;
                }
            }
        }
    }
    printf("end\n");
    // Reset visited array for the second pass
    memset(visited, 0, sizeof(visited));
    memset(new_buffer, 0, w * h);  // Clear the new buffer

    // Second pass: extract the largest cluster
    if (max_cluster_x != -1 && max_cluster_y != -1) {
        floodFill(buffer, new_buffer, max_cluster_x, max_cluster_y, w, h, true);
    }

}


// not copilot
static pthread_mutex_t mutex;
static struct msg_det{
  uint8_t cmd;
  bool updated;
};

uint8_t Buffer_2[HEIGHT*WIDTH];
uint8_t Buffer_3[HEIGHT*WIDTH];
uint8_t Buffer_4[HEIGHT*WIDTH];

struct msg_det move_global[1];
static struct image_t *g4_floor_det_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{  
  short h = img->h;
  short w = img->w;
  uint8_t *source = (uint8_t *)img->buf;
  uint8_t *dest = Buffer_2;
  for(short i = 0; i<h; i++){
    for(short j = 0; j<w; j+=2){
      short y1 = (short)source[1];
      short u = (short)source[0] - 128;
      short y2 = (short)source[3];
      short v = (short)source[2] - 128;
      short R1 = y1 + ((1436 * v) >> 10);
      short G1 = y1 - ((352 * u + 731 * v) >> 10);
      short B1 = y1 + ((1814 * u) >> 10);
      short R2 = y2 + ((1436 * v) >> 10);
      short G2 = y2 - ((352 * u + 731 * v) >> 10);
      short B2 = y2 + ((1814 * u) >> 10);
      if(R1 < R_lim && B1 < B_lim && G1>G_lim){dest[0] = 255;} else {dest[0] = 0;};
      if(R2 < R_lim && B2 < B_lim && G2>G_lim){dest[1] = 255;} else {dest[1] = 0;};
      dest += 2;
      source += 4;
    }
  }
  isolateLargestCluster(Buffer_2, Buffer_3, w, h);

  pthread_mutex_lock(&mutex);
  move_global[0].cmd = 1;
  move_global[0].updated = true;
  pthread_mutex_unlock(&mutex);

  if(det_draw1){
    uint8_t *source2 = (uint8_t *)img->buf;
    uint8_t *dest2 = Buffer_2;
    for(short i = 0; i<h; i++){
      for(short j = 0; j<w; j+=2){
        // check if in image or not
        source2[1] = dest2[0];
        source2[0] = 128;
        source2[3] = dest2[1];
        source2[2] = 128;  
        dest2+= 2;
        source2 += 4;
      }
    }
  }
  if(det_draw2){
    uint8_t *source2 = (uint8_t *)img->buf;
    uint8_t *dest2 = Buffer_3;
    for(short i = 0; i<h; i++){
      for(short j = 0; j<w; j+=2){
        // check if in image or not
        source2[1] = dest2[0];
        source2[0] = 128;
        source2[3] = dest2[1];
        source2[2] = 128;  
        dest2+= 2;
        source2 += 4;
      }
    }
  }
  

  return img; // func did not make a new image
}

void g4_floor_det_init(void)
{
  memset(move_global, 0, sizeof(struct msg_det));
  pthread_mutex_init(&mutex, NULL);
  cv_add_to_device(&FLOOR_DET_CAMERA, g4_floor_det_func, FLOOR_DETECTOR_FPS, 0);
}


void g4_floor_det_periodic(void)
{
  static struct msg_det move[1];
  pthread_mutex_lock(&mutex);
  memcpy(move, move_global, sizeof(struct msg_det));
  pthread_mutex_unlock(&mutex);
  if(move[0].updated){AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, move[0].cmd, (int16_t)0); move[0].updated = false;};
}