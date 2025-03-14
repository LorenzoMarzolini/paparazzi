
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


bool visited[WIDTH][HEIGHT];
static int floodFill(uint8_t *buffer, uint8_t *new_buffer, int x, int y, short w, short h, bool write_to_new_buffer) {
  int cluster_size = 0;
  int queue[HEIGHT * WIDTH][2]; 
  int front = 0, rear = 0;

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
          new_buffer[cx * w + cy] = 255; 
          
      }
      int neighbors[8][2] = {
          {cx - 1, cy}, {cx + 1, cy}, {cx, cy - 1}, {cx, cy + 1},           
          {cx - 1, cy - 1}, {cx - 1, cy + 1}, {cx + 1, cy - 1}, {cx + 1, cy + 1}
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
  return cluster_size;
}

static void isolateLargestCluster(uint8_t *buffer, uint8_t *new_buffer, short w, short h) {
    memset(visited, 0, sizeof(visited)); 
    int max_cluster_size = 0;
    int max_cluster_x = -1, max_cluster_y = -1;

    for (short i = 0; i < h; i++) {
        for (short j = 0; j < w; j++) {
            if (!visited[i][j] && buffer[i * w + j] == 255) {
                int cluster_size = floodFill(buffer, NULL ,i, j, w, h, false); 
                if (cluster_size > max_cluster_size) {
                    max_cluster_size = cluster_size;
                    max_cluster_x = i;
                    max_cluster_y = j;
                }
            }
        }
    }
    memset(visited, 0, sizeof(visited));
    memset(new_buffer, 0, w * h); 

    if (max_cluster_x != -1 && max_cluster_y != -1) {
        floodFill(buffer, new_buffer, max_cluster_x, max_cluster_y, w, h, true);
    }

}

static pthread_mutex_t mutex;
static struct msg_det{
  uint8_t cmd;
  bool updated;
};

uint8_t Buffer_2[HEIGHT*WIDTH];
uint8_t Buffer_3[HEIGHT*WIDTH];
uint8_t Buffer_4[HEIGHT*WIDTH];
bool right = false;
bool left = false;
bool spin = false;
bool gap = false;
uint8_t cmd = 0;
struct msg_det move_global[1];
uint8_t hort_check[WIDTH];
uint8_t vert_check[WIDTH];
int left_sum = 0;
int right_sum = 0;;


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

  if(true){
  int middle = h / 2; 
  int slice = (int)(h / 10) / 2; 

  spin = false;
  left = false;
  right = false;
  gap = false;
  for (int y = 0; y < h; y++) {
      hort_check[y] = Buffer_3[y * w + (h - 1)];
  }
  int active_pixels = 0;
  for (int y = 0; y < h; y++) {
      if (hort_check[y] > 0) {
          active_pixels++;
      }
  }
  if (active_pixels < 30) {
      spin = true;
  }
  if(!spin){
    for (int x = 0; x < h; x++) {
      vert_check[x] = 0;
  }

  for (int x = 0; x < h; x++) {
      for (int y = 0; y < w; y++) {
          if (Buffer_3[x * w + y] > 0) {
              vert_check[x] = 255; 
              break;               
          }
      }
  }
  left_sum = 0;
  right_sum = 0;
  for (int x = 0; x < middle; x++) {
    if (vert_check[x] > 0) {
        left_sum++;
    }
  }
  for (int x = middle; x < h; x++) {
      if (vert_check[x] > 0) {
          right_sum++;
      }
  }

  for (int x = middle - slice; x <= middle + slice; x++) {
    if (x < 0 || x >= h) continue; 

    if (vert_check[x] == 0) { 
        gap = true;
        break;
    }
}
if(gap){
    printf("Left:%d Right:%d", left_sum, right_sum);
    if (right_sum > left_sum) {
        right = true;
    } else if (left_sum > right_sum) {
        left = true;
    }
    else {right = true;}
  }
    }
  }
  cmd = 1;
  if(spin){cmd = 4;}
  else {
    if (left){ cmd = 2;}
    else if (right){cmd = 3;};
  }
  pthread_mutex_lock(&mutex);
  move_global[0].cmd = cmd;
  move_global[0].updated = true;
  pthread_mutex_unlock(&mutex);
  if(det_draw1){
    uint8_t *source2 = (uint8_t *)img->buf;
    uint8_t *dest2 = Buffer_2;
    for(short i = 0; i<h; i++){
      for(short j = 0; j<w; j+=2){
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
        source2[1] = dest2[0];
        source2[0] = 128;
        source2[3] = dest2[1];
        source2[2] = 128;  
        dest2+= 2;
        source2 += 4;
      }
    }
  }
  return img;
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