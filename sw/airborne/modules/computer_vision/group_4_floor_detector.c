#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>
#include <pthread.h>

#include "modules/computer_vision/lib/vision/image.h"
#include "group_4_floor_detector.h"
#include "modules/core/abi.h"

uint8_t R_lim = 100;
uint8_t G_lim = 100;
uint8_t B_lim = 70;

bool det_draw1 = FLOOR_DET_DRAW;
bool det_draw2 = FLOOR_DET_DRAW2;

typedef struct msg_det {
  uint8_t cmd;
  bool updated;
} msg_det;

msg_det move_global[1];

uint8_t Buffer_2[HEIGHT * WIDTH];
uint8_t Buffer_3[HEIGHT * WIDTH];
uint8_t Buffer_4[HEIGHT * WIDTH];
bool right = false;
bool left = false;
bool spin = false;
bool gap = false;
uint8_t cmd = 0;
uint8_t hort_check[WIDTH];
uint8_t vert_check[WIDTH];
int left_sum = 0;
int right_sum = 0;

static pthread_mutex_t mutex;

// SVM CONSTS
#define NUM_FEATURES 136
#define NUM_CLASSES 1
#define DSF 30

static const double svm_coefficients[NUM_CLASSES][NUM_FEATURES] = {
    {
      -0.29309039136074766, 0.2855126472852318, -0.8815636193507481, 0.0, 0.0, 0.0, 0.0,
      -0.3337508185709339, 0.23844752986268913, -0.15382879127505972, 0.23929505300714823, 0.0,
      0.0, 0.0, 0.0, 0.0, -0.5682652762871319, 0.4647882095870011, 0.23929505300714823, 0.0,
      0.0, 0.0719534099285286, 0.0, 0.0, 0.14656320814933838, 0.4933103353281445, 0.6486319256539266,
      0.663852885783037, 0.8284074507977163, 0.0, 0.27159452992287464, 0.0, -0.3749108220788966,
      0.2501885149777319, 0.8117795439714822, 0.9617818197030583, 0.0, 0.0, 0.14514857436763817,
      0.0, 1.1866219733394845, 0.08744717527240337, 0.3925148762414158, -0.38173110991786613,
      -0.8629583206454123, -0.3645830538769453, 0.0, 0.0, 0.8081412213021018, -0.15069315253234916,
      -0.12304145668571342, 0.663852885783037, 0.4245578327758888, 0.0, 0.0, 0.0,
      0.6317056704282755, 0.5234855735918738, -0.525328658931751, 0.3489268212557386, -0.3659140685961238,
      -0.4984317297048043, -1.1119499087596283, -0.07005461463543923, 1.2348396131201549,
      -0.11624982467874223, 0.12649444615486832, -0.14950490844906578, 0.2569200536867092,
      -0.1285377125795339, 0.0, -0.4984317297048043, 1.0152209601291105, -1.1058379293020348,
      -0.08002725588843024, -0.15319361002577997, 0.6313925646115038, -0.4984317297048043,
      -0.06636591305872509, 0.6977584776702289, 0.8618014018748432, 0.5803086456917608,
      -0.14819587452768074, 0.10963176824859033, -0.6433528736116896, -1.132226414156248,
      -0.003688701576714145, -0.9798100600651549, 1.2233444067196388, -0.35997686241684446,
      -0.3843231391033499, -0.7337095667579443, -0.43846720458161864, 0.1631476183175557,
      -0.4212387465552579, -0.003688701576714145, 0.8683895950398859, -0.5219829661155828,
      0.07203485041539442, -0.07563101152015025, -0.386302374282904, -0.003688701576714145,
      0.0, 0.0, 0.7865426274838512, 1.1332411382298015, -0.018672953765361433,
      -0.38129197758602357, 0.0, -0.07319516443910958, 0.0, -0.5148151610321557,
      0.3544489765629956, 0.14556986472667627, 0.3091078448277635, 0.23929505300714823,
      0.0, -0.38813735977827457, -0.3337508185709339, 0.0, -0.3915735602882702,
      0.32059870830523396, -0.0846158505558704, 0.7376938040722347, 0.23792833072702183,
      -0.6336669774010782, -0.38813735977827457, 0.0, -0.3917182784415165, 0.2506162325930821,
      0.07778854673411759, 0.0, 0.0, 0.0, 0.0, 0.0
    }
};

static const double svm_intercepts[NUM_CLASSES] = { -7.278287104923872 };




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




uint8_t* downsample_binary(const uint8_t *binary, int width, int height, int dsf, int *new_width, int *new_height) {
    *new_width = width / dsf;
    *new_height = height / dsf;
    uint8_t *down = malloc((*new_width) * (*new_height) * sizeof(uint8_t));
    for (int i = 0; i < *new_height; i++) {
        for (int j = 0; j < *new_width; j++) {
            down[i * (*new_width) + j] = binary[i * dsf * width + j * dsf];
        }
    }
    return down;
}

void extract_features(const uint8_t *down, int new_width, int new_height, double *feature_vector) {
    int total_pixels = new_width * new_height;
    int step = total_pixels / NUM_FEATURES;
    if (step < 1) step = 1;
    int idx = 0;
    for (int i = 0; i < total_pixels && idx < NUM_FEATURES; i += step) {
        feature_vector[idx++] = (down[i] == 1) ? 1.0 : 0.0;
    }
    for (; idx < NUM_FEATURES; idx++) {
        feature_vector[idx] = 0.0;
    }
}

int classify_image(const double feature_vector[NUM_FEATURES]) {
    double decision_value = 0.0;
    for (int j = 0; j < NUM_FEATURES; j++) {
        decision_value += svm_coefficients[0][j] * feature_vector[j];
    }
    decision_value += svm_intercepts[0];
    int ret = (decision_value < 0.0) ? 1 : 2;
    // return (decision_value > 0.0) ? 1 : 0;
    return ret;
}

int classify_floor(const uint8_t *binary, int width, int height) {
    int ds_width, ds_height;
    uint8_t *down = downsample_binary(binary, width, height, DSF, &ds_width, &ds_height);

    double feature_vector[NUM_FEATURES];
    extract_features(down, ds_width, ds_height, feature_vector);
    free(down);
    int predicted_class = classify_image(feature_vector);
    return predicted_class;
}

static struct image_t *g4_floor_det_func(struct image_t *img, uint8_t camera_id __attribute__((unused))) {  
    short h = img->h;
    short w = img->w;

    uint8_t *source = (uint8_t *)img->buf;
    uint8_t *dest = Buffer_2;
    
    for (short i = 0; i < h; i++) {
        for (short j = 0; j < w; j += 2) {
            short y1 = (short)source[1];
            short u  = (short)source[0] - 128;
            short y2 = (short)source[3];
            short v  = (short)source[2] - 128;
            short R1 = y1 + ((1436 * v) >> 10);
            short G1 = y1 - ((352 * u + 731 * v) >> 10);
            short B1 = y1 + ((1814 * u) >> 10);
            short R2 = y2 + ((1436 * v) >> 10);
            short G2 = y2 - ((352 * u + 731 * v) >> 10);
            short B2 = y2 + ((1814 * u) >> 10);
            dest[0] = (R1 < R_lim && B1 < B_lim && G1 > G_lim) ? 255 : 0;
            dest[1] = (R2 < R_lim && B2 < B_lim && G2 > G_lim) ? 255 : 0;
            dest += 2;
            source += 4;
        }
    }

    int predicted_move = classify_floor(Buffer_2, w, h);

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



    uint8_t final_cmd;
    if (cmd > 1 && predicted_move == 2) {
        final_cmd = cmd;
    } else if (cmd == 1 && predicted_move == 1) {
        final_cmd = 1;
    } else if (cmd == 1 && predicted_move == 2) {
        final_cmd = 2;
    } else {
        final_cmd = cmd;
    }


    pthread_mutex_lock(&mutex);
    move_global[0].cmd = final_cmd;
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


void g4_floor_det_init(void) {
    memset(move_global, 0, sizeof(msg_det));
    pthread_mutex_init(&mutex, NULL);
    cv_add_to_device(&FLOOR_DET_CAMERA, g4_floor_det_func, FLOOR_DETECTOR_FPS, 0);
}

void g4_floor_det_periodic(void) {
    static msg_det move[1];
    pthread_mutex_lock(&mutex);
    memcpy(move, move_global, sizeof(msg_det));
    pthread_mutex_unlock(&mutex);
    if (move[0].updated) {
        AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, move[0].cmd, (int16_t)0);
        move[0].updated = false;
    }
}
