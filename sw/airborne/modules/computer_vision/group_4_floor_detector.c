

#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"
#include "group_4_floor_detector.h"
#include "modules/core/abi.h"


uint8_t R_lim = 100;
uint8_t G_lim = 100;
uint8_t B_lim = 70;

bool det_draw1 = FLOOR_DET_DRAW;


static pthread_mutex_t mutex;
static struct msg_det{
  uint8_t cmd;
  bool updated;
};

uint8_t Buffer_2[520*240];

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