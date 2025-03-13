

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



struct msg_det move_global[1];
static struct image_t *g4_floor_det_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{  
  short h = img->h;
  short w = img->w;
  
  for(short i = 0; i<h; i++){
    for(short j = 0; j<w; j+=2){
      // check if in image or not
    }
  }

  pthread_mutex_lock(&mutex);
  move_global[0].cmd = 1;
  move_global[0].updated = true;
  pthread_mutex_unlock(&mutex);

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