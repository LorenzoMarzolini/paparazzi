

#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"
#include "group_4_floor_detector.h"


void g4_floor_det_init(void)
{
  cv_add_to_device(&FLOOR_DET_CAMERA, g4_floor_det_func, 0, 0);
}




static struct image_t *g4_floor_det_func(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{  
  short h = img->h;
  short w = img->w;
  
  for(short i = 0; h; i++){
    for(short j = 0; w; j+=2){
      // check if in image or not
    }
  }


  return img; // func did not make a new image
}