#include <stdint.h>
#include "modules/computer_vision/cv.h"

#ifndef TCT_floor_DET
#define TCT_floor_DET


extern void g4_floor_det_init(void);
extern void g4_floor_det_periodic(void);

extern uint8_t R_lim;
extern uint8_t G_lim;
extern uint8_t B_lim;
extern bool det_draw1;
extern bool det_draw2;
#endif