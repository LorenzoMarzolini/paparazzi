// Some setup stuff I don't know
#include "modules/team4_opticflow_module/team4_opticalflow.h"

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

#include <stdio.h>

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define TEAM4_OPTICFLOW_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[team4_opticflow->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if TEAM4_OPTICFLOW_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// define global variables
int32_t flow_x = 0.0;
int32_t flow_y = 0.0;
int32_t flow_der_x = 0.0;
int32_t flow_der_y = 0.0;
float quality = 0.0;
float size_divergence = 0.0;

float velocity_z = 0.0;

#ifndef FLOW_OPTICFLOW_ID
#define FLOW_OPTICFLOW_ID ABI_BROADCAST
#endif  

static abi_event opticflow_ev;
static void opticalflow_cb(uint8_t __attribute__((unused)) sender_id, uint32_t __attribute__((unused)) stamp,
                         int32_t _flow_x, int32_t _flow_y, int32_t _flow_der_x, int32_t _flow_der_y,
                         float _quality, float _size_divergence)
{ 
  flow_x = _flow_x;
  flow_y = _flow_y;
  flow_der_x = _flow_der_x;
  flow_der_y = _flow_der_y;
  quality = _quality;
  size_divergence = _size_divergence;
}


void team4_opticalflow_init(void)
{
  // bind to the optical flow results
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &opticflow_ev, opticalflow_cb);
}

void team4_opticalflow_periodic(void)
{
  if(!autopilot_in_flight()){
    return;
  }

  // do some fancy control here
  VERBOSE_PRINT("flow_y: %d", flow_y);
}
