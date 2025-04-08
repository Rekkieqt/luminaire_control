#ifndef CMD_H
#define CMD_H
#include "ring_buffer.h"
#include "comms.h"

void get_command(float v, float L, float u, float& ener, float& flicker, float& vis_err, unsigned long& N,
                 ring_buffer* _data_log, canbus_comm* hermes, msg_to_can* inner_frame);
void set_dutycycle(float val);
#endif