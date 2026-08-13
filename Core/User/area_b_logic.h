#ifndef AREA_B_LOGIC_H
#define AREA_B_LOGIC_H

#include <stdint.h>

typedef struct
{
    uint8_t should_irrigate;
    uint8_t next_position;
    uint16_t next_state;
} AreaBPositionDecision;

AreaBPositionDecision AreaB_DecidePosition(uint8_t current_position,
                                           uint8_t situation);

#endif
