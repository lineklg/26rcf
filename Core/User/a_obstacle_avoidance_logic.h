#ifndef A_OBSTACLE_AVOIDANCE_LOGIC_H
#define A_OBSTACLE_AVOIDANCE_LOGIC_H

#include <stdint.h>

uint16_t AObstacleAvoidance_NextState(uint16_t state_id,
                                      uint8_t *detour_started);
uint32_t AObstacleAvoidance_DistanceDelayMs(float distance_m,
                                            float speed_mps);

#endif
