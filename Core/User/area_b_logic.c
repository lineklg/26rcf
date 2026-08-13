#include "area_b_logic.h"

AreaBPositionDecision AreaB_DecidePosition(uint8_t current_position,
                                           uint8_t situation)
{
    AreaBPositionDecision decision;

    decision.should_irrigate = situation != 0U;
    decision.next_position = current_position + 1U;
    decision.next_state = current_position % 2U == 0U ? 3U : 6U;
    return decision;
}
