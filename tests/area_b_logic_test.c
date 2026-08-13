#include "area_b_logic.h"

#include <assert.h>

static void TestZeroEvenPositionSkipsToRight(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(0U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 1U);
    assert(decision.next_state == 3U);
}

static void TestZeroOddPositionSkipsToCenter(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(1U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 2U);
    assert(decision.next_state == 6U);
}

static void TestNonzeroPositionRequiresIrrigation(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(2U, 3U);

    assert(decision.should_irrigate == 1U);
    assert(decision.next_position == 3U);
    assert(decision.next_state == 3U);
}

static void TestLastPositionAdvancesPastArea(void)
{
    AreaBPositionDecision decision = AreaB_DecidePosition(5U, 0U);

    assert(decision.should_irrigate == 0U);
    assert(decision.next_position == 6U);
    assert(decision.next_state == 6U);
}

static void TestConsecutiveZeroPositionsAdvanceInOrder(void)
{
    AreaBPositionDecision left = AreaB_DecidePosition(2U, 0U);
    AreaBPositionDecision right = AreaB_DecidePosition(left.next_position, 0U);

    assert(left.next_position == 3U);
    assert(left.next_state == 3U);
    assert(right.next_position == 4U);
    assert(right.next_state == 6U);
}

int main(void)
{
    TestZeroEvenPositionSkipsToRight();
    TestZeroOddPositionSkipsToCenter();
    TestNonzeroPositionRequiresIrrigation();
    TestLastPositionAdvancesPastArea();
    TestConsecutiveZeroPositionsAdvanceInOrder();
    return 0;
}
