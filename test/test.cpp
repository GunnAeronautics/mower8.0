#include <unity.h>
#include "test.h"

// There is no need for testing data printing mode

void test_PadTransition(void)
{
    flightState = 1;
    flightStateAdvancementTrigger = 3;

    altitudeAccel = 0.0f;
    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, flightState, "Flight state should not transition on pad");

    altitudeAccel = 50.0f;
    flightStateAdvancementTrigger = 3;
    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(2, flightState, "Flight state should transition to 2 on launch");
}

void test_Ascent(void)
{
    flightState = 2;
    altitudeAccel = 1.0f;
    timeElapsed = 0;
    altitudeV580 = 1.0f;
    flightStateAdvancementTrigger = 2;

    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, flightStateAdvancementTrigger, "Advancement Trigger should reset to 0");

    altitudeAccel = -1.0f;
    for (int i = 0; i <= 11; i++)
    {
        loop();
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(3, flightState, "Flight state should transition to 3 on coast");
}

void test_DeployableCoastUpCheck(void)
{
    flightState = 3;
    altitudeV580 = 1.0f;
    timeElapsed = 100;
    flightStateAdvancementTrigger = 0;

    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, flightStateAdvancementTrigger, "trigger not incremented yet");

    altitudeV580 = 0.0f;
    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, flightStateAdvancementTrigger, "trigger increments when velocity < 0.5");

    altitudeV580 = 1.0f;
    timeElapsed = 20000;
    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(2, flightStateAdvancementTrigger, "trigger increments with 10s safety");

    flightStateAdvancementTrigger = 0;
    for (int i = 0; i <= 11; i++)
    {
        loop();
    }
    TEST_ASSERT_EQUAL_INT_MESSAGE(4, flightState, "Flight state should reach 4 on coast down");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, flightStateAdvancementTrigger, "trigger reset to zero");
}

void test_DeployableCoastUpBangBang(void)
{
    flightState = 3;
    altitudeV580 = 1.0f;
    timeElapsed = 100;
    altitude = 250.0f;
    flightStateAdvancementTrigger = 0;

    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, dragFlapDeployed, "Drag flaps should deploy when above apogee");

    altitude = 200.0f;
    altitudeV580 = 1.0f;
    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, dragFlapDeployed, "Drag flaps should not remain deployed when below desired apogee");

    lastDeployTime = -10000;
    altitude = 228.5;
    altitudeV580 = 2.0;
    integratedVelocity = 8.0;
    timeElapsed = 9000;
    dragFlapDeployed = false;

    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, dragFlapDeployed, "Airbrake PID may not deploy immediately in this code path");

    lastDeployTime = -10000;
    altitude = -10.0f;
    dragFlapDeployed = true;

    loop();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, dragFlapDeployed, "Drop flaps when below ground level (edge case)");
}

void test_DeployableCoastDown(void)
{
    flightState = 4;
    timeElapsed = 36500;
    altitude = 0.0f;
    parachuteDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, parachuteDeployed, "Parachute should deploy when predicted ground time too short");

    timeElapsed = 0;
    altitude = 10.0f;
    parachuteDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, parachuteDeployed, "Parachute should deploy when very low");

    timeElapsed = 0;
    altitude = 100.0f;
    parachuteDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, parachuteDeployed, "Parachute deployment expected at 100m with large remaining time");

    microReady = true;
    parachuteDeployed = true;
    timeElapsed = 0;
    altitude = 100.0f;
    altitudeV580 = 1.0f;
    dragFlapDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, dragFlapDeployed, "No flap opening near ground unless condition met");

    microReady = true;
    parachuteDeployed = true;
    timeElapsed = -70000;
    dragFlapDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, parachuteDeployed, "Parachute remains deployed when rapid descent");

    microReady = true;
    parachuteDeployed = true;
    timeElapsed = 100;
    dragFlapDeployed = false;
    downwardLogic();
    TEST_ASSERT_EQUAL_INT_MESSAGE(1, parachuteDeployed, "Parachute remains deployed around low time");
}
