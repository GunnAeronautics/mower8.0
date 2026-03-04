#include <gtest/gtest.h>

#include "test.h"

// There is no need for testing data printing mode

// Pad transition test
TEST(LOOP, PadTransition)
{

    flightState = 1;

    altitudeAccel = 0.0f;
    loop();
    ASSERT_EQ(flightState, 1) << "Flight state should not transition on pad. Double check pad code";

    altitudeAccel = 50.0f;
    loop();
    ASSERT_EQ(flightState, 2) << "Flight state should transition to 2 on launch. Double check pad code";
}

TEST(LOOP, Ascent)
{
    flightState = 2;
    altitudeAccel = 1.0f;
    timeElapsed = 0;
    altitudeV = 1.0f;
    flightStateAdvancementTrigger = 2;

    loop();
    ASSERT_EQ(flightStateAdvancementTrigger, 0) << "Advancement Trigger should reset to 0";

    altitudeAccel = -1.0f;
    for (int i = 0; i <= 11; i++)
    {
        loop();
    }
    ASSERT_EQ(flightState, 3) << "Flight state should transition to 3 on coast. Double check ascent code";
}

TEST(LOOP, DeployableCoastUpCheck)
{
    flightState = 3; // coast up
    altitudeV = 1.0f;
    timeElapsed = 100;
    flightStateAdvancementTrigger = 0;

    loop();
    ASSERT_EQ(flightStateAdvancementTrigger, 0) << "Advancement trigger conditions not met, should not increment";

    altitudeV = 0.0f;
    loop();
    ASSERT_EQ(flightStateAdvancementTrigger, 1) << "Advancement trigger should increment when velocity < 0.5 m/s";
    altitudeV = 1.0f;
    timeElapsed = 20000;
    loop();
    ASSERT_EQ(flightStateAdvancementTrigger, 2) << "Advancement trigger should increment when 10 second safety is met";
    flightStateAdvancementTrigger = 0;
    for (int i = 0; i <= 11; i++)
    {
        loop();
    }
    ASSERT_EQ(flightState, 4) << "Flight state should transition to 4 on coast down. Double check coast down code";
    ASSERT_EQ(flightStateAdvancementTrigger, 0) << "Advancement Trigger should reset to 0";
}

TEST(LOOP, DeployableCoastUpBangBang)
{    
    flightState = 3;                                             // coast up
    altitudeV = 1.0f;
    timeElapsed = 100;
    altitude = 250.0f;                                           // 228.6 is the desired apogee
    flightStateAdvancementTrigger = 0;
    bool dragFlapDeployed = false;

    // alt > desired apogee, overshoot safety
    loop();
    EXPECT_EQ(dragFlapDeployed, true) << "Drag flaps should be deployed.";

    // alt < desired apogee
    altitude = 200.0f; 
    altitudeV = 1.0f; //10

    loop();
    EXPECT_EQ(dragFlapDeployed, false) << "Drag flaps should be Closed!";

    lastDeployTime = -10000;
    altitude = 228.5;        // <= desired appogyee
    altitudeV = 2.0;        // >= 0.5 and enough to exceed apogee
    integratedVelocity = 8.0;  // predictApogee > 228.6
    timeElapsed = 9000;     // <= 10000
    dragFlapDeployed = false;

    loop();
    EXPECT_EQ(dragFlapDeployed, true) << "Drag flaps should be deployed when predicted apogee is above desired apogee.";
    
    lastDeployTime = -10000;
    altitude = -10.0f; //testing edge edge case
    dragFlapDeployed = true;
    
    loop();
    EXPECT_EQ(dragFlapDeployed, false) << "Drag flaps should be closed when altitude is lower than 228.6 at > 500 delta deploy";


}

TEST(LOOP, DeployableCoastDown)
{    
    flightState = 4;                                             // coast down
    timeElapsed = 36500; // part one of if = 1* 6.65
    altitude = 0.0f;
    parachuteDeployed = false;
    downwardLogic(); // testing for only first if
    EXPECT_EQ(parachuteDeployed, true) << "Predicted time to ground is too fast but the parachute was not deployed";

    timeElapsed = 0;
    altitude = 10.0f;
    parachuteDeployed = false;
    downwardLogic(); // testing for only second if
    EXPECT_EQ(parachuteDeployed, true) << "Rocket is too low but the parachute was not deployed";

    
    timeElapsed = 0;
    altitude = 100.0f;
    parachuteDeployed = false;
    downwardLogic(); // testing for non deploy case
    EXPECT_EQ(parachuteDeployed, false) << "Unexpected parachute deployment";

    microReady = true;
    parachuteDeployed = true;
    timeElapsed = 0;
    altitude = 100.0f;
    altitudeV = 1.0f;
    dragFlapDeployed = false;
    downwardLogic();
    EXPECT_EQ(parachuteDeployed, false) << "Flap NOT retracting near ground";

    microReady, parachuteDeployed = true;
    timeElapsed = -70000; // too fast
    dragFlapDeployed = false;
    downwardLogic();
    EXPECT_EQ(parachuteDeployed, true) << "Flap NOT opening when going too fast";

    microReady, parachuteDeployed = true;
    timeElapsed = 100; // too fast
    dragFlapDeployed = false;
    downwardLogic();
    EXPECT_EQ(parachuteDeployed, false) << "Flap Opening when going too fast";
}
