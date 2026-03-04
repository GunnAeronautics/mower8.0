#include <gtest/gtest.h>

#include "Airbreaks.h"

// There is no need for testing data printing mode

// Pad transition test
TEST(LOOP, PadTransition) {

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
    for(int i = 0; i <= 11; i++){
        loop();  
    }
    ASSERT_EQ(flightState, 3) << "Flight state should transition to 3 on coast. Double check ascent code";
    
}



TEST(LOOP, DeployableCoastDownCheck)
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

}

TEST(FactorialTestFalling, DeployableFall)
{
    int flightState = 4; // falling
    float DESIRED_APOGEE = 228.6;
    float altitudeV = 1.0f;
    float maxAltitude = 300.0f;
    double altitude = 250.0f;
    int timeElapsed = 100;
    int flightStateAdvancementTrigger = 0;
    bool dragFlapDeployed = false;
    float COEF_DRAG_FLAPDEPLOYED = 0.004333;
    int lastDeployTime = 0;

    // alt > desired apogee, overshoot safety
    altitude = 250.0f;
    loop();
    EXPECT_EQ(dragFlapDeployed, true) << "Drag flaps should be deployed.";

    // alt < desired apogee
    altitude = 200.0f;
    loop();

}

TEST(FactorialTestFalling, DeployableFallingLogic)
{
     int flightState = 4; // falling
    float DESIRED_APOGEE = 228.6;
    float altitudeV = 1.0f;
    float maxAltitude = 300.0f;
    double altitude;
    int timeElapsed = 100;
    int flightStateAdvancementTrigger = 0;
    bool dragFlapDeployed = false;
    float COEF_DRAG_FLAPDEPLOYED = 0.004333;



    int lastDeployTime = 0;

    // alt > desired apogee, overshoot safety
    altitude = 250.0f;
    loop();
    EXPECT_EQ(dragFlapDeployed, true) << "Drag flaps should be deployed.";

    // alt < desired apogee
    altitude = 200.0f;
    dragFlapDeployed = false;

    loop();
    EXPECT_EQ(dragFlapDeployed, false) << "Drag flaps should be deployed.";

}

