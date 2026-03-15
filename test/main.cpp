#include <unity.h>

extern void test_PadTransition();
extern void test_Ascent();
extern void test_DeployableCoastUpCheck();
extern void test_DeployableCoastUpBangBang();
extern void test_DeployableCoastDown();

void setUp(void) {}
void tearDown(void) {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_PadTransition);
    RUN_TEST(test_Ascent);
    RUN_TEST(test_DeployableCoastUpCheck);
    RUN_TEST(test_DeployableCoastUpBangBang);
    RUN_TEST(test_DeployableCoastDown);

    return UNITY_END();
}
