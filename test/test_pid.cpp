#include <Arduino.h>
#include <unity.h>
#include "PIDController.h"

void test_pid_zero_error(void)
{
    PIDController pid(1, 1, 1);
    pid.reset();
    float out = pid.update(10, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0.0, out);
}

void test_pid_positive_error(void)
{
    PIDController pid(2, 0, 0);
    pid.reset();
    float out = pid.update(5, 3); // error = 2 → output = kp*2 = 4
    TEST_ASSERT_FLOAT_WITHIN(0.01, 4.0, out);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_zero_error);
    RUN_TEST(test_pid_positive_error);
    UNITY_END();
}

void loop() {}
