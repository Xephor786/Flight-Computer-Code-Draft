#include <gtest/gtest.h>
#include "../include/altimeter.h"
// uncomment line below if you plan to use GMock
// #include <gmock/gmock.h>

// TEST(...)
// TEST_F(...)

#if defined(ARDUINO)
#include <Arduino.h>
TEST(DummyTest, BasicAssertions) {
    // You can use any assertion macros here
    double initial_alt=1;
    bool result = check_apogee(initial_alt);
    EXPECT_FALSE(result); // "Apogee should not be reached at initial altitude.";
}
void setup()
{
    // should be the same value as for the `test_speed` option in "platformio.ini"
    // default value is test_speed=115200
    Serial.begin(115200);

    ::testing::InitGoogleTest();
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock();
}

void loop()
{
  // Run tests
  if (RUN_ALL_TESTS())
  ;

  // sleep for 1 sec
  delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS())
    ;

    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
#endif