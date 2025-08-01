
// g++ --std=c++11 test.cpp

#include "test.h"

#include "test.h"
#include "FastLED.h"

#include "fl/namespace.h"
FASTLED_USING_NAMESPACE

#define NUM_LEDS 1000
#define DATA_PIN 2
#define CLOCK_PIN 3

TEST_CASE("Simple") {
    static CRGB leds[NUM_LEDS];  // Use static to avoid global constructor warning
    FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
}
