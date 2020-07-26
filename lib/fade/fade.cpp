/**
 * @file fade.cpp
 * @author Karl Thorén (karl.h.thoren@gmail.com)
 * @brief Fade handler.
 * @version 0.1
 * @date 2020-07-26
 *
 * Inspired by https://www.baldengineer.com/fading-led-analogwrite-millis-example.html
 *
 */
#include <Arduino.h>
#include "fade.h"

fade::fade(uint8_t ledPin, uint8_t minPWM, uint8_t maxPWM)
    : minPWM(minPWM)
    , maxPWM(maxPWM)
    , fadeDirection(fade::UP)
    , fadeValue(0u)
    , previousFadeMillis(0u)
    , ledPin(ledPin)
{
}

fade::~fade()
{
}

void fade::doFade(unsigned long thisMillis)
{
    // is it time to update yet?
    // if not, nothing happens
    if (thisMillis - previousFadeMillis >= fadeInterval)
    {
        // yup, it's time!
        if (fadeDirection == UP)
        {
            fadeValue = fadeValue + fadeIncrement;
            if (fadeValue >= maxPWM)
            {
                // At max, limit and change direction
                fadeValue = maxPWM;
                fadeDirection = DOWN;
            }
        }
        else
        {
            //if we aren't going up, we're going down
            fadeValue = fadeValue - fadeIncrement;
            if (fadeValue <= minPWM)
            {
                // At min, limit and change direction
                fadeValue = minPWM;
                fadeDirection = UP;
            }
        }
        // Only need to update when it changes
        analogWrite(ledPin, fadeValue);

        // reset millis for the next iteration (fade timer only)
        previousFadeMillis = thisMillis;
    }
}
