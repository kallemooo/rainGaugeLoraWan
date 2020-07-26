/**
 * @file fade.h
 * @author Karl Thorén (karl.h.thoren@gmail.com)
 * @brief Simple PWM fade handler.
 * @version 0.1
 * @date 2020-07-26
 *
 * Inspired by https://www.baldengineer.com/fading-led-analogwrite-millis-example.html
 *
 */
#ifndef FADE_H
#define FADE_H
#include <Arduino.h>

class fade
{
public:
    fade(uint8_t ledPin, uint8_t minPWM = 0u, uint8_t maxPWM = 255u);
    virtual ~fade();

    void doFade(unsigned long thisMillis);

private:
    // define directions for LED fade
    static const uint8_t UP   = 0u;
    static const uint8_t DOWN = 1u;

    // How smooth to fade?
    static const uint8_t fadeIncrement = 5;

    // How fast to increment?
    static const uint8_t fadeInterval = 50u;

    // Min and max PWM
    uint8_t minPWM;
    uint8_t maxPWM;

    // State Variable for Fade Direction
    uint8_t fadeDirection;

    // Global Fade Value
    // but be bigger than byte and signed, for rollover
    int16_t fadeValue;

    // millis() timing Variable, just for fading
    unsigned long previousFadeMillis;

    // LED pin.
    uint8_t ledPin;
};

#endif // FADE_H
