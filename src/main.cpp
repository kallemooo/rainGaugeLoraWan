/**
 * @file main.cpp
 * @author Karl Thorén (karl.h.thoren@gmail.com)
 * @brief Main application.
 * @version 0.1
 * @date 2020-07-26
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <util/atomic.h>
#include <fade.h>
#include "common.h"

#ifndef APPEUI_KEY
#error "No APPEUI key defined. See example_lorawan_keys.ini for info."
#define APPEUI_KEY ("This value should be specified in lorawan_keys.ini")
#endif

#ifndef DEVEUI_KEY
#error "No DEVEUI_KEY key defined. See example_lorawan_keys.ini for info."
#define DEVEUI_KEY ("This value should be specified in lorawan_keys.ini")
#endif

#ifndef APPKEY_KEY
#error "No APPKEY_KEY key defined. See example_lorawan_keys.ini for info."
#define APPKEY_KEY ("This value should be specified in lorawan_keys.ini")
#endif

// Configured in hte lorawan_keys.ini file.
static const u1_t PROGMEM APPEUI[8] = {APPEUI_KEY};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// Configured in hte lorawan_keys.ini file.
static const u1_t PROGMEM DEVEUI[8] = {DEVEUI_KEY};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// Configured in hte lorawan_keys.ini file.
static const u1_t PROGMEM APPKEY[16] = {APPKEY_KEY};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

typedef enum _st_t
{
    ST_JOINING = 1,
    ST_JOIN_DONE,
    ST_TX_RX_PENDING,
    ST_IDLE
} st_t;

static st_t state = ST_JOINING;

#if defined(ARDUINO_AVR_FEATHER32U4)
/**
 * @brief Pin mapping for Adafruit Feather 32u4 LoRa.
 *
 * Using Arduino pin 1 for RFM95 DIO 1 as Arduino pin 1 is one of the external interrupt pins.
 */
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 1, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 4000000,
};
#else
#error "Unknown target"
#endif

static osjob_t sendJob;

// Schedule TX every this many seconds. Multiplied with TX_INTERVAL_MULTIPLEXER.
static const uint16_t TX_INTERVAL = 60u;
static const uint16_t TX_INTERVAL_MULTIPLEXER_MIN = 1u;
static uint8_t TX_INTERVAL_MULTIPLEXER = 15u;

/**
 * @brief Sleep counter max value.
 *
 * When this is reached the device sleeps until next external interrupt.
 * Is set to one hour wait for more rain.
 */
static const uint16_t sleepCntMax = 60u*60u/8u;
static volatile uint16_t sleepCnt = 1u;

// Fade handler.
static fade fader(LED_BUILTIN);

// Rain counter.
static volatile uint32_t counts = 0u;

// Battery voltage.
static volatile float measuredvbat = 0.0f;

// Number of main function loops before allowing sleep.
static const uint8_t maxLoopsBeforeSleep = 5u;
static uint8_t loopsBeforeSleep = 0u;


static uint64_t previousMillis = 0u;

static void idleState();
void countRaindrop();

/**
 * @brief Measures battery voltage.
 */
static void measureVbat()
{
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2.0f;    // we divided by 2, so multiply back
    measuredvbat *= 3.3f;    // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024u; // convert to voltage
}

/**
 * @brief Fetches the data and sends it.
 *
 * @param j Job identifier.
 */
void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (!(LMIC.opmode & OP_TXRXPEND))
    {
        state = ST_TX_RX_PENDING;
        analogWrite(LED_BUILTIN, 50u);
        // Copy counter value in interrupt safe context.
        uint32_t c;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            c = counts;
        }

        // Send buffer.
        uint8_t buff[5];

        // Data encoded in 3 bytes.
        // First two bytes and bit 7 of the third byte is the counter.
        // 17 bits is used for the counter. It will easy cover heavy rain for days.
        buff[0] = ((uint8_t)((c) &  0xFFu));
        buff[1] = ((uint8_t)((c) >> 8u));
        uint8_t top  = ((uint8_t)((c) >> 16u));
        if (top > 0u)
        {
            top = 0x80u;
        }

        // 7 bits of the third byte is the vbat voltage.
        // Multiply with 100 and remove 330 so it fits to 7 bits.
        // It will not reach below 3v as the device will powerdown before that.
        // Maximum voltage is not above 4.3 v so 7 bits is ok
        uint8_t vbat = (uint8_t)((uint16_t)(measuredvbat * 100u) - 330u) & 0x7Fu;
        buff[2] = top | vbat;
        buff[3] = highByte(sleepCnt);
        buff[4] = lowByte(sleepCnt);
        LMIC_setTxData2(1u, buff, sizeof(buff), 0u);
    }
}

/**
 * @brief LMIC event handler.
 *
 * @param ev Current signaled event.
 */
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_JOINING:
        state = ST_JOINING;
        // Start with SF9.
        LMIC_setDrTxpow(EU868_DR_SF9, 14);
        Serial.println(F("EV_J"));
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_J_TXC"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        state = ST_JOIN_DONE;
        digitalWrite(LED_BUILTIN, LOW);
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_J_F"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJ_F"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXC"));
        if (LMIC.txrxFlags & TXRX_ACK)
        {
            Serial.println(F("R A"));
        }
        if (LMIC.dataLen)
        {
            // Channel is at LMIC.dataBeg - 1.
            u1_t channel = LMIC.frame[LMIC.dataBeg - 1];
            Serial.print(F("Receive len: "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" b"));
            Serial.print(F("Channel: "));
            Serial.println(channel);
            if (1u == channel)
            {
                // Channel for changing transmit interval multiplexer.
                if (2u == LMIC.dataLen && 1u == LMIC.frame[LMIC.dataBeg])
                {
                    u1_t newMultiplexer = LMIC.frame[LMIC.dataBeg + 1];
                    if (newMultiplexer >= TX_INTERVAL_MULTIPLEXER_MIN)
                    {
                        // New value is approved. Apply.
                        TX_INTERVAL_MULTIPLEXER = newMultiplexer;
                        Serial.print(F("New TX multiplexer: "));
                        Serial.println(TX_INTERVAL_MULTIPLEXER);
                    }
                }
            }
            if (2u == channel)
            {
                // Channel for clearing of raindrop counter.
                if ((1u == LMIC.dataLen) && (0xEA == LMIC.frame[LMIC.dataBeg]))
                {
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                    {
                        counts = 0u;
                    }
                    Serial.println(F("Cleared counts variable."));
                }
            }
        }
        // Schedule next transmission
        os_setTimedCallback(&sendJob, os_getTime() + sec2osticks(TX_INTERVAL * TX_INTERVAL_MULTIPLEXER), do_send);
        if (!(LMIC.opmode & OP_POLL))
        {
            analogWrite(LED_BUILTIN, 2u);
            state = ST_IDLE;
        }
        else
        {
            analogWrite(LED_BUILTIN, 10u);
            Serial.println(F("OP_POLL"));
            // Confirmed downlink requested.
        }

        break;
    case EV_RESET:
        Serial.println(F("EV_R"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXC"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXS"));
        state = ST_TX_RX_PENDING;
        analogWrite(LED_BUILTIN, 50u);
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

/**
 * @brief Setup handler
 *
 * Configures LoRaWan LMIC driver and enables the external interrupt.
 */
void setup()
{
    measureVbat();
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);

    // Turn on the LED to indicate startup mode.
    analogWrite(LED_BUILTIN, 2u);
    Serial.begin(115200);
    delay(100);
    if (UDADDR & _BV(ADDEN))
    {
        while (!Serial)
        {
            delay(1); // wait for serial port to connect. Needed for native USB
        }
        Serial.println(F("S"));
    }

    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), countRaindrop, FALLING);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1u / 100u);

    measureVbat();
    Serial.print(F("VBat: "));
    Serial.println(measuredvbat);

    // Start job (sending starts OTAA too)
    do_send(&sendJob);
    state = ST_JOINING;
}


static volatile unsigned long last_interrupt_time = 0;
static volatile bool wakeup_by_interrupt = false;

/**
 * @brief Interrupt handler.
 *
 */
void countRaindrop()
{
    wakeup_by_interrupt = true;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 10ms, assume it's a bounce and ignore it.
    if ((interrupt_time - last_interrupt_time) > 10u)
    {
        counts++;
    }
    last_interrupt_time = interrupt_time;
}

/**
 * @brief Main program loop.
 *
 */
void loop()
{
    os_runloop_once();

    switch (state)
    {
    case ST_JOINING:
    {
        fader.doFade(millis());
    }
    break;
    case ST_JOIN_DONE:
    {
        state = ST_TX_RX_PENDING;
    }
    break;
    case ST_IDLE:
    {
        idleState();
    }
    default:
    break;
    }

    if (UDADDR & _BV(ADDEN))
    {
        // Only run this part if USB is connected.
        unsigned long currentMillis = millis();
        if ((currentMillis - previousMillis) >= 10000u)
        {
            // Copy counter value in interrupt safe context.
            uint32_t c;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                c = counts;
            }
            Serial.print(F("counts: "));
            Serial.print(c);
            Serial.print(F(" VBat: "));
            Serial.println(measuredvbat);
            previousMillis = currentMillis;
        }
    }
    measureVbat();
}

/**
 * @brief Idle state handler.
 *
 * Handles sleep states.
 */
static void idleState()
{
    if (!(UDADDR & _BV(ADDEN)))
    {
        loopsBeforeSleep++;
        if (loopsBeforeSleep > maxLoopsBeforeSleep)
        {
            digitalWrite(LED_BUILTIN, LOW);
            loopsBeforeSleep = 0u;
            if (measuredvbat < 3.5f)
            {
                // To low battery voltage. Powerdown to not discharge battery
                // below the level that will destroy the battery.
                // Disconnect interrupt and put device in deep sleep forever.
                detachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN));
                LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
            }

            if (sleepCnt > sleepCntMax)
            {
                // No rain for a long time. Sleep until next rainfall.
                LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
                sleepCnt = 0u;
            }
            else
            {
                LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
                if (wakeup_by_interrupt)
                {
                    // Wakeup by interrupt. Set counter to 1.
                    sleepCnt = 1u;
                }
                else
                {
                    // Wakeup by timeout.
                    sleepCnt++;
                }
            }

            wakeup_by_interrupt = false;

            // Give the AVR back the sleep time
            //Needs '#include <util/atomic.h>'
            const unsigned long slept = 8u * 1000u; // 8 s sleep time.
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                extern volatile unsigned long timer0_millis;
                extern volatile unsigned long timer0_overflow_count;
                timer0_millis += slept;
                // timer0 uses a /64 prescaler and overflows every 256 timer ticks
                timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000u) / (64 * 256);
            }

            // VERY IMPORTANT after sleep to update os_time and not cause txbeg and os_time
            // out of sync which causes send delays with the RFM95 on and eating power
            os_getTime();
        }
    }

    if (0u == sleepCnt)
    {
        // Wakeup after long time sleep. Sent message to fetch any incoming data.
        os_setCallback(&sendJob, do_send);
        sleepCnt++;
    }
}
