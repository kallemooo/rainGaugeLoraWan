#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>

extern unsigned long timer0_overflow_count;

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
#define LEDPIN 13
#define RAIN_GAUGE_PIN 0

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
};

static osjob_t sendJob;

// Schedule TX every this many seconds. Multipled with TX_INTERVAL_MULTIPLEXER.
const uint16_t TX_INTERVAL = 60ul;
const uint16_t TX_INTERVAL_MULTIPLEXER_MIN = 15ul;
uint8_t TX_INTERVAL_MULTIPLEXER = TX_INTERVAL_MULTIPLEXER_MIN;

// Delay direct send job.
const uint16_t TX_DIRECT_SEND_DELAY = 60ul * TX_INTERVAL_MULTIPLEXER_MIN;

#define MAX_NO_OF_COUNTS (3u)
volatile static uint32_t counts[MAX_NO_OF_COUNTS] = {0u, 0u, 0u};

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (!(LMIC.opmode & OP_TXRXPEND))
    {
        LMIC_setTxData2(1, (xref2u1_t)counts, sizeof(counts) / sizeof(counts[0]), 0u);
        state = ST_TX_RX_PENDING;
    }
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_JOINING:
        // Start with SF9.
        LMIC_setDrTxpow(EU868_DR_SF9, 14);
        Serial.println(F("EV_J"));
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_J_TXC"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            state = ST_JOIN_DONE;
            analogWrite(LEDPIN, 2);
        }
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
            u1_t channel = LMIC.frame[LMIC.dataBeg - 1];
            Serial.print(F("Receive len: "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" b"));
            Serial.print(F("Channel: "));
            Serial.println(channel);
            if (1u == channel)
            {
                // Channel for changing transmit interval multiplexer.
                if (1u == LMIC.dataLen)
                {
                    u1_t newMultiplexer = LMIC.frame[LMIC.dataBeg];
                    if (newMultiplexer >= TX_INTERVAL_MULTIPLEXER_MIN)
                    {
                        // New value is approved. Apply.
                        TX_INTERVAL_MULTIPLEXER = newMultiplexer;
                        Serial.print(F("New TX multiplexer: "));
                        Serial.println(TX_INTERVAL_MULTIPLEXER);
                    }
                }
            }
        }
        // Schedule next transmission
        os_setTimedCallback(&sendJob, os_getTime() + sec2osticks(TX_INTERVAL * TX_INTERVAL_MULTIPLEXER), do_send);
        digitalWrite(LEDPIN, LOW);

        if (!(LMIC.opmode & OP_POLL))
        {
            state = ST_IDLE;
        }
        else
        {
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
        state = ST_TX_RX_PENDING;
        Serial.println(F("EV_TXS"));
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void countRaindrop()
{
    counts[0]++;
}

void setup()
{
    pinMode(LEDPIN, OUTPUT);
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);

    // Turn on the LED to indicate startup mode.
    analogWrite(LEDPIN, 2);
    Serial.begin(115200);
    yield();
    delay(100);
    yield();
#if defined(__AVR_ATmega32U4__)
    if (UDADDR & _BV(ADDEN))
    {
        while (!Serial)
        {
            ; // wait for serial port to connect. Needed for native USB
        }
        Serial.println(F("S"));
    }
#endif

    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN), countRaindrop, FALLING);

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setAdrMode(1);
    LMIC_setLinkCheckMode(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1u / 100u);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendJob);
    state = ST_JOINING;
}

// define directions for LED fade
#define UP (0u)
#define DOWN (1u)

// constants for min and max PWM
const uint8_t minPWM = 0u;
const uint8_t maxPWM = 255u;

// State Variable for Fade Direction
uint8_t fadeDirection = UP;

// Global Fade Value
// but be bigger than byte and signed, for rollover
uint8_t fadeValue = 0;

// How smooth to fade?
const byte fadeIncrement = 5;

// millis() timing Variable, just for fading
unsigned long previousFadeMillis;

// How fast to increment?
const uint8_t fadeInterval = 50u;

void doTheFade(unsigned long thisMillis)
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
        analogWrite(LEDPIN, fadeValue);

        // reset millis for the next iteration (fade timer only)
        previousFadeMillis = thisMillis;
    }
}

ostime_t nextDirectSend = 0u;
#define MAX_LOOPS_BEFORE_SLEEP 5
uint8_t loopsBeforeSleep = 0u;

void loop()
{
    os_runloop_once();

    switch (state)
    {
    case ST_JOINING:
    {
        unsigned long currentMillis = millis();
        doTheFade(currentMillis);
    }
    break;
    case ST_JOIN_DONE:
    {
        state = ST_TX_RX_PENDING;
        counts[0] = 0u;
    }
    break;
    case ST_IDLE:
    {
        unsigned long currentMillis = millis();
        if ((currentMillis - previousFadeMillis) >= 1000u)
        {
            Serial.print(F("counts[0]: "));
            Serial.print(counts[0]);
            Serial.print(F(" counts[1]: "));
            Serial.print(counts[1]);
            Serial.print(F(" counts[2]: "));
            Serial.println(counts[2]);
            previousFadeMillis = currentMillis;
        }

        loopsBeforeSleep++;
        if (loopsBeforeSleep > MAX_LOOPS_BEFORE_SLEEP)
        {
            loopsBeforeSleep = 0u;
#if defined(__AVR_ATmega32U4__)
            if (!(UDADDR & _BV(ADDEN)))
            {
                LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
                // Give the AVR back the slept time back (simple version)
                cli();
                timer0_overflow_count += 8u * 64u * clockCyclesPerMicrosecond();
                sei();

                // VERY IMPORTANT after sleep to update os_time and not cause txbeg and os_time
                // out of sync which causes send delays with the RFM95 on and eating power
                os_getTime();
            }
#endif
        }

        bool doTrySend = false;

        if (doTrySend)
        {
            if (hal_checkTimer(nextDirectSend))
            {
                // Event detected. Send and prepare a delay for the next time an event can be sent.
                nextDirectSend = os_getTime();
                os_setTimedCallback(&sendJob, nextDirectSend, do_send);
                nextDirectSend += sec2osticks(TX_DIRECT_SEND_DELAY);
            }
        }
    }
    break;
    default:
        break;
    }
}