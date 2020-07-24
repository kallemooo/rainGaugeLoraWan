#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <util/atomic.h>

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
    ST_JOIN_DONE
} st_t;

static st_t state = ST_JOINING;
#define LEDPIN 13
// Arduino pin 3 is AVR PD0 and INT0.
#define RAIN_GAUGE_PIN 3
#define VBATPIN A9

#if defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
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

// Schedule TX every this many seconds. Multipled with TX_INTERVAL_MULTIPLEXER.
const uint16_t TX_INTERVAL = 60u;
const uint16_t TX_INTERVAL_MULTIPLEXER_MIN = 1u;
uint8_t TX_INTERVAL_MULTIPLEXER = TX_INTERVAL_MULTIPLEXER_MIN;

void countRaindrop();

volatile static uint32_t counts = 0u;
volatile static float measuredvbat = 0.0f;

static void measureVbat()
{
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2.0f;    // we divided by 2, so multiply back
    measuredvbat *= 3.3f;    // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024u; // convert to voltage
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (!(LMIC.opmode & OP_TXRXPEND))
    {
        // Copy counter value in interrupt safe context.
        uint32_t c;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            c = counts;
        }

        // Send buffer.
        uint8_t buff[3];

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
        LMIC_setTxData2(1u, buff, sizeof(buff), 0u);
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
            digitalWrite(LEDPIN, LOW);
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
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void setup()
{
    measureVbat();
    pinMode(LEDPIN, OUTPUT);
    pinMode(RAIN_GAUGE_PIN, INPUT_PULLUP);

    // Turn on the LED to indicate startup mode.
    analogWrite(LEDPIN, 2);
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

static void doTheFade(unsigned long thisMillis)
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

static volatile unsigned long last_interrupt_time = 0;
static volatile bool wakeup_by_interrupt = false;
// Interrupt handler.
void countRaindrop()
{
    wakeup_by_interrupt = true;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than 2ms, assume it's a bounce and ignore
    if ((interrupt_time - last_interrupt_time) > 2u)
    {
        counts++;
    }
    last_interrupt_time = interrupt_time;
}

void loop()
{
    os_runloop_once();

    if (ST_JOINING == state)
    {
        unsigned long currentMillis = millis();
        doTheFade(currentMillis);
    }
    else
    {
        if (!(UDADDR & _BV(ADDEN)))
        {
            // Check if it is OK to go to sleep for 15 ms.
            if (0 == os_queryTimeCriticalJobs(os_getTime() + ms2osticks(15u)))
            {
                if (measuredvbat < 3.5f)
                {
                    // To low battery voltage. Powerdown to not discharge battery
                    // below the level that vill destroy the battery.
                    // Disconnect interrupt and put device in deep sleep forever.
                    detachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_PIN));
                    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
                }

                // OK to go to sleep.
                wakeup_by_interrupt = false;
                LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
                if (!wakeup_by_interrupt)
                {
                    // Wakeup by timeout. This is ignored when waked up by interrupt.
                    // Give the AVR back the sleep time
                    //Needs '#include <util/atomic.h>'
                    const unsigned long slept = 15u; // 15 ms sleep time.
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                    {
                        extern volatile unsigned long timer0_millis;
                        extern volatile unsigned long timer0_overflow_count;
                        timer0_millis += slept;
                        // timer0 uses a /64 prescaler and overflows every 256 timer ticks
                        timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000u) / (64 * 256);
                    }
                }
                // VERY IMPORTANT after sleep to update os_time and not cause txbeg and os_time
                // out of sync which causes send delays with the RFM95 on and eating power
                os_getTime();
            }
        }
        else
        {
            // Only run this part if USB is connected.
            unsigned long currentMillis = millis();
            if ((currentMillis - previousFadeMillis) >= 10000u)
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
                previousFadeMillis = currentMillis;
            }
        }
    }
    measureVbat();
}