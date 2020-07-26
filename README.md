![GitHub](https://img.shields.io/github/license/kallemooo/rainGaugeLoraWan) ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/kallemooo/rainGaugeLoraWan/CI)
# LoRaWAN&trade; connected rain gauge
Simple tipping bucket rain gauge connected via LoRaWAN&trade;.
The device counts number of times the bucket tip/tumble, and sends the value using LoRaWAN&trade;.

A receiver application can then calculate the rainfall, and if needed reset the counter.

## Rainfall calculation
The rainfall calculation is inspired by the [Arduino-Rain-Gauge-Calibration][argc] instruction.

A tipping bucket rain gauge with the dimensions 11 cm by 5 cm respectively is used, giving a catchment area of 55 cm&sup2;.
A collection of 10 millilitres of rain is 10 ml/55 cm&sup2; = 0.181818182 cm = 1.81818182 mm of rain.

In the tipping bucket rain gauge, the bucket tips/tumbles 5 times for 10 ml (or 1.81 mm of rain) and so a single tip is for (10/5) ml = 2ml (or 0,364636364 mm).

## Hardware
* An [Adafruit Feather 32u4 RFM95 LoRa Radio][32u4].
* Power source is a LiPo battery.
* A tipping bucket rain gauge.
* RJ-11 jack for the tipping bucket connection.
* Wires to connect the RJ-11 to the Feather.

![Hardware](/img/hardware.png "The hardware used.")

### Pin usage.
#### RFM95 connection
All needed pins of the the RFM95 pins is on the Feather directly connected to pins on the 32u4 except for RFM95 DIO1. For RFM95 DIO1 is Arduino pin 1 selected as it is external interrupt #3, and also located next to dio1 on the Feather.
[Adafruit Feather 32u4 RFM95 pin mapping][RFM95pin].

#### Tipping bucket connection
The tipping bucket needs two pins on the Feather, GND and input.
As input pin is Arduino pin 3 (external interrupt 0) selected.

Weak pull-up needs to be enabled for Arduino pin 3 so when the tipping bucket tips/tumbles a low signal can be detected.

## Software
[PlatformIO][PlatformIO] using the [Adafruit Feather 32u4 PlatformIO board][feather32u4] is the base.
The [Arduino-LMIC library][arduino-lmic] provides the LoRaWAN&trade; support.

The firmware only counts number of tips/tumbles done by the tipping bucket and sends it using LoRaWAN&trade;. In the message is also the battery voltage level reported.

During LoRaWan join sequence the LED is fading. The fade class is build from the [Arduino fading led example][fade].

### Power save handling
Power saving is done in two main states. Raining mode and no rain mode.

A third mode sets the device in SLEEP_FOREVER mode without any interrupts enabled if the battery voltage is below 3.5 v.
This is to save the battery from being destroyed by discharging.

#### Raining mode
Sleep is done in 8 second intervals and after each 8 s of sleep the LMIC state machine is checked and data if it is time data is sent.

Default data send period is 15 seconds.

#### No rain mode
If there have been no rain detected for approximate one hour the device sets sleep mode SLEEP_FOREVER and waits for the next time it rains so an external interrupt wakes up the device.

When the device wakes up a message is sent to fetch any incoming command.

## Integration to [the things network][ttn]
To simplify integration to [the things network][ttn] a decoder and encoder can be used to simplify usage of the MQTT API.

### Reset rain counter
MQTT command to reset the rain counter in the device.
The parameter **reset** shall have the value **234** (0xEA) for the reset command to be accepted by the device.

```shell
mosquitto_pub -h <Region>.thethings.network -u "<AppID>" -P "<AppKey>" -t '<AppID>/devices/<DevID>/down' -m '{"port":2,"confirmed":true,"payload_fields":{"reset":234}}'
```

### Set transmit interval multiplexer
MQTT command to set transmit interval to 15 minutes.

```shell
mosquitto_pub -h <Region>.thethings.network -u "<AppID>" -P "<AppKey>" -t '<AppID>/devices/<DevID>/down' -m '{"port":2,"confirmed":true,"payload_fields":{"multiplexer":15}}'
```

### JavaScript decoder for [the things network][ttn]

```javascript
function Decoder(bytes, port)
{
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  if (port === 1)
  {
    if (bytes.length >= 3)
    {
      decoded.counts = (((bytes[2] & 0x80) >> 7) << 16) | (bytes[1]<<8) | bytes[0];
      decoded.vbat = ((bytes[2] & 0x7F) + 330) / 100.0;
    }
  }

  return decoded;
}
```

### JavaScript encoder for [the things network][ttn]

```javascript
function Encoder(object, port) {
  // Encode downlink messages sent as
  // object to an array or buffer of bytes.
  var bytes = [];

  if (port === 1 && ((object.multiplexer > 0) && (object.multiplexer < 255)))
  {
    bytes[0] = 1;
    bytes[1] = object.multiplexer;
  }
  else if (port === 2 && object.reset === 0xea)
  {
    bytes[0] = object.reset;
  }

  return bytes;
}
```

## References
* [The things network][ttn]
* [Arduino-Rain-Gauge-Calibration][argc]
* [Adafruit Feather 32u4 RFM95 LoRa Radio][32u4]
* [Adafruit Feather 32u4 RFM95 pin mapping][RFM95pin]
* [Adafruit Feather 32u4 PlatformIO board][feather32u4]
* [PlatformIO][PlatformIO]
* [Arduino-LMIC library][arduino-lmic]
* [Arduino fading led example][fade]

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job.)

[ttn]: <https://thethingsnetwork.org> "The things network"

[argc]: <https://www.instructables.com/id/Arduino-Rain-Gauge-Calibration>

[32u4]: <https://www.adafruit.com/product/3078> "Adafruit Feather 32u4 RFM95 LoRa Radio- 868 or 915 MHz - RadioFruit"

[RFM95pin]: <https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts#rfm-slash-semtech-radio-module-2825006-11> "Adafruit Feather 32u4 RFM95 pin mapping"

[feather32u4]: <https://docs.platformio.org/en/latest/boards/atmelavr/feather32u4.html> "Adafruit Feather 32u4 PlatformIO board"

[PlatformIO]: <https://platformio.org> "PlatformIO"

[arduino-lmic]: <https://github.com/mcci-catena/arduino-lmic> "MCCI Catena Arduino-LMIC library"

[fade]: <https://www.baldengineer.com/fading-led-analogwrite-millis-example.html> "Arduino fading led example"