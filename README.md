![GitHub](https://img.shields.io/github/license/kallemooo/rainGaugeLoraWan) ![GitHub Workflow Status](https://img.shields.io/github/workflow/status/kallemooo/rainGaugeLoraWan/CI)
# LoRaWAN&trade; connected rain gauge
Simple tipping bucket rain gauge connected via LoRaWAN&trade;.
The device counts number of times the bucket tip/tumble, and sends the value using LoRaWAN&trade;.

A receiver application can then calculate the rainfall, and if needed reset the counter.

## Rainfall calculation
The rainfall calculation is inspired by the [Arduino-Rain-Gauge-Calibration][argc] instruction.

I am using a tipping bucket rain gauge with the dimensions 11 cm by 5 cm respectively giving a catchment area of 55 cm&sup2;.
A collection of 10 millilitres of rain is 10 ml/55 cm&sup2; = 0.181818182 cm = 1.81818182 mm of rain.

In the tipping bucket rain gauge, the bucket tips 5 times for 10 ml (or 1.81 mm of rain) and so a single tip is for (10/5) ml = 2ml (or 0,364636364 mm).

## Hardware
* An [Adafruit Feather 32u4 RFM95 LoRa Radio][32u4].
* A tipping bucket rain gauge.

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
    if (bytes.length == 3)
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

  if (port === 1 && ((object.multiplexer > 1) && (object.multiplexer < 250)))
  {
    bytes[0] = object.multiplexer;
  }
  if (port === 2 && object.reset === 0xea)
  {
    bytes[0] = object.reset;
  }

  return bytes;
}
```

## References
* [Arduino-Rain-Gauge-Calibration][argc]
* [The things network][ttn]
* [Adafruit Feather 32u4 RFM95 LoRa Radio][32u4]

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job.)

[ttn]: <https://thethingsnetwork.org> "The things network"

[argc]: <https://www.instructables.com/id/Arduino-Rain-Gauge-Calibration>

[32u4]: <https://www.adafruit.com/product/3078> "Adafruit Feather 32u4 RFM95 LoRa Radio- 868 or 915 MHz - RadioFruit"
