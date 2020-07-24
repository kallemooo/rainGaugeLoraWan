# Lora wan connected rain gauge
Simple tipping bucket rain gauge connected via Lora Wan.
The device counts number of times the bucket tip/tumble, and sends the value using Lora Wan.

A receiver application can then calculate the rainfall, and if needed reset the counter.

## Integration to thethingsnetwork.org
To simplify integration to thethingsnetwork.org a decoder and encoder can be used
so the MQTT API can be used.

### Reset rain counter
MQTT command to reset the rain counter in the device.

```shell
mosquitto_pub -h <Region>.thethings.network -u "<AppID>" -P "<AppKey>" -t '<AppID>/devices/<DevID>/down' -m '{"port":2,"confirmed":true,"payload_fields":{"reset":234}}'
```

### Set transmit intervall multiplexer
MQTT command to set transmit intervall to 15 minutes.

```shell
mosquitto_pub -h <Region>.thethings.network -u "<AppID>" -P "<AppKey>" -t '<AppID>/devices/<DevID>/down' -m '{"port":2,"confirmed":true,"payload_fields":{"multiplexer":15}}'
```

### JavaScript decoder for thethingsnetwork.org

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

### JavaScript encoder for thethingsnetwork.org

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

## Links
The device is inspired by https://www.instructables.com/id/Arduino-Rain-Gauge-Calibration