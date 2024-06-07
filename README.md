# LoRa Sensor Reader

This is a (very) simple project to transmit sensor data, as encrypted JSON, over the wire using LoRa.

The receiver just prints it over Serial.

## Hardware

- Uses RAK19007 + RAK4631
- Receiver is using IO1 with an internal pullup

## Receiving
```
sudo stty -F /dev/ttyACM0 115200
sudo cat /dev/ttyACM0
```

## Encrypting
- Currently uses a hard-coded key on both rx and tx modules
- Currently always uses the max 128 byte payload length
- To reduce the risk of replay attacks, will always be incremeneing a sequence number and boot counter.  Computer script should ensure these are always increasing.
