# EXO Sonde to LoRaWAN Bridge

This project reads data from an [EXO sonde (YSI/Xylem)](https://www.xylem.com/siteassets/brand/ysi/resources/manual/exo-user-manual-web.pdf) via 
Modbus RTU and transmits the data over LoRaWAN using an [**Arduino MKR WAN 1310**](https://store.arduino.cc/products/arduino-mkr-wan-1310) 
board + [antenna](https://store.arduino.cc/products/dipole-pentaband-waterproof-antenna).

---

## Directory Set Up

- [codec](./codec/): contains the decoder in python.
- [main](./main/): contains the main source code for the arduino mkrwan 1310. If deploying, use this source code.
- [setup](./setup/): contains the setup code for the arduino mkrwan 1310. This is a copy of the main source code but with `serial` print out to help users monitor/debug their code or retrieve the `DevEUI`.

## Description

The system periodically reads parameters from the EXO sonde, extracting valid sensor values. These values are packed into a structured LoRaWAN payload including:

- Hardware/software version 
- Device ID
- Date and time of measurement
- Sampling period
- Valid parameter values (up to 30)
- CRC byte for data integrity

The payload is transmitted over a LoRaWAN network using OTAA.

---

## Connections

- **EXO Sonde** connected to **Signal Output Adapter for EXO Sonde** 
- **Arduino**: `VCC`, `GND`, `14-TX`, `13-RX` connected to Signal Adapter (hardware `Serial1` for communication with EXO sonde)
- For monitoring/debugging: **Laptop** connected to the Arduino via USB (hardware default `Serial` interface) 
>NOTE: monitoring/debugging is only used in setup.ino NOT in main.ino
---

## Requirements

### Arduino Libraries:

- [`SensorModbusMaster`](https://github.com/EnviroDIY/SensorModbusMaster)
- [`CRC`](https://github.com/RobTillaart/CRC)
- [`MKRWAN`](https://docs.arduino.cc/libraries/mkrwan)

Install via the [Arduino IDE](https://www.arduino.cc/en/software/) Library Manager.

---

## Configuration

Device ID, HW/SW version and MAX_PAYLOAD_SIZE required (in `main.ino`).

LoRaWAN credentials required (in `arduino_secrets.h`):

```cpp
#define SECRET_APP_EUI "xxxxxxxxxxxxxxxx"
#define SECRET_APP_KEY "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

---

## Serial Output Example

```
Sample period: 120
idx  Code   Status   Raw 16-bit register values
0    51     0        (DATE) 4240 4857
1    54     0        (TIME) 2180 4815
3    230    0	     49E9 413D
...

Number of valid parameters: 23
Bytes required for parameters: 138

Payload (HEX): 
00 00 12 40 42 57 48 57 48 80 21 78 ... [sensor values] ... CRC
```

---

## LoRaWAN Payload Structure

The LoRaWAN payload will be divided into multiple packets, each conforming to the MAX_PAYLOAD_SIZE limit and including a Header and CRC.

```
-------------- HEADER --------------
[0]   -> Reserved byte
[1]   -> HW/SW version
[2]   -> Device ID
[3-6] -> Date (4 bytes - 2 Modbus registers)
[7-10] -> Time (4 bytes - 2 Modbus registers)
-------------- PAYLOAD --------------
[11-13] -> Sampling period (3 bytes: 1 code (0) + 2 value) (on first packet)
[14-(N-1)] -> Valid parameters (6 bytes each: 1 code + 1 status + 4 values)
-------------- CRC --------------
[N]   -> CRC8 byte
```

### Downlink Command Structure
>TODO: write the command structure


