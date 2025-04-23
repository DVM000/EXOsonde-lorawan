# EXO Sonde to LoRaWAN Bridge

This project reads data from an EXO sonde (YSI/Xylem) via Modbus RTU and transmits the data over LoRaWAN using an [**Arduino MKR WAN 1310**](https://store.arduino.cc/products/arduino-mkr-wan-1310) 
board + [antenna](https://store.arduino.cc/products/dipole-pentaband-waterproof-antenna).

---

## Description

The system periodically reads parameters from the EXO sonde, extracting valid sensor values. These values are packed into a structured LoRaWAN payload including:

- Device ID
- Date and time of measurement
- Sampling period
- Valid parameter values (up to 32)
- CRC byte for data integrity

The payload is transmitted over a LoRaWAN network using OTAA.

---

## Connections

- **EXO Sonde** connected to **Signal Output Adapter for EXO Sonde** 
- **Arduino**: `VCC`, `GND`, `14-TX`, `13-RX` connected to Signal Adapter (hardware `Serial1` for communication with EXO sonde)
- For monitoring/debugging: Laptop connected to the Arduino via USB using the default `Serial` interface 

---

## Requirements

### Arduino Libraries:

- [`SensorModbusMaster`](https://github.com/EnviroDIY/SensorModbusMaster)
- [`CRC`](https://github.com/RobTillaart/CRC)
- [`MKRWAN`](https://docs.arduino.cc/libraries/mkrwan)

Install via the Arduino IDE Library Manager.

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
3	230	    0	     49E9 413D
...

Number of valid parameters: 22 + sample_period
Bytes required for parameters + sample_period: 134

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
[11-12] -> Sampling period (on first packet)
[12-(N-1)] -> Valid parameters (6 bytes each: 1 code + 1 status + 4 values)
-------------- CRC --------------
[N]   -> CRC8 byte
```




