/* 
 * Reading data from EXO sonde via Modbus + sending LoRaWAN packet with valid parameters
 * Datasheet: https://www.xylem.com/siteassets/brand/ysi/resources/manual/exo-user-manual-web.pdf
*/

#include <Arduino.h>
#include <SensorModbusMaster.h> // https://github.com/EnviroDIY/SensorModbusMaster
#include "CRC8.h"               // https://github.com/RobTillaart/CRC
#include "CRC.h"
#include <MKRWAN.h>             // https://docs.arduino.cc/libraries/mkrwan/
#include "arduino_secrets.h"    // containing SECRET_APP_EUI and SECRET_APP_KEY

bool verbose = false;    // shows extra debugging information

// Sonde device configuration
uint8_t devID = 0x12;   // Sonde device ID
uint8_t version = 0x00; // Sonde Hardware/Software version

// Bus configuration
byte modbusAddress = 0x01;
int32_t modbusBaudRate = 9600;

HardwareSerial& modbusSerial = Serial1; // modBus communication with Sonde
modbusMaster modbus;

// LoRaWAN variables
LoRaModem modem;
String appEui = SECRET_APP_EUI; // OTAA credentials
String appKey = SECRET_APP_KEY;
const int JOIN_TIMEOUT = 60000; // max waiting time for joining

// LoRaWAN packet variables
const int METADATA_BYTES = 1 + 1 + 1 + 8 + 1;  // Reserved + version + deviceID + Date+Time (8B) + CRC
const int PARAM_BYTES = 1 + 1 + 4;             // 1 byte of code + 1 byte of status + 2 uint16_t registers per parameter 
const int MAX_PAYLOAD_SIZE = METADATA_BYTES + 6*PARAM_BYTES; // minimum for 1 packet: MEDATADABYTES + 1*PARAM_BYTES
// There is also LoRaWAN payload limit for each Spreaing Factor: 51 for SF10, 222 for SF8, ... // https://www.semtech.com/design-support/faq/faq-lorawan/P20
const int MAX_paramsPerPacket = (MAX_PAYLOAD_SIZE - METADATA_BYTES) / PARAM_BYTES; // ( maximum payload - (header+CRC) ) / bytes_per_parameter


// Functions
bool JoinNetwork(int maxRetries = 5, int retryDelay = 5000){
    bool connected = false;

    // https://www.semtech.com/design-support/faq/faq-lorawan/
    modem.minPollInterval(60); // independent of this setting, the modem will not allow sending more than one message every 2 minutes
    modem.setPort(10);         // Application-specific Fport
    modem.dataRate(0);         // Data Rate. 0: SF10 BW 125 kHz
    modem.setADR(true);        // (dinamically) Adaptive Data Rate

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        connected = modem.joinOTAA(appEui, appKey, JOIN_TIMEOUT);
        if (connected) {
            break;
        } else { 
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    if (!connected) {
        while (1) {}
        return false;
    }

    return true;
}

bool SendPacket(uint8_t* payload, int numBytes, int maxRetries = 5, int retryDelay = 5000) {
    bool success = false;

    if (payload == nullptr || numBytes <= 0) {
        return false;
    }

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        modem.dataRate(1);         // Data Rate. 0: SF10 BW 125 kHz
        modem.beginPacket();
        modem.write(payload, numBytes);
        int ok = modem.endPacket(true) > 0;  
        if (ok) {
            success = true;
            break;
        } else {   
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    return success;
}

void setup() {

    modbusSerial.begin(modbusBaudRate);  // modbus communicatio with Sonde device
    modbus.begin(modbusAddress, modbusSerial, 6);

    if (!modem.begin(US915)) { // US915: (902–928 MHz)
        while (1) {}
    }
    JoinNetwork();
}


void loop() {
    
    const int numParams = 32;  // maximum number of parameters
    uint16_t sample_period;    
    uint16_t codes[numParams];
    uint16_t statuses[numParams];
    uint16_t values[2*numParams]; // 2 uint16 registers per floating-point parameter value, little endian
    byte data_byte; // read data from register

    // ------------------------------------------------------------------------------------------------------
    // Forcing read
    // ------------------------------------------------------------------------------------------------------
    modbus.byteToRegister(0x03, 1, 2);
    for (int i = 1; i < 15; i++) // wait 15 seconds <--- change this to change packet frequency
    {
         delay(1000);
    }

    // ------------------------------------------------------------------------------------------------------
    // Read data
    // ------------------------------------------------------------------------------------------------------
    // Read sample period register (register 0)
    sample_period = modbus.uint16FromRegister(0x03, 0);

    // Read 32 parameter codes (registers 128–159)
    for (int i = 0; i < numParams; i++) {
        codes[i] = modbus.uint16FromRegister(0x03, 128 + i);   
    }

    // Read 32 parameter statuses (registers 256–287). Valid parameters correspond only to codes[i]!=0
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            statuses[i] = modbus.uint16FromRegister(0x03, 256 + i);
        }
    }

    // Read 32 parameter values (registers 384–447. 64 registers, 2 per floating-point value, little endian)
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            values[2*i]   = modbus.uint16FromRegister(0x03, 384 + 2*i, bigEndian);
            values[2*i+1] = modbus.uint16FromRegister(0x03, 384 + 2*i+1, bigEndian);
        }
    } 

    // count valid parameters (code != 0)
    int validCount = 1; // sample_period is 1st parameter
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) {
            validCount++;
            // not counting as parameter to be sent in payload. It will go in Header
            if (codes[i] == 51) { validCount--; } 
            if (codes[i] == 54) { validCount--; } 
        }
    }

    // ------------------------------------------------------------------------------------------------------
    // Build Lorawan packet
    // ------------------------------------------------------------------------------------------------------
    /*
        ------------------- HEADER -------------------
        [0]   -> reserved Byte (1 Byte)
        [1]   -> HW/SW version (1 byte)
        [2]   -> Device ID (1 Byte)
        [3-6] -> Date (4 Bytes)         // Register 51
            [3-4] -> 1st uint16_t register -> bytes: [low, high]
            [5-6] -> 2nd uint16_t register -> bytes: [low, high]
        [7-10] -> Time (4 Bytes)        // Register 54
        ------------------- PAYLOAD -------------------
        [11-13]  -> Sample Period (1 Byte code + 2 Bytes)  // Register 0   (on 1st packet)
        [14-(N-1)] -> Valid parameters (6 Bytes per parameter: 1 byte code + 1 byte status + 2 uint16_t ModBus registers): 
            [i-(i+1)]     -> code
            [(i+2)-(i+3)] -> status
            [i-(i+1)]     -> 1st uint16_t register -> bytes: [low, high]
            [(i+2)-(i+3)] -> 2nd uint16_t register -> bytes: [low, high]
        ------------------- CRC -------------------
        [N] -> CRC (1 Bytes)
    */
 
    if (MAX_PAYLOAD_SIZE < METADATA_BYTES+PARAM_BYTES) { // not enough MAX_PAYLOAD_SIZE even for 1 parameter
        while(1) {}
    }
    int totalPackets = ceil( (float)(validCount) / MAX_paramsPerPacket );    // Up to MAX_paramsPerPacket params per packet
    int paramsPerPacket = ceil( (float)(validCount) / totalPackets );         // Actual number of params per packet (equal number for all packets)

    // Get date & time from registers
    uint16_t DATEl, DATEh, TIMEl, TIMEh;
    int idxDate = -1; int idxTime = -1;
    for (int i = 0; i < numParams; i++) {
        if (codes[i] == 51) idxDate = 2*i;
        if (codes[i] == 54) idxTime = 2*i;
    }
   
    if (idxDate >= 0) {
        DATEl = values[idxDate];
        DATEh = values[idxDate+1];
    }
    else {
        DATEl = 0x00;
        DATEh = 0x00;
    }
        
    if (idxTime >= 0) {
        TIMEl = values[idxTime];
        TIMEh = values[idxTime+1];
    }
    else {
        TIMEl = 0x00;
        TIMEh = 0x00;
    }

    // Build packets
    int i = 0; // parameter to be sent

    for (int pkt = 0; pkt < totalPackets; pkt++) {

        uint8_t payload[MAX_PAYLOAD_SIZE];
        int index = 0;

        // HEADER 
        payload[index++] = 0x00;              // 1- Reserved Byte
        payload[index++] = version;           // 2- HW/SW version B
        payload[index++] = devID; //.toInt(); // 3- deviceId

        payload[index++] = DATEl & 0xFF;      // 4- Date (8 less significant bits of 1st uint16)
        payload[index++] = DATEl >> 8;        //         (8 more significant bits)
        payload[index++] = DATEh & 0xFF;      // 4- Date (8 less significant bits of 2nd uint16)
        payload[index++] = DATEh >> 8;        //          (8 more significant bits)

        payload[index++] = TIMEl & 0xFF;      // 5- Time
        payload[index++] = TIMEl >> 8;       
        payload[index++] = TIMEh & 0xFF;      // 5- Time (8 less significant bits of 2nd uint16)
        payload[index++] = TIMEh >> 8;        

        // PAYLOAD: PARAMETERS
        int remainingParams = paramsPerPacket;
        if (pkt == 0) {  // sample period on 1st packet
            payload[index++] = 0x00;                 // code=0 to facilitate sample_period decoding
            payload[index++] = sample_period & 0xFF; // 6- register 0 - sample_period 
            payload[index++] = sample_period >> 8; 
            remainingParams--;
        }

        if (index + 4 >= MAX_PAYLOAD_SIZE) continue; // avoid overload

        while (remainingParams > 0 && i < numParams) {
            if (codes[i] != 0 && codes[i] != 51  &&  codes[i] != 54) {
                payload[index++] = codes[i] & 0xFF;       // 7a- Parameter code
                payload[index++] = statuses[i] & 0xFF;    // 7b- Parameter status

                payload[index++] = values[2 * i] & 0xFF;   // 7c - parameter values (4 bytes). Values for parameter codes[i] are stored in values[2*i] and values[2*i+1]
                payload[index++] = values[2 * i] >> 8;
                payload[index++] = values[2 * i + 1] & 0xFF;
                payload[index++] = values[2 * i + 1] >> 8; 
                remainingParams--;
            }
            i++;
        }

        // CRC
        CRC8 crc;
        crc.add((uint8_t*) payload, index);
        uint8_t crc_value = crc.calc();
        payload[index++] = crc_value;  // 8- CRC

        // ------------------------------------------------------------------------------------------------------
        // Send LoRaWAN packet
        // ------------------------------------------------------------------------------------------------------
        bool err = SendPacket(payload, index);
    } // end for packets
    //while(1) {};
}

