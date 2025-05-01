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
const int32_t serialBaud = 115200; 

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
    Serial.print("Module version is: ");
    Serial.print(modem.version());
    Serial.print(". device EUI is: ");
    Serial.println(modem.deviceEUI());

    // https://www.semtech.com/design-support/faq/faq-lorawan/
    modem.minPollInterval(60); // independent of this setting, the modem will not allow sending more than one message every 2 minutes
    modem.setPort(10);         // Application-specific Fport
    modem.dataRate(0);         // Data Rate. 0: SF10 BW 125 kHz
    modem.setADR(true);        // (dinamically) Adaptive Data Rate

    Serial.print("--- Joining via OTAA... (timeout: "); Serial.print(JOIN_TIMEOUT/1000); Serial.println(" sec) --- ");
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        connected = modem.joinOTAA(appEui, appKey, JOIN_TIMEOUT);
        if (connected) {
            break;
        } else {
            Serial.print("x ");   
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    if (!connected) {
        Serial.println(" -> Join fail.");
        while (1) {}
        return false;
    }

    Serial.println(" -> Join pass.");
    return true;
}

bool SendPacket(uint8_t* payload, int numBytes, int maxRetries = 5, int retryDelay = 5000) {
    bool success = false;

    if (payload == nullptr || numBytes <= 0) {
        Serial.println("Empty or null payload, skipping send.");
        return false;
    }

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        modem.dataRate(1);         // Data Rate. 0: SF10 BW 125 kHz
        modem.beginPacket();
        modem.write(payload, numBytes);
        int ok = modem.endPacket(true) > 0;  
        if (ok) {
            Serial.println(" ->  Message sent correctly.");
            success = true;
            break;
        } else {
            Serial.print("x ");    
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    if (!success){
        Serial.print(" -> Error sending message after "); Serial.print(maxRetries); Serial.println(" attempts.");}
    return success;
}

void printPayloadHex(uint8_t* payload, int numBytes) {
    Serial.print("Payload (HEX): ");
    for (int i = 0; i < numBytes; i++) {
        if (payload[i] < 0x10) Serial.print("0"); // zero padding
        Serial.print(payload[i], HEX);
        Serial.print(" ");
      }
    Serial.println();
    Serial.println("               [reserved (1) | version (1) | devId (1) | DATE (4) | TIME (4) | sample_period (2) / VALID PARAMETERS (6*n) | CRC (1)]");
    }


void setup() {

    Serial.begin(serialBaud);            // serial bus communication with laptop
    while (!Serial);                     // wait until port is ready on MKR board

    modbusSerial.begin(modbusBaudRate);  // modbus communicatio with Sonde device
    modbus.begin(modbusAddress, modbusSerial, 6);

    //Serial.println("Starting LoRa modem...");
    if (!modem.begin(US915)) { // US915: (902–928 MHz)
        Serial.println("Failed to start modem module");
        while (1) {}
    }
    Serial.print("Modem started successfully. ");
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
    Serial.print("\n--- Forcing read (waiting for 15 seconds) ---  "); 
    modbus.byteToRegister(0x03, 1, 2);
    for (int i = 1; i < 15; i++) // wait 15 seconds
    {
         delay(1000);
         Serial.print(i);  Serial.print(" ");
    }

    // ------------------------------------------------------------------------------------------------------
    // Read data
    // ------------------------------------------------------------------------------------------------------
    // Read sample period register (register 0)
    sample_period = modbus.uint16FromRegister(0x03, 0);
    Serial.println(" "); Serial.print("Sample period: "); Serial.println(sample_period);

    // Read 32 parameter codes (registers 128–159)
    if (verbose) { Serial.println(" "); Serial.println("Codes:"); }
    for (int i = 0; i < numParams; i++) {
        codes[i] = modbus.uint16FromRegister(0x03, 128 + i); 
        if (verbose) { Serial.print(codes[i]);  Serial.print(","); }    
    }

    // Read 32 parameter statuses (registers 256–287). Valid parameters correspond only to codes[i]!=0
    if (verbose) { Serial.println(" "); Serial.println("Statuses:"); }
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            statuses[i] = modbus.uint16FromRegister(0x03, 256 + i);
        }
        if (verbose) { Serial.print(statuses[i]);  Serial.print(","); }
    }

    // Read 32 parameter values (registers 384–447. 64 registers, 2 per floating-point value, little endian)
    if (verbose) { Serial.println(" "); Serial.println("Values:"); }
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            values[2*i]   = modbus.uint16FromRegister(0x03, 384 + 2*i, bigEndian);
            values[2*i+1] = modbus.uint16FromRegister(0x03, 384 + 2*i+1, bigEndian);
            if (verbose) {  Serial.print(values[2*i]); Serial.print(","); Serial.print(values[2*i+1]); Serial.print(","); }
        }
    }
    if (verbose) {
        for (int i = 0; i < 2*numParams; i++) {
            Serial.print(values[i]); Serial.print(","); }
    } 

    // Print valid parameters (code != 0)
    int validCount = 1; // sample_period is 1st parameter
    Serial.println("idx\tCode\tStatus\tRaw 16-bit register values");
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) {
            validCount++;
            Serial.print(i); Serial.print("\t");
            Serial.print(codes[i]); Serial.print("\t");
            Serial.print(statuses[i]); Serial.print("\t");
            if (codes[i] == 51) { Serial.print("(DATE) ");  validCount--; } // not counting as parameter to be sent in payload. It will go in Header
            if (codes[i] == 54) { Serial.print("(TIME) ");  validCount--; } 
            Serial.print(values[2*i], HEX); Serial.print(" "); Serial.println(values[2*i+1], HEX); 
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

    Serial.print("Number of valid parameters: "); Serial.println(validCount); 
    //Serial.println(" + sample_period"); 
    Serial.print("Bytes required for parameters: "); Serial.println(validCount*PARAM_BYTES);

    Serial.print("MAX_PAYLOAD: "); Serial.println(MAX_PAYLOAD_SIZE);  
    if (MAX_PAYLOAD_SIZE < METADATA_BYTES+PARAM_BYTES) { // not enough MAX_PAYLOAD_SIZE even for 1 parameter
        Serial.print("ERROR: Can not send any parameter with PAYLOAD_SIZE = "); Serial.println(MAX_PAYLOAD_SIZE);
        while(1) {}
    }
    int totalPackets = ceil( (float)(validCount) / MAX_paramsPerPacket );    // Up to MAX_paramsPerPacket params per packet
    int paramsPerPacket = ceil( (float)(validCount) / totalPackets );         // Actual number of params per packet (equal number for all packets)
    Serial.print("Total packets: "); Serial.print(totalPackets);
    Serial.print(", with parameters per packet: "); Serial.println(paramsPerPacket);

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
        Serial.print("--- Packet #"); Serial.print(pkt+1); Serial.print(", with params: ");

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
        if (verbose) { Serial.print(DATEl, HEX); Serial.print(" -> "); Serial.print(payload[3], HEX); Serial.print(" "); Serial.println(payload[4], HEX); }

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
            Serial.print("0 (sample_period), ");
        }

        if (index + 4 >= MAX_PAYLOAD_SIZE) continue; // avoid overload

        while (remainingParams > 0 && i < numParams) {
            if (codes[i] != 0 && codes[i] != 51  &&  codes[i] != 54) {
                payload[index++] = codes[i] & 0xFF;       // 7a- Parameter code
                payload[index++] = statuses[i] & 0xFF;    // 7b- Parameter status

                payload[index++] = values[2 * i] & 0xFF;   // 7c - parameter values (4 bytes). Values for parameter codes[i] are stored in values[2*i] and values[2*i+1]
                payload[index++] = values[2 * i] >> 8;
                //Serial.print(values[2*i], HEX); Serial.print(" -> "); Serial.print(payload[index-1], HEX); Serial.print(" "); Serial.println(payload[index-2], HEX);
                payload[index++] = values[2 * i + 1] & 0xFF;
                payload[index++] = values[2 * i + 1] >> 8;
                //Serial.print(values[2*i+1], HEX); Serial.print(" -> "); Serial.print(payload[index-1], HEX); Serial.print(" "); Serial.println(payload[index-2], HEX); 
                remainingParams--;
                Serial.print(i); Serial.print(": "); Serial.print(codes[i]); Serial.print(", ");
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
        Serial.print(". Payload: "); Serial.println(index);
        if (true) { printPayloadHex(payload, index);  }
        Serial.println("--- Sending packet... --- ");
        bool err = SendPacket(payload, index);

        Serial.println();
    } // end for packets
    //while(1) {};
}

