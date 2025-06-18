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
const bool DEBUG = false; // set to true to enable serial debugging
const bool verbose = false; // set to true to enable verbose output

// Debugging
#define dbg_print(x)     if (DEBUG) Serial.print(x)
#define dbg_println(x)   if (DEBUG) Serial.println(x)
#define dbg_printhex(x)  if (DEBUG) Serial.print(x, HEX)
#define dbg_printhexln(x) if (DEBUG) Serial.println(x, HEX)

// Sonde device configuration
uint8_t devID = 0x12;   // Sonde device ID
uint8_t version = 0x02; // Sonde Hardware/Software version

// Bus configuration
byte modbusAddress = 0x01;
int32_t modbusBaudRate = 9600;
const int32_t serialBaud = 115200; 

HardwareSerial& modbusSerial = Serial1; // modBus communication with Sonde
modbusMaster modbus;

// Modbus registers
const int SAMPLE_PERIOD_REGISTER = 0;
const int FORCE_SAMPLE_REGISTER = 1;
const int FORCE_WIPE_REGISTER = 2;
const int MIN_PARAM_TYPE_REGISTER = 128; // 128-159
const int MAX_PARAM_TYPE_REGISTER = 159;
const int MIN_PARAM_STATUS_REGISTER = 256; // 256-287
const int MAX_PARAM_STATUS_REGISTER = 287;
const int MIN_PARAM_VALUE_REGISTER = 384; // 384-447
const int MAX_PARAM_VALUE_REGISTER = 447;
const int MAX_PARAM_CODES = 32; // 32 parameters
const uint8_t VALID_PARAM_CODES[] = {
    1, 2, 3, 4, 5, 6, 7, 10, 12, 17, 18, 19, 20, 21, 22, 23,
    28, 37, 47, 48, 51, 52, 53, 54, 95, 101, 106, 108, 110,
    112, 145, 190, 191, 193, 194, 201, 202, 204, 211, 212,
    214, 215, 216, 217, 218, 223, 225, 226, 227, 228, 229,
    230, 237, 238, 239, 240, 241, 242, 243
};

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

// Default lorawan transmit period (seconds), min = 60 sec, max = 7200 sec
uint16_t TRANSMIT_PERIOD = 300;
// Default adapter sample period (seconds), min = 15 sec, max = 3600 sec
uint16_t ADAPTER_PERIOD = TRANSMIT_PERIOD / 2;
// Force Sample flag
volatile bool FORCE_SAMPLE = false;

// Heartbeat parameter (always send, increments 1…255 then wraps to 1)
uint8_t heartbeatCounter = 1;
const int HEARTBEAT_PARAM_CODE = 255;

// Functions
bool JoinNetwork(int maxRetries = 5, int retryDelay = 5000){
    bool connected = false;
    dbg_print("Module version is: ");
    dbg_print(modem.version());
    dbg_print(". device EUI is: ");
    dbg_println(modem.deviceEUI());

    // https://www.semtech.com/design-support/faq/faq-lorawan/
    modem.minPollInterval(60); // independent of this setting, the modem will not allow sending more than one message every 2 minutes
    modem.setPort(10);         // Application-specific Fport
    modem.dataRate(0);         // Data Rate. 0: SF10 BW 125 kHz
    modem.setADR(true);        // (dinamically) Adaptive Data Rate

    dbg_print("--- Joining via OTAA... (timeout: "); dbg_print(JOIN_TIMEOUT/1000); dbg_println(" sec) --- ");
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        connected = modem.joinOTAA(appEui, appKey, JOIN_TIMEOUT);
        if (connected) {
            break;
        } else {
            dbg_print("x ");   
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    if (!connected) {
        dbg_println(" -> Join fail.");
        while (1) {}
        return false;
    }

    dbg_println(" -> Join pass.");
    return true;
}

bool SendPacket(uint8_t* payload, int numBytes, int maxRetries = 5, int retryDelay = 5000) {
    bool success = false;

    if (payload == nullptr || numBytes <= 0) {
        dbg_println("Empty or null payload, skipping send.");
        return false;
    }

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        modem.dataRate(1);         // Data Rate. 0: SF10 BW 125 kHz
        modem.beginPacket();
        modem.write(payload, numBytes);
        int ok = modem.endPacket(true) > 0;  
        if (ok) {
            dbg_println(" ->  Message sent correctly.");
            success = true;
            break;
        } else {
            dbg_print("x ");    
            if (attempt < maxRetries) 
                delay(retryDelay * attempt);  // exponential backoff 
        }
    }
    if (!success){
        dbg_print(" -> Error sending message after "); dbg_print(maxRetries); dbg_println(" attempts.");}
    return success;
}

void printPayloadHex(uint8_t* payload, int numBytes) {
    dbg_print("Payload (HEX): ");
    for (int i = 0; i < numBytes; i++) {
        if (payload[i] < 0x10) dbg_print("0"); // zero padding
        dbg_printhex(payload[i]);
        dbg_print(" ");
      }
    dbg_println();
    dbg_println("               [reserved (1) | version (1) | devId (1) | DATE (4) | TIME (4) | sample_period (2) / VALID PARAMETERS (6*n) | CRC (1)]");
}

void changeTransmitPeriod(uint16_t newPeriod) {
    if (newPeriod < 60) {
        dbg_print("CMD: Ignored - Transmit period too low: ");
        dbg_println(newPeriod);
    } else if (newPeriod > 7200) {
        dbg_print("CMD: Ignored - Transmit period too high: ");
        dbg_println(newPeriod);
    } else {
        TRANSMIT_PERIOD = constrain(newPeriod, 60, 7200); // safety
        dbg_print("CMD: Transmit period set to ");
        dbg_print(TRANSMIT_PERIOD);
        dbg_println(" seconds");

        // Set adapter sample period to half of send period
        ADAPTER_PERIOD = TRANSMIT_PERIOD / 2;
        ADAPTER_PERIOD = constrain(ADAPTER_PERIOD, 15, 3600);  // safety
        bool success = modbus.byteToRegister(0x03, SAMPLE_PERIOD_REGISTER, ADAPTER_PERIOD);

        if (success) {
            dbg_print("Adapter sample period set to "); dbg_print(ADAPTER_PERIOD); dbg_println(" seconds");
        } else {
            dbg_println("WARNING: Failed to write adapter sample period");
        }
    }
}

void ForceSample() {
    // Send force sample command to adapter with one retry attempt
    bool success = modbus.byteToRegister(FORCE_SAMPLE_REGISTER,0x03, 2);
    if (!success) {
        success = modbus.byteToRegister(FORCE_SAMPLE_REGISTER,0x03, 2);
    }

    // Check if the command was sent successfully
    if (success) {
        // Wait for adapter to complete sampling (minimum 15 sec)
        for (int i = 1; i <= 15; i++) {
            delay(1000);
            dbg_print(i); dbg_print(" ");
        }
        FORCE_SAMPLE = true;  // Skip wait in main loop
    } else {
        dbg_println("CMD Error: Failed to send force sample command to adapter");
    }
}

void ForceWipe() {
    // Send force wipe command to adapter with one retry attempt
    bool success = modbus.byteToRegister(FORCE_WIPE_REGISTER,0x03, 2);
    if (!success) {
        success = modbus.byteToRegister(FORCE_WIPE_REGISTER,0x03, 2);
    }

    // Check if the command was sent successfully
    if (success) {
        dbg_println("CMD: Force wipe command sent to adapter");
    } else {
        dbg_println("CMD Error: Failed to send force wipe command to adapter");
    }
}

bool isValidParameterCode(uint8_t code) {
    for (uint8_t valid : VALID_PARAM_CODES) {
        if (code == valid) return true;
    }
    return false;
}

void changeParamType(int Params, uint8_t rcv[]) {
    bool writeSuccess = true;

    for (int i = 0; i < Params; i++) {
        uint8_t paramCode = rcv[i + 1];
        dbg_print(paramCode); dbg_print(" ");

        if (!isValidParameterCode(paramCode)) {
            dbg_print("\nCMD Error: Invalid parameter code "); dbg_println(paramCode);
            writeSuccess = false;
            break;
        }

        if (!modbus.byteToRegister(0x03, MIN_PARAM_TYPE_REGISTER + i, paramCode)) {
            dbg_print("\nCMD Error: Failed to write parameter code "); dbg_println(paramCode);
            writeSuccess = false;
            break;
        }
    }

    if (writeSuccess) {
        modbus.byteToRegister(0x03, MIN_PARAM_TYPE_REGISTER + Params, 0);  // Terminate with 0
        dbg_println("\nCMD: Parameter types updated successfully.");
    } else {
        dbg_println("CMD: Failed to write parameter types.");
    }
}

void HandleDownlinkCommand() {
    if (!modem.available()) {
        dbg_println("No downlink message received.");
        return;
    }

    uint8_t rcv[64];  // adjust size based on max expected command length
    int len = 0;

    while (modem.available()) {
        rcv[len++] = (uint8_t)modem.read();
    }

    dbg_print("Downlink [");
    dbg_print(len);
    dbg_print(" bytes]: ");
    for (int i = 0; i < len; i++) {
        dbg_print("0x");
        if (rcv[i] < 0x10) dbg_print("0");
        dbg_printhex(rcv[i]);
        dbg_print(" ");
    }
    dbg_println();

    if (len == 0) return;

    uint8_t command = rcv[0];

    switch (command) {
        case 0x01: // Change LoRaWAN Transmit Period
            if (len >= 3) {
                dbg_println("CMD: Change Lorawan Transmit Period triggered");
                uint16_t newPeriod = rcv[1] | (rcv[2] << 8);
                changeTransmitPeriod(newPeriod);
            } else {
                dbg_println("CMD Error: Sample period payload too short");
            }
            break;
        case 0x02: // Force Sample
            dbg_println("CMD: Force Sample triggered");
            ForceSample();
            break;
        case 0x03: // Force Wipe
            dbg_println("CMD: Force Wipe triggered");
            ForceWipe();
            break;
        case 0x04: // Change Parameter Types
            if (len < 2) {
                dbg_println("CMD Error: No parameter types provided.");
                break;
            } else {
                dbg_print("CMD: Change Parameter Types triggered");
                const int Params = min(len - 1, MAX_PARAM_CODES);  // max 32 parameters
                changeParamType(Params, rcv);
            }
            break;
        default:
            dbg_print("CMD Error: Unknown command code 0x");
            dbg_printhexln(command);
            break;
    }
}

int ReadSensorData( uint16_t& sample_period, uint16_t* codes, uint16_t* statuses, uint16_t* values, int numParams)
{

    // ------------------------------------------------------------------------------------------------------
    // Read data
    // ------------------------------------------------------------------------------------------------------
    // Read sample period register (register 0)
    sample_period = modbus.uint16FromRegister(0x03, SAMPLE_PERIOD_REGISTER);
    dbg_println(" "); dbg_print("Sample period: "); dbg_println(sample_period);

    // Read 32 parameter codes (registers 128–159)
    if (verbose) { dbg_println(" "); dbg_println("Codes:"); }
    for (int i = 0; i < numParams; i++) {
        codes[i] = modbus.uint16FromRegister(0x03, MIN_PARAM_TYPE_REGISTER + i); 
        if (verbose) { dbg_print(codes[i]);  dbg_print(","); }    
    }

    // Read 32 parameter statuses (registers 256–287). Valid parameters correspond only to codes[i]!=0
    if (verbose) { dbg_println(" "); dbg_println("Statuses:"); }
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            statuses[i] = modbus.uint16FromRegister(0x03, MIN_PARAM_STATUS_REGISTER + i);
        }
        if (verbose) { dbg_print(statuses[i]);  dbg_print(","); }
    }

    // Read 32 parameter values (registers 384–447. 64 registers, 2 per floating-point value, little endian)
    if (verbose) { dbg_println(" "); dbg_println("Values:"); }
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) { // more efficient: only read valid parameters
            values[2*i]   = modbus.uint16FromRegister(0x03, MIN_PARAM_VALUE_REGISTER + 2*i, bigEndian);
            values[2*i+1] = modbus.uint16FromRegister(0x03, MIN_PARAM_VALUE_REGISTER + 2*i+1, bigEndian);
            if (verbose) {  dbg_print(values[2*i]); dbg_print(","); dbg_print(values[2*i+1]); dbg_print(","); }
        }
    }
    if (verbose) {
        for (int i = 0; i < 2*numParams; i++) {
            dbg_print(values[i]); dbg_print(","); }
    }
    
    // Print valid parameters (code != 0)
    int validCount = 1; // sample_period is 1st parameter
    dbg_println("idx\tCode\tStatus\tRaw 16-bit register values");
    for (int i = 0; i < numParams; i++) {
        if (codes[i] != 0) {
            validCount++;
            dbg_print(i); dbg_print("\t");
            dbg_print(codes[i]); dbg_print("\t");
            dbg_print(statuses[i]); dbg_print("\t");
            if (codes[i] == 51) { dbg_print("(DATE) ");  validCount--; } // not counting as parameter to be sent in payload. It will go in Header
            if (codes[i] == 54) { dbg_print("(TIME) ");  validCount--; } 
            dbg_printhex(values[2*i]); dbg_print(" "); dbg_printhexln(values[2*i+1]); 
        }
    }

    dbg_print("Number of valid parameters: "); dbg_println(validCount); 
    //dbg_println(" + sample_period"); 
    dbg_print("Bytes required for parameters: "); dbg_println(validCount*PARAM_BYTES);

    return validCount;
}

void heartbeat(uint16_t DATEl, uint16_t DATEh, uint16_t TIMEl, uint16_t TIMEh) {
    // ------------------------------------------------------------------------------------------------------
    // Build and send heartbeat packet
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
        [11]  -> Heartbeat parameter code (1 Byte)
        [13] -> Heartbeat value (1 Byte)
        ------------------- CRC -------------------
        [N] -> CRC (1 Bytes)
    */
    uint8_t payload[MAX_PAYLOAD_SIZE];
    int index = 0;

    // --- HEADER ---
    payload[index++] = 0x00;           // reserved
    payload[index++] = version;        // HW/SW version
    payload[index++] = devID;          // device ID

    payload[index++] = DATEl & 0xFF;   // 4- Date (8 less significant bits of 1st uint16)
    payload[index++] = DATEl >> 8;     //         (8 more significant bits)
    payload[index++] = DATEh & 0xFF;   // 4- Date (8 less significant bits of 2nd uint16)
    payload[index++] = DATEh >> 8;     //          (8 more significant bits)

    payload[index++] = TIMEl & 0xFF;   // 5- Time
    payload[index++] = TIMEl >> 8;       
    payload[index++] = TIMEh & 0xFF;   // 5- Time (8 less significant bits of 2nd uint16)
    payload[index++] = TIMEh >> 8;

    // --- HEARTBEAT PARAM (2 bytes) ---
    payload[index++] = HEARTBEAT_PARAM_CODE;  // code
    payload[index++] = heartbeatCounter;      // value
    if (++heartbeatCounter == 0) heartbeatCounter = 1; // roll over 1…255

    // --- CRC ---
    CRC8 crc;
    crc.add((uint8_t*) payload, index);
    payload[index++] = crc.calc();

    // --- SEND ---
    dbg_println("Heartbeat Payload:");
    if (true) { printPayloadHex(payload, index);  }
    dbg_println("--- Sending packet... --- ");
    SendPacket(payload, index);
    dbg_println();
}

void BuildAndSendLoRaPackets(uint16_t sample_period, uint16_t* codes, uint16_t* statuses, uint16_t* values, int numParams, int validCount) {
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
    dbg_print("MAX_PAYLOAD: "); dbg_println(MAX_PAYLOAD_SIZE);  
    if (MAX_PAYLOAD_SIZE < METADATA_BYTES+PARAM_BYTES) { // not enough MAX_PAYLOAD_SIZE even for 1 parameter
        dbg_print("ERROR: Can not send any parameter with PAYLOAD_SIZE = "); dbg_println(MAX_PAYLOAD_SIZE);
        return;
    }

    int totalPackets = ceil((float)(validCount) / MAX_paramsPerPacket);
    int paramsPerPacket = ceil((float)(validCount) / totalPackets);
    dbg_print("Total packets: "); dbg_print(totalPackets);
    dbg_print(", with parameters per packet: "); dbg_println(paramsPerPacket);

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

    int i = 0;  // Index for parameters

    for (int pkt = 0; pkt < totalPackets; pkt++) {
        dbg_print("--- Packet #"); dbg_print(pkt+1); dbg_print(", with params: ");

        uint8_t payload[MAX_PAYLOAD_SIZE];
        int index = 0;

        // --- HEADER ---
        payload[index++] = 0x00;              // 1- Reserved Byte
        payload[index++] = version;           // 2- HW/SW version B
        payload[index++] = devID; //.toInt(); // 3- deviceId

        payload[index++] = DATEl & 0xFF;      // 4- Date (8 less significant bits of 1st uint16)
        payload[index++] = DATEl >> 8;        //         (8 more significant bits)
        payload[index++] = DATEh & 0xFF;      // 4- Date (8 less significant bits of 2nd uint16)
        payload[index++] = DATEh >> 8;        //          (8 more significant bits)
        if (verbose) { dbg_printhex(DATEl); dbg_print(" -> "); dbg_printhex(payload[3]); dbg_print(" "); dbg_printhexln(payload[4]); }

        payload[index++] = TIMEl & 0xFF;      // 5- Time
        payload[index++] = TIMEl >> 8;       
        payload[index++] = TIMEh & 0xFF;      // 5- Time (8 less significant bits of 2nd uint16)
        payload[index++] = TIMEh >> 8;   

        // --- PAYLOAD (PARAMETERS) ---
        int remainingParams = paramsPerPacket;
        if (pkt == 0) {  // sample period on 1st packet
            payload[index++] = 0x00;                 // code=0 to facilitate sample_period decoding
            payload[index++] = sample_period & 0xFF; // 6- register 0 - sample_period 
            payload[index++] = sample_period >> 8; 
            remainingParams--;
            dbg_print("0 (sample_period), ");
        }

        if (index + 4 >= MAX_PAYLOAD_SIZE) continue; // avoid overload

        while (remainingParams > 0 && i < numParams) {
            if (codes[i] != 0 && codes[i] != 51  &&  codes[i] != 54) {
                payload[index++] = codes[i] & 0xFF;       // 7a- Parameter code
                payload[index++] = statuses[i] & 0xFF;    // 7b- Parameter status

                payload[index++] = values[2 * i] & 0xFF;   // 7c - parameter values (4 bytes). Values for parameter codes[i] are stored in values[2*i] and values[2*i+1]
                payload[index++] = values[2 * i] >> 8;
                //dbg_printhex(values[2*i]); dbg_print(" -> "); dbg_printhex(payload[index-1]); dbg_print(" "); dbg_printhexln(payload[index-2]);
                payload[index++] = values[2 * i + 1] & 0xFF;
                payload[index++] = values[2 * i + 1] >> 8;
                //dbg_printhex(values[2*i+1]); dbg_print(" -> "); dbg_printhex(payload[index-1]); dbg_print(" "); dbg_printhexln(payload[index-2]); 
                remainingParams--;
                dbg_print(i); dbg_print(": "); dbg_print(codes[i]); dbg_print(", ");
            }
            i++;
        }

        // --- CRC ---
        CRC8 crc;
        crc.add((uint8_t*) payload, index);
        uint8_t crc_value = crc.calc();
        payload[index++] = crc_value;  // 8- CRC

        // --- SEND ---
        dbg_print(". Payload: "); dbg_println(index);
        if (true) { printPayloadHex(payload, index);  }
        dbg_println("--- Sending packet... --- ");
        SendPacket(payload, index);
        dbg_println();
    }
    heartbeat(DATEl, DATEh, TIMEl, TIMEh); //send the heartbeat packet
    HandleDownlinkCommand();  // Check if any downlink is received
}

void setup() {

    if (DEBUG) {
        Serial.begin(serialBaud);            // serial bus communication with laptop
        while (!Serial);                     // wait until port is ready on MKR board
    }

    modbusSerial.begin(modbusBaudRate);  // modbus communication with Sonde device
    modbus.begin(modbusAddress, modbusSerial, 6);

    // Set Adapter samples
    modbus.byteToRegister(0x03, SAMPLE_PERIOD_REGISTER, ADAPTER_PERIOD);  
    dbg_println("Set adapter sample period");

    //dbg_println("Starting LoRa modem...");
    if (!modem.begin(US915)) { // US915: (902–928 MHz)
        dbg_println("Failed to start modem module");
        while (1) {}
    }
    dbg_print("Modem started successfully. ");
    JoinNetwork();
}

void loop() {
    uint16_t codes[MAX_PARAM_CODES];
    uint16_t statuses[MAX_PARAM_CODES];
    uint16_t values[2 * MAX_PARAM_CODES];
    uint16_t sample_period;

    // ------------------------------------------------------------------------------------------------------
    // Waiting for Read
    // ------------------------------------------------------------------------------------------------------
    // Only wait if it's NOT a force sample
    if (!FORCE_SAMPLE) {
        TRANSMIT_PERIOD = constrain(TRANSMIT_PERIOD, 60, 7200); // safety
        dbg_print("\n--- Waiting for "); dbg_print(TRANSMIT_PERIOD); dbg_println(" seconds ---");
    
        for (int i = 1; i <= TRANSMIT_PERIOD; i++) // wait TRANSMIT_PERIOD seconds
        {
             delay(1000);
             dbg_print(i);  dbg_print(" ");
        }
    } 
    
    // ------------------------------------------------------------------------------------------------------
    // Start transmission
    // ------------------------------------------------------------------------------------------------------
    int validCount = ReadSensorData(sample_period, codes, statuses, values, MAX_PARAM_CODES);
    if (validCount <= 0) {
        dbg_println("ReadSensorData retured no data — proceeding with heartbeat only");
        validCount = 0;
    }
    BuildAndSendLoRaPackets(sample_period, codes, statuses, values, MAX_PARAM_CODES, validCount);

    // Reset force sample flag
    FORCE_SAMPLE = false;
}

