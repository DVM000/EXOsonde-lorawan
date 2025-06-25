/* 
 * Reading data from EXO sonde via Modbus + sending LoRaWAN packet with valid parameters
 * Datasheet: https://www.xylem.com/siteassets/brand/ysi/resources/manual/exo-user-manual-web.pdf
*/

#include <Arduino.h>
#include <SensorModbusMaster.h> // https://github.com/EnviroDIY/SensorModbusMaster
#include "CRC8.h"               // https://github.com/RobTillaart/CRC
#include "CRC.h"
#include <MKRWAN.h>             // https://docs.arduino.cc/libraries/mkrwan/
#include <Adafruit_SleepyDog.h> //https://docs.arduino.cc/libraries/adafruit-sleepydog-library/
#include "arduino_secrets.h"    // containing SECRET_APP_EUI and SECRET_APP_KEY
#include <FlashStorage.h> // https://docs.arduino.cc/libraries/flashstorage/

/* -------------------------------------------------------------------------------------
DEBUGGING 
    * DEBUG: set to true to enable serial debugging
    * verbose: set to true to enable verbose output
    
    NOTE: if enabled, the mkrwan will not work if the end device does not except the 
        serial port
---------------------------------------------------------------------------------------*/
const bool DEBUG = false; // set to true to enable serial debugging
const bool verbose = false; // set to true to enable verbose output
#define dbg_print(x)     if (DEBUG) Serial.print(x)
#define dbg_println(x)   if (DEBUG) Serial.println(x)
#define dbg_printhex(x)  if (DEBUG) Serial.print(x, HEX)
#define dbg_printhexln(x) if (DEBUG) Serial.println(x, HEX)

/* -------------------------------------------------------------------------------------
EXOSONDE DEVICE GLOBAL VARIABLES
---------------------------------------------------------------------------------------*/
uint8_t devID = 0x12;   // Sonde device ID
uint8_t version = 0x03; // Sonde Hardware/Software version

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
const int DATE_REGISTER = 51;
const int TIME_REGISTER = 54;
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

/* -------------------------------------------------------------------------------------
LORAWAN GLOBAL VARIABLES
---------------------------------------------------------------------------------------*/
LoRaModem modem;
String appEui = SECRET_APP_EUI; // OTAA credentials
String appKey = SECRET_APP_KEY;

// LoRaWAN packet variables
const int METADATA_BYTES = 1 + 1 + 1 + 8 + 1;  // Reserved + version + deviceID + Date+Time (8B) + CRC
const int PARAM_BYTES = 1 + 1 + 4;             // 1 byte of code + 1 byte of status + 2 uint16_t registers per parameter 
const int MAX_PAYLOAD_SIZE = METADATA_BYTES + 6*PARAM_BYTES; // minimum for 1 packet: MEDATADABYTES + 1*PARAM_BYTES
// There is also LoRaWAN payload limit for each Spreaing Factor: 51 for SF10, 222 for SF8, ... // https://www.semtech.com/design-support/faq/faq-lorawan/P20
const int MAX_paramsPerPacket = (MAX_PAYLOAD_SIZE - METADATA_BYTES) / PARAM_BYTES; // ( maximum payload - (header+CRC) ) / bytes_per_parameter

// max waiting time (milliseconds) for joining
const int JOIN_TIMEOUT = 90000; 
// Default lorawan transmit period (seconds), min = 60 sec, max = 7200 sec
const uint16_t DEFAULT_TRANSMIT_PERIOD = 300;
uint16_t TRANSMIT_PERIOD = DEFAULT_TRANSMIT_PERIOD;
// Default adapter sample period (seconds), min = 15 sec, max = 3600 sec
uint16_t ADAPTER_PERIOD = TRANSMIT_PERIOD / 2;
// Force Sample flag
volatile bool FORCE_SAMPLE = false;

// Heartbeat parameter (always send, increments 1…255 then wraps to 1)
uint8_t heartbeatCounter = 1;
const int HEARTBEAT_PARAM_CODE = 255;

/* -------------------------------------------------------------------------------------
STATEFUL CONFIGURATION
---------------------------------------------------------------------------------------*/
typedef struct {
    uint16_t txPeriod;
} PersistentConfig;
FlashStorage(config_store, PersistentConfig);

void saveConfig() {
    /*-------------------------------------------------------------------------------------------------------
    Save the current configuration to persistent storage
    This function checks if the values have changed before writing to avoid unnecessary writes.
    -------------------------------------------------------------------------------------------------------*/
    dbg_print("[CONFIG] Saving configuration... ");
    PersistentConfig current = config_store.read();
    if (current.txPeriod != TRANSMIT_PERIOD) {
        config_store.write({TRANSMIT_PERIOD});
        dbg_println("Done.");
        dbg_print("- TRANSMIT_PERIOD: "); dbg_println(TRANSMIT_PERIOD);
    } else {
        dbg_println("No changes.");
    }
}

void loadConfig() {
    /*-------------------------------------------------------------------------------------------------------
    Load the configuration from persistent storage.
    This function reads the configuration and sets the transmit period and adapter period accordingly.
    If the stored value is out of bounds, it uses the default TRANSMIT_PERIOD.
    --------------------------------------------------------------------------------------------------------*/
    dbg_print("[CONFIG] Loading configuration... ");
    PersistentConfig config = config_store.read();
    TRANSMIT_PERIOD = (config.txPeriod >= 60 && config.txPeriod <= 7200) ? config.txPeriod : DEFAULT_TRANSMIT_PERIOD;
    ADAPTER_PERIOD = TRANSMIT_PERIOD / 2;
    dbg_println("Done.");
    dbg_print("- TRANSMIT_PERIOD: "); dbg_println(TRANSMIT_PERIOD);
}

/* -------------------------------------------------------------------------------------
LED FUNCTIONS
---------------------------------------------------------------------------------------*/
void blinkJoinPassed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a successful join to the LoRaWAN network
    --------------------------------------------------------------------------------------------------------*/
    int blinkDelay = 100;
    for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(blinkDelay);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(blinkDelay);
    }
}

void blinkJoinFailed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a failed join to the LoRaWAN network
    --------------------------------------------------------------------------------------------------------*/
    int blinkDelay = 300;
    for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(blinkDelay);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(blinkDelay);
    }
    mydelay(1000);
}

void blinkSensorReadPassed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a successful sensor read
    --------------------------------------------------------------------------------------------------------*/
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(50);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(150);
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(50);
    digitalWrite(LED_BUILTIN, LOW);
}

void blinkSensorReadFailed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a failed sensor read
    --------------------------------------------------------------------------------------------------------*/
    for (int i = 0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(250);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(100);
    }
}

void blinkTxPassed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a successful transmission of a LoRaWAN packet
    --------------------------------------------------------------------------------------------------------*/
    for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(80);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(80);
    }
}

void blinkTxFailed() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate a failed transmission of a LoRaWAN packet
    --------------------------------------------------------------------------------------------------------*/
    for (int i = 0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(100);
    digitalWrite(LED_BUILTIN, LOW);
    mydelay(300);
    }
}

void blinkWaiting() {
    /*-------------------------------------------------------------------------------------------------------
    Blink the built-in LED to indicate waiting for an operation to complete
    --------------------------------------------------------------------------------------------------------*/
    digitalWrite(LED_BUILTIN, HIGH);
    mydelay(200);
    digitalWrite(LED_BUILTIN, LOW);
}

/* -------------------------------------------------------------------------------------
WATCHDOG FUNCTIONS
---------------------------------------------------------------------------------------*/
void enableWatchdog() {
    /*-------------------------------------------------------------------------------------------------------
    Enable the watchdog timer, this will reboot the system if it is not reset in time
    -------------------------------------------------------------------------------------------------------*/
    int countdownMS = Watchdog.enable(16000); //max is 16 seconds
    dbg_print("[WDT] Watchdog enabled with timeout: ");
    dbg_print(countdownMS / 1000); // print in seconds
    dbg_println(" seconds");
}

void pingWatchdog(const char* tag = "") {
    /*------------------------------------------------------------------------------------------------------
    Reset the watchdog timer, this pings the watchdog to prevent a system reboot
    -------------------------------------------------------------------------------------------------------*/
    Watchdog.reset();
    if(verbose && strlen(tag) > 0) {
        dbg_print("[WDT] Watchdog ping");
        dbg_print(" at: ");
        dbg_println(tag);
    }
}

void mydelay(int ms) {
    /* ------------------------------------------------------------------------------------------------------
    Custom delay function to ping the watchdog during long delays
    ms: delay in milliseconds
    -------------------------------------------------------------------------------------------------------*/
    int elapsed = 0;
    while (elapsed < ms) {
        pingWatchdog();
        delay(100);
        elapsed += 100;
    }
}

/* -------------------------------------------------------------------------------------
EXOSONDE FUNCTIONS
---------------------------------------------------------------------------------------*/
bool setSamplePeriod() {
    /* ------------------------------------------------------------------------------------------------------
    Set the sample period in the adapter by writing to the Modbus register
    --------------------------------------------------------------------------------------------------------*/
    bool highByteSuccess = modbus.byteToRegister(SAMPLE_PERIOD_REGISTER, 1, (ADAPTER_PERIOD >> 8));  // upper byte
    bool lowByteSuccess  = modbus.byteToRegister(SAMPLE_PERIOD_REGISTER, 2, (ADAPTER_PERIOD & 0xFF)); // lower byte
    return highByteSuccess && lowByteSuccess;
}

void changeTransmitPeriod(uint16_t newPeriod) {
    /* ------------------------------------------------------------------------------------------------------
    Change the LoRaWAN transmit period and update the adapter sample period accordingly
    newPeriod: new transmit period in seconds, must be between 60 and 7200 seconds
    --------------------------------------------------------------------------------------------------------*/
    if (newPeriod < 60) {
        dbg_print("[EXO] Ignored - Transmit period too low: ");
        dbg_println(newPeriod);
    } else if (newPeriod > 7200) {
        dbg_print("[EXO] Ignored - Transmit period too high: ");
        dbg_println(newPeriod);
    } else {
        TRANSMIT_PERIOD = constrain(newPeriod, 60, 7200); // safety
        dbg_print("[EXO] Transmit period set to ");
        dbg_print(TRANSMIT_PERIOD);
        dbg_println(" seconds");

        // Set adapter sample period to half of send period
        ADAPTER_PERIOD = TRANSMIT_PERIOD / 2;
        ADAPTER_PERIOD = constrain(ADAPTER_PERIOD, 15, 3600);  // safety
        saveConfig();

        bool success = setSamplePeriod();
        if (success) {
            dbg_print("Adapter sample period set to "); dbg_print(ADAPTER_PERIOD); dbg_println(" seconds");
        } else {
            dbg_println("WARNING: Failed to write adapter sample period");
        }
    }
}

void ForceSample() {
    /* ------------------------------------------------------------------------------------------------------
    Force a sample on the adapter by sending a command to it
    It will wait for the adapter to complete sampling, which takes at least 15 seconds.
    --------------------------------------------------------------------------------------------------------*/
    // Send force sample command to adapter with one retry attempt
    bool success = modbus.byteToRegister(FORCE_SAMPLE_REGISTER, 2, 2);
    if (!success) {
        success = modbus.byteToRegister(FORCE_SAMPLE_REGISTER, 2, 2);
    }

    // Check if the command was sent successfully
    if (success) {
        // Wait for adapter to complete sampling (minimum 15 sec)
        for (int i = 1; i <= 15; i++) {
            pingWatchdog("ForceSample()");
            mydelay(1000);
            dbg_print(i); dbg_print(" ");
        }
        FORCE_SAMPLE = true;  // Skip wait in main loop
    } else {
        dbg_println("[EXO] Error: Failed to send force sample command to adapter");
    }
}

void ForceWipe() {
    /* ------------------------------------------------------------------------------------------------------
    Force a wipe on the adapter by sending a command to it
    This command is used to reset the adapter and clear its memory.
    --------------------------------------------------------------------------------------------------------*/
    // Send force wipe command to adapter with one retry attempt
    bool success = modbus.byteToRegister(FORCE_WIPE_REGISTER, 2, 2);
    if (!success) {
        success = modbus.byteToRegister(FORCE_WIPE_REGISTER, 2, 2);
    }

    // Check if the command was sent successfully
    if (success) {
        dbg_println("[EXO] Force wipe command sent to adapter");
    } else {
        dbg_println("[EXO] Error: Failed to send force wipe command to adapter");
    }
}

bool isValidParameterCode(uint8_t code) {
    /* ------------------------------------------------------------------------------------------------------
    Check if the given parameter code is valid
    code: parameter code to check
    Returns: true if the code is valid, false otherwise
    --------------------------------------------------------------------------------------------------------*/
    for (uint8_t valid : VALID_PARAM_CODES) {
        pingWatchdog("isValidParameterCode() checking code");
        if (code == valid) return true;
    }
    return false;
}

void changeParamType(int Params, uint8_t rcv[]) {
    /* ------------------------------------------------------------------------------------------------------
    Change the parameter types in the adapter by writing to the Modbus registers
    Params: number of parameters to change
    rcv: array containing the parameter codes to write, first byte is the command code
    --------------------------------------------------------------------------------------------------------*/
    bool writeSuccess = true;

    for (int i = 0; i < Params; i++) {
        pingWatchdog("changeParamType() writing parameter code");
        uint8_t paramCode = rcv[i + 1];
        dbg_print(paramCode); dbg_print(" ");

        if (!isValidParameterCode(paramCode)) {
            dbg_print("\n[EXO] Error: Invalid parameter code "); dbg_println(paramCode);
            writeSuccess = false;
            break;
        }

        if (!modbus.byteToRegister(MIN_PARAM_TYPE_REGISTER + i, 2, paramCode)) {
            dbg_print("\n[EXO] Error: Failed to write parameter code "); dbg_println(paramCode);
            writeSuccess = false;
            break;
        }
    }

    if (writeSuccess) {
        modbus.byteToRegister(MIN_PARAM_TYPE_REGISTER + Params, 2, 0);  // Terminate with 0
        dbg_println("\n[EXO] Parameter types updated successfully.");
    } else {
        dbg_println("[EXO] Failed to write parameter types.");
    }
}

int ReadSensorData( uint16_t& sample_period, uint16_t* codes, uint16_t* statuses, uint16_t* values, int numParams)
{
    /* ------------------------------------------------------------------------------------------------------
    Read sensor data from the EXO sonde via Modbus
    sample_period: reference to store the sample period value
    codes: array to store parameter codes
    statuses: array to store parameter statuses
    values: array to store parameter values (2 registers per value)
    numParams: number of parameters to read (maximum is MAX_PARAM_CODES)
    Returns: number of valid parameters read
    --------------------------------------------------------------------------------------------------------*/
    pingWatchdog("ReadSensorData() start");

    // Read sample period register (register 0)
    sample_period = modbus.uint16FromRegister(0x03, SAMPLE_PERIOD_REGISTER);
    dbg_println(" "); dbg_print("[EXO] Sample period: "); dbg_println(sample_period);

    // Read 32 parameter codes (registers 128–159)
    if (verbose) { dbg_println(" "); dbg_println("[EXO] Codes:"); }
    for (int i = 0; i < numParams; i++) {
        pingWatchdog("ReadSensorData() reading codes");
        codes[i] = modbus.uint16FromRegister(0x03, MIN_PARAM_TYPE_REGISTER + i); 
        if (verbose) { dbg_print(codes[i]);  dbg_print(","); }    
    }

    // Read 32 parameter statuses (registers 256–287). Valid parameters correspond only to codes[i]!=0
    if (verbose) { dbg_println(" "); dbg_println("[EXO] Statuses:"); }
    for (int i = 0; i < numParams; i++) {
        pingWatchdog("ReadSensorData() reading statuses");
        if (codes[i] != 0) { // more efficient: only read valid parameters
            statuses[i] = modbus.uint16FromRegister(0x03, MIN_PARAM_STATUS_REGISTER + i);
        }
        if (verbose) { dbg_print(statuses[i]);  dbg_print(","); }
    }

    // Read 32 parameter values (registers 384–447. 64 registers, 2 per floating-point value, little endian)
    if (verbose) { dbg_println(" "); dbg_println("[EXO] Values:"); }
    for (int i = 0; i < numParams; i++) {
        pingWatchdog("ReadSensorData() reading values");
        if (codes[i] != 0) { // more efficient: only read valid parameters
            values[2*i]   = modbus.uint16FromRegister(0x03, MIN_PARAM_VALUE_REGISTER + 2*i, bigEndian);
            values[2*i+1] = modbus.uint16FromRegister(0x03, MIN_PARAM_VALUE_REGISTER + 2*i+1, bigEndian);
            if (verbose) {  dbg_print(values[2*i]); dbg_print(","); dbg_print(values[2*i+1]); dbg_print(","); }
        }
    }
    if (verbose) {
        for (int i = 0; i < 2*numParams; i++) {
            pingWatchdog("ReadSensorData() printing values");
            dbg_print(values[i]); dbg_print(","); }
    }
    
    // Print valid parameters (code != 0)
    int validCount = 1; // sample_period is 1st parameter
    dbg_println("[EXO] idx\tCode\tStatus\tRaw 16-bit register values");
    for (int i = 0; i < numParams; i++) {
        pingWatchdog("ReadSensorData() printing valid parameters");
        if (codes[i] != 0) {
            validCount++;
            dbg_print(i); dbg_print("\t");
            dbg_print(codes[i]); dbg_print("\t");
            dbg_print(statuses[i]); dbg_print("\t");
            if (codes[i] == DATE_REGISTER) { dbg_print("(DATE) ");  validCount--; } // not counting as parameter to be sent in payload. It will go in Header
            if (codes[i] == TIME_REGISTER) { dbg_print("(TIME) ");  validCount--; } 
            dbg_printhex(values[2*i]); dbg_print(" "); dbg_printhexln(values[2*i+1]); 
        }
    }

    dbg_print("[EXO] Number of valid parameters: "); dbg_println(validCount); 
    //dbg_println(" + sample_period"); 
    dbg_print("[EXO] Bytes required for parameters: "); dbg_println(validCount*PARAM_BYTES);

    pingWatchdog("ReadSensorData() end");
    return validCount;
}

void EnableDateTimeRegister() {
    /* ------------------------------------------------------------------------------------------------------
    Enable the date and time registers (51 and 54) in the adapter by ensuring they are present in the 
    parameter codes.
    -------------------------------------------------------------------------------------------------------*/
    uint16_t currentCodes[MAX_PARAM_CODES];
    uint8_t filteredCodes[MAX_PARAM_CODES];
    int filteredCount = 0;
    bool hasDate = false;
    bool hasTime = false;

    // Step 1: Read current param codes
    for (int i = 0; i < MAX_PARAM_CODES; i++) {
        pingWatchdog("EnableDateTimeRegister() reading codes");
        currentCodes[i] = modbus.uint16FromRegister(0x03, MIN_PARAM_TYPE_REGISTER + i);
        dbg_print(currentCodes[i]); dbg_print(",");
    }

    // Step 2: Filter out 52-53 and keep 51, track 51 and 54
    for (int i = 0; i < MAX_PARAM_CODES; i++) {
        pingWatchdog("EnableDateTimeRegister() filtering codes");
        uint16_t code = currentCodes[i];
        if (code == 0 || code == 52 || code == 53) continue;

        if (code == DATE_REGISTER) hasDate = true;
        if (code == TIME_REGISTER) hasTime = true;

        filteredCodes[filteredCount++] = code;
    }

    // Step 3: If already full (>= 32 after filtering), make room for 51 and 54
    int requiredSpace = 0;
    if (!hasDate) requiredSpace++;
    if (!hasTime) requiredSpace++;

    if (filteredCount + requiredSpace > MAX_PARAM_CODES) {
        dbg_println("[EXO] Full param list. Dropping last parameters to make room for 51 and 54...");
        filteredCount = MAX_PARAM_CODES - requiredSpace;
    }

    // Step 4: Ensure 51 and 54 are present
    if (!hasDate && filteredCount < MAX_PARAM_CODES) {
        filteredCodes[filteredCount++] = DATE_REGISTER;
    }
    if (!hasTime && filteredCount < MAX_PARAM_CODES) {
        filteredCodes[filteredCount++] = TIME_REGISTER;
    }

    // Step 5: Write updated param codes back to adapter
    changeParamType(filteredCount, filteredCodes);
}

/* -------------------------------------------------------------------------------------
LoRaWAN FUNCTIONS
---------------------------------------------------------------------------------------*/
bool JoinNetwork(int maxRetries = 5, int retryDelay = 5000){
    /* ------------------------------------------------------------------------------------------------------
    Join the LoRaWAN network using OTAA (Over-The-Air Activation)
    maxRetries: maximum number of join attempts, minimum is 1 , maximum is 5
    retryDelay: delay between join attempts in milliseconds, minimum is 5000, maximum is 10000
    Note: The modem must be initialized before calling this function.
    If the join fails, the function will block indefinitely waiting for a reboot.
    --------------------------------------------------------------------------------------------------------*/
    retryDelay = constrain(retryDelay, 5000, 10000); // safety: 5-10 seconds
    maxRetries = constrain(maxRetries, 1, 5); // safety: 1-5 retries
    bool connected = false;
    dbg_print("[LORA] Module version is: ");
    dbg_print(modem.version());
    dbg_print(". device EUI is: ");
    dbg_println(modem.deviceEUI());

    // https://www.semtech.com/design-support/faq/faq-lorawan/
    modem.minPollInterval(60); // independent of this setting, the modem will not allow sending more than one message every 2 minutes
    modem.setPort(10);         // Application-specific Fport
    modem.dataRate(0);         // Data Rate. 0: SF10 BW 125 kHz
    modem.setADR(true);        // (dinamically) Adaptive Data Rate

    dbg_print("[LORA] --- Joining via OTAA... (timeout: "); dbg_print(JOIN_TIMEOUT/1000); dbg_println(" sec) --- ");
    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        blinkWaiting();
        connected = modem.joinOTAA(appEui, appKey, JOIN_TIMEOUT);
        if (connected) {
            break;
        } else {
            dbg_print("x ");   
            if (attempt < maxRetries) { 
                mydelay(retryDelay * attempt);  // exponential backoff
            } 
        }
    }
    if (!connected) {
        dbg_println(" -> Join fail.");
        return false;
    }

    dbg_println(" -> Join pass.");
    return true;
}

bool SendPacket(uint8_t* payload, int numBytes, int maxRetries = 5, int retryDelay = 5000) {
    /* ------------------------------------------------------------------------------------------------------
    Send a LoRaWAN packet with the given payload and number of bytes
    payload: pointer to the payload data
    numBytes: number of bytes in the payload
    maxRetries: maximum number of send attempts, minimum is 1, maximum is 5
    retryDelay: delay between send attempts in milliseconds, minimum is 5000, maximum is 10000
    Note: The modem must be initialized before calling this function.
    -------------------------------------------------------------------------------------------------------*/
    bool success = false;
    retryDelay = constrain(retryDelay, 5000, 10000); // safety: 5-10 seconds
    maxRetries = constrain(maxRetries, 1, 5); // safety: 1-5 retries

    if (payload == nullptr || numBytes <= 0) {
        dbg_println("[LORA] Empty or null payload, skipping send.");
        return false;
    }

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
        modem.dataRate(1);         // Data Rate. 0: SF10 BW 125 kHz
        modem.beginPacket();
        modem.write(payload, numBytes);
        int ok = modem.endPacket(true) > 0;  
        if (ok) {
            dbg_println(" ->  Message sent correctly.");
            blinkTxPassed();
            success = true;
            break;
        } else {
            dbg_print("x ");    
            if (attempt < maxRetries) {
                blinkWaiting();
                mydelay(retryDelay * attempt);  // exponential backoff 
            }
        }
    }
    if (!success){
        blinkTxFailed();
        dbg_print(" -> Error sending message after "); dbg_print(maxRetries); dbg_println(" attempts.");
    }
    return success;
}

void printPayloadHex(uint8_t* payload, int numBytes) {
    /* ------------------------------------------------------------------------------------------------------
    Print the payload in hexadecimal format for debugging
    payload: pointer to the payload data
    numBytes: number of bytes in the payload
    -------------------------------------------------------------------------------------------------------*/
    dbg_print("Payload (HEX): ");
    for (int i = 0; i < numBytes; i++) {
        if (payload[i] < 0x10) dbg_print("0"); // zero padding
        dbg_printhex(payload[i]);
        dbg_print(" ");
      }
    dbg_println();
    dbg_println("               [reserved (1) | version (1) | devId (1) | DATE (4) | TIME (4) | sample_period (2) / VALID PARAMETERS (6*n) | CRC (1)]");
}

void heartbeat(uint16_t DATEl, uint16_t DATEh, uint16_t TIMEl, uint16_t TIMEh) {
    /*------------------------------------------------------------------------------------------------------
    Build and send heartbeat packet

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
    -------------------------------------------------------------------------------------------------------*/
    dbg_print("[LORA]--- Heartbeat Packet"); dbg_print(", with params: ");
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
    if (verbose) { dbg_printhex(DATEl); dbg_print(" -> "); dbg_printhex(payload[3]); dbg_print(" "); dbg_printhexln(payload[4]); }
    dbg_println("[LORA] Heartbeat Payload:");
    if (true) { printPayloadHex(payload, index);  }
    dbg_println("--- Sending packet... --- ");
    SendPacket(payload, index);
    dbg_println();
}

void HandleDownlinkCommand() {
    /* ------------------------------------------------------------------------------------------------------
    Handle downlink commands received from the LoRaWAN network
    This function checks if there is any downlink message available and processes it.
    --------------------------------------------------------------------------------------------------------*/
    pingWatchdog("HandleDownlinkCommand() start");
    if (!modem.available()) {
        dbg_println("[LORA] No downlink message received.");
        return;
    }

    uint8_t rcv[64];  // adjust size based on max expected command length
    int len = 0;

    while (modem.available()) {
        rcv[len++] = (uint8_t)modem.read();
    }

    dbg_print("[LORA] Downlink [");
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
                dbg_println("[LORA] Change Lorawan Transmit Period triggered");
                uint16_t newPeriod = rcv[1] | (rcv[2] << 8);
                changeTransmitPeriod(newPeriod);
            } else {
                dbg_println("[LORA] Error: Sample period payload too short");
            }
            break;
        case 0x02: // Force Sample
            dbg_println("[LORA] Force Sample triggered");
            ForceSample();
            break;
        case 0x03: // Force Wipe
            dbg_println("[LORA] Force Wipe triggered");
            ForceWipe();
            break;
        case 0x04: // Change Parameter Types
            if (len < 2) {
                dbg_println("[LORA] Error: No parameter types provided.");
                break;
            } else {
                dbg_print("[LORA] Change Parameter Types triggered");
                const int Params = min(len - 1, MAX_PARAM_CODES);  // max 32 parameters
                changeParamType(Params, rcv);
            }
            break;
        case 0x05: // Force a mkrwan reboot
            dbg_println("[LORA] Force Reboot triggered");
            dbg_println("[LORA] Rebooting the MKR WAN 1310...");
            while(1){}
            break;
        default:
            dbg_print("[LORA] Error: Unknown command code 0x");
            dbg_printhexln(command);
            break;
    }
    pingWatchdog("HandleDownlinkCommand() end");
}

void BuildAndSendLoRaPackets(uint16_t sample_period, uint16_t* codes, uint16_t* statuses, uint16_t* values, int numParams, int validCount) {
    /*------------------------------------------------------------------------------------------------------
    Build and send LoRaWAN packets with valid parameters from the EXO sonde

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
    
    --------------------------------------------------------------------------------------------------------*/

    dbg_print("[LORA] MAX_PAYLOAD: "); dbg_println(MAX_PAYLOAD_SIZE);  
    if (MAX_PAYLOAD_SIZE < METADATA_BYTES+PARAM_BYTES) { // not enough MAX_PAYLOAD_SIZE even for 1 parameter
        dbg_print("[LORA] ERROR: Can not send any parameter with PAYLOAD_SIZE = "); dbg_println(MAX_PAYLOAD_SIZE);
        return;
    }

    int totalPackets = ceil((float)(validCount) / MAX_paramsPerPacket);
    int paramsPerPacket = ceil((float)(validCount) / totalPackets);
    dbg_print("[LORA] Total packets: "); dbg_print(totalPackets);
    dbg_print(", with parameters per packet: "); dbg_println(paramsPerPacket);

    // Get date & time from registers
    uint16_t DATEl, DATEh, TIMEl, TIMEh;
    int idxDate = -1; int idxTime = -1;
    for (int i = 0; i < numParams; i++) {
        if (codes[i] == DATE_REGISTER) idxDate = 2*i;
        if (codes[i] == TIME_REGISTER) idxTime = 2*i;
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
        pingWatchdog("BuildAndSendLoRaPackets() building packets");
        dbg_print("[LORA]--- Packet #"); dbg_print(pkt+1); dbg_print(", with params: ");

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

        if (index + 4 >= MAX_PAYLOAD_SIZE) { // avoid overload
            dbg_println("[LORA] ERROR: Not enough space for more parameters in payload, skipping packet.");
            continue;
        }  

        while (remainingParams > 0 && i < numParams) {
            if (codes[i] != 0 && codes[i] != DATE_REGISTER  &&  codes[i] != TIME_REGISTER) {
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

/* -------------------------------------------------------------------------------------
MKRWAN 1310 SETUP AND LOOP FUNCTIONS
---------------------------------------------------------------------------------------*/
void setup() {
    /* ------------------------------------------------------------------------------------------------------
    Setup function to initialize the MKRWAN 1310 board
    --------------------------------------------------------------------------------------------------------*/
    if (DEBUG) {
        Serial.begin(serialBaud);            // serial bus communication with laptop
        while (!Serial);                     // wait until port is ready on MKR board
    }
    dbg_println("[MKRWAN] Starting setup()");

    // enable led light
    pinMode(LED_BUILTIN, OUTPUT);

    // modbus communication with Sonde device
    modbusSerial.begin(modbusBaudRate);  
    modbus.begin(modbusAddress, modbusSerial, 6);

    // Load persistent configuration
    loadConfig();

    // Set Adapter samples
    setSamplePeriod();
    dbg_println("[MKRWAN] Set adapter sample period");

    if (!modem.begin(US915)) { // US915: (902–928 MHz)
        dbg_println("[MKRWAN] Failed to start modem module");
    }
    dbg_println("[MKRWAN] Modem started successfully. ");
    bool isJoined = JoinNetwork();

    //enable watchdog
    enableWatchdog();

    // Enable date/time register if not already enabled
    EnableDateTimeRegister();

    // Print device information
    if(isJoined){
        dbg_println("[MKRWAN] Joined LoRaWAN network successfully.");
    } else {
        dbg_println("[MKRWAN] Failed to join LoRaWAN network. Rebooting...");
        while(1){}
    }
    pingWatchdog("setup() end");
}

void loop() {
    /* ------------------------------------------------------------------------------------------------------
    Main loop function to read sensor data and send it via LoRaWAN
    ---------------------------------------------------------------------------------------------------------*/
    uint16_t codes[MAX_PARAM_CODES];
    uint16_t statuses[MAX_PARAM_CODES];
    uint16_t values[2 * MAX_PARAM_CODES];
    uint16_t sample_period;

    /* Waiting for Read
    ----------------------------------------------------------------------------------------------------*/
    // Only wait if it's NOT a force sample
    if (!FORCE_SAMPLE) {
        TRANSMIT_PERIOD = constrain(TRANSMIT_PERIOD, 60, 7200); // safety
        dbg_print("\n[MKRWAN] --- Waiting for "); dbg_print(TRANSMIT_PERIOD); dbg_println(" seconds ---");
    
        for (int i = 1; i <= TRANSMIT_PERIOD; i++) // wait TRANSMIT_PERIOD seconds
        {
            pingWatchdog("loop() waiting");
            if (i % 5 == 0) blinkWaiting();
            mydelay(1000);
            dbg_print(i);  dbg_print(" ");
        }
    } 
    
    /* Start transmission
    ----------------------------------------------------------------------------------------------------*/
    int validCount = ReadSensorData(sample_period, codes, statuses, values, MAX_PARAM_CODES);
    if (validCount <= 0) {
        dbg_println("[MKRWAN] ReadSensorData retured no data — proceeding with heartbeat only");
        blinkSensorReadFailed();
        validCount = 0;
    } else {
        blinkSensorReadPassed();
    }
    BuildAndSendLoRaPackets(sample_period, codes, statuses, values, MAX_PARAM_CODES, validCount);

    // Reset force sample flag
    FORCE_SAMPLE = false;
    pingWatchdog("loop() end of loop");
}

