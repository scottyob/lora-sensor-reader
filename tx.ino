/**
   @file Transmitter.ino
   @author Scott O'Brien (from rakwireless.com)
   @brief Transmitter node for GPIO telemetry with encryption
   @version 0.1
   @date 2020-08-21

   @copyright Copyright (c) 2020

   @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)

  Libraries:
  - (Board) Rak Wireless 4630 board
  - ArduinoJson
  - ArduinoBearSSL
  
*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include <ArduinoBearSSL.h>
#include "AES128.h"

using namespace Adafruit_LittleFS_Namespace;

// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

#define LED_BUILTIN 35

// Define LoRa parameters (AU, Long_Fast equivilent)
//#define RF_FREQUENCY 915125000  // Hz
//#define TX_OUTPUT_POWER 22    // dBm
//#define LORA_BANDWIDTH 1    // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
//#define LORA_SPREADING_FACTOR 11 // [SF7..SF12]
//#define LORA_CODINGRATE 1   // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
//#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
//#define LORA_SYMBOL_TIMEOUT 0 // Symbols
//#define LORA_FIX_LENGTH_PAYLOAD_ON false
//#define LORA_IQ_INVERSION_ON false
//#define TX_TIMEOUT_VALUE 3000

// Define LoRa parameters (AU, Long_Slow equivilent)
#define RF_FREQUENCY 915125000  // Hz
#define TX_OUTPUT_POWER 22    // dBm
#define LORA_BANDWIDTH 0    // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 4   // [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define TX_TIMEOUT_VALUE 12000

// Global variables to store state and sequences
#define BUFFER_LEN 128
#define SLEEP_TIME 1000 * 60 * 1 // 1 minute
static RadioEvents_t RadioEvents;
static bool switchState;
static uint8_t TxdBuffer[BUFFER_LEN] = {0};
static bool sending = false;
static uint32_t sequenceNum = 0;
static uint8_t bootCounter = 0;


void setup()
{

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  Serial.println("=====================================");
  Serial.println("LoRap2p Tx Test");
  Serial.println("=====================================");

  // Initialize the IO port for reading the water tower
  pinMode(WB_IO1, INPUT_PULLUP);  // Enable internal pull-up resistor

  // Initialize LoRa chip.
  lora_rak4630_init();
  
  // Initialize the Radio callbacks
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = NULL;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = NULL;
  RadioEvents.RxError = NULL;
  RadioEvents.CadDone = NULL;

  // Initialize the Radio
  Radio.Init(&RadioEvents);

  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set Radio TX configuration
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  // Initialize boot counter from flash
  static const char settings_name[] = "boot";

  // Initialize Internal File System, format if required
  InternalFS.begin();
  File file(InternalFS); // Filesystem object for boot count
  file.open(settings_name, FILE_O_READ);
  if (!file) {
    InternalFS.format();
  } else {
    file.read((uint8_t*)&bootCounter, 1);
    file.close();
  }

  // Write the new bootCounter to flash
  bootCounter++;
  InternalFS.remove(settings_name);
  file.open(settings_name, FILE_O_WRITE);
  file.write((uint8_t *)&bootCounter, 1);
  file.close();
}

void loop()
{
  switchState = digitalRead(WB_IO1);

  if (!sending) {
    send();
    yield();
  }

  delay(SLEEP_TIME);

}

/**@brief Function to be executed on Radio Tx Done event
*/
void OnTxDone(void)
{
  sending = false;
}

/**@brief Function to be executed on Radio Tx Timeout event
*/
void OnTxTimeout(void)
{
  Serial.println("ERROR:  TIMEOUT");
  sending = false;
}

void send()
{
  Serial.println("Sending");
  JsonDocument doc;
  doc["node"] = "tower-1";
  doc["state"] = switchState;
  doc["sequenceNum"] = sequenceNum++;
  doc["bootCounter"] = bootCounter;
  auto byteLen = serializeJson(doc, TxdBuffer, BUFFER_LEN);

  // Perform an encryption on the data
  static uint8_t TxdBufferEncrypted[BUFFER_LEN] = {0};
  memcpy(TxdBufferEncrypted, TxdBuffer, BUFFER_LEN);
  Serial.print("Before encryption: ");
  Serial.println((char*)TxdBufferEncrypted);

  uint8_t key[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02};
  uint8_t enc_iv[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01};
  uint8_t dec_iv[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01};
  AES128Class AES128;
  AES128.runEnc(key, 16, TxdBufferEncrypted, BUFFER_LEN, enc_iv);   // expect 0x65D0F7758B094114AFA6D33A5EA0716A

  sending = true;
  Radio.Send(TxdBufferEncrypted, BUFFER_LEN);
}
