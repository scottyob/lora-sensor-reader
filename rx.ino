/**
   @file LoRaP2P_RX.ino
   @author rakwireless.com
   @brief Receiver node for LoRa point to point communication
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
*/
#include <Arduino.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <ArduinoJson.h>
#include <SPI.h>
#include <ArduinoBearSSL.h>
#include "AES128.h"

// Function declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

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


// No idea why this is needed
// #define LORA_RXCONT true

static RadioEvents_t RadioEvents;

#define BUFFER_LEN 128
static uint8_t RcvBuffer[BUFFER_LEN];

void setup()
{

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  Serial.println("=====================================");
  Serial.println("LoRaP2P Rx Test");
  Serial.println("=====================================");
  // Initialize LoRa chip.
  lora_rak4630_init();
  // Initialize the Radio callbacks
  RadioEvents.TxDone = NULL;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = NULL;
  RadioEvents.RxTimeout = NULL;
  RadioEvents.RxError = OnRxError;
  RadioEvents.CadDone = NULL;

  // Initialize the Radio
  Radio.Init(&RadioEvents);

  // Set Radio channel
  Radio.SetChannel(RF_FREQUENCY);

  // Set Radio RX configuration
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Start LoRa
  Serial.println("Starting Radio.Rx");
  Radio.Rx(0);
  //  Radio.Rx(RX_TIMEOUT_VALUE);
}

void loop()
{
  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions.

  Radio.IrqProcess();

  //  We are on FreeRTOS, give other tasks a chance to run
  delay(100);
  yield();

}

/**@brief Function to be executed on Radio Rx Done event
*/
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  delay(10);
  memcpy(RcvBuffer, payload, size);

  // Attempt to decrypt
  uint8_t key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02};
  uint8_t enc_iv[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01};
  AES128Class AES128;
  AES128.runDec(key, 16, RcvBuffer, BUFFER_LEN, enc_iv);

  // Attempt to get JSON document
  JsonDocument doc;
  auto result = deserializeJson(doc, RcvBuffer);
  Serial.print("# JSON Deserialize Result: ");
  Serial.println(result.c_str());

  if (result == DeserializationError::Ok) {
    doc["rssi_dBm"] = rssi;
    doc["snr"] = snr;
    serializeJson(doc, Serial);
    Serial.println("");
  }

  Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Timeout event
*/
void OnRxTimeout(void)
{
  delay(10);
  Serial.println("OnRxTimeout");
  Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Error event
*/
void OnRxError(void)
{
  delay(10);
  Serial.println("OnRxError");
  Radio.Rx(0);
}
