/*
   RadioLib SX1262 Transmit Example

   This example transmits packets using SX1262 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

#include <RadioLib.h>
#include "boards.h"

#if defined(USING_SX1268_433M)
uint8_t txPower = 22;
float radioFreq = 433.0;
SX1268 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#elif defined(USING_SX1262_868M)
uint8_t txPower = 22;
float radioFreq = 868.0;
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

#endif

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void)
{
    // check if the interrupt is enabled
    if (!enableInterrupt)
    {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}

void setup()
{
    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    // initialize SX126x with default settings
    Serial.print(F("[SX126x] Initializing ... "));
    int state = radio.begin(radioFreq);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
            ;
    }

    // set the function that will be called
    // when new packet is received
    radio.setDio1Action(setFlag);

    // start listening for LoRa packets
    Serial.print(F("[SX126x] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("success!"));
    }
    else
    {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true)
            ;
    }

    // if needed, 'listen' mode can be disabled by calling
    // any of the following methods:
    //
    // radio.standby()
    // radio.sleep()
    // radio.transmit();
    // radio.receive();
    // radio.readData();
    // radio.scanChannel();
}

void loop()
{
    // check if the flag is set
    if (receivedFlag)
    {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;

        // you can read received data as an Arduino String
        String str;
        int state = radio.readData(str);

        // you can also read received data as byte array
        /*
          byte byteArr[8];
          int state = radio.readData(byteArr, 8);
        */

        if (state == RADIOLIB_ERR_NONE)
        {
            // packet was successfully received
            Serial.println(F("[SX126x] Received packet!"));

            // print data of the packet
            Serial.print(F("[SX126x] Data:\t\t"));
            Serial.println(str);

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("[SX126x] RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("[SX126x] SNR:\t\t"));
            Serial.print(radio.getSNR());
            Serial.println(F(" dB"));

#ifdef HAS_DISPLAY
            if (u8g2)
            {
                u8g2->clearBuffer();
                char buf[256];
                u8g2->drawStr(0, 12, "Received OK!");
                snprintf(buf, sizeof(buf), "RX:%s", str);
                u8g2->drawStr(0, 24, buf);
                snprintf(buf, sizeof(buf), "RSSI:%.2f", radio.getRSSI());
                u8g2->drawStr(0, 36, buf);
                snprintf(buf, sizeof(buf), "SNR:%.2f", radio.getSNR());
                u8g2->drawStr(0, 48, buf);
                u8g2->sendBuffer();
            }
#endif
        }
        else if (state == RADIOLIB_ERR_CRC_MISMATCH)
        {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));
        }
        else
        {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }

        // put module back to listen mode
        radio.startReceive();

        // we're ready to receive more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}
