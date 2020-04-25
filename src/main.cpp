/**
 * @brief Main source file. Contains the main routines.
 * 
 * @file main.cpp
 */

/* Includes -------------------------------------------- */
/* MCP CAN module */
#include "mcp_can.h"

/* Hex tools */
#include "HexTools.hpp"

/* Arduino framework */
#include <Arduino.h>

/* Defines --------------------------------------------- */
#define LOG_BAUDRATE 115200U

/* GPIO pins */
#define MCP_INT_PIN 2

/* CAN SPI defines */
#define CAN_BAUDRATE            CAN_500KBPS
#define CAN_MESSAGE_MAX_SIZE    8U
#define CAN_SPI_CS              10

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif /* IRAM_ATTR */

/* Global variables ------------------------------------ */
MCP_CAN gCAN(CAN_SPI_CS);

/* Static variables ------------------------------------ */

/* Helper functions ------------------------------------ */
void printCANMsg(const uint32_t &pCOBID, const uint8_t &pLen, const uint8_t * const pData, const uint8_t &pFlags) {
    Serial.print(uint32ToHexStr(pCOBID));
    Serial.print(" [");
    Serial.print(pLen);
    Serial.print("] ");
    for(uint8_t i = 0U; (i < pLen) && (i < CAN_MESSAGE_MAX_SIZE); ++i) {
        Serial.print(uint8ToHexStr(pData[i], false));
        Serial.print(" ");
    }
    Serial.print("(");
    Serial.print(uint8ToHexStr(pFlags));
    Serial.println(")");
}

void printTxCANMsg(const uint32_t &pCOBID, const uint8_t &pLen, const uint8_t * const pData, const uint8_t &pFlags) {
    Serial.print("Tx < ");
    printCANMsg(pCOBID, pLen, pData, pFlags);
}

void printRxCANMsg(const uint32_t &pCOBID, const uint8_t &pLen, const uint8_t * const pData, const uint8_t &pFlags) {
    Serial.print("Rx > ");
    printCANMsg(pCOBID, pLen, pData, pFlags);
}

uint8_t initCAN(void) {
    uint8_t lResult = gCAN.begin(MCP_ANY, CAN_BAUDRATE, MCP_8MHZ);
    gCAN.setMode(MCP_NORMAL);

    return lResult;
}

/* Interrupts ------------------------------------------ */
void IRAM_ATTR MCP_GeneralInt_Falling(void) {
    Serial.println("[ INT ] <MCP_GeneralInt> FALLING");
}

void IRAM_ATTR MCP_GeneralInt_Rising(void) {
    Serial.println("[ INT ] <MCP_GeneralInt> RISING");
}

/* On-boot routine ------------------------------------- */
void setup(void) {
    /* Set up Serial port */
    Serial.begin(LOG_BAUDRATE);

    /* Setup CAN bus */
    Serial.println("[BOOT ] Setting up the MCP2515 CAN module");
    if(CAN_OK != gCAN.begin(MCP_ANY, CAN_BAUDRATE, MCP_8MHZ)) {
        Serial.println("[ERROR] CAN setup failed");
        while(true);
    }
    gCAN.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
    
    /* Set up interrupts */
    pinMode(MCP_INT_PIN, INPUT);
    attachInterrupt(MCP_INT_PIN, MCP_GeneralInt_Falling, FALLING);
    attachInterrupt(MCP_INT_PIN, MCP_GeneralInt_Rising, RISING);

    /* End of setup */
    Serial.println("[BOOT ] Boot successful !");
}

/* Event callbacks ------------------------------------- */

/* Loop routine ---------------------------------------- */
void loop(void) {
    static uint8_t sMCPError = 0x00U;
    /* Check for errors */
    if(CAN_OK != gCAN.checkError()) {
        Serial.println("[ERROR] Detected error w/ MCP_CAN::checkError");
        sMCPError = gCAN.getError();
        if(MCP_EFLG_RX1OVR == (MCP_EFLG_RX1OVR & sMCPError)) {
            //Serial.println("        Rx buffer 1 overflow");
        }
        if(MCP_EFLG_RX0OVR == (MCP_EFLG_RX0OVR & sMCPError)) {
            //Serial.println("        Rx buffer 0 overflow");
        }
        if(MCP_EFLG_TXBO == (MCP_EFLG_TXBO & sMCPError)) {
            Serial.println("        BUS-OFF");
        }
        if(MCP_EFLG_TXEP == (MCP_EFLG_TXEP & sMCPError)) {
            Serial.println("        Tx ERROR-PASSIVE");
        }
        if(MCP_EFLG_RXEP == (MCP_EFLG_RXEP & sMCPError)) {
            Serial.println("        Rx ERROR-PASSIVE");
        }
        if(MCP_EFLG_TXWAR == (MCP_EFLG_TXWAR & sMCPError)) {
            Serial.println("        Tx ERROR-WARNING");
        }
        if(MCP_EFLG_RXWAR == (MCP_EFLG_RXWAR & sMCPError)) {
            Serial.println("        Rx ERROR-WARNING");
        }
        if(MCP_EFLG_EWARN == (MCP_EFLG_EWARN & sMCPError)) {
            Serial.println("        ERROR-WARNING");
        }
    }

    /* Check if there are any available CAN messages */
    while(CAN_MSGAVAIL == gCAN.checkReceive()) {
        uint32_t lRxCOBID = 0x000U;
        uint8_t  lRxFlags = 0U;
        uint8_t  lRxLen   = 0U;
        uint8_t  lRxData[CAN_MESSAGE_MAX_SIZE];
        memset(lRxData, 0U, CAN_MESSAGE_MAX_SIZE);

        if(CAN_OK != gCAN.readMsgBuf(&lRxCOBID, &lRxFlags, &lRxLen, lRxData)) {
            Serial.println("[ERROR] Message avaiable, but failed to read.");
        } else {
            if((0x00000000U == lRxCOBID) && (0x00 == lRxFlags) && (0 == lRxLen)) {
                Serial.println("[ERROR] Msg available, but nothing received.");

                /* Wait for things to cool down */
                delay(100U);

                /* Reinit the CAN controller */
                if(CAN_OK != initCAN()) {
                    Serial.println("[ERROR] CAN reinitialization failed");
                    while(true);
                }
            } else {
                printRxCANMsg(lRxCOBID, lRxLen, lRxData, lRxFlags);
            }
        }
    }

    delay(10U);
}
