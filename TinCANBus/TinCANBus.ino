/*
  CANBed FD - https://www.longan-labs.cc/1030009.html
*/

#include <SPI.h>
#include <EEPROM.h>
#include "mcp2518fd_can.h"
#include "Adafruit_VL53L0X.h"

//#define DEBUG

// pinS for CAN-FD Shield,
//const int SPI_CS_PIN = 9;
//const int CAN_INT_PIN = 2;

// pinS for CANBed FD
const int SPI_CS_PIN = 17;
const int CAN_INT_PIN = 7;
const int LED_PIN = 13;

uint16_t DEVICE_NUM_WITH_HEADER;
uint8_t DEVICE_NUM = 0x00;

const uint8_t DEVICE_NUM_VALID_HEADER = 0xAB; 
const int EEPROM_DEV_NUM_ADDR = 0;

const unsigned int ROBORIO_DEV_ID = 0x00;

// Source: https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
const unsigned long FRC_ROBOT_CONTROLLER = 0x01010000;
const unsigned long FRC_HEARTBEAT_API = 0x00001840;
const unsigned long FRC_HEARTBEAT_FLT = FRC_ROBOT_CONTROLLER + FRC_HEARTBEAT_API;

const unsigned long TEAM_USE_FLT = ((uint32_t)0x0A << 24) + ((uint32_t)0x08 << 16);

const unsigned long DEV_CFG_API_CLASS = 0;
const unsigned long SET_DEV_NUM_API_INDEX = 1;
const unsigned long SET_DEV_NUM_ID =
                                TEAM_USE_FLT + (DEV_CFG_API_CLASS << 10) +
                                (SET_DEV_NUM_API_INDEX << 6);

const unsigned long SENSOR_API_CLASS = 1;
const unsigned long SENSOR_READ_API_INDEX = 1;
const unsigned long SENSOR_READ_ID =
                                TEAM_USE_FLT + (SENSOR_API_CLASS << 10) +
                                (SENSOR_READ_API_INDEX << 6);



unsigned long lastHeartbeat;
unsigned long lastLedBeat = 0;
unsigned long lastTofUpdate = 0;
uint16_t tofRange;
boolean ledState = false;


mcp2518fd TinCAN(SPI_CS_PIN); // Set CS pin

Adafruit_VL53L0X tof = Adafruit_VL53L0X();


void setup() {
#ifdef DEBUG
    Serial.begin(115200); // USB Serial
    while(!Serial);
#endif
    while (CAN_OK != TinCAN.begin(CAN20_1000KBPS)) {             // init can bus : baudrate = 1Mbps
#ifdef DEBUG
        Serial.println("CAN init fail, retry...");
#endif
        delay(100);
    }
#ifdef DEBUG
    Serial.println("CAN init ok!");
#endif

    if (!tof.begin()) {
#ifdef DEBUG
      Serial.println(F("Failed to boot VL53L0X"));
#endif
    }

    // Get device number from non-volatile location
    EEPROM.get(EEPROM_DEV_NUM_ADDR, DEVICE_NUM_WITH_HEADER);
               
    // If header byte doesn't match, then EEPROM has not been written before.
    if (DEVICE_NUM_WITH_HEADER & 0x0FF != DEVICE_NUM_VALID_HEADER) {
      DEVICE_NUM_WITH_HEADER = (DEVICE_NUM_VALID_HEADER + (((uint16_t)DEVICE_NUM) << 8);
      EEPROM.write(EEPROM_DEV_NUM_ADDR, DEVICE_NUM_WITH_HEADER);
    } else {
      DEVICE_NUM = (uint8_t)(DEVICE_NUM_WITH_HEADER >> 8);
    }

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // init_Filt_Mask(filter number, ext, filter, mask)
    // There're 32 set of filter/mask for MCP2517FD, filter number can be set to 0~31
    //TinCAN.init_Filt_Mask(0, 0, 0, 0);     // get all standard frame
    //TinCAN.init_Filt_Mask(1, 1, 0, 0);     // get all extended frame

    // FRC Heartbeat filter
    TinCAN.init_Filt_Mask(0, 1, FRC_HEARTBEAT_FLT, 0x1FFFFFFF);

    // Team Use filter
    TinCAN.init_Filt_Mask(1, 1, TEAM_USE_FLT + DEVICE_NUM, 0x1FFF003F);

    tof.startRangeContinuous();

}


void loop() {
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == TinCAN.checkReceive()) // check if data coming
    {

        TinCAN.readMsgBuf(&len, buf);  // You should call readMsgBuff before getCanId
        unsigned long id = TinCAN.getCanId();
        unsigned char ext = TinCAN.isExtendedFrame();

        if (id == FRC_HEARTBEAT_FLT) {
          lastHeartbeat = millis();
          ledState = true;
        }
        if (id == SET_DEV_NUM_ID + DEVICE_NUM) {
          EEPROM.write(EEPROM_DEV_NUM_ADDR, (DEVICE_NUM_VALID_HEADER + (((uint16_t)buf[0]) << 8));
        }

#ifdef DEBUG
        Serial.print(ext ? "GET EXTENDED FRAME FROM ID: 0X" : "GET STANDARD FRAME FROM ID: 0X");
        Serial.println(id, HEX);

        Serial.print("Len = ");
        Serial.println(len);
            // print the data
        for (int i = 0; i < len; i++) {
            Serial.print(buf[i]);
            Serial.print("\t");
        }
        Serial.println();
#endif
    }

    unsigned long now = millis();
    // heartbeat timeout
    if (now - lastHeartbeat >= 100) {
      ledState = false;
    }
    // blink LED a bit faster than once per second
    if (!ledState || now - lastLedBeat >= 375) {
      lastLedBeat = now;
      digitalWrite(LED_PIN, ledState ? !digitalRead(LED_PIN) : LOW);
    }

    if (tof.isRangeComplete()) {
      tofRange = tof.readRange();
#ifdef DEBUG
      Serial.print("Distance in mm: ");
      Serial.println(tofRange);
#endif
    }
    if (now - lastTofUpdate > 500) {
      lastTofUpdate = now;
      TinCAN.sendMsgBuf(SENSOR_READ_ID + DEVICE_NUM, 1, 2, (byte*)&tofRange);
    }

}


// END FILE
