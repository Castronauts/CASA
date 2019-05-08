/**
  @file ArloRobot.h

  @author Parallax Inc.

  @brief Arlo Robot (with DHB-10 motor controller) functions for Arduino.

  @version 0.5

  @copyright
  Copyright (c) Parallax Inc 2016. All rights MIT licensed;
                see license.txt.
*/

#ifndef PARALLAX_ARLO_h
#define PARALLAX_ARLO_h

//#define DISPLAY_ACTIVITY

#include <SoftwareSerial.h>
#include <arduino.h>

#define DHB10_MAX_MOTOR_PWR 127
#define DT_PREV_ENCODER_CHECK 40
#define LAST_EXCG_STR_LEN 96
#define EXCG_STR_LEN 96
#define LAST_EXCG_TX_MIN 16
#define DEFAULT_TOP_SPEED 200

class ArloRobot
{
  public:
    char lastExchange[LAST_EXCG_STR_LEN];
    int timeoutDHB10 = 900;
  public:
    // Set up/tear down
    void begin(SoftwareSerial &serial);
    void end();

    // Movements
    void writeCounts(long left, long right);
    void writeSpeeds(int left, int right);
    void writeMotorPower(int left, int right);

    // Measurements
    int readCountsLeft();
    int readCountsRight();
    int readSpeedLeft();
    int readSpeedRight();
    
    // Communication Modes
    void writePulseMode();
    
    // Information
    int readFirmwareVer();
    int readHardwareVer();
    int readSpeedLimit();

    // Configuration
    void writeConfig(char *configString, int value);
    int readConfig(char *configString);
    void writeSpeedLimit(int countsPerSecond);
    void clearCounts();

    //Nonvolatile Configuration Storage
    void storeConfig(char *configString);
    void restoreConfig(); 

    void checkCharacter(char c, int j);

  private:
    SoftwareSerial *dhb10;
    long returnVal[3];
    long paramVal[3];
    char c[EXCG_STR_LEN];
    unsigned long lastReadCount = 0;
    char lastCommand[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    int topSpeed = DEFAULT_TOP_SPEED;

  private:
    char *com(char *command, int paramCount, int retCount);
}; 

#endif
