/**
  @file ArloRobot.cpp

  @author Parallax Inc.

  @brief Arlo Robot (with DHB-10 motor controller) functions for Arduino.

  @version 0.5

  @copyright
  Copyright (c) Parallax Inc 2016. All rights MIT licensed;
                see license.txt.
*/

#include "ArloRobot.h"

// Set up/tear down

void ArloRobot::begin(SoftwareSerial &serial)
{
  dhb10 = &serial;
  dhb10->print("txpin ch2\r");                 // Receive on different pin
  delay(20);                                   // Wait for reply
  dhb10->flush();                              // Clear input buffer
}

//
void ArloRobot::end()
{
  dhb10->end();
}
//
// Movements

void ArloRobot::writeCounts(long left, long right)
{
  paramVal[0] = left;
  paramVal[1] = right;
  paramVal[2] = topSpeed;
  com("MOVE", 3, 0);
}

void ArloRobot::writeSpeeds(int left, int right)
{
  left = constrain(left, -topSpeed, topSpeed);
  right = constrain(right, -topSpeed, topSpeed);
  paramVal[0] = left;
  paramVal[1] = right;
  com("GOSPD", 2, 0);
}

void  ArloRobot::writeMotorPower(int left, int right)
{
  left = constrain(left, -DHB10_MAX_MOTOR_PWR, DHB10_MAX_MOTOR_PWR);
  right = constrain(right, -DHB10_MAX_MOTOR_PWR, DHB10_MAX_MOTOR_PWR);
  paramVal[0] = left;
  paramVal[1] = right;
  com("GO", 2, 0);
}


// Measurements

int ArloRobot::readCountsLeft()
{
  
  #ifdef DISPLAY_ACTIVITY
    Serial.print("left strcmp ");
    Serial.println(strcmp("DIST", lastCommand));
    Serial.print("left millis ");
    Serial.println(millis() - lastReadCount);
  #endif
  
  if(((millis() - lastReadCount) > DT_PREV_ENCODER_CHECK || (strcmp("DIST", lastCommand))))
  {
    com("DIST", 0, 2);
  }
  lastReadCount = millis();
  return returnVal[0];
}

int ArloRobot::readCountsRight()
{
  #ifdef DISPLAY_ACTIVITY
    Serial.print("right strcmp ");
    Serial.println(strcmp("DIST", lastCommand));
    Serial.print("right millis ");
    Serial.println(millis() - lastReadCount);
  #endif 
  
  if(((millis() - lastReadCount) > DT_PREV_ENCODER_CHECK || (strcmp("DIST", lastCommand))))
  {
    com("DIST", 0, 2);
  }
  lastReadCount = millis();
  return returnVal[1];
}

int ArloRobot::readSpeedLeft()
{
  if(((millis() - lastReadCount) > DT_PREV_ENCODER_CHECK || (strcmp("SPD", lastCommand))))
  {
    com("SPD", 0, 2);
  }
  lastReadCount = millis();
  return returnVal[0];
}

int ArloRobot::readSpeedRight()
{
  if(((millis() - lastReadCount) > DT_PREV_ENCODER_CHECK || (strcmp("SPD", lastCommand))))
  {
    com("SPD", 0, 2);
  }
  lastReadCount = millis();
  return returnVal[1];
}

void ArloRobot::writePulseMode()
{
  com("PULSE", 0, 0);
}


// Information

int ArloRobot::readFirmwareVer()
{
  com("VER", 0, 1);
  return returnVal[0];
}

int ArloRobot::readHardwareVer()
{
  com("HWVER", 0, 1);
  return returnVal[0];
}

int ArloRobot::readSpeedLimit()
{
  return topSpeed;
}


// Configuration
void ArloRobot::writeConfig(char *configString, int value)
{
  paramVal[0] = value;
  com(configString, 1, 0);
}

int ArloRobot::readConfig(char *configString)
{
  com(configString, 0, 1);
  return returnVal[0];
}

void ArloRobot::writeSpeedLimit(int countsPerSecond)
{
  topSpeed = countsPerSecond;
}

void ArloRobot::clearCounts()
{
  com("RST", 0, 0);
}


//Nonvolatile Configuration Storage

void ArloRobot::storeConfig(char *configString)
{
  char temp[] = {'S','T','O','R','E', ' ' ,0,0,0,0,0,0,0,0,0,0,0} ;
  strcpy(&temp[6], configString);
  com(temp, 0, 0);
}

void ArloRobot::restoreConfig()
{
  com("RESTORE", 0, 0);
}    


//Private

char *ArloRobot::com(char *command, int paramCount, int retCount)
{
  dhb10->listen();
  memset(c, 0, LAST_EXCG_STR_LEN);
  memset(lastExchange, ' ', LAST_EXCG_STR_LEN - 1);
  lastExchange[LAST_EXCG_STR_LEN - 1] = 0;
  int idx = 0;

  #ifdef DISPLAY_ACTIVITY
    Serial.print(command);
    for(int i = 0; i < paramCount; i++)
    {
      Serial.print(" ");
      Serial.print(paramVal[i], DEC);
    }
    Serial.print("\\r     ");                   // Display message 
  #endif
  
  dhb10->print(command);                     // Send message to DHB-10
  strcpy(lastExchange, command);
  idx = strlen(command);
  for(int i = 0; i < paramCount; i++)
  {
    dhb10->print(" ");
    lastExchange[idx++] = ' ';
    //lastExchange[idx] = 0;
    dhb10->print(paramVal[i], DEC);
    //idx += sprintf(&lastExchange[idx], "%d", paramVal[i]);
    char *p = itoa(paramVal[i], &lastExchange[idx], 10);
    idx += strlen(p);    
  }
  dhb10->print('\r');                          // Send message to DHB-10
  lastExchange[idx++] = '\\';
  lastExchange[idx++] = 'r';
  //lastExchange[idx++] = 0;
  if(idx < LAST_EXCG_TX_MIN) idx = LAST_EXCG_TX_MIN;
  
  long ti = millis();
  dhb10->readBytes(c, 1);                      // Get first byte of reply
  long tf = millis();
  
  #ifdef DISPLAY_ACTIVITY
     Serial.print("millis = ");
     Serial.println(tf - ti);
     Serial.print("Dec = ");
     Serial.println(c[0], DEC);
  #endif  
  
  if((tf - ti) > timeoutDHB10)
  {
    #ifdef DISPLAY_ACTIVITY
      Serial.println("No reply (timeout)");     // Display timeout message
    #endif  
    strcpy(&lastExchange[idx], "No reply (timeout).");
  }
  else if(c[0] == 0)
  {
    #ifdef DISPLAY_ACTIVITY
      Serial.println("No reply");     // Display timeout message
    #endif  
    strcpy(&lastExchange[idx], "No reply.");
  }
  else if(c[0] == '\r')                            // If DHB-10 replied
  {
    #ifdef DISPLAY_ACTIVITY
      Serial.println("\\r");                    // Display terminating character
    #endif 
    strcpy(&lastExchange[idx], "\\r");
  }
  else if(c[0] == 'E')
  {
    dhb10->readBytesUntil('\r', &c[1], EXCG_STR_LEN - 2);

    #ifdef DISPLAY_ACTIVITY
      Serial.write(c);
      Serial.println("\\r");
    #endif  
    strcpy(&lastExchange[idx], c); 
    idx += strlen(c);
    strcpy(&lastExchange[idx], "\\r"); 
  }
  else
  {
    dhb10->readBytesUntil('\r', &c[1], EXCG_STR_LEN - 2);
    #ifdef DISPLAY_ACTIVITY
      Serial.write(c);
      Serial.println(">>>   Else   <<<");
      Serial.println("\\r");
    #endif  
    strcpy(&lastExchange[idx], c); 
    idx += strlen(c);
    strcpy(&lastExchange[idx], "\\r"); 
  }
  
  for(int i = 0; i < retCount; i++) returnVal[i] = 0;

  int j = -1;
  for(int i = 0; i < retCount; i++)
  {
    while(isAlpha(c[++j]));
    if(isdigit(c[j]) || (c[j] == '-'))
    {
      //returnVal[i] = atoi(&c[j]);
      returnVal[i] = atol(&c[j]);
      while(isDigit(c[++j]));
    }
    #ifdef DISPLAY_ACTIVITY
      Serial.print("rv[");
      Serial.print(i, DEC);
      Serial.print("] = ");
      Serial.println(returnVal[i], DEC);
    #endif  
  }  
  strcpy(lastCommand, command);
  //lastCommand = command;
  return c;
}
