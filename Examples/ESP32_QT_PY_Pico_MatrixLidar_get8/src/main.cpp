#include <Arduino.h>

// --- Forward Declarations ---
void setup(void);
void loop(void);

/*!
 * @file get8_8Data.ino
 * @brief This is a demo for retrieving all TOF data. Running this demo will allow you to get all TOF data.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [tangjie](jie.tang@dfrobot.com)
 * @version  V1.0
 * @date  2025-04-03
 * @url https://github.com/DFRobot/DFRobot_MatrixLidar
 */

#include "Wire.h"
#include "DFRobot_MatrixLidar.h"

DFRobot_MatrixLidar_I2C tof(0x33, &Wire1); // VERY IMPORTANT 2ND PARAM: Wire1 or Wire
uint16_t buf[64];

// Define Adafruit QT Py ESP32 Pico STEMMA QT hardware I2C pins
#define STEMMA_SDA 19
#define STEMMA_SCL 22

void setup(void){
  Serial.begin(115200);
  delay(2000);
  int resp;
  //tof = new DFRobot_MatrixLidar_I2C(0x33, &Wire1); // VERY IMPORTANT 2ND PARAM: Wire1 or Wire
  Wire1.setPins(STEMMA_SDA, STEMMA_SCL);
  Wire1.begin(); 
  //Wire1.setClock(400000); // 400kHz Fast Mode is highly recommended for Matrix ToF data

  //resp = tof->init();
  //Serial.print("tof->init() returned: ");
  //Serial.println(resp);
  //delay(2000);
  //if(resp != 0){
  //  Serial.println("exiting sketch.");
  //  exit(0);
  //}

  while(tof.begin() != 0){
    Serial.println("begin error !!!!!");
  }
  Serial.println("begin success");
  //config matrix mode
  while(tof.setRangingMode(eMatrix_8X8) != 0){
    Serial.println("init error !!!!!");
    delay(1000);
  }
  Serial.println("init success");
}

void loop(void){
  tof.getAllData(buf);
  int val = -1;
  for(uint8_t i = 0; i < 8; i++){
    Serial.print("Y");
    Serial.print(i);
    Serial.print(":\t");
    for(uint8_t j = 0; j < 8; j++){
      val = buf[i * 8 + j];
      Serial.printf("%04d\t", val);
    }
    Serial.println("");
  }
  Serial.println("------------------------------");
  delay(500);
}

