/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       12
 *    D3       13
 *    CMD      15
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      14
 *    VSS      GND
 *    D0       2  (add 1K pull up after flashing)
 *    D1       4
 */

#include "FS.h"
#include "SD_MMC.h"

#define ONE_BIT_MODE  true





void setup(){
    Serial.begin(115200);

    pinMode(2, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);
    pinMode(15, INPUT_PULLUP);
    
    if(!SD_MMC.begin("/sdcard", ONE_BIT_MODE)){
        Serial.println("Card Mount Failed");
        return;
    }
    
    File myFile = SD_MMC.open("/RamChu.txt", FILE_WRITE);
}

void loop(){
    int x = 1234;
    File myFile = SD_MMC.open("/RamChu.txt", FILE_APPEND);
    if (myFile){
    Serial.println("FILE OK !!!");
  } else {
    Serial.println("FILE NOT OK !!!");
  }
    myFile.print(x);
    myFile.println(", tá na hora de molhar o biscoito");
}
