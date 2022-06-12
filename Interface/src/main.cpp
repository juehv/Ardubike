#include <Arduino.h>
#include <FS.h>     // SD Card ESP32
#include <SD_MMC.h> // SD Card ESP32

#include <soc/soc.h>           // Disable brownour problems
#include <soc/rtc_cntl_reg.h>  // Disable brownour problems

// second serial port for receiving data from controller
#define RXD2 16
#define TXD2 17

#include <BluetoothSerial.h>
BluetoothSerial ESP_BT;

bool sdCardMounted = false;
String path = "/Log.txt";
byte* serialBuffer = new byte[100];
/* setup function */
void setup(void)
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  if (!SD_MMC.begin())
  {
    Serial.println("SD Card Mount Failed");
  }
  else
  {
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD Card attached");
    }
    else
    {
      Serial.println("SD Card Mount Successful");
      sdCardMounted = true;
    }
  }

  Serial.println("Start BT");
  delay(100);
  ESP_BT.begin("Ardubike");

  Serial.println("Setup done.");
}

void loop(void)
{
  if (Serial.available())
  {
    ESP_BT.write(Serial.read());
  }
  if (ESP_BT.available())
  {
    Serial.write(ESP_BT.read());
  }

  if (Serial2.available())
  {
    ESP_BT.write(char(Serial2.read()));
    // size_t size = Serial2.readBytes(serialBuffer, 100);

    // ESP_BT.print("NANO: ");
    // ESP_BT.write(serialBuffer, size);
    // Serial.print("NANO:");
    // Serial.write(serialBuffer, size);

    // if (sdCardMounted){
    //   fs::FS &fs = SD_MMC; 
    //   File file = fs.open(path.c_str(), FILE_APPEND);
    //   if(file){
    //     file.write(serialBuffer, size); 
    //     file.write('\n');
    //   }
    //   file.close();
    // }
  }

  delay(1);
}