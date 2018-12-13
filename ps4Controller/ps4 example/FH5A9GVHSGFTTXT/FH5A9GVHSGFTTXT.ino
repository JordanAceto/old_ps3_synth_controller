#include <PS4BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

USB Usb;

BTD Btd(&Usb);

PS4BT PS4(&Btd, PAIR);

unsigned char red = 0, green = 0, blue = 0;
boolean lRed = false, lGreen = false, lBlue = false, printRGB = false;
char state = 0; // 0 = Accelerometer controlled, 1 = RS + LT

void setup() {
  Serial.begin(19200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}
void loop() {
  Usb.Task();

  if (PS4.connected()) {
      PS4.setLed(red, green, blue);
      
      switch(state){
      
        case 0:  // Accelerometers
          if(!lRed){
            red = (unsigned char) (PS4.getSensor(aX) / 256.0) + 128;
            //if(red > 
          }
          if(!lGreen){
            green = (unsigned char) (PS4.getSensor(aY) / 256.0) + 128;
          }
          if(!lBlue){
            blue = (unsigned char) (PS4.getSensor(aZ) / 256.0) + 128;
          }
          break;
        case 1: // R Thumbstick and L Trigger
          if(!lRed){
            red = PS4.getAnalogHat(RightHatX);
          }
          if(!lGreen){
            green = PS4.getAnalogHat(RightHatY);
          }
          if(!lBlue){
            blue = PS4.getAnalogButton(L2);
          }
          break;
        default:
          break;
      }
      
      if(PS4.getButtonClick(UP)){
         if(state == 1){
           state = 0;
         } else {
           state = 1; 
         }
      }
      if(PS4.getButtonClick(CROSS)){
          printRGB = !printRGB;
      }
      if(printRGB){
          Serial.print(F("\nR: "));
          Serial.print(red);
          Serial.print(F("\tG: "));
          Serial.print(green);
          Serial.print(F("\tB: "));
          Serial.print(blue);
          Serial.print(F("\n\r"));
      } 
      if(PS4.getButtonClick(OPTIONS)){
          // Print Accelerometer data 
          Serial.print(F("\nAccel X: "));
          Serial.print(PS4.getSensor(aX));
          Serial.print(F("\tAccel Y: "));
          Serial.print(PS4.getSensor(aY));
          Serial.print(F("\tAccel Z: "));
          Serial.print(PS4.getSensor(aZ));
          Serial.print(F("\n\r"));
      }
      if(PS4.getButtonClick(SHARE)){
          // Print Gyro data
          Serial.print(F("\nGyro X: "));
          Serial.print(PS4.getSensor(gX));
          Serial.print(F("\tGyro Y: "));
          Serial.print(PS4.getSensor(gY));
          Serial.print(F("\tGyro Z: "));
          Serial.print(PS4.getSensor(gZ));
          Serial.print(F("\n\r"));
      }
      if(PS4.getButtonClick(SQUARE)){
         lRed = !lRed;
         if(lRed){
            Serial.print(F("Red paused\n"));
          } else {
           
            Serial.print(F("Red unpaused\n"));
         }
      }
      if(PS4.getButtonClick(TRIANGLE)){
         lGreen = !lGreen;
         if(lGreen){
            Serial.print(F("Green paused\n"));
          } else {
           
            Serial.print(F("Green unpaused\n"));
         }
      }
      if(PS4.getButtonClick(CIRCLE)){
         lBlue = !lBlue;
         if(lBlue){
            Serial.print(F("Blue paused\n"));
          } else {
           
            Serial.print(F("Blue unpaused\n"));
         }
      }
      
      if(PS4.getButtonClick(PS)){
       PS4.disconnect();
       Serial.end();
      }
      
    }
}
