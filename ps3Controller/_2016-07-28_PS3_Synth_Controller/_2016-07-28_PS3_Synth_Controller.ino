/* TO DO ---
 *  
 *  -RETHINK SHOULDER BUTTONS? (MIGHT BE OK)
 *  
 *  -WRITE 1V/OCT DAC CODE (MIGHT BE GOOD TO GO)
 *  
 *  -DAC MULTIPLIER OVERFLOWS AT HIGH END
 *  
 *  -STARTING WITH SLOTON == TRUE CREATES ODDNESS WHEN USING THE OCTAVE SWITCH FOR THE FIRST TIME
 *  
 *  -ADD UNMULTIPLIER CLOCK OUT?
 *  
 *  -ADD CLOCK SYNC IN?
 *  
 *  -IMPROVE CLOCK ADJUSTMENT RESOLUTION AT FAST SPEEDS
 *  
 *  -CLOCK SPEED UP/SLOW DOWN BUTTONS FEEL ODD WHEN CLOCK PERIOD IS LONG, AS-IS CLOCK PERIOD MUST COMPLETE BEFORE NEW SHORTER CLOCK PERIOD UPDATES
 *  
 *  -IMPROVE CLOCK MULTIPLIER?
 *  
 *  -SCROLLING THROUGH PAT LENGTH THE FIRST TIME IS ODD, GOES 8, 7, 6, 5, 4, 3, 2, 3, 2, THEN IS NORMAL AFTER THAT
 *  
 *  -FINISH PATTER GENERATOR LED DRIVER (NEOPIXEL STICKS IN MAIL, EXPERIMENT WHEN THEY ARRIVE)
 *  
 *  -FINISH ANALOG OUTS, CHECK DIRECTION OF JOYSTICKS, CONSTRAIN(?) ACCELEROMETER
 *  
 *  -WHEN DONE, PASTE IN BLUE TOOTH SECTION
 *  
 *  -UPDATE SPECIFICATION
 *  
 *  TIDY UP
 *  
 */



/***********************************************************************************
 *     -----PLAYSTATION 3 CONTROLLER USED TO CONTROL ANALOG SYNTHESIZERS-----      *
 *                                                                                 *
 *              HARDWARE IS ARDUINO MEGA AND USB HOST SHIELD                       *                                               
 *                                                                                 *
 *           Generates analog signals for each of the four joystick                *
 *           axis, analog signals for accelerometer X and Y axis,                  *
 *           a one volt per octave control signal, four different                  *
 *           gate signals, and a tap tempo clock signal.                           *
 *                                                                                 *
 *           The joystick position value is sent to four PWM                       *
 *           DACs and scaled with opamps.                                          *
 *                                                                                 *
 *           The accelerometer X and Y values are scaled and sent to               *
 *           two PWM DACs, and then scaled with opamps.                            *
 *                                                                                 *
 *           The shoulder buttons are used as a four bit binary                    *
 *           keyboard, while the UP/DOWN arrow act as an octave                    *
 *           switch.  This is fed into an external MCP4822 DAC                     *
 *           serially, and then scaled with an opamp.                              *
 *                                                                                 *
 *           Pressing any of the four shoulder buttons sends a                     *
 *           gate signal high.                                                     *
 *                                                                                 *
 *           The SQUARE, TRIANGLE, and X buttons each send a gate                  *
 *           signal high.                                                          *
 *                                                                                 *
 *           The CIRCLE button is used to tap in a tempo to the                    *
 *           clock signal.  The LEFT and RIGHT buttons slow down                   *
 *           and speed up the clock.  The SELECT button scrolls                    *
 *           through the multiplier that is applied to the clock                   *
 *           frequency.  1x, 2x, 3x, 4x, 6x and 8x are available.                  *
 *           The START button starts and stops the clock signal.                   *
 *                                                                                 *
 ***********************************************************************************/

 
byte nonsense_var = 0;  //this line maybe required for IDE compatibility
// removed 04-12 #include <SoftwareSerial.h>  //Serial communication from any port 
#include <EEPROM.h>  // EEPROM Storage

//------------------PS3 CONTROLLER INITIALIZATION BELOW--------------
#include <PS3BT.h>
#include <SPP.h>
#include <usbhub.h>
#include <PS3USB.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */

SPP SerialBT(&Btd); // This will set the name to the defaults: "Arduino" and the pin to "0000"

//PS3BT PS3(&Btd); // This will just create the instance
PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0xE7, 0x96, 0xA6); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
//PS3USB PS3usb(&Usb); // This will just create the instance
PS3USB PS3usb(&Usb,0x00,0x15,0x83,0xE7,0x96,0xA6); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;

//------------------PS3 CONTROLLER INITIALIZATION ABOVE--------------


//            VARIABLES AND CONSTANTS

byte              plugged             =     0;                //is PS3 controller plugged in?


//        SEQUENCER AND PATTERN GENERATOR 

unsigned int      rawPatLength        =     6;                //because modulo
unsigned int      patLength           =     8;                //how long is the pattern? 2 to 8

unsigned int      slot                =     0;                //position of pattern generator for editing, 0 to 7
boolean           slotOn              =     true;             //while true the slot selector is active, and you can turn individual steps on and off
boolean           slotTimerRun        =     false;            //while true the slot timeout timer is running
boolean           slotOutatime        =     true;             //true when the slot timout timer runs out
unsigned long     slotTimeOut         =     0;                //determines how long you have to edit a step before timing out

boolean           pattern[8]          =     {};               //pattern array, true means active step, edited by selecting position with LEFT/RIGHT, writing with SELECT
unsigned int      patIndex            =     0;                //where in the pattern are we?
boolean           stepOn              =     false;            //clock output is high when true

unsigned int      greenLED            =     255;              //LED outputs are powers of 2,
unsigned int      redLED              =     0;                //used to set output ports
unsigned int      yellowLED           =     0;                //directly.  Needs lots of wires, may be smarter to use addressable LEDs and control them serially?  Works tho.

int               sequence[8]         =     {};               //edited by selecting position with LEFT/RIGHT, programming note with shoulder buttons and UP/DOWN, and writting with SQUARE 

//        TAP TEMPO CLOCK

unsigned int      rawClockMult        =     0;                 //because modulo and don't want zero for actual multiplier, maybe there is a smarter way to do it?
unsigned int      clockMultiplier     =     2;                 //multiplier for clock speed, 0.5x, 1x, 1.5x, 2x, 3x or 4x.  Actually accomplishes this by dividing the clock period. CURRENTLY INCORRECT.

unsigned int      taps                =     0;                 //how many recent taps have there been?
unsigned long     lastTap             =     0;                 //when the last tap happened 
unsigned int      tapLength           =     500;               //average of time between recent taps
unsigned long     timerArray[4]       =     {};                //array of most recent tap counts
unsigned int      tapIndex            =     0;                 //where in the array of tap counts are we?
unsigned int      biggest             =     500;               //the array is split into big,
unsigned int      smallest            =     500;               //medium and small, this is to
unsigned int      middle              =     500;               //eliminate outliers and avoid jarring transitions

long              clockAdjust         =     0;                 //speed up/slow down clock with right and left D-pad
unsigned int      adjustment          =     0;                 //adjustment value is a percentage of the clock period

unsigned long     clockPeriod         =     500;               //clock period in milliseconds
unsigned long     outatime            =     0;                 //this is when the timer will trigger next

boolean           clockReady          =     true;              //toggled by start button, used to make sure clock starts at the beginning of a cycle
boolean           clockOn             =     true;              //when clockOn is true the clock signal appears at pin 13
boolean           toggle              =     true;              //toggles true/false/true/false at clock rate
boolean           risingEdge          =     true;


//        JOYSTICK

int               rightY              =     127;              //value of each joystick axis, 0 to 255
int               rightX              =     127; 
int               leftY               =     127;
int               leftX               =     127;

const int         RIGHT_Y_OUT         =     2;                 //pin to output scaled joystick value
const int         RIGHT_X_OUT         =     3;                 //via PWM DAC
const int         LEFT_Y_OUT          =     4;
const int         LEFT_X_OUT          =     5;


//        ACCELEROMETER

const int         ACCEL_X_OUT         =     11;                 //pin to output scaled X and Y value
const int         ACCEL_Y_OUT         =     12;                 //via PWM DAC


//        1V/OCTAVE

byte              shoulderNow         =     1;                  //vslue of shoulder buttons while pressed, 1 to 15
byte              shoulderValue       =     1;                  //updated value of shoulder button combination, 1 to 15


int               octave              =     0;                  //up and down arrows select octave shift, +6, +5, +4, +3, +2, +1 or +0, pressing up counts up, pressing down counts down
int               octaveNow           =     0;                  //value of octave switch while pressed, is either +2, +3, or +4

int               oneVoltPerOct       =     0;                  //wip, number consisting of shoulder value plus octave switch, use external DAC?           

boolean           shoulderPressed     =     false;              //true if any shoulder button is pressed
const byte        SHOULDER_GATE       =     A4;                 //pin for shoulder gate signal, goes high whenever a shoulder button is pressed

unsigned long     thisShoulder        =     0;
unsigned long     lastShoulder        =     0;                  //when the last shoulder button was pressed

const int         PIN_CS              =     53;
const int         GAIN_1              =     0x1;
const int         GAIN_2              =     0x0;



//      RIGHT BUTTON PAD GATES

const byte        SQUARE_GATE         =     6;                  //pin for square button gate signal
const byte        CROSS_GATE          =     7;                  //pin for x button gate signal
const byte        TRI_GATE            =     8;                  //pin for triabgle button gate signal

         
//        LOOP COUNTING

int               interval            =     25;
long              currentMillis       =     0;
long              lastMillis          =     0;
long              loops               =     0;
long              lastloops           =     0;

//        BATTERY STATE LEDs

byte              LedState            =     0;

//------------------------------SETUP LOOP---------------
void setup(){

//-------------------- PATTERN GENERATOR ----------------

for (int i = 0; i < 8; i ++) {          //set pattern to all gates on
    pattern[i] = true;
}

for (int i = 0; i < 8; i ++) {          //set sequence to all notes middle C
    sequence[i] = 37;
}

//DDRA = B11111111;                       //set all PORT A pins to output, pins 22 to 29
//DDRC = B11111111;                       //set all PORT C pins to output, pins 37 to 30


//---------------------- MCP4822 DAC SPI ---------------

pinMode(PIN_CS, OUTPUT);                //setup SPI communication via pins 51, 52 & 53
digitalWrite(PIN_CS, HIGH);             //51 is MOSI, -> MCP4822 pin 4
SPI.begin();                            //52 is SCK, -> MCP4822 pin 3
SPI.setClockDivider(SPI_CLOCK_DIV2);    //53 is SS, -> MCP4822 pin 2



//---------------------- OUTPUTS/INPUTS -----------------

pinMode(2, OUTPUT);                     //RIGHT_Y_OUT
pinMode(3, OUTPUT);                     //RIGHT_X_OUT
pinMode(4, OUTPUT);                     //LEFT_Y_OUT
pinMode(5, OUTPUT);                     //LEFT_X_OUT

pinMode(11, OUTPUT);                    //ACCEL_X_OUT
pinMode(12, OUTPUT);                    //ACCEL_Y_OUT

pinMode(A4, OUTPUT);                    //SHOULDER_GATE

pinMode(6, OUTPUT);                     //SQUARE_GATE
pinMode(7, OUTPUT);                     //CROSS_GATE
pinMode(8, OUTPUT);                     //TRI_GATE

pinMode(13, OUTPUT);                    //blinky for testing
  
//Pin9 = used for USB shield
//Pin10 = used for USB shield


  
//------------------------------BEGIN SERIAL COMMUNICATION----------------
  Serial.begin(38400); 

//--------------------------------BEGIN PS3 INITIALIZATION------------------
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
//--------------------------------END PS3 INITIALIZATION------------------


}

void loop() {
  Usb.Task(); //PS3 get info command
  PS3.attachOnInit(BTonInit);
  PS3usb.attachOnInit(USBonInit);
  SerialBT.attachOnInit(SerialBTonInit);

/*  
  long currentMillis = millis();
  loops++;

  if(currentMillis - lastMillis > 1000){
    lastloops=loops;
    lastMillis = currentMillis;
    loops = 0;
    Serial.print("main loops last second:");
    Serial.println(lastloops);
  }
*/

//---------------UNPLUGGED CONTROLLER ROUTINE-------------
//------------(includes bracket at end of main loop)------
  if(PS3.PS3Connected == 0 && PS3usb.PS3Connected == 0){ //skip loop if no controller found
    //Serial.println(" controller unplugged ");

  }else if(PS3usb.PS3Connected == 0){ 

    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3.disconnect();
    }

    //PS3.printStatusString();
    if(PS3.getStatus(Shutdown)){
      if(LedState != 1){
        PS3.setAllOff();
        PS3.setLedToggle(LED1);
        LedState=1;
      }
    }else if(PS3.getStatus(Dying)){
      if(LedState != 2){
        PS3.setAllOff();
        PS3.setLedToggle(LED2);
        LedState=2;
      }
   }else if (PS3.getStatus(Low)){
      if(LedState != 3){
        PS3.setAllOff();
        PS3.setLedToggle(LED3);
        LedState=3;
      }
    }else if (PS3.getStatus(High) || PS3.getStatus(Full)){
       if(LedState != 4){
        PS3.setAllOff();
        PS3.setLedToggle(LED4);
        LedState=4;
      }
    }
    
    
      

      


 
 } else if(PS3.PS3Connected == 0){ //USB COMMANDS BELOW-----------------------------------------------------------------------------------------------------------------

    if (PS3usb.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3usb.Release();
    }
  
  //*********************** PATTERN GENERATOR ***********************************************//

  
  if (PS3usb.getButtonClick(R3)) {
      rawPatLength ++;
  }
  
  if (PS3usb.getButtonClick(L3)) {
      rawPatLength --;
  } 
  patLength = (rawPatLength % 7) + 2;

  //Serial.println(patLength);
  
  if (PS3usb.getButtonClick(RIGHT)) {
       if (slotOutatime == true) {
          slot --;
       }
       slot ++;
       slot = slot % patLength;
       yellowLED = 1;
       yellowLED <<= slot;
       slotOn = true;
       slotTimerRun = true;
       slotOutatime = false;
       slotTimeOut = millis();
  }
  
  if (PS3usb.getButtonClick(LEFT)) {
      if (slotOutatime == true) {
          slot ++;
      }
      slot --;
      slot = slot % patLength;
      yellowLED = 1;
      yellowLED <<= slot;
      slotOn = true;
      slotTimerRun = true;
      slotOutatime = false;
      slotTimeOut = millis();
  }

  
  if (slotOn == true) {
      if (PS3usb.getButtonClick(SQUARE)) {
          sequence[slot] = shoulderNow + octaveNow;
          slotTimerRun = true;
          slotOutatime = false;
          slotTimeOut = millis();
      }
      
      if (PS3usb.getButtonClick(SELECT)) {
          pattern[slot] = !pattern[slot];
          slotTimerRun = true;
          slotOutatime = false;
          slotTimeOut = millis();                    
      }
  }

  if (slotTimerRun == true) {
      if ((millis() - slotTimeOut) > 4000) {
          yellowLED = 0;
          slotOn = false;
          slotTimerRun = false;
          slotOutatime = true; 
      }
  }

  //Serial.println(yellowLED);

/*  
  for (int i = 0; i < patLength; i++) {
      Serial.print(sequence[i]);
      Serial.print("   ");
  }
  Serial.println(" ");
*/

  greenLED = 0;
  
/*  
  for (int i = 0; i < patLength; i++) {
      if (pattern[i]) {
          greenLED += (1 << i);
      }
  }
*/ 
  //Serial.println(greenLED);
  
  greenLED = (greenLED ^ redLED) | yellowLED;
  //PORTA = greenLED;

  
  
  
  //*********************** TAP TEMPO CLOCK **************************************************//
  

  if((millis() - lastTap) > 5000) {                               //reset the number of recent taps
      taps = 0;                                                   //and index if it has been more
      tapIndex = 0;                                               //than 5 seconds since the last tap
      lastTap = millis();                                      
  }

  if (PS3usb.getButtonClick(START)) {
      rawClockMult ++;
      rawClockMult = rawClockMult % 6;
      clockMultiplier = rawClockMult + 2;
      if (clockMultiplier == 5) {
          clockMultiplier = 6;
      } else if (clockMultiplier == 6) {
          clockMultiplier = 8;
      }
  }

  Serial.println(clockMultiplier);
    
  if(PS3usb.getButtonClick(CIRCLE)) {                                         
      timerArray[tapIndex] = millis() - lastTap;
      lastTap = millis(); 
      toggle = false;
      outatime = 0;
      clockAdjust = 0;
      tapIndex = tapIndex % 3;                      
      tapIndex += 1;
      taps += 1;                                        
  }
  
  if (taps >= 3) {
      
      biggest = max(max(timerArray[1], timerArray[2]), timerArray[3]);    
      
      smallest = min(min(timerArray[1], timerArray[2]), timerArray[3]);
      
      middle = max(min(timerArray[1], timerArray[2]), min(max(timerArray[1], timerArray[2]), timerArray[3]));
      
      if (middle - smallest > 100) {                                //remove outliers
          smallest = middle;
      }
      
      if (biggest - middle > 100) {
          biggest = middle; 
      }
      
      tapLength = (biggest + smallest + middle) / 3;
  }
  
  adjustment = (clockPeriod / 700) + 1;                             
  
  if(PS3usb.getButtonPress(TRIANGLE) && clockPeriod > 1) {            //speed up the clock generator
      if(PS3usb.getAnalogButton(TRIANGLE) > 200) {
        clockAdjust -= (adjustment * 10);
      } else {
          clockAdjust -= adjustment;
      }
  }   

  if(PS3usb.getButtonPress(CROSS) && clockPeriod < 20000) {          //slow down the clock generator
      if(PS3usb.getAnalogButton(CROSS) > 200) {
          clockAdjust += (adjustment * 10);
      } else {
          clockAdjust += adjustment;    
      }
  }

  //Serial.println(clockPeriod);

  clockPeriod = (tapLength + (clockAdjust / 4)) / clockMultiplier;  

  if( millis() >= outatime ) {
      toggle = !toggle;
      outatime = millis() + clockPeriod;
  }
  
/* 
  if (PS3usb.getButtonClick(START)) {                               //toggles the clock on and off       
      clockReady = !clockReady;
  }  

  if (clockReady && !toggle) {                                      //this ensures that the clock turns on and off while it is low       
      clockOn = true;                                               //so that there are not jarring transitions
  } else if (!clockReady && !toggle) {
      clockOn = false;
  }
*/
  
  if (clockOn) {              
      if (toggle) {
          if (risingEdge) {
              oneVoltPerOct = sequence[patIndex] + octave;
              if (slotOn == false) {
                  oneVoltPerOct += shoulderNow;
              } else {
                  oneVoltPerOct ++;
              }

              //Serial.println(oneVoltPerOct * 50);
              
              setOutput(oneVoltPerOct * 50);
              redLED = 0;
              if (pattern[patIndex]) {
                  stepOn = true;
                  redLED = 1 << patIndex;
              }
              redLED = redLED | yellowLED;
              PORTC = redLED;
              //oneVoltPerOct = octave + shoulderValue;
              //setOutput(oneVoltPerOct * 50);
              patIndex ++;
              patIndex = patIndex % patLength;
              risingEdge = false;
          }
          //digitalWrite(13, HIGH);
      } else {
          stepOn = false;
          risingEdge = true;
          digitalWrite(13, LOW);
      }
  } else {
      //digitalWrite(13, LOW);
  }

  //Serial.println(patIndex);

  if (stepOn) {
      digitalWrite(13, HIGH);
  } else {
      digitalWrite(13, LOW);
  }

 
  
  //*********************JOYSTICKS********************************************************//

  rightY = (PS3usb.getAnalogHat(RightHatY));          //read the joystick
  analogWrite(RIGHT_Y_OUT, rightY);                     //and write the value via PWM

  rightX = (PS3usb.getAnalogHat(RightHatX));
  analogWrite(RIGHT_X_OUT, rightX);

  leftY = (PS3usb.getAnalogHat(LeftHatY));
  analogWrite(LEFT_Y_OUT, leftY);
  
  leftX = (PS3usb.getAnalogHat(LeftHatX));
  analogWrite(LEFT_X_OUT, leftX);



  //********************* ACCELEROMETER X AND Y ******************************************//

  //Serial.print(" X: ");
  //Serial.print(PS3usb.getSensor(aX) - 380);

  //Serial.print("         Y: ");
  //Serial.println(PS3usb.getSensor(aY) - 380);

  //Serial.print("         Z: ");
  //Serial.println(PS3usb.getSensor(aZ) - 400);



//************************ D-PAD OCTAVE SWITCH *************************************************//


  
  if (slotOn == false) {
    if(PS3usb.getButtonClick(UP)) {            //increment the octave
        octave += 12;
    }
    
    if(PS3usb.getButtonClick(DOWN)) {          //decrement the octave
        octave -= 12;
    }
  }
  
  if (octave > 24) {                         //but don't go above +6 octaves 
      octave = 24;
  }

  if (octave < -36) {                          //or below +0 octaves
      octave = -36;
  }

  //Serial.println(octave);
  
  if (PS3usb.getButtonPress(UP)) {
      octaveNow = 48;
  } else if (PS3usb.getButtonPress(DOWN)) {
      octaveNow = 24;
  } else {
      octaveNow = 36;
  }
 
  
//*********************SHOULDER BUTTONS************************************************//

          
  if(PS3usb.getButtonPress(L1) || PS3usb.getButtonPress(R1) ||        //if any shoulder buttons are pressed
    PS3usb.getButtonPress(L2) || PS3usb.getButtonPress(R2)) {
      shoulderPressed = true;
      digitalWrite(SHOULDER_GATE, HIGH);                               //send the shoulder gate high  
      thisShoulder = millis();

      shoulderNow = 0;  
      
      if(PS3usb.getButtonPress(L1)) {                                 //and update the new value by adding each bit to shoulderValue
          shoulderNow += 1;                                           //L1 is the LSB, R1 is bit 1, L2 is bit 2, R2 is the MSB 
      }

      if(PS3usb.getButtonPress(R1)) {
          shoulderNow += 2;
      }

      if(PS3usb.getButtonPress(L2)) {
          shoulderNow += 4;
      }

      if(PS3usb.getButtonPress(R2)) {
          shoulderNow += 8;
      }

      
      if (thisShoulder - lastShoulder >= interval) {                  //only update shoulderValue if some time has passed, to allow for sloppy playing
       
          lastShoulder = thisShoulder;                                //if interval is too big shoulder buttons will feel sluggish, if interval is too small you will get errors
      
          shoulderValue = shoulderNow;
      }
  } else {
        shoulderNow = 1;
        shoulderPressed = false;
        digitalWrite(SHOULDER_GATE, LOW);
  }


  //Serial.println(shoulderValue);

  //oneVoltPerOct = octave + shoulderValue;

  //setOutput(oneVoltPerOct * 50);
  

   
   
   //code to send this information off into the world so it can be used goes here.
   //desired result -> one can "play" the shoulder buttons like a keyboard, using
   //the binary sequence.  

  
    

    
//**********************************  //RIGHT BUTTON PAD GATES// *********************************************//


  
  //            SQUARE BUTTON: GATE SIGNAL 1, GOES HIGH AS LONG AS SQUARE IS PRESSED
  if(PS3usb.getButtonPress(SQUARE)) {
      digitalWrite(SQUARE_GATE, HIGH);
  } else {
      digitalWrite(SQUARE_GATE, LOW);
  }

  
  //             X BUTTON: GATE SIGNAL 2, GOES HIGH AS LONG AS X IS PRESSED
  if(PS3usb.getButtonClick(CROSS)) {  
      digitalWrite(CROSS_GATE, HIGH);
  } else {
      digitalWrite(CROSS_GATE, LOW);
  }
  
    
  //            TRIANGLE BUTTON: GATE SIGNAL 3, GOES HIGH AS LONG AS TRIANGLE IS PRESSED
  if(PS3usb.getButtonPress(TRIANGLE)) {
      digitalWrite(TRI_GATE, HIGH);
  } else {
      digitalWrite(TRI_GATE, LOW);
  }



  }   //bracket for unplugged controller routine, (PS3Connected)
}     //end bracket for main loop





 

void BTonInit(){ //runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" BT CONNECTED ");
}

void SerialBTonInit(){ //runs once at moment of controller initialization
  //LedState = 0; 
}

void USBonInit(){ //runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" USB CONNECTED ");
}

void setOutput(unsigned int val)
{
  digitalWrite(PIN_CS, LOW);
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
   
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS, HIGH);
}

