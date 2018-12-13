/* TO DO ---
 *  
 *  -check sequencer editing, seemed like it wrote to wrong step once
 *  
 *  -ADD UNMULTIPLIED CLOCK OUT
 *  
 *  -ADD CLOCK SYNC IN
 *  
 *  -RESET IS WHACKY, SHORTER PATTERN LENGTHS MAKE IT RESET TO STEPS OTHER THAN 1
 *  
 *  -IMPROVE CLOCK ADJUSTMENT RESOLUTION AT FAST SPEEDS
 *  
 *  -CLOCK SPEED UP/SLOW DOWN BUTTONS FEEL ODD WHEN CLOCK PERIOD IS LONG, AS-IS CLOCK PERIOD MUST COMPLETE BEFORE NEW SHORTER CLOCK PERIOD UPDATES
 *  
 *  -IMPROVE CLOCK MULTIPLIER?
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
 *           a one volt per octave sequencer signal, a gate pattern                *
 *           signal, and a tap tempo clock signal.                                 *
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
 *           The CIRCLE button is used to tap in a tempo to the                    *
 *           clock signal.  The TRIANGLE and CROSS buttons slow down               *
 *           and speed up the clock.  The START button scrolls                     *
 *           through the multiplier that is applied to the clock                   *
 *           frequency.  1x, 2x, 3x, 4x, 6x and 8x are available.                  *
 *                                                                                 *
 *           To program the sequencer, use the RIGHT and LEFT buttons              *                                                        
 *           to select the step to edit, this will cause the selected              *
 *           step to blink and be brighter than normal. While the step             *
 *           is blinking, hold down a combination of shoulder buttons              *
 *           and UP/DOWN buttons, then press SQUARE to enter the value.            *
 *           The editable step will time out after 4 sseconds. The LED             *
 *           display gives information about the sequence, cooler colors           *
 *           are lower note values, warmer colors are higher note values.          *
 *                                                                                 *
 *           You can also program the gate pattern in a  similar way.              *
 *           While the editable step is blinking, press SELECT to turn             *
 *           the gate for thar step on or off. Steps with active gates             *
 *           will be brighter than steps with innactive gates.                     *
 *                                                                                 *
 *           While the sequencer is playing, you can use the shoulder             *
 *           buttons and the UP/DOWN buttons to change the root note and           *
 *           octave. Shoulder values will revert to the root value when you        *
 *           release the shoulder buttons, octave values will persist.             *
 *                                                                                 *                                                                                 
 *           To change the length of the pattern, press L3 or R3. L3               *
 *           removes steps, R3 adds steps. The minimum number of steps             *
 *           is 2. Steps outside of the active pattern will be dark,               *
 *           but will light up and blink if you are editing that step.             *
 *                                                                                 *
 *           If you are not editing a step, you can reset the sequences            *
 *           to step 1 by pressing the SQUARE button.                              *
 *                                                                                 *
 *                                                                                 *
 *                                                                                 *
 ***********************************************************************************/

 
byte nonsense_var = 0;        // this line maybe required for IDE compatibility

#include <EEPROM.h>          


//------------------NEOPIXEL FAST LED INITIALIZATION ----------------
#include <FastLED.h>
#define LED_PIN   A1
#define NUM_LEDS  16
CRGB leds[NUM_LEDS];


//------------------ PS3 CONTROLLER INITIALIZATION BELOW --------------
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
USBHub Hub1(&Usb);                                          // Some dongles have a hub inside
BTD Btd(&Usb);                                              // You have to create the Bluetooth Dongle instance like so


SPP SerialBT(&Btd);                                         // This will set the name to the defaults: "Arduino" and the pin to "0000"

//PS3BT PS3(&Btd);                                          // This will just create the instance
PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0xE7, 0x96, 0xA6);        // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
//PS3USB PS3usb(&Usb);                                      // This will just create the instance
PS3USB PS3usb(&Usb,0x00,0x15,0x83,0xE7,0x96,0xA6);          // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printTemperature, printAngle;



//------------------PS3 CONTROLLER INITIALIZATION ABOVE--------------


//            VARIABLES AND CONSTANTS

byte              plugged             =     0;                // is PS3 controller plugged in?


//        NEOPIXEL LEDS USING FASTLED LIBRARY

unsigned long     slotLedTimeout      =     0;                // used to make the editable step blink,
boolean           slotLedBlink        =     false;            // so you can tell where it is

int               brightAdjust        =     1;                // makes the active step brighter so you can tell where it is

int               seqLEDscaled[16]    =     {};               // array of values for LEDs, colors proportional to 1V/oct value
int               seqInit             =     215;              // initial value for LEDs, to make a pretty rainbow at startup


//        SEQUENCER AND PATTERN GENERATOR 

unsigned int      rawPatLength        =     14;               // because modulo and don't want zero length, maybe there is a smarter way to do it?
unsigned int      patLength           =     16;               // how long is the pattern? 2 to 16

unsigned int      slot                =     0;                // position of pattern generator for editing, 0 to 15. LED at this step will blink while editable
boolean           slotOn              =     false;            // while true the slot selector is active, and you can turn individual steps on and off and edit their 1V/oct value
boolean           slotTimerRun        =     false;            // while true the slot timeout timer is running
boolean           slotOutatime        =     true;             // true when the slot timout timer runs out
unsigned long     slotTimeOut         =     0;                // determines how long you have to edit a step before timing out

boolean           pattern[16]         =     {};               // pattern array, true means active step, edited by selecting position with LEFT/RIGHT, writing with SELECT
unsigned int      patIndex            =     0;                // where in the pattern are we? LED at this step will be brighter
boolean           stepOn              =     false;            // gate output is high when true
boolean           reset               =     false;            // if true, reset the sequencer to step 1 next clock cycle

int               sequence[16]        =     {};               // array of sequencer values, edited by selecting position with LEFT/RIGHT,                                                               //programming note with shoulder buttons and UP/DOWN, and writting with SQUARE 


//        TAP TEMPO CLOCK

unsigned int      rawClockMult        =     0;                 // because modulo and don't want zero for multiplier, maybe there is a smarter way to do it?
unsigned int      clockMultiplier     =     8;                 // multiplier for clock speed, 0.5x, 1x, 1.5x, 2x, 3x or 4x.  Actually accomplishes this by dividing the clock period. CURRENTLY INCORRECT.

unsigned int      taps                =     0;                 // how many recent taps have there been?
unsigned long     lastTap             =     0;                 // when the last tap happened 
unsigned int      tapLength           =     500;               // average of time between recent taps
unsigned long     timerArray[4]       =     {};                // array of most recent tap counts
unsigned int      tapIndex            =     0;                 // where in the array of tap counts are we?

unsigned int      biggest             =     500;               // the array is split into big,
unsigned int      smallest            =     500;               // medium and small, this is to
unsigned int      middle              =     500;               // eliminate outliers and avoid jarring transitions CURRENTLY UNSURE HOW EFFECTIVE THIS IS

long              clockAdjust         =     0;                 // speed up/slow down clock with right and left D-pad
unsigned int      adjustment          =     0;                 // adjustment value is a percentage of the clock period

unsigned int      clockPeriod         =     500;               // clock period in milliseconds
unsigned long     outatime            =     0;                 // this is when the timer will trigger next

boolean           clockReady          =     true;              // toggled by start button, used to make sure clock starts at the beginning of a cycle CURRENTLY WRONG, DOES NOTHING
boolean           clockOn             =     true;              // when clockOn is true the clock signal appears at pin 13
boolean           toggle              =     true;              // toggles true/false/true/false at clock rate
boolean           risingEdge          =     true;              // true right as the clock goes high


//        JOYSTICK

int               rightY              =     127;               // value of each joystick axis, 0 to 255
int               rightX              =     127; 
int               leftY               =     127;
int               leftX               =     127;

const int         RIGHT_Y_OUT         =     2;                 // pin to output scaled joystick value
const int         RIGHT_X_OUT         =     3;                 // via PWM DAC
const int         LEFT_Y_OUT          =     4;
const int         LEFT_X_OUT          =     5;


//        ACCELEROMETER

int               accelX              =     127;                // value of accelerometer X and Y values, 0 to 255
int               accelY              =     127;

const int         ACCEL_X_OUT         =     11;                 // pin to output scaled X and Y value
const int         ACCEL_Y_OUT         =     12;                 // via PWM DAC


//        1V/OCTAVE

byte              shoulderNow         =     1;                  // vslue of shoulder buttons while pressed, 1 to 15, for programming and transposing the sequencer

int               octave              =     0;                  // up and down arrows select octave shift, for transposing the sequencer
int               octaveNow           =     0;                  // value of octave switch while pressed, is either +2, +3, or +4, for programming the sequencer

unsigned int      oneVoltPerOct       =     0;                  // main 1V/octave output, sequence plus transposition, uses external MCP4822 DAC via SPI

boolean           shoulderPressed     =     false;              // true if any shoulder button is pressed
const byte        SHOULDER_GATE       =     A4;                 // pin for shoulder gate signal, goes high whenever a shoulder button is pressed

const int         PIN_CS              =     53;                 // these have to do 
const int         GAIN_1              =     0x1;                // with the SPI ->
const int         GAIN_2              =     0x0;                // MCP4822 DAC



//        BATTERY STATE LEDs

byte              LedState            =     0;


void setup(){
 

//---------------------- MCP4822 DAC SPI ---------------

  pinMode(PIN_CS, OUTPUT);                // setup SPI communication via pins 51, 52 & 53
  digitalWrite(PIN_CS, HIGH);             // 51 is MOSI, -> MCP4822 pin 4, white/green
  SPI.begin();                            // 52 is SCK, -> MCP4822 pin 3, white/red
  SPI.setClockDivider(SPI_CLOCK_DIV2);    // 53 is SS, -> MCP4822 pin 2, white/orange
  
  
  
//---------------------- OUTPUTS/INPUTS -----------------
                                          //                         wire color
  pinMode(A1, OUTPUT);                    // serial data to LEDs     blue
  pinMode(A4, OUTPUT);                    // SHOULDER_GATE
  
  pinMode(2, OUTPUT);                     // RIGHT_Y_OUT             orange
  pinMode(3, OUTPUT);                     // RIGHT_X_OUT             pink
  pinMode(4, OUTPUT);                     // LEFT_Y_OUT              white
  pinMode(5, OUTPUT);                     // LEFT_X_OUT              brown
   
  //Pin9 = used for USB shield
  //Pin10 = used for USB shield
  
  pinMode(11, OUTPUT);                    // ACCEL_X_OUT             grey
  pinMode(12, OUTPUT);                    // ACCEL_Y_OUT             purple
  
  pinMode(13, OUTPUT);                    // blinky for testing      red/green
    
  
  
//-------------------- PATTERN GENERATOR ----------------
  
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS); 
  
  for (int i = 0; i < patLength; i ++) {                 

    pattern[i] = true;                          // set pattern to all gates on
    
    sequence[i] = 37;                           // set sequence to all notes middle C
    
    seqLEDscaled[i] = seqInit;                  // make a beautiful rainbow
    leds[i] = CHSV(seqLEDscaled[i], 255, 50);
    delay(100);
    seqInit -= 14;
    FastLED.show();                             // show the besutiful rainbow  
  }
    
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
}



void loop() {
  Usb.Task(); //PS3 get info command
  PS3.attachOnInit(BTonInit);
  PS3usb.attachOnInit(USBonInit);
  SerialBT.attachOnInit(SerialBTonInit);


//---------------UNPLUGGED CONTROLLER ROUTINE-------------
//------------(includes bracket at end of main loop)------
  if (PS3.PS3Connected == 0 && PS3usb.PS3Connected == 0) {          // skip loop if no controller found
    Serial.println(" controller unplugged ");

  } else if (PS3usb.PS3Connected == 0) { 

    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3.disconnect();
    }
 
  } else if (PS3.PS3Connected == 0) { // USB COMMANDS BELOW-----------------------------------------------------------------------------------------------------------------

    if (PS3usb.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3usb.Release();
    }
  
//*********************** PATTERN GENERATOR AND SEQUENCER ***********************************************//

  
    if (PS3usb.getButtonClick(R3)) {
      rawPatLength ++;
    }
    
    if (PS3usb.getButtonClick(L3)) {
      rawPatLength --;
    } 
    patLength = (rawPatLength % 15) + 2;
  
  
    if (PS3usb.getButtonClick(RIGHT)) {                          // move the editable slot position right
      if (slotOutatime == true) {
         slot --;
      }
      slot ++;
      slot = slot % 16;
      slotOn = true;
      slotTimerRun = true;
      slotOutatime = false;
      slotTimeOut = millis();
    }
    
    if (PS3usb.getButtonClick(LEFT)) {                           // or left
      if (slotOutatime == true) {
        slot ++;
      }
      slot --;
      slot = slot % 16;
      slotOn = true;
      slotTimerRun = true;
      slotOutatime = false;
      slotTimeOut = millis();
    }
  
    
    if (slotOn == true) {                                        // if you have not timed out
      if (PS3usb.getButtonClick(SQUARE)) {                       // you can edit the sequence
        sequence[slot] = shoulderNow + octaveNow;
        seqLEDscaled[slot] = ((sequence[slot] * -6) + 378); 
        slotTimerRun = true;
        slotOutatime = false;
        slotTimeOut = millis();
      }
        
      if (PS3usb.getButtonClick(SELECT)) {                       // and the gate pattern
        pattern[slot] = !pattern[slot];
        slotTimerRun = true;
        slotOutatime = false;
        slotTimeOut = millis();                    
      }
    }
  
    
  
    if (slotTimerRun == true) {                                  // this is where the slot timer times out
      if ((millis() - slotTimeOut) > 4000) {
        slotOn = false;
        slotTimerRun = false;
        slotOutatime = true; 
      }
    }
  
  
    for (int i = 0; i < 16; i++) {                                            // this is where we program the LEDs                   
      if (i == patIndex) {                                                      
        brightAdjust = 2;                                                     // brightness adjustment makes the 
      } else {                                                                // active step brighter so you can
        brightAdjust = 1;                                                     // see where it is
      }
      
      if (pattern[i]) {                                                       // here we show the gate pattern
        leds[i] = CHSV(seqLEDscaled[i], 255, 50 * brightAdjust);              // bright LEDs mean gate on
      } else {                                                                // dim LEDs mean gate off
        leds[i] = CHSV(seqLEDscaled[i], 255, 10 * brightAdjust);              // black LEDs are outside the current pattern 
      }
  
      if (i >= patLength) {
        leds[i] = CRGB::Black;
      }
      
      if (slotOn) {                                                           // here we show the editable step
        if ((millis() - slotLedTimeout) > 200) {                              // with a blinky LED
        slotLedBlink = !slotLedBlink;                                         // this shows up even outside the current pattern
        slotLedTimeout = millis();
        }
        if (slotLedBlink) {
            leds[slot] = CHSV(seqLEDscaled[slot], 255, 125); 
        } else {
            leds[slot] = CRGB::Black; 
        }
      }      
    }
    
    FastLED.show();                                                           // here is where we actually show all the LEDs
  
    
    
    
//*********************** TAP TEMPO CLOCK **************************************************//
    
  
    if ((millis() - lastTap) > 2000) {                            // reset the number of recent taps
      taps = 0;                                                   // and index if it has been more
      tapIndex = 0;                                               // than 2 seconds since the last tap
      lastTap = millis();                                      
    }
  
    if (PS3usb.getButtonClick(START)) {                           // here we set how many times the
      rawClockMult ++;                                            // pattern increments per tap
      rawClockMult = rawClockMult % 6;
      clockMultiplier = rawClockMult + 2;
      if (clockMultiplier == 5) {
        clockMultiplier = 6;
      } else if (clockMultiplier == 6) {
        clockMultiplier = 8;
      }
    }
  
      
    if (PS3usb.getButtonClick(CIRCLE)) {                          // tap!                         
      timerArray[tapIndex] = millis() - lastTap;
      lastTap = millis(); 
      toggle = false;
      outatime = 0;
      clockAdjust = 0;
      tapIndex = tapIndex % 3;                      
      tapIndex += 1;                                              // is this on purpose? index never is zero
      taps += 1;                                        
    }
    
    if (taps >= 3) {
        
      biggest = max(max(timerArray[1], timerArray[2]), timerArray[3]);    
      
      smallest = min(min(timerArray[1], timerArray[2]), timerArray[3]);
      
      middle = max(min(timerArray[1], timerArray[2]), min(max(timerArray[1], timerArray[2]), timerArray[3]));
      
      if (middle - smallest > 100) {                              // remove outliers, unsure if this works well or is usefull
        smallest = middle;
      }
      
      if (biggest - middle > 100) {
        biggest = middle; 
      }
      
      tapLength = (biggest + smallest + middle) / 3;
    }
    
    adjustment = (clockPeriod / 700) + 1;                             
    
    if (PS3usb.getButtonPress(TRIANGLE) && clockPeriod > 1) {          // speed up the clock generator
      if (PS3usb.getAnalogButton(TRIANGLE) > 200) {                    // if you push hard, speed up more
        clockAdjust -= (adjustment * 10);
      } else {
        clockAdjust -= adjustment;
      }
    }   
  
    if (PS3usb.getButtonPress(CROSS) && clockPeriod < 20000) {         // slow down the clock generator
      if (PS3usb.getAnalogButton(CROSS) > 200) {                       // if you push hard, slow down more
        clockAdjust += (adjustment * 10);
      } else {
        clockAdjust += adjustment;    
      }
    }
  
  
    clockPeriod = (tapLength + (clockAdjust / 4)) / clockMultiplier;   // the length of a clock period is made up of tap times, adjustment and multiplier
  
    if ( millis() >= outatime ) {                                      // here we flop back and forth, making
      toggle = !toggle;                                                // a 50% square wave at 2x clock period
      outatime = millis() + clockPeriod;
    }
    
    if (slotOn == false) {                                             // if you are not editing a step, you can
      if (PS3usb.getButtonClick(SQUARE)) {                             // reset the sequencer by pressing SQUARE
          reset = true;
      }
    }
  
    if (clockOn) {              
      if (toggle) {
        if (risingEdge) {                                              // at the rising edge of a clock cycle
          oneVoltPerOct = sequence[patIndex] + octave;                 // set the sequence to the current step value
          if (slotOn == false) {                                       // if you are not editing, add the value
            oneVoltPerOct += shoulderNow;                              // of the shoulder buttons for transposing
          } else {
            oneVoltPerOct ++;                                          // if editing, add 1 to keep the root the same
          }
                        
          setOutput(4096 - (oneVoltPerOct * 45));                      // send the scaled upside down sequence to the DAC
                                                                       // an inverting opamp scales it further
          if (pattern[patIndex]) {      
            stepOn = true;                                    
          }
          
          if (reset == true) {                                        
            patIndex = 15;                                             // reset if appropriate
            reset = false;
          } else { 
            patIndex ++;                                               // or else increment the step
            patIndex = patIndex % patLength;
            risingEdge = false;
          }
        }       
      } else {                                                        
        stepOn = false;
        risingEdge = true;
      }
    }
  
    if (stepOn) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
  
   
    
//*********************JOYSTICKS********************************************************//
  
    rightY = (PS3usb.getAnalogHat(RightHatY));           // read the joystick 
    rightY = 255 - rightY;                               // and write the value via PWM
    analogWrite(RIGHT_Y_OUT, rightY);                    
                                                            
    rightX = (PS3usb.getAnalogHat(RightHatX));           // Y values are "upside down",
    analogWrite(RIGHT_X_OUT, rightX);                    // math flips them "right side up"
                                                         
    leftY = (PS3usb.getAnalogHat(LeftHatY));
    leftY = 255 - leftY;
    analogWrite(LEFT_Y_OUT, leftY);
    
    leftX = (PS3usb.getAnalogHat(LeftHatX));
    analogWrite(LEFT_X_OUT, leftX);
  
  
  
//********************* ACCELEROMETER X AND Y ******************************************//
  
    accelX = (PS3usb.getSensor(aX));                     // both X and Y are "upside down" and centered oddly
    accelX -= 380;                                       // math flips them "right side up" and centers them
    accelX = 255 - (constrain(accelX, 0, 255));
    analogWrite(ACCEL_X_OUT, accelX);                    // as is X and Y clip when moved 
                                                         // quickly, might be fine or might                                                        
    accelY = (PS3usb.getSensor(aY));                     // want to change scaling
    accelY -= 380; 
    accelY = 255 - (constrain(accelY, 0, 255));
    analogWrite(ACCEL_Y_OUT, accelY);
  
    
  
  
//************************ D-PAD OCTAVE SWITCH *************************************************//
  
  
    
    if (slotOn == false) {                       // octave is used while the sequence is playing, only works when slot is inactive
      if(PS3usb.getButtonClick(UP)) {            // increment the octave
        octave += 12;
      }
      
      if(PS3usb.getButtonClick(DOWN)) {          // decrement the octave
        octave -= 12;
      }
    }
    
    if (octave > 24) {                           // but don't go above +5 octaves 
      octave = 24;
    }
  
    if (octave < -24) {                          // or below +0 octaves
      octave = -24;
    }
  
    
    if (PS3usb.getButtonPress(UP)) {             // octaveNow is used while editing the sequence, only works when slot is active
      octaveNow = 48;
    } else if (PS3usb.getButtonPress(DOWN)) {
      octaveNow = 24;
    } else {
      octaveNow = 36;
    }
   
    
//*********************SHOULDER BUTTONS************************************************//
  
            
    if (PS3usb.getButtonPress(L1) || PS3usb.getButtonPress(R1) ||     // if any shoulder buttons are pressed
      PS3usb.getButtonPress(L2) || PS3usb.getButtonPress(R2)) {
        shoulderPressed = true;
        digitalWrite(SHOULDER_GATE, HIGH);                            // send the shoulder gate high  
        shoulderNow = 0;  
        
        if(PS3usb.getButtonPress(L1)) {                               // and update the new value by adding each bit to shoulder value
          shoulderNow += 1;                                           // L1 is the LSB, R1 is bit 1, L2 is bit 2, R2 is the MSB 
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
    } else {
      shoulderNow = 1;
      shoulderPressed = false;
      digitalWrite(SHOULDER_GATE, LOW);
    }

  }   // bracket for unplugged controller routine, (PS3Connected)

}     // end bracket for main loop




//************************ FUNCTIONS *************************************//
 

void BTonInit() {                                       // runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" BT CONNECTED ");
}

void SerialBTonInit() {                                 //runs once at moment of controller initialization
  //LedState = 0; 
}

void USBonInit() {                                      // runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" USB CONNECTED ");
}

void setOutput(unsigned int val) {                      // this sets the LEDs to whatever colors and patter has been programmed

  digitalWrite(PIN_CS, LOW);
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;
   
  PORTB &= 0xfb;
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  PORTB |= 0x4;
  digitalWrite(PIN_CS, HIGH);
}

void batteryState() {                                   // shows the battery state of the PS3 controller with the built in LEDS   
  if (PS3.getStatus(Shutdown)) {
    if (LedState != 1) {
      PS3.setAllOff();
      PS3.setLedToggle(LED1);
      LedState=1;
    }
  } else if (PS3.getStatus(Dying)) {
    if (LedState != 2) {
      PS3.setAllOff();
      PS3.setLedToggle(LED2);
      LedState=2;
    }
  } else if (PS3.getStatus(Low)) {
    if (LedState != 3) {
      PS3.setAllOff();
      PS3.setLedToggle(LED3);
      LedState=3;
    }
  } else if (PS3.getStatus(High) || PS3.getStatus(Full)) {
    if (LedState != 4) {
      PS3.setAllOff();
      PS3.setLedToggle(LED4);
      LedState=4;
    }
  }
}
