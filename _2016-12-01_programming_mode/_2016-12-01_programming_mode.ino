
/*   
 *  STUFF TO FIX:
 *  
 *  
 *  SAVED SPEED HAS NO EFFECT WHILE IN PROPORTIONAL MODE, SHOULD IT?  
 *    
 *  USB CODE NOT UP TO DATE?
*/



/* EEPROM STORAGE
 * ADDRESSES AND PURPOSE:
 * 0,1                revision number
 * 2,3,4,5            Z-lift Down Time (millis)
 * 6,7,8,9            Z-lift Pause Time (millis)
 * 10,11,12,13        Z-Lift Up Time (millis)
 * 14,15,16,17        Cruise Interval Time (millis)
 * 18,19,20,21        Total Vacuum On Time (seconds)
 * 
 * 22                 inHg            boolean
 * 23                 kPa,            boolean
 * 24, 25, 26, 27     tripPointOne    int    
 * 28, 29, 30, 31     tripPointTwo    int
 * 32                 variVacuum      boolean
 * 33, 34, 35, 36     torqueTrim      int
 * 
 * 37, 38             saved speed
 */

#define REVISION 0x0001         // changing this will reinitialize the EEPROM with values from variable initialization below
                                // note that vacuum on time will be reset as well (Blake, do you wish this was different?)

byte nonsense_var = 0;          //this line maybe required for IDE compatibility
 
#include <EEPROM.h>             // EEPROM Storage

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
//Sena UD100 dongle requires the hub line below
USBHub Hub1(&Usb); // Some dongles have a hub inside 
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */

SPP SerialBT(&Btd); // This will set the name to the defaults: "Arduino" and the pin to "0000"


PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0xE7, 0x96, 0xA6); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
PS3USB PS3usb(&Usb); // This will just create the instance
//PS3USB PS3usb(&Usb,0x00,0x15,0x83,0xE7,0x96,0xA6); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

//bool printTemperature, printAngle;

//------------------PS3 CONTROLLER INITIALIZATION ABOVE--------------


//            VARIABLES

word      pot_final       =       0; 

int       error           =       0;
int       val             =       0;       // Variable to hold analog reading from speedPot 

const int speedPot        =       A1;      // Potentiometer on analog pin 1
int       cruise          =       0;
int       cruiseDirR      =       0;
int       cruiseDirL      =       0;
int       cruiseLast      =       0;

int       leftadjust      =       0;
int       rightadjust     =       0;
int       RD;
int       LD;
int       LRradjust       =       0;
int       LRladjust       =       0;
int       Selectpressed   =       0;

//        FP TORQUE POT INPUT
const int torquePot       =       A2;     // Potentiometer on analog pin 2
int       torqueLast      =       0;      // switch for LCD display, so that you don't need to write to it each loop
unsigned int       FPtorque        =       0;      // FP torque value to send via PWM
int       displayTorque   =       0;      // FP torque value to write to LCD display (normalized 0-99)
int       smoothTorque    =       0;      // averaged display value
String    LCDtorqueString =       "";     // we turn the torque value into a string so that we can write it to the LCD display
int       FPunlocked      =       0;      // FP is locked when 0, and open when 1

//        DRIVING VARIABLES 
String    motorL          =       "M1: ";
String    motorR          =       "M2: ";
int       drivemode       =       -1;     // indicates forward movement, used with select
short     drivespeedr     =       0;      // stores speed of right side
short     drivespeedl     =       0;      // stores speed of left side
 
word      Scount          =       0;
word      saved           =       512;
word      potread;
int       LCDspeed        =       0;      // drive speed for LCD (normalized 0-99)
int       lastLCDspeed    =       0;      // so that you don't need to write speed to LCD every loop, only when it has changed
int       smoothLCDspeed  =       0;      // averaged LCD speed
String    LCDspeedString  =       "";     // we turn the speed value into a string so that we can write it to the LCD display

boolean   Spressed        =       0;
boolean   s_switched;

//        PROPORTIONAL JOYSTICK VARIABLES
boolean   proportionalSpeed =    false;

long      LjoystickSpeed  =         0;
long      RjoystickSpeed  =         0;

//        LIMITS
byte      lowLimit        =         80;
byte      highLimit       =         200;

//        VARIABLES FOR VACUUM READING
long      lastA1          =         0;       // when did the last A1 request happen?
int       vacSense        =         2000;    // this is the number that "A1: get" returns, 2000 is about 0 inHg, 10 is about 8 inHg. it's "upside down"
long      lastValidA1     =         -501;    // when did the last valid A1 reading happen? Initialized such that we don't write to either display until a valid A1 is recieved
int       vacSenseMath    =         0;       // was int, changed to print decimal point to LCD, doesn't seem to be used anywhere
String    inputString     =         "";      // a string to hold incoming data
boolean   stringComplete  =         false;   // whether the string is complete

int       sensorArray[8]  =         {};      // array of recent vacuum sensor readings, to be averaged together
int       sensorIndex     =         0;       // where are we in the array?
int       sensorAverage   =         2000;    // you know what this is, value of 2000 = 0 inHg, 0 kPa
int       lastSensor      =         0;       // so we can check to see if the sensor value has changed

boolean   inHg            =         true;    // if true we are working in inches of mercury, if false we are working in kilopascals
boolean   kPa             =         false;   // if true we are woring in kilopascals, if false we are working in inches of mercury

String    sensorString    =         "";      // we turn the sensor value into a string so that we can write it to the LCD display, used for small characters only, not big digits

int       sensorTens      =         0;       // value representing the tens part of the sensor value, in inHg. to make writing big numbers to the LCD easier. only used for kPa
int       lastTens        =         0;       // we keep track of the last value so that we don't need to write to the LCD every time through the loop, only when it changes

int       sensorOnes      =         0;       // value representing the ones part of the sensor value, in inHg. to make writing big numbers to the LCD easier
int       lastOnes        =         0;       // we keep track of the last value so that we don't need to write to the LCD every time through the loop, only when it changes

int       sensorTenths    =         0;       // value representing the tenths part of the sensor value, in inHg. to make writing big numbers to the LCD easier
int       lastTenths      =         0;       // we keep track of the last value so that we don't need to write to the LCD every time through the loop, only when it changes

int       bigDigitPlace   =         1;       // which place are we going to write a big digit to? starts at 1, increments differently depending on inHg or kPa 


int       vacCheck        =         0;       // represents inHg, but with no decimal (1.8 inHg = 18), to check state of vacuum level for FP locking purposes, and to to aid in writing to the LCD 
int       inHgCheck       =         0;       // represents inHg, but with no decimal (5.4 inHg = 54), to aid in writing to the LCD 
int       kPaCheck        =         0;       // represents kPa, but with no decimal (15.3 kPa = 153), to aid in writing to the LCD 
int       tripPointOne    =         25;      // first trip point, determines when warning light comes on. In inHg, but without decimal, so we don't need so many floats 
int       tripPointTwo    =         15;      // second trip point, determines when Fall Protector locks. In inHg, but without decimal, so we don't need so many floats 

int       vacPWM          =         0;       // value to be written to remote display, not needed with new LCD only approach, might be handy for calibration though

boolean   vacWarn         =         true;    // true while inHg < trip point one, warning comes on here
boolean   vacWarnLast     =         true;    // this represents what vacWarn was the loop before this one
boolean   vacWarnChange   =         true;    // this keeps track of when vacWarn changes, so we don't need to do anything about it every loop

boolean   vacLow          =         true;    // true while inHg < trip point two, Fall Protector locks up while true (as long as you are not manualy unlocking it)
boolean   vacLowLast      =         true;    // this represents what vacLow was the loop before this one
boolean   vacLowChange    =         true;    // this keeps track of when vacLow changes, so we don't need to do anything about it every loop

boolean   vibrate         =         false;   // controller is vibrating while true, to alarm user of falling vacuum
long      vibrateTime     =         0;       // when did vibrating start? used to time length of vibrating


//        VACUUM CONTROL VARIABLES
boolean   variVacuum      =         true;    // true for variable vacuum, false for standard vacuum
boolean   variVacuumTemp  =         true;    // temporary version, for editing while in programming mode
int       vacPot          =         A0;      // Potentiometer on analog pin 2
boolean   vacSwitchOn     =         0;       // 1/true = vac switch is turned on 
int       vacScaled       =         -2047;   // -2047 is hard OFF, 2047 is hard ON


//        LOOP COUNTING VARIABLES
long      lastMillis      =         0;
long      loops           =         0;
long      lastloops       =         0;
int       flipFlop        =         0;

//        BATTERY STATE LEDs
byte      LedState        =         0;

//        RESET
boolean   connectOnce     =         0;
//uint8_t vbus;

//        TOOLS/ACCESSORIES
boolean   toolOn          =         false;    // true while the tool is on, clicking the CIRCLE button toggles it ON/OFF
boolean   toolWrite       =         true;     // true if you are going to write the tool status to the LCD this loop

boolean   zLiftChange     =         false;    // this changes when you start or stop pressing UP/DOWN, helps to only write to the LCD when its state changes
//int       zLiftOn         =         0;        // this doesn't seem to be used anywhere?        
boolean   zLiftWrite      =         true;     // true if you are going to write the z-lift status to the LCD this loop


//        Z-LIFT AUTOMATION
boolean   sensorCycle     =         false;    // true while the sensor is active
long      zTime           =         0;        // to keep track of time for sensor automation, plunge/wait/retract
int       zCycle          =         1;        // to clear program or not
int       beginProgram    =         0;
long      upTime          =         2000;
long      pauseTime       =         2000;
long      downTime        =         2000;
int       pausedAlready   =         0;
int       downBegin       =         0;
int       pausedBegin     =         0;
int       upBegin         =         0;
boolean   cPressed        =         false;

//        DRIVE AUTOMATION
int       beginDriveTime  =         0;
int       driveProgramStart =       0;
long      driveTime       =         0;
boolean   triPressed      =         false;
boolean   xPressed        =         false;
int       beginCruiseTime =         0;
long      currentDriveStart =       0;
//long      currentDriveTime  =       0;
boolean   driveInterval   =         false;

//        DEMO LOOP 
long     demoTime         =         0;
long     demo2Time        =         0;
int      demoOn           =         0;
int      demo2On          =         0;
int      firstTurn        =         0;

//        RECORD KEEPING
long      machineVacTime  =         0;
long      totalVacTime    =         0;
int       vacStart        =         0;
long      machineRunTime  =         0;
long      totalRunTime    =         0;
int       runTimeStart    =         0;

float     timeFloat       =         0;

boolean   LCDwired        =         false;    // is the LCD wired to the control station?
boolean   wiredFirstTime  =         true;     // does it need to be initialized?

int       LCDcount        =         0;
long      LCDtime         =         0;

//      PROGRAMMING MODE VARIABLES

long      programTime     =         0;        // used to enter and exit programming mode by pressing and holding START for 5 seconds 
boolean   progTimerStart  =         true;     // if true, the next time you press START the timer to enter programming mode will start
boolean   progEnterExit   =         true;     // if true, you can enter and exit programming mode

boolean   programMode     =         false;     // while true, you will be in programming mode
boolean   firstProg       =         true;     // true when you first enter programming mode, to shut off motors and setup the LCD

int       menuLength      =         9;        // how many items are there in the menu?

String    menuItems[9]    =          {"UNIT:", "WARN:", "LOCK:", 
                                      "VAC : ", "TORQ: ", "TIME: ",
                                      "P3.1: ", "P3.2: ", "P3.3: "};   // array of names of parameters that can be edited while in programming mode


String    tempEntries[9]  =          {"t0  ", "t1  ", "t2  ",
                                      "t3  ", "t4  ", "t5  ",
                                      "t6  ", " t7  ", "t8  "};        // array of string representations of temporary variables which can be edited, [vac units, trip points, drive intervals, etc.]

String    savedEntries[9] =          {"s0  ", "s1  ", "s2  ",
                                      "s3  ", "s4  ", "s5  ",
                                      "s6  ", "s7  ", "s8  "};         // array of string representations of saved variables [vac units, trip points, drive intervals, etc.]

boolean   inHgTemp        =         true;
boolean   kPaTemp         =         false;
boolean   unitChange      =         false;    // when true, you've just change units from inHg to kPa or vica versa, do some math to get the right units

int       tripPointOneTemp=         25;       // temporary versions of trip points one and two, so that we can modify them
int       tripPointTwoTemp=         15;       // in programming mode, but only permanently change them with the CROSS button

float     tripPointOneFloat =       2.5;      // floating point versions of trip points one and two, so
float     tripPointTwoFloat =       1.5;      // we can convert between inHg and kPa without truncation errors

const float unitMult      =         3.38639;  // multiply inHg by this to get kPa, divide kPa by this to get inHg

int       torqueScale     =         10;       // scaling factor for Fall Protector torque, turns down max torque
int       torqueScaleTemp =         10;       // temporary version for editing

long      editBlinkTime   =         0;        // timer to make the entry you are editing blink so you know what's up
boolean   editBlink       =         false;    // flops back and forth, to make the entry you are editing blink

boolean   editing         =         false;    // while true, you are editing an entry, the entry will blink to let you know
boolean   entrySaved      =         false;    // while in programming mode, tap CROSS to save the entry you are editing, it will stop blinking

int       menuIndex       =         0;        // where are we in the menu?
int       lastMenuIndex   =         0;        // used to check if we changed the menu index
boolean   moveSlot        =         true;     // true when you need to print the asterisk to the LCD to show the editable item
int       menuPage        =         0;        // which page are we on? 
int       lastPage        =         -1;       // keep track of page so you don't need to rewrite page every loop, initialized such that we do write the first time

int       hours           =         0;
int       minutes         =         0;
String    hourString      =         "000";
String    minuteString    =         "00";

//------------------------------SETUP LOOP---------------
void setup(){

drivespeedl = pot_final;
drivespeedr = pot_final;

//-----------------------OUTPUTS/INPUTS---------------

pinMode(vacPot, INPUT);             // vac pot input, pin A0
pinMode(speedPot, INPUT);           // climber speed pot, pin A1
pinMode(torquePot, INPUT);          // FP torque pot, pin A2
pinMode(2, INPUT_PULLUP);           // Vac Enable Switch
pinMode(3, OUTPUT);                 // Clutch
pinMode(4, OUTPUT);                 // Brake
pinMode(5, OUTPUT);                 // LL motor LOWER
pinMode(6, OUTPUT);                 // LL motor LIFT
pinMode(7, OUTPUT);                 // FP Torque, PWM

// Pin 8 = nothing
// Pin 9 = used for USB shield
// Pin 10 = used for USB shield
// Pin 11 = nothing

pinMode(12, INPUT_PULLUP);          // LCD wired switch, goes LOW when LCD is connected with wires
pinMode(13, OUTPUT);                // PWM out to vac display
analogWrite(13, 60);                // initialize the vac display to a value so that it reads 0.0 at startup

// Pin 16 = Tx2, to LCD 
// Pin 17 = Rx2, not used
// Pin 18 = Tx1, to climber
// Pin 19 = Rx1 from climber



  
//------------------------------BEGIN SERIAL COMMUNICATION----------------

Serial.begin(38400);                // communication with computer serial monitor
Serial1.begin(38400);               // communication with climber
Serial2.begin(9600);                // communication to wired LCD display, transmit only


//--------------------------------BEGIN PS3 INITIALIZATION------------------

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    //delay(8000);
    //asm volatile ("  jmp 0");
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
//---------------------------------END PS3 INITIALIZATION---------------------------

short revision;                                     // this chunk here initializes the EEPROM
EEPROM.get(0, revision);                            // whenever you change revision numbers or
if (revision != REVISION) {                         // program a robot the first time.
                                                    // it just fills the EEPROM with values
                                                    // from the initial variable declarations
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  
  EEPROM.put(2, downTime);              
  EEPROM.put(6, pauseTime);
  EEPROM.put(10, upTime);
  EEPROM.put(14, driveTime);
  EEPROM.put(18, totalVacTime);
  EEPROM.put(22, inHg);
  EEPROM.put(23, kPa);
  EEPROM.put(24, tripPointOne);
  EEPROM.put(28, tripPointTwo);
  EEPROM.put(32, variVacuum);
  EEPROM.put(33, torqueScale);
  EEPROM.put(37, saved);
 
  revision = REVISION;
  EEPROM.put(0, revision);

}




//    LOAD EEPROM VALUES FOR SAVED SETTINGS

EEPROM.get(2, downTime);
EEPROM.get(6, pauseTime);
EEPROM.get(10, upTime);
EEPROM.get(14, driveTime);
EEPROM.get(18, totalVacTime);

EEPROM.get(22, inHg);
EEPROM.get(23, kPa);

EEPROM.get(24, tripPointOne);
EEPROM.get(28, tripPointTwo);

EEPROM.get(32, variVacuum);
EEPROM.get(33, torqueScale);

EEPROM.get(37, saved);


Serial.println(" Retrieved EEPROM ");
Serial.print(" down time: ");
Serial.println(downTime);
Serial.print(" pause time: ");
Serial.println(pauseTime);
Serial.print(" up time: ");
Serial.println(upTime);
Serial.print(" drive interval: ");
Serial.println(driveTime);
Serial.print(" total vac-on time: ");
Serial.println(totalVacTime);
Serial.print(" total Climber-on time: ");
Serial.println(totalRunTime);

}

void loop() {
  
  Usb.Task(); //PS3 get info command
  PS3.attachOnInit(BTonInit);
  PS3usb.attachOnInit(USBonInit);
  SerialBT.attachOnInit(SerialBTonInit);




// ------------------------- CHECK LCD DISPLAY, IS IT WIRED OR NOT? -----------------------

  LCDwired = !digitalRead(12);                              // true while the LCD is plugged in to the Control Station
                                                            // we use !digitalRead because it's configured as an INPUT_PULLUP

  LCDwired = true;                                          // JUST FOR TESTING! DELETE ME ONCE EVERYTHING WORKS
  
  if (LCDwired == true) {                                   // if the LCD is wired to the control station
    if (wiredFirstTime == true) {                           // and has not been initialized yet
      displayInit4x40();                                    // do it!
      wiredFirstTime = false;
    } 
  } else {                                                  // it's not not plugged in here,
    wiredFirstTime = true;                                  // it will need initialization the next time it is plugged in
  }


//------------------------------------- LCD BLINKY COLON  ----------------------------------

  if (millis() - lastMillis > 500) {                        // this block makes a blinky colon after "FP", so you can tell
    lastMillis = millis();                                  // that the LCD display is actually working,
    if (flipFlop == 1) {                                    // if it stops blinking, you are disconnected :(
      flipFlop = 0;
      displayWrite4x40(18, 1, ":");
    } else {
      flipFlop = 1;
      displayWrite4x40(18, 1, " ");
    }
  } 



  
//------------------------ FALL PROTECTOR TORQUE, READ POT AND WRITE TO LCD ---------------------------- 

  FPtorque = analogRead(torquePot) >> 2;                    // FP torque range is 0 to 255 (n >> 2 is equivalent to n/4, but faster)

  if (FPtorque <= 50) {
    displayTorque = (FPtorque * 100) >> 8;                  // display Torque range is 0 to 99 (n >> 8 is equivalent to n/256, but faster)
  }else {
    displayTorque = ((FPtorque * 100) >> 8) + 1;            // without this cheeky line the display only gets up to 98, our little secret
  }

  FPtorque = (FPtorque * (torqueScale + 10)) / 20;          // here we scale the torque value with the saved parameter from programming mode 
  
  smoothTorque = (displayTorque + smoothTorque) >> 1;       // take the average of the last two torque readings (n >> 1 is equivalent to n/2, but faster)
   
  if (smoothTorque != torqueLast) {                         // only bother writing to the LCD display if the value has changed 
    
    torqueLast = smoothTorque;
    
    LCDtorqueString = String(smoothTorque);                 // turns the value into a string so we can write it to the LCD

    if (smoothTorque >= 10) {                               // if the value is >= 10 write both digits
      displayWrite4x40(19, 1, LCDtorqueString);
    } else {                                                // if the value is < 10 write a 0 first to keep the formatting nice
      displayWrite4x40(19, 1, "0" + LCDtorqueString);
    } 
  }



       
//------------------------READ SPEED POT / PROCESS SPEED ---------------------------
  
  val = analogRead(speedPot);                                 // Read potentiometer for drive speed
  
  pot_final = val * 2;                                        // analog input range is 0 - 1023, we want range of 0 - 2047




//--------------------------SWITCH SPEED BETWEEN POT AND SAVED -------------------


    
  if (s_switched == false) {
    drivespeedl = pot_final;
    drivespeedr = pot_final;

    
    LCDspeed = (pot_final * 5) / 103;                         // pot speed of zero to 99 for LCD screen.
    
    if (LCDspeed >= 50) {                                     // without this cheeky block the display only gets up to 98, our little secret
      LCDspeed ++; 
    }

   
    if (Spressed == true) {
      displayWrite4x40(9, 3, "SAVED:");                       // "SAVED" means the value saved to EEPROM controls climber speed
    }
  } else if (s_switched == true) {
    EEPROM.get(37, saved);                                     
    drivespeedl = saved;
    drivespeedr = saved;
    //Serial.print(" saved drive speed variable: ");
    //Serial.println(drivespeedl);
    LCDspeed = (saved * 5) / 103;                             // saved speed of zero to 99 for LCD screen.

    if (LCDspeed >= 50) {                                     // without this cheeky block the display only gets up to 98, our little secret
      LCDspeed ++; 
    }

    if (Spressed == true) {
      displayWrite4x40(9, 3, "SPEED:");                       // "SPEED", means the knob controls climber speed
    }
  }
 


//-------------------------------WRITE SPEED TO LCD----------------------------



  smoothLCDspeed = (LCDspeed + smoothLCDspeed) >> 1;          // take the average of the last two speed readings (n >> 1 is equivalent to n/2, but faster)

  if (smoothLCDspeed != lastLCDspeed) {                       // only bother writing to the LCD display if the value has changed 
    
    lastLCDspeed = smoothLCDspeed;
    
    LCDspeedString = String(smoothLCDspeed);                  // turn the value into a string so we can write it to the LCD
    
    if (smoothLCDspeed >= 10) {                               // if the value is >= 10 write both digits
      displayWrite4x40(15, 3, LCDspeedString);
    } else {                                                  // if the value is < 10 write a 0 first to keep the formatting nice
      displayWrite4x40(15, 3, "0" + LCDspeedString);
    } 
  }  
 


  
//------------------CHANGE STRAIGHT-LINE ADJUST PERCENTAGE ---------------------------
  
  LRradjust = (5 * drivespeedr) / 100;
  LRladjust = (5 * drivespeedl) / 100;

  RD = rightadjust + drivespeedr;
  LD = leftadjust + drivespeedl;

  RD = constrain(RD, 0, 2047);
  LD = constrain(LD, 0, 2047);

//------------------------------------- CRUISE CONTROL----------------------------------
  
  if (cruise == 1) { // CONSTANT SPEED FORWARD DUAL
    //if(driveTime == 0) {
    if (driveInterval == false) {
      //Serial.println(" untimed drive ");
      Serial1.print(motorL);
      Serial1.println(-1 * LD * cruiseDirL);  // will reverse buttons based on drivemode (forward or reverse)
      Serial1.print(motorR);
      Serial1.println(-1 * RD * cruiseDirR);
      //Serial.print(" DUAL CONSTANT SPEED FORWARD ");
    //}else if(driveTime != 0){
    } else if (driveInterval == true) {  
      if (beginCruiseTime == 0) {
        currentDriveStart = millis();
        beginCruiseTime = 1;    
        //Serial.print("set timed drive: ");
        //Serial.println(currentDriveStart);    
      }
      //currentDriveTime = millis() - currentDriveStart;
      //Serial.print(" current drive time: ");
      //Serial.println(currentDriveTime);
      if ((millis() - currentDriveStart) <= driveTime) {
        //Serial.print(" timed drive: ");
        //Serial.println(driveTime);
        Serial1.print(motorL);
        Serial1.println(-1 * LD * cruiseDirL);  // will reverse buttons based on drivemode (forward or reverse)
        Serial1.print(motorR);
        Serial1.println(-1 * RD * cruiseDirR);
      } else {
        beginCruiseTime = 0;
        cruise = 0;
        triPressed = false;
        xPressed = false;
        //Serial.println(" end timed drive ");
      }

      //flip triangle boolean perhaps
    }
  } 



//----------------------------------- CONTROL VACUUM AND REQUEST VACUUM SENSOR READING ------------------------------------

  
  vacuumControl();                                    // check the vacuum enable switch and control the vacuum as required  

  getVacuum();                                        // ask climber for vacuum reading and update the LCD



//--------------------------------VACUUM SENSOR TRIP POINTS --------------------------------------- 

  if (vacCheck <= tripPointOne) {                           // check to see if the vacuum level is below trip point one
    vacWarn = true;                                         // 
  
    if (vacWarnLast != vacWarn) {                           // this block is entered when the vacuum is lowish, but was high last time around
      vacWarnChange = true;                                 // since we just went to a lowish vacuum condition 
      Serial2.write(0xFE);                                    
      Serial2.write(0x57);                                  // this block of serial commands drives GPO1 (labeled "PB0" on adafruit LCD backpack)
      Serial2.write(1);                                     // high, to turn on the warning LED since we're below the warning point 
    } else {
      vacWarnChange = false;                                // if the vacuum is lowish, and it was also lowish last time, there has been no change
    }
  
  } else {                                                  // here the vacuum level is above trip point one
    vacWarn = false; 
    if (vacWarnLast != vacWarn) {                           // if the vacuum just went high, and was low last time around
      vacWarnChange = true;
      Serial2.write(0xFE);
      Serial2.write(0x56);                                  // this block of serial commands drives GPO1 (labeled "PB0" on adafruit LCD backpack)
      Serial2.write(1);                                     // low, to turn off the warning LED since we're above the warning point
    } else {
      vacWarnChange = false;
    }
  
  }

  vacWarnLast = vacWarn;

  
  
  
  
  
  if (vacCheck <= tripPointTwo) {                           // check to see if the vacuum level is below trip point two
    
    vacLow = true;                                          // vacLow variable is used in the Fall Protector block of code
    
    if (vacLowLast != vacLow) {                             // this block is entered when the vacuum is low, but was high last time around
      vacLowChange = true;                                  // since we just went to a low vacuum condition (we also use "vacLow" to update the LCD)
      vibrate = true;                                       // we had better warn the user by rumbling the handheld controller
      vibrateTime = millis();                               // make a note of when we started rumbling       
    } else {
      vacLowChange = false;                                 // if the vacuum is low, and it was also low last time, there has been no change
    }
  
    if (vibrate == true) {  
                                           
      if (millis() - vibrateTime < 500) {                                  
        PS3.setRumbleOn(RumbleHigh);
        PS3usb.setRumbleOn(RumbleHigh);
      } else {
        PS3.setRumbleOff();
        PS3usb.setRumbleOff();
        vibrate = false;
      }
    }

  } else {                                                  // here the vacuum level is above trip point two
    vacLow = false;                                         
    if (vibrate == true) {                                  // if you happen to be rumbling the controller
      PS3.setRumbleOff();                                   // stahp
      PS3usb.setRumbleOff();
      vibrate = false;
    }
    if (vacLowLast != vacLow) {                             // if the vacuum just went high, and was low last time around
      vacLowChange = true;                                  // there has been a change, this is used to update the LCD display
    } else {
      vacLowChange = false;
    }
  }

  vacLowLast = vacLow;

/*
  if (vacLow == true) {
    Serial.println("OH SHIT!");
  } else if (vacWarn == true) {
    Serial.println("WARNING!");
  } else {
    Serial.println("AWWW YES");
  }
*/

  

//-------------------------------- UPDATE CRUISE CONTROL STATE ------------------------------

  if (cruise == 1 && cruiseLast == 0) {                       // cruise on, but was off last cycle
    cruiseLast = 1;
  } else if (cruise == 0 && cruiseLast == 1) {                // cruise off but was on last cycle
    cruiseLast = 0;
    displayWrite4x40(18, 3, "   ");                           // cruise is off
    //Serial.print("cruise off");
  }





//---------------CONTROLLER-------------------------------------------
//--------------------------ROUTINE-----------------------------------
//-------------------------------------STARTS-------------------------
//----------------------------------------------HERE------------------
//---------------UNPLUGGED CONTROLLER ROUTINE-------------------------
//------------(includes bracket at end of main loop)------------------
  
  if (PS3.PS3Connected == 0 && PS3usb.PS3Connected == 0) { //skip loop and make climber safe if no controller found
    //Serial.println(" controller unplugged ");
    
    Serial1.println("M1: 0");                     // turn off the drive motors
    Serial1.println("M2: 0");
    cruise = 0;

    Serial1.println("M3: 0");                     // turn off the Z-Lift motor
    
    Serial1.println("M4: 0");                     // turn off the tool
    toolOn = 0;
    displayWrite4x40(17, 2, "    ");              // clear the "TOOL" indicator on the LCD
    
    digitalWrite(3, LOW);                         // lock FP clutch
    digitalWrite(4, LOW);                         // lock FP brake
    digitalWrite(5, LOW);                         // turn LL motor off
    digitalWrite(6, LOW);                         // turn off the LL motor
    analogWrite(7, 0);                            // turn off the LL motor

    if (connectOnce == 1) {
      Serial.print("Resetting!");
      delay(1000);
      asm volatile ("  jmp 0");
    }

 //------------BLUETOOTH---------------------------------------------------
 //--------------------------CONTROLLER------------------------------------
 //-------------------------------------------STARTS-----------------------
 //------------------------------------------------------HERE--------------  
  
  } else if (PS3usb.PS3Connected == 0) { 

    connectOnce = 1;

    setLEDs();                                                  // show the PS3 controller battery status with 1 of 4 red LEDs

    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3.disconnect();
      connectOnce = 0;
    }



  
//                SELECT BUTTON   
    if (PS3.getButtonClick(SELECT)) {    
      if (drivemode == 1) {
        //Serial.print(" BEGIN REV-SINGLE MODE ");
        drivemode = -1; //must be "-1", used in math equations
        motorL = "M1: ";
        motorR = "M2: ";
      } else if (drivemode == -1) {
        drivemode = 1; //must be "1", used in math equations
        motorL = "M2: ";
        motorR = "M1: ";
        //Serial.print(" BEGIN SINGLE MODE ");
      }
    }
    

    RjoystickSpeed = map(PS3.getAnalogHat(RightHatY), 0, 255, -2047, 2047);       // scale the right joystick proportional value
    RjoystickSpeed = (RjoystickSpeed * pot_final) / 2047;                         // and allow the speed pot to turn it up or down

    LjoystickSpeed = map(PS3.getAnalogHat(LeftHatY), 0, 255, -2047, 2047);        // scale the left joystick proportional value
    LjoystickSpeed = (LjoystickSpeed * pot_final) / 2047;                         // and allow the speed pot to turn it up or down


//          CHECKS FOR PROPORTIONAL JOYSTICK READINGS 
 
  
//                START BUTTON    
    
    if (progEnterExit == true) {                                                    // if you are allowed (allowed means you are not coming straight from entering/exiting)
      if (PS3.getButtonPress(START)) {                                              // you can enter or exit programming mode by pressing and holding START for 5 seconds
        if (progTimerStart == true) {                                               // if you just started pressing START
          programTime = millis();                                                   // begin the timer
          progTimerStart = false;
        }
          
        if (millis() - programTime >= 5000) {                                       // if you've been pressing it for 5 seconds    
          programMode = !programMode;                                               // enter or exit programming mode
          progEnterExit = false;                                                    // this forbids entering or exiting programming mode again, so we don't oscillate back and forth between modes
          firstProg = true;                                                         // when you enter programming mode from here, do the initialize routine
        }
      } 
    } 
    
    if (!PS3.getButtonPress(START)) {                                               // it you are not pressing START
      progTimerStart = true;                                                        // the next time you press it the timer will start over
      progEnterExit = true;                                                         // and you are allowed to enter or exit programming mode   
    }



    
    if (PS3.getButtonClick(START)) {    
      if (proportionalSpeed == false) {
        //Serial.print(" PROPORTIONAL SPEED SET ");
        proportionalSpeed = true; 
        displayWrite4x40(15, 4, "ANALOG");
        highLimit = 150;
        lowLimit = 80;
      } else if (proportionalSpeed == true) {
        //Serial.print(" NON-PROPORTIONAL SPEED SET ");
        proportionalSpeed = false;
        displayWrite4x40(15, 4, "ON/OFF");
        highLimit = 200;
        lowLimit = 55;
      }
    }
    
    if (proportionalSpeed == true) {
        drivespeedl = abs(LjoystickSpeed);
        drivespeedr = abs(RjoystickSpeed);
    }
    /*if(proportionalSpeed == false){
        drivespeedl = pot_final;
        drivespeedr = pot_final;
    }*/
      
//if button pressed enter a NEW DRIVEMODE
//--> New drive mode changes speed depending on the 

    
    // DRIVE CONTROLS - SINGLE (no dual controls here)

    //FORWARD AND REVERSE
    if (PS3.getAnalogHat(LeftHatY) < lowLimit) {
      Serial1.print(motorL); //M2 is right side motor  M2: 2046
      Serial1.println(drivemode * drivespeedl);
      //Serial.print(" LEFT FORWARD: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (PS3.getAnalogHat(LeftHatY) > highLimit) {
      Serial1.print(motorL); //M2 is right side motor
      Serial1.println((drivemode * -1) * drivespeedl);
      //Serial.print(" LEFT BACK: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (cruise == 0) {
      leftadjust = 0;
      rightadjust = 0;
      Serial1.print(motorL);
      Serial1.println("0");
      //Serial.print(" LEFT STOPPED: ");
      //Serial.print(drivemode, DEC);
    }
    
    // RIGHT FORWARD AND REVERSE
    if (PS3.getAnalogHat(RightHatY) < lowLimit) {
      Serial1.print(motorR); // M1 is left side motor
      Serial1.println (drivemode * drivespeedr);
      //Serial.print(" RIGHT FORWARD: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (PS3.getAnalogHat(RightHatY) > highLimit) {
      Serial1.print(motorR); // M1 is left side motor
      Serial1.println((drivemode * -1) * drivespeedr);
      //Serial.print(" RIGHT BACK: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (cruise == 0) {
      leftadjust = 0;
      rightadjust = 0;
      Serial1.print(motorR);
      Serial1.println("0");
      //Serial.print(" RIGHT STOPPED: ");
      //Serial.print(drivemode, DEC);
    }

//-------------------------------D-PAD ------------------------------


    if (((PS3.getButtonPress(UP)) ||            // this first section helps make it so that we
      (PS3.getButtonPress(DOWN)))               // only write to the LCD display when something 
      != zLiftChange) {                         // has changed, this covers the Z-Lift, the
                                                // tool is covered in the CIRCLE button section
      
      zLiftWrite = true;                         // zLiftChange changes right when you start or
      zLiftChange = !zLiftChange;               // stop pressing UP/DOWN, and makes toolWrite
    }                                           // true when a change has happened
    
    
    
//               D UP/DOWN
    if (PS3.getButtonPress(UP)) {               // Z-lift up
        Serial1.print("M3: ");
        Serial1.println(-2047);
        if (zLiftWrite == true) {
          displayWrite4x40(14, 2, "UP");
          zLiftWrite = false;
        }
    } else if (PS3.getButtonPress(DOWN)) {      // Z-lift down
        Serial1.print("M3: ");
        Serial1.println(2047);
       if (zLiftWrite == true) {
          displayWrite4x40(14, 2, "DN");
          zLiftWrite = false;
        }
    } else if (sensorCycle == false) {          // Z-lift off, not pressing UP or DOWN, and not in a sensor cycle ( will we ever have both a TOOL and SENSOR CYCLE used together? )
      Serial1.print("M3: ");
      Serial1.println(0);
      if (zLiftWrite = true); {
        displayWrite4x40(14, 2, "  ");
        zLiftWrite = false;
      }
    }
      
      

 
//              D LEFT 
    if (PS3.getButtonClick(LEFT)) {
      leftadjust = leftadjust + LRladjust;
      rightadjust = rightadjust - LRradjust;
    }   

//            D RIGHT 
    if (PS3.getButtonClick(RIGHT)) {
      leftadjust = leftadjust - LRradjust;
      rightadjust = rightadjust + LRladjust;
    }   

  
//----------------------- SQUARE BUTTON: Save and switch speeds-------------------
    if (PS3.getButtonPress(SQUARE)) {
      //Serial.print(" Square pressed ");
      Spressed = true;
      Scount = Scount + 1;
    }
      
    if ((!PS3.getButtonPress(SQUARE)) && (Spressed == true)) {
      Spressed = false;
      if (Scount > 60) {
       //Serial.print(" SQUARE SAVED ");
       EEPROM.put(37, pot_final);
       //EEPROM.write(0, lowByte(pot_final));
       //EEPROM.write(1, highByte(pot_final));
      }
      if ((Scount < 60) && (s_switched == false)) {
        s_switched = true;
        //Serial.print(" Set to saved ");
      } else if ((Scount < 60) && (s_switched == true)) {
        s_switched = false;
        //Serial.print(" set to pot ");
      }
      Scount = 0;
    }
    
 
 
  //---------------------- CIRCLE BUTTON: TOOL ON/OFF ------------------------- uncomment either this or "automatic sensor plunge" block, depending on the accesories used

    if (PS3.getButtonClick(CIRCLE)) {                                 // here we turn the tool on and off
      if (!toolOn) {                                                   
        Serial1.println("P3: 2047");                                  
      } else {                                                        
        Serial1.println("P3: -2047");                                 // we use a Sabertooth MC to turn on/off a relay
      }  
      toolOn = !toolOn;
      toolWrite = true;                                               // this guy here is so that we only write to the LCD 
    }                                                                 // when the tool changes, not every loop

    
    if (toolWrite == true) {                                          // here we write the tool state to the LCD
      if (toolOn == true) {
        displayWrite4x40(17, 2, "TOOL");
    } else {
      displayWrite4x40(17, 2, "    ");
    }
      toolWrite = false;
    }

 
 //---------------------- CIRCLE BUTTON: automatic sensor plunge / Z-lift ----- uncomment either this or "TOOL ON/OFF" block, depending on the accesories used
/*
    //PRESSHOLD L3 and PRESS CIRCLE TO CLEAR PROGRAM
    if (PS3.getButtonPress(CIRCLE) && PS3.getButtonPress(L3)) {
      zCycle = 0;
    }
    
    if ((zCycle == 0) && PS3.getButtonPress(CIRCLE) && (beginProgram == 0)) {
      beginProgram = 1;
      pausedAlready = 0;
    }
  
    //HOLD DOWN THEN PAUSE THEN UP TO PROGRAM TIMES 
    if (beginProgram == 1) {
      if (PS3.getButtonPress(DOWN) && (downBegin == 0)) {
        downTime = millis();
        downBegin = 1;
        //Serial.println(" down ");
      } else if (!PS3.getButtonPress(DOWN) && !PS3.getButtonPress(UP) && (pausedBegin == 0) && (pausedAlready == 0) && (downBegin == 1)) {
        downTime = millis() - downTime;
        pauseTime = millis();
        pausedBegin = 1;
        //Serial.println(" paused ");
      } else if (PS3.getButtonPress(UP) && (upBegin == 0)) {
        pauseTime = millis() - pauseTime;
        upTime = millis();
        upBegin = 1;
        pausedAlready = 1;
        //Serial.println(" up ");
      } else if (!PS3.getButtonPress(UP) && (pausedAlready == 1)) {
        upTime = millis() - upTime;
        zCycle = 1;
        beginProgram = 0;
        downBegin = 0;
        pausedBegin = 0;
        upBegin = 0;
        pausedAlready = 0;
  
        EEPROM.put(2, downTime);
        EEPROM.put(6, pauseTime);
        EEPROM.put(10, upTime);
        
        //Serial.println(" Finished Program ");
        //Serial.println(" EEPROM Saved ");
      }
    }
       
    if (PS3.getButtonClick(CIRCLE) && !PS3.getButtonPress(L3) && zCycle == 1) {              
      cPressed = true;
      if (cPressed == true) {
        if (sensorCycle == false) {
          sensorCycle = true;
          zTime = millis();
          displayWrite2x16(0x4C, " INT");
        }
      }
    }
  
    if ((sensorCycle == true) && (!PS3.getButtonPress(DOWN) || !PS3.getButtonPress(UP))) {
                                    
      if ((millis() - zTime) < downTime) {                                     // start a sensor reading cycle by lowering sensor
        Serial1.println("M3: 2047");                     
        //Serial.println("PLUNGE");
      }
      if (((millis() - zTime) >= downTime) && ((millis() - zTime) < (downTime + pauseTime))) {    // after sensor hits the surface, turn it off for a time  
        Serial1.println("M3: 0");
        //Serial.println("WAIT");
      }
      if (((millis() - zTime) >= (downTime + pauseTime)) && ((millis() - zTime) < (downTime + pauseTime + upTime))) {    // after sensor has been on the surface, lift it up again  
        Serial1.println("M3: -2047");
        //Serial.println("RETRACT");
      }
      if ((millis() - zTime) >= (downTime + pauseTime + upTime)) {                                     // after sensor has lifted up, stop the z-lift motor  
        Serial1.println("M3: 0");
        sensorCycle = false; 
        displayWrite2x16(0x4C, "    ");
        //Serial.println("OFF");
      } 
    }
    
    if ((sensorCycle == true) && (PS3.getButtonPress(DOWN) || PS3.getButtonPress(UP))) {
      Serial1.println("M3: 0");
      sensorCycle = false;
      displayWrite2x16(0x4C, "    ");
    }
*/
  
 //--------------------------TRIANGLE BUTTON: Constant speed forward----------------   

    //PROGRAM DRIVE TIME:
    if (PS3.getButtonPress(TRIANGLE) && PS3.getButtonPress(L3) && (beginDriveTime == 0)) { //IF PROGRAMMING MODE
      driveTime = millis();
      beginDriveTime = 1;
      //Serial.println(" set initial drive time ");
    } else if (!PS3.getButtonPress(TRIANGLE) && !PS3.getButtonPress(L3) && (beginDriveTime == 1)) { //RELEASE BUTTONS
      driveProgramStart = 1;
      //Serial.println(" program started ");
    } else if (PS3.getButtonPress(TRIANGLE) && (!PS3.getButtonPress(L3)) && (driveProgramStart == 1)) {  //PRESS TRI TO SAVE SPEED
      //SAVE DRIVE TIME
      driveTime = millis() - driveTime;
      EEPROM.put(14, driveTime);
      driveInterval = true;
      displaySetup2x16(0x06);
      timeFloat = driveTime/100;
      timeFloat = timeFloat/10;

      //Serial.println(timeFloat, 1);
      beginDriveTime = 0;
      driveProgramStart = 0;
      beginCruiseTime = 0; //new
      //Serial.print(" programmed drivetime: ");
      //Serial.println(driveTime);
    } else if ((!PS3.getButtonPress(TRIANGLE)) && (PS3.getButtonPress(L3)) && (driveProgramStart == 1)) { //PRESS L3 TO CANCEL
      //CANCEL PROGRAMMING
      //driveTime = 0;
      //EEPROM.put(14, driveTime);
      EEPROM.get(14, driveTime);
      driveInterval = !driveInterval;
      beginDriveTime = 0;
      driveProgramStart = 0;
      beginCruiseTime = 0; //new
      triPressed = false;
      xPressed = false;
      cruise = 0;
      //Serial.print(" programmed drivetime: ");
      //Serial.println(driveTime);
    }

    if (PS3.getButtonClick(TRIANGLE)) {
      triPressed = !triPressed;  
      if (triPressed == true) {
        cruiseDirL = 1;
        cruiseDirR = 1;
        cruise = 1;
        if (driveInterval == true) {
          displaySetup2x16(0x06);
          if (timeFloat <= 9.9) {
            SerialBT.print(timeFloat, 1);
          } else {
            SerialBT.print(timeFloat, 0);
          }
        } else {
          displayWrite4x40(18, 3, "FWD");
        }
        //Serial.println("cruise button on");
      } else if ((triPressed == false) && (!PS3.getButtonPress(L3))) {
        cruiseDirL = 0;
        cruiseDirR = 0;
        cruise = 0;
        //displayWrite4x40(18, 3, "   ");
        //Serial.println("cruise button off");
      }
    }
    
  /*  if(PS3.getButtonClick(TRIANGLE)) {
      //Serial.print(" TRIANGLE, Constant speed now forward ");
      cruiseDirL = 1;
      cruiseDirR = 1;
      cruise = 1;
      SerialBT.write(0xFE); 
      SerialBT.write(0x45);
      SerialBT.write(0x06);
      delay(1);
      SerialBT.print("FWD");
    }*/

 //-------------------------X BUTTON: Constant speed reverse------------------------
   /* if(PS3.getButtonClick(CROSS)) {
      //Serial.print(" X-BUTTON, Constant speed now reverse ");
      cruiseDirR = -1;
      cruiseDirL = -1;
      cruise = 1;
      SerialBT.write(0xFE); 
      SerialBT.write(0x45);
      SerialBT.write(0x06);
      delay(1);
      SerialBT.print("REV");
      }*/

    if (PS3.getButtonClick(CROSS)) {
      xPressed = !xPressed;  
      if (xPressed == true) {
        cruiseDirL = -1;
        cruiseDirR = -1;
        cruise = 1;
        if (driveInterval == true) {
          displaySetup2x16(0x06);
          if (timeFloat <= 9.9) {
            SerialBT.print(timeFloat, 1);
          } else {
            SerialBT.print(timeFloat, 0);
          }
        } else {
          displayWrite4x40(18, 3, "REV");
        }
        //Serial.println("Rcruise button on");
      } else if (xPressed == false) {
        cruiseDirL = 0;
        cruiseDirR = 0;
        cruise = 0;
        //displayWrite4x40(18, 3, "   ");
        //Serial.println("Rcruise button off");
      }
    }
 
//-------------------------R3-L3----DEMO PROGRAM--------------------------------------
    if (PS3.getButtonClick(R3)) {
      cruise = 1;
      cruiseDirR = 1;
      cruiseDirL = 1;
      displayWrite2x16(0x06, "DM1");
      demoTime = millis();
      demoOn = 1;
    } 
  
    if (demoOn == 1) {
      if ((millis() - demoTime) > 24000) {
        cruiseDirR = cruiseDirR * -1;
        cruiseDirL = cruiseDirL * -1;
        demoTime = millis();
        demo2Time = millis();
        firstTurn = 1;
      }
    }


  /*if (PS3.getButtonClick(L3)){
    cruise = 1;
    cruiseDirR = 1;
    cruiseDirL = 1;
    SerialBT.write(0xFE); 
    SerialBT.write(0x45);
    SerialBT.write(0x06);
    delay(1);
    SerialBT.print("DM2");
    demoTime = millis();
    //demo2Time = millis();
    demo2On = 1;
    firstTurn = 0;
  }

  if(demo2On == 1 && firstTurn == 0){
      //Serial.print(" first loop: ");
      //Serial.println(firstTurn);
      if((millis() - demoTime)>20000){
        cruiseDirR = cruiseDirR * -1;
        cruiseDirL = cruiseDirL * 1;
        demo2Time = millis();
        firstTurn = 1;
    }
  }

  if(demo2On == 1 && (firstTurn == 1 || firstTurn == 2)){
    //Serial.print(" second loop: ");
    //Serial.println(firstTurn);
    if((millis() - demo2Time)>5000){
      //Serial.print(" second loop - 2: ");
      //Serial.println(firstTurn);
      cruiseDirR = cruiseDirR * -1;
      cruiseDirL = cruiseDirL * -1;
      //demoTime = millis();
      demo2Time = millis();
      firstTurn = firstTurn + 1;
      if(firstTurn == 3){
        //Serial.print(" second loop - 3: ");
        //Serial.println(firstTurn);
        demoTime = millis();
        firstTurn = 0;
        cruise = 1;
        cruiseDirR = cruiseDirR * 1;
        cruiseDirL = cruiseDirL * -1;
      }
    }
  }*/

//--------FALL PROTECTOR----------------------------------------------------------
    
    //TENSION   
    if (PS3.getButtonPress(R2) || vacLow == 0) {                                  // L1 = LOWER
      analogWrite(7, FPtorque);                                                   // R1 = LIFT
      //Serial.print(" TENSION ON ");                                             // L2 = SPOOL FREE
    } else {                                                                      // R2 or VACUUM > 1.5inHG = TENSION 
      analogWrite(7, 0);                                                          
    }

    //CLUTCH
    if (PS3.getButtonPress(R2) || PS3.getButtonPress(L2) || vacLow == 0) {
      digitalWrite(3, HIGH);
      //Serial.print(" CLUTCH FREE ");
    } else {
      digitalWrite(3, LOW);
    }

    //BRAKE
    if (PS3.getButtonPress(R2) || PS3.getButtonPress(L2)
    || PS3.getButtonPress(R1) || PS3.getButtonPress(L1) 
    || vacLow == 0) {
      digitalWrite(4, HIGH);
      //Serial.print(" BRAKE FREE ");
      if (FPunlocked == 0) { // vacLowChange = 1 means the vacuum has just changed from high to low, or vice versa 
        FPunlocked = 1;
        //Serial.println("unlocked");
        displayWrite4x40(9, 1, "      ");
      }
    } else {
      digitalWrite(4, LOW);
      if (FPunlocked == 1) { // FPunlocked==1 means yes the FP was previously unlocked
        FPunlocked = 0;
        displayWrite4x40(9, 1, "LOCKED");
      }
    }

    //LL MOTOR LIFT
    if (PS3.getButtonPress(R1)) {   
      digitalWrite(6, HIGH);                                    
      //Serial.print(" LL MOTOR LIFT ");                        

    } else {
      digitalWrite(6, LOW);
    }

    //LL MOTOR LOWER
    if (PS3.getButtonPress(L1)) {
      digitalWrite(5, HIGH);
      //Serial.print(" LL MOTOR LOWER ");

    } else {
      digitalWrite(5, LOW);
    }


 //------------------USB---------------------------------------------------
 //--------------------------CONTROLLER------------------------------------
 //-------------------------------------------STARTS-----------------------
 //------------------------------------------------------HERE--------------
  } else if (PS3.PS3Connected == 0) { 

    connectOnce = 1;
    
    if (PS3usb.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3usb.Release();
      connectOnce = 0;
    }

    
  
//                SELECT BUTTON   
    if (PS3usb.getButtonClick(SELECT)) {    
      if (drivemode == 1) {
        //Serial.print(" BEGIN REV-SINGLE MODE ");
        drivemode = -1; //must be "-1", used in math equations
        motorL = "M1: ";
        motorR = "M2: ";
      } else if (drivemode == -1) {
        drivemode = 1; //must be "1", used in math equations
        motorL = "M2: ";
        motorR = "M1: ";
        //Serial.print(" BEGIN SINGLE MODE ");
      }
    }
    
    RjoystickSpeed = map(PS3usb.getAnalogHat(RightHatY), 0, 255, -2047, 2047);    // scale the right joystick proportional value
    RjoystickSpeed = (RjoystickSpeed * pot_final) / 2047;                         // and allow the speed pot to turn it up or down

    LjoystickSpeed = map(PS3usb.getAnalogHat(LeftHatY), 0, 255, -2047, 2047);     // scale the left joystick proportional value
    LjoystickSpeed = (LjoystickSpeed * pot_final) / 2047;                         // and allow the speed pot to turn it up or down


//          CHECKS FOR PROPORTIONAL JOYSTICK READINGS 
 
  
//                START BUTTON    
    
    
    if (progEnterExit == true) {                                                    // if you are allowed (allowed means you are not coming straight from entering/exiting)
      if (PS3usb.getButtonPress(START)) {                                           // you can enter or exit programming mode by pressing and holding START for 5 seconds
        if (progTimerStart == true) {                                               // if you just started pressing START
          programTime = millis();                                                   // begin the timer
          progTimerStart = false;
        }
          
        if (millis() - programTime >= 5000) {                                       // if you've been pressing it for 5 seconds    
          programMode = !programMode;                                               // enter or exit programming mode
          progEnterExit = false;                                                    // this forbids entering or exiting programming mode again, so we don't oscillate back and forth between modes
          firstProg = true;                                                         // when you enter programming mode from here, do the initialize routine
        }
      } 
    } 
    
    if (!PS3usb.getButtonPress(START)) {                                            // it you are not pressing START
      progTimerStart = true;                                                        // the next time you press it the timer will start over
      progEnterExit = true;                                                         // and you are allowed to enter or exit programming mode   
    }
    
    
    
 
    
    
    if (PS3usb.getButtonClick(START)) {    
      if (proportionalSpeed == false) {
        //Serial.print(" PROPORTIONAL SPEED SET ");
        proportionalSpeed = true; 
        displayWrite4x40(15, 4, "ANALOG");
        highLimit = 150;
        lowLimit = 80;
      } else if (proportionalSpeed == true) {
        //Serial.print(" NON-PROPORTIONAL SPEED SET ");
        proportionalSpeed = false;
        displayWrite4x40(15, 4, "ON/OFF");
        highLimit = 200;
        lowLimit = 55;
      }
    }
    
    if (proportionalSpeed == true) {
        drivespeedl = abs(LjoystickSpeed);
        drivespeedr = abs(RjoystickSpeed);
    }
    /*if(proportionalSpeed == false){
        drivespeedl = pot_final;
        drivespeedr = pot_final;
    }*/
      
//if button pressed enter a NEW DRIVEMODE
//--> New drive mode changes speed depending on the 

    
    // DRIVE CONTROLS - SINGLE (no dual controls here)

    //FORWARD AND REVERSE
    if (PS3usb.getAnalogHat(LeftHatY) < lowLimit) {
      Serial1.print(motorL); //M2 is right side motor  M2: 2046
      Serial1.println(drivemode * drivespeedl);
      //Serial.print(" LEFT FORWARD: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (PS3usb.getAnalogHat(LeftHatY) > highLimit) {
      Serial1.print(motorL); //M2 is right side motor
      Serial1.println((drivemode * -1) * drivespeedl);
      //Serial.print(" LEFT BACK: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (cruise == 0) {
      leftadjust = 0;
      rightadjust = 0;
      Serial1.print(motorL);
      Serial1.println("0");
      //Serial.print(" LEFT STOPPED: ");
      //Serial.print(drivemode, DEC);
    }
    
    // RIGHT FORWARD AND REVERSE
    if (PS3usb.getAnalogHat(RightHatY) < lowLimit) {
      Serial1.print(motorR); // M1 is left side motor
      Serial1.println (drivemode * drivespeedr);
      //Serial.print(" RIGHT FORWARD: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (PS3usb.getAnalogHat(RightHatY) > highLimit) {
      Serial1.print(motorR); // M1 is left side motor
      Serial1.println((drivemode * -1) * drivespeedr);
      //Serial.print(" RIGHT BACK: ");
      //Serial.print(drivemode, DEC);
      cruise = 0;
      demoOn = 0;
      demo2On = 0;
    } else if (cruise == 0) {
      leftadjust = 0;
      rightadjust = 0;
      Serial1.print(motorR);
      Serial1.println("0");
      //Serial.print(" RIGHT STOPPED: ");
      //Serial.print(drivemode, DEC);
    }

//-------------------------------D-PAD ------------------------------


    if (((PS3usb.getButtonPress(UP)) ||         // this first section helps make it so that we
      (PS3usb.getButtonPress(DOWN)))            // only write to the LCD display when something 
      != zLiftChange) {                         // has changed, this covers the Z-Lift, the
                                                // tool is covered in the CIRCLE button section
      
      toolWrite = true;                         // zLiftChange changes right when you start or
      zLiftChange = !zLiftChange;               // stop pressing UP/DOWN, and makes toolWrite
    }                                           // true when a change has happened
    
    
    
//               D UP/DOWN
    if (PS3usb.getButtonPress(UP)) {            // Z-lift up
        Serial1.print("M3: ");
        Serial1.println(-2047);
        if (toolWrite == true) {
          displayWrite2x16(0x4C, "T-UP");
          toolWrite = false;
        }
    } else if (PS3usb.getButtonPress(DOWN)) {   // Z-lift down
        Serial1.print("M3: ");
        Serial1.println(2047);
       if (toolWrite == true) {
          displayWrite2x16(0x4C, "T-DN");
          toolWrite = false;
        }
    } else if (sensorCycle == false) {          // Z-lift off, not pressing UP or DOWN, and not in a sensor cycle ( will we ever have both a TOOL and SENSOR CYCLE used together? )
      Serial1.print("M3: ");
      Serial1.println(0);
      if (toolWrite == true) {
          if (toolOn == 1) {
            displayWrite2x16(0x4C, "TOOL");
        } else {
          displayWrite2x16(0x4C, "    ");
        }
        toolWrite = false;
      }
    }

    


 
//              D LEFT 
    if (PS3usb.getButtonClick(LEFT)) {
      leftadjust = leftadjust + LRladjust;
      rightadjust = rightadjust - LRradjust;
    }   

//            D RIGHT 
    if (PS3usb.getButtonClick(RIGHT)) {
      leftadjust = leftadjust - LRradjust;
      rightadjust = rightadjust + LRladjust;
    }   

  
//----------------------- SQUARE BUTTON: Save and switch speeds-------------------
    if (PS3usb.getButtonPress(SQUARE)) {
      //Serial.print(" Square pressed ");
      Spressed = true;
      Scount = Scount + 1;
    }
      
    if ((!PS3usb.getButtonPress(SQUARE)) && (Spressed == true)) {
      Spressed = false;
      if (Scount > 60) {
       //Serial.print(" SQUARE SAVED ");
       EEPROM.put(37, pot_final);
       //EEPROM.write(0, lowByte(pot_final));
       //EEPROM.write(1, highByte(pot_final));
      }
      if ((Scount < 60) && (s_switched == false)) {
        s_switched = true;
        //Serial.print(" Set to saved ");
      } else if ((Scount < 60) && (s_switched == true)) {
        s_switched = false;
        //Serial.print(" set to pot ");
      }
      Scount = 0;
    }
    
 
 
  //---------------------- CIRCLE BUTTON: TOOL ON/OFF ------------------------- uncomment either this or "automatic sensor plunge" block, depending on the accesories used

    if (PS3usb.getButtonClick(CIRCLE)) {                              // this block just turns the tool on and off
      if (!toolOn) {                                                  // the code that writes "TOOL" or "____" to the 
        Serial1.println("P3: 2047");                                  // LCD is actually in the D-pad up/down section
      } else {                                                        
        Serial1.println("P3: -2047");                                 // we use a Sabertooth MC to turn on/off a relay
      }  
      toolOn = !toolOn;
      toolWrite = true;                                               // this guy here is so that we only write to the LCD 
    }                                                                 // when the tool changes, not every loop
    

 
 //---------------------- CIRCLE BUTTON: automatic sensor plunge / Z-lift ----- uncomment either this or "TOOL ON/OFF" block, depending on the accesories used
/*
    //PRESSHOLD L3 and PRESS CIRCLE TO CLEAR PROGRAM
    if (PS3usb.getButtonPress(CIRCLE) && PS3usb.getButtonPress(L3)) {
      zCycle = 0;
    }
    
    if ((zCycle == 0) && PS3usb.getButtonPress(CIRCLE) && (beginProgram == 0)) {
      beginProgram = 1;
      pausedAlready = 0;
    }
  
    //HOLD DOWN THEN PAUSE THEN UP TO PROGRAM TIMES 
    if (beginProgram == 1) {
      if (PS3usb.getButtonPress(DOWN) && (downBegin == 0)) {
        downTime = millis();
        downBegin = 1;
        //Serial.println(" down ");
      } else if (!PS3usb.getButtonPress(DOWN) && !PS3usb.getButtonPress(UP) && (pausedBegin == 0) && (pausedAlready == 0) && (downBegin == 1)) {
        downTime = millis() - downTime;
        pauseTime = millis();
        pausedBegin = 1;
        //Serial.println(" paused ");
      } else if (PS3usb.getButtonPress(UP) && (upBegin == 0)) {
        pauseTime = millis() - pauseTime;
        upTime = millis();
        upBegin = 1;
        pausedAlready = 1;
        //Serial.println(" up ");
      } else if (!PS3usb.getButtonPress(UP) && (pausedAlready == 1)) {
        upTime = millis() - upTime;
        zCycle = 1;
        beginProgram = 0;
        downBegin = 0;
        pausedBegin = 0;
        upBegin = 0;
        pausedAlready = 0;
  
        EEPROM.put(2, downTime);
        EEPROM.put(6, pauseTime);
        EEPROM.put(10, upTime);
        
        //Serial.println(" Finished Program ");
        //Serial.println(" EEPROM Saved ");
      }
    }
       
    if (PS3usb.getButtonClick(CIRCLE) && !PS3usb.getButtonPress(L3) && zCycle == 1) {              
      cPressed = true;
      if (cPressed == true) {
        if (sensorCycle == false) {
          sensorCycle = true;
          zTime = millis();
          displayWrite2x16(0x4C, " INT");
        }
      }
    }
  
    if ((sensorCycle == true) && (!PS3usb.getButtonPress(DOWN) || !PS3usb.getButtonPress(UP))) {
                                    
      if ((millis() - zTime) < downTime) {                                     // start a sensor reading cycle by lowering sensor
        Serial1.println("M3: 2047");                     
        //Serial.println("PLUNGE");
      }
      if (((millis() - zTime) >= downTime) && ((millis() - zTime) < (downTime + pauseTime))) {    // after sensor hits the surface, turn it off for a time  
        Serial1.println("M3: 0");
        //Serial.println("WAIT");
      }
      if (((millis() - zTime) >= (downTime + pauseTime)) && ((millis() - zTime) < (downTime + pauseTime + upTime))) {    // after sensor has been on the surface, lift it up again  
        Serial1.println("M3: -2047");
        //Serial.println("RETRACT");
      }
      if ((millis() - zTime) >= (downTime + pauseTime + upTime)) {                                     // after sensor has lifted up, stop the z-lift motor  
        Serial1.println("M3: 0");
        sensorCycle = false; 
        displayWrite2x16(0x4C, "    ");
        //Serial.println("OFF");
      } 
    }
    
    if ((sensorCycle == true) && (PS3usb.getButtonPress(DOWN) || PS3usb.getButtonPress(UP))) {
      Serial1.println("M3: 0");
      sensorCycle = false;
      displayWrite2x16(0x4C, "    ");
    }
*/
  
 //--------------------------TRIANGLE BUTTON: Constant speed forward----------------   

    //PROGRAM DRIVE TIME:
    if (PS3usb.getButtonPress(TRIANGLE) && PS3usb.getButtonPress(L3) && (beginDriveTime == 0)) { //IF PROGRAMMING MODE
      driveTime = millis();
      beginDriveTime = 1;
      //Serial.println(" set initial drive time ");
    } else if (!PS3usb.getButtonPress(TRIANGLE) && !PS3usb.getButtonPress(L3) && (beginDriveTime == 1)) { //RELEASE BUTTONS
      driveProgramStart = 1;
      //Serial.println(" program started ");
    } else if (PS3usb.getButtonPress(TRIANGLE) && (!PS3usb.getButtonPress(L3)) && (driveProgramStart == 1)) {  //PRESS TRI TO SAVE SPEED
      //SAVE DRIVE TIME
      driveTime = millis() - driveTime;
      EEPROM.put(14, driveTime);
      driveInterval = true;
      displaySetup2x16(0x06);
      timeFloat = driveTime/100;
      timeFloat = timeFloat/10;

      //Serial.println(timeFloat, 1);
      beginDriveTime = 0;
      driveProgramStart = 0;
      beginCruiseTime = 0; //new
      //Serial.print(" programmed drivetime: ");
      //Serial.println(driveTime);
    } else if ((!PS3usb.getButtonPress(TRIANGLE)) && (PS3usb.getButtonPress(L3)) && (driveProgramStart == 1)) { //PRESS L3 TO CANCEL
      //CANCEL PROGRAMMING
      //driveTime = 0;
      //EEPROM.put(14, driveTime);
      EEPROM.get(14, driveTime);
      driveInterval = !driveInterval;
      beginDriveTime = 0;
      driveProgramStart = 0;
      beginCruiseTime = 0; //new
      triPressed = false;
      xPressed = false;
      cruise = 0;
      //Serial.print(" programmed drivetime: ");
      //Serial.println(driveTime);
    }

    if (PS3usb.getButtonClick(TRIANGLE)) {
      triPressed = !triPressed;  
      if (triPressed == true) {
        cruiseDirL = 1;
        cruiseDirR = 1;
        cruise = 1;
        if (driveInterval == true) {
          displaySetup2x16(0x06);
          if (timeFloat <= 9.9) {
            SerialBT.print(timeFloat, 1);
          } else {
            SerialBT.print(timeFloat, 0);
          }
        } else {
          displayWrite2x16(0x06, "FWD");
        }
        //Serial.println("cruise button on");
      } else if ((triPressed == false) && (!PS3usb.getButtonPress(L3))) {
        cruiseDirL = 0;
        cruiseDirR = 0;
        cruise = 0;
        displayWrite2x16(0x06, "   ");
        //Serial.println("cruise button off");
      }
    }
    
  /*  if(PS3usb.getButtonClick(TRIANGLE)) {
      //Serial.print(" TRIANGLE, Constant speed now forward ");
      cruiseDirL = 1;
      cruiseDirR = 1;
      cruise = 1;
      SerialBT.write(0xFE); 
      SerialBT.write(0x45);
      SerialBT.write(0x06);
      delay(1);
      SerialBT.print("FWD");
    }*/

 //-------------------------X BUTTON: Constant speed reverse------------------------
   /* if(PS3usb.getButtonClick(CROSS)) {
      //Serial.print(" X-BUTTON, Constant speed now reverse ");
      cruiseDirR = -1;
      cruiseDirL = -1;
      cruise = 1;
      SerialBT.write(0xFE); 
      SerialBT.write(0x45);
      SerialBT.write(0x06);
      delay(1);
      SerialBT.print("REV");
      }*/

    if (PS3usb.getButtonClick(CROSS)) {
      xPressed = !xPressed;  
      if (xPressed == true) {
        cruiseDirL = -1;
        cruiseDirR = -1;
        cruise = 1;
        if (driveInterval == true) {
          displaySetup2x16(0x06);
          if (timeFloat <= 9.9) {
            SerialBT.print(timeFloat, 1);
          } else {
            SerialBT.print(timeFloat, 0);
          }
        } else {
          displayWrite2x16(0x06, "REV");
        }
        //Serial.println("Rcruise button on");
      } else if (xPressed == false) {
        cruiseDirL = 0;
        cruiseDirR = 0;
        cruise = 0;
        displayWrite2x16(0x06, "   ");
        //Serial.println("Rcruise button off");
      }
    }
 
//-------------------------R3-L3----DEMO PROGRAM--------------------------------------
    
    if (PS3usb.getButtonClick(R3)) {
      cruise = 1;
      cruiseDirR = 1;
      cruiseDirL = 1;
      displayWrite2x16(0x06, "DM1");
      demoTime = millis();
      demoOn = 1;
    } 
  
    if (demoOn == 1) {
      if ((millis() - demoTime) > 24000) {
        cruiseDirR = cruiseDirR * -1;
        cruiseDirL = cruiseDirL * -1;
        demoTime = millis();
        demo2Time = millis();
        firstTurn = 1;
      }
    }


  /*if (PS3usb.getButtonClick(L3)){
    cruise = 1;
    cruiseDirR = 1;
    cruiseDirL = 1;
    SerialBT.write(0xFE); 
    SerialBT.write(0x45);
    SerialBT.write(0x06);
    delay(1);
    SerialBT.print("DM2");
    demoTime = millis();
    //demo2Time = millis();
    demo2On = 1;
    firstTurn = 0;
  }

  if(demo2On == 1 && firstTurn == 0){
      //Serial.print(" first loop: ");
      //Serial.println(firstTurn);
      if((millis() - demoTime)>20000){
        cruiseDirR = cruiseDirR * -1;
        cruiseDirL = cruiseDirL * 1;
        demo2Time = millis();
        firstTurn = 1;
    }
  }

  if(demo2On == 1 && (firstTurn == 1 || firstTurn == 2)){
    //Serial.print(" second loop: ");
    //Serial.println(firstTurn);
    if((millis() - demo2Time)>5000){
      //Serial.print(" second loop - 2: ");
      //Serial.println(firstTurn);
      cruiseDirR = cruiseDirR * -1;
      cruiseDirL = cruiseDirL * -1;
      //demoTime = millis();
      demo2Time = millis();
      firstTurn = firstTurn + 1;
      if(firstTurn == 3){
        //Serial.print(" second loop - 3: ");
        //Serial.println(firstTurn);
        demoTime = millis();
        firstTurn = 0;
        cruise = 1;
        cruiseDirR = cruiseDirR * 1;
        cruiseDirL = cruiseDirL * -1;
      }
    }
  }*/

//--------FALL PROTECTOR----------------------------------------------------------
    
    //TENSION   
    if (PS3usb.getButtonPress(R2) || vacLow == 0) {                               // L1 = LOWER
      analogWrite(7, FPtorque);                                                   // R1 = LIFT
      //Serial.print(" TENSION ON ");                                             // L2 = SPOOL FREE
    } else {                                                                      // R2 or VAUCUUM > 1.5inHG = TENSION 
      analogWrite(7, 0);                                                          
    }

    //CLUTCH
    if (PS3usb.getButtonPress(R2) || PS3usb.getButtonPress(L2) || vacLow == 0) {
      digitalWrite(3, HIGH);
      //Serial.print(" CLUTCH FREE ");
    } else {
      digitalWrite(3, LOW);
    }

    //BRAKE
    if (PS3usb.getButtonPress(R2) || PS3usb.getButtonPress(L2)
    || PS3usb.getButtonPress(R1) || PS3usb.getButtonPress(L1) 
    || vacLow == 0) {
      digitalWrite(4, HIGH);
      //Serial.print(" BRAKE FREE ");
      if (FPunlocked == 0) { // vacLowChange = 1 means the vacuum has just changed from high to low, or vice versa 
        FPunlocked = 1;
        //Serial.println("unlocked");
        displayWrite4x40(9, 1, "UNLOCK");
      }
    } else {
      digitalWrite(4, LOW);
      if (FPunlocked == 1) { // FPunlocked==1 means yes the FP was previously unlocked
        FPunlocked = 0;
        displayWrite4x40(9, 1, "LOCKED");
      }
    }

    //LL MOTOR LIFT
    if (PS3usb.getButtonPress(R1)) {   
      digitalWrite(6, HIGH);                                    
      //Serial.print(" LL MOTOR LIFT ");                        

    } else {
      digitalWrite(6, LOW);
    }

    //LL MOTOR LOWER
    if (PS3usb.getButtonPress(L1)) {
      digitalWrite(5, HIGH);
      //Serial.print(" LL MOTOR LOWER ");

    } else {
      digitalWrite(5, LOW);
    }
 
  } //bracket for unplugged controller routine, (PS3usbConnected)





//******************************************* PROGRAMMING MODE ************************************************************************

  while (programMode == true) {
    
    if (firstProg == true) {                                              // stuff to do once the first time you enter programming mode 
      
      Serial1.println("M1: 0");                                           // turn off the drive motors
      Serial1.println("M2: 0");
      cruise = 0;

      Serial1.println("M3: 0");                                           // turn off the Z-Lift motor
      
      Serial1.println("M4: 0");                                           // turn off the tool
      toolOn = 0;
      
      digitalWrite(3, LOW);                                               // lock FP clutch
      digitalWrite(4, LOW);                                               // lock FP brake
      digitalWrite(5, LOW);                                               // turn LL motor off
      digitalWrite(6, LOW);                                               // turn off the LL motor
      analogWrite(7, 0);                                                  // turn off the LL motor
      
      
      Serial2.write(0xFE);                                                // clear the LCD screen
      Serial2.write(0x58);                                             
      delay(10);   

      displayWrite4x40(9, 1, "PROGRAM MODE");
      
      displayVacuum();                                                    // write the proper units and vacuum reading to the LCD

      if (inHg == true) {
        savedEntries[0] = "inHg"; 
      } else {
        savedEntries[0] = "kPa ";
      }

      savedEntries[1] = getVacString(tripPointOne);

      savedEntries[2] = getVacString(tripPointTwo);

      if (variVacuum == true) {
        savedEntries[3] = "VARI"; 
      } else {
        savedEntries[3] = "NORM";
      }

      savedEntries[4] = torqueScale;


      hours = totalVacTime / (60 * 60);                                   // here we turn to total vacuum time into a string and load
      minutes = (totalVacTime % (60 * 60)) / (60);                        // it into savedEntries[5]

      if (hours < 10) {                                                   // this is some formatting so that it comes out nice, format is 
        hourString = "00" + String(hours);                                // [hours] [hours] [hours] [:] [minutes] [minutes]
      } else if (hours < 100) {
        hourString = "0" + String(hours);
      } else {
        hourString = String(hours);
      }

      if (minutes < 10) {
        minuteString = "0" + String(minutes);
      } else {
        minuteString = String(minutes);
      }

      savedEntries[5] = hourString + ":" + minuteString;                  // and here we finally load in the formatted vacuum time
                                                                          
      
      menuIndex = 0;                                                      // start from the first item

      menuPage = 0;                                                       // on the first page

      for (int i = 0; i < 3; i ++) {                                      
        displayWrite4x40(9, i + 2, menuItems[i + (menuPage * 3)]);        // show the first page of menu items, each page is three items
        displayWrite4x40(15, i + 2, savedEntries[i + (menuPage * 3)]);    // show the first page of menu entries
      }

      displayWrite4x40(8, 2, "*");                                        // and show the editable slot with the asterisk

      firstProg = false;                                                  // it's not the first time through the programming mode loop
                                                                          // anymore, so skip the initialization next time
    
    }                                                                     // bracket for programming mode initialization



//************************************************ EXIT PROGRAMMING MODE ****************************************************************************************
    
    if (progEnterExit == true) {                                          // if you are allowed (allowed means you are not coming straight from entering/exiting)
      if (PS3.getButtonPress(START) || PS3usb.getButtonPress(START)) {    // you can enter or exit programming mode by pressing and holding START for 5 seconds
        if (progTimerStart == true) {                                     // if you just started pressing START
          programTime = millis();                                         // begin the timer
          progTimerStart = false;
        }
        
        if (millis() - programTime >= 5000) {                             // if you've been pressing it for 5 seconds    
          programMode = !programMode;                                     // enter or exit programming mode
          progEnterExit = false;                                          // this forbids entering or exiting programming mode again, so we don't oscillate back and forth between modes
          displayInit4x40();                                              // when exiting programming mode, initialize the LCD for normal operation
        }
      } 
    } 
    
    if (!PS3.getButtonPress(START) && !PS3usb.getButtonPress(START)) {    // it you are not pressing START
      progTimerStart = true;                                              // the next time you press it the timer will start over
      progEnterExit = true;                                               // and you are allowed to enter or exit programming mode   
    }

//*******************************************************************************************************************************************************************************************
    
    Usb.Task();                                                           // PS3 get info command
    PS3.attachOnInit(BTonInit);
    PS3usb.attachOnInit(USBonInit);
    SerialBT.attachOnInit(SerialBTonInit);

    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\n PS DISCONNECT! "));
      PS3.disconnect();
      connectOnce = 0;
    }

    if (PS3.PS3Connected == true) {
      setLEDs(); 
    } else {
      LedState = 0;
    }

    vacuumControl();                                                      // vacuum is controllable while in programming mode
    
    getVacuum();                                                          // get the vacuum level and write it to the LCD ***DOES NOT WORK WHILE IN PROGRAMMING MODE, WHY?***

    if (PS3.getButtonClick(DOWN) || PS3usb.getButtonClick(DOWN)) {        // increment the menu index
    
      menuIndex ++;
      if (menuIndex >= menuLength) {
        menuIndex = menuLength - 1;                                       // (this stops the asterisk at the bottom of the list)
      }
      moveSlot = true;                                                    // and print the asterisk in the correct spot
    }

    if (PS3.getButtonClick(UP) || PS3usb.getButtonClick(UP)) {            // decrement the menu index
      menuIndex --;
      if (menuIndex <= 0) {
        menuIndex = 0;                                                    // (this stops the asterisk at the top of the list)
      }
      moveSlot = true;                                                    // and print the asterisk in the correct spot
    }


    if (moveSlot == true) {                                               // if the editable slot has moved

      editing = false;                                                    // don't immediately start editing the new slot, 
                                                                          // only start editing after you change a parameter 
                                                                          // with the LEFT and RIGHT buttons 
      
      inHgTemp = inHg;                                                    // force all temporary variables to be the same as
      kPaTemp = kPa;                                                      // their saved variable counterparts, this resets
      tripPointOneTemp = tripPointOne;                                    // them if you have changed their temporary value 
      tripPointTwoTemp = tripPointTwo;                                    // but not saved that value
      variVacuumTemp = variVacuum;
      torqueScaleTemp = torqueScale;
      
      
      for (int i = 0; i < 3; i ++) {
        displayWrite4x40(8, i + 2, " ");                                  // clear the asterisk
        displayWrite4x40(15, i + 2, savedEntries[i + (menuPage * 3)]);    // show the correct page of menu entries     
      }
      
      displayWrite4x40(8, (menuIndex % 3) + 2, "*");                      // and show the editable slot with the asterisk     
      moveSlot = false;
    
    }

  
    menuPage = menuIndex / 3;                                             // each page has three items 

    if (menuPage != lastPage) {                                           // if the page has changed, fill in the new page
      lastPage = menuPage;
      
      for (int c = 14; c <= 20; c ++) {                                   // clear out the field of entries
        for (int r = 2; r <= 4; r ++) {
          displayWrite4x40(c, r, " ");
        }
      }
      
      for (int i = 0; i < 3; i ++) {                                      
        displayWrite4x40(9, i + 2, menuItems[i + (menuPage * 3)]);        // show the correct page of menu items, each page is three items
        displayWrite4x40(15, i +2, savedEntries[i + (menuPage * 3)]);     // show the correct page of menu entries 
      }
    }





    if (editing == true) {                                                // if you are editing,
      if (millis() - editBlinkTime > 333) {                               // make the parameter you
        editBlinkTime = millis();                                         // are editing blink on and
        editBlink = !editBlink;                                           // off three times per second
      } 
      
      if (editBlink == true) {
        displayWrite4x40(15, (menuIndex % 3) + 2, "    ");                      // here it blinks off
      } else {
        displayWrite4x40(15, (menuIndex % 3)+ 2, tempEntries[menuIndex]);      // here it blinks on
      }
    }

    

//***************************** EDIT VACUUM UNITS ********************************************************************

    if (menuIndex == 0) {                                                 // the item at index 0 is vacuum units
      
      if (PS3.getButtonClick(RIGHT) || PS3.getButtonClick(LEFT)           // if you press LEFT or RIGHT
        || PS3usb.getButtonClick(RIGHT) || PS3usb.getButtonClick(LEFT)) {

        editing = true;                                                   // you are now editing this parameter
        entrySaved = false;                                               // but you have not saved it yet
        
        inHgTemp = !inHgTemp;                                             // change from inHg to kPa and vice versa
        kPaTemp = !kPaTemp;                                               // every time you press LEFT or RIGHT
        
        if (inHgTemp == true) {
          tempEntries[0] = "inHg";                                        // enter the new units into the array of temporary entries
        } else {
          tempEntries[0] = "kPa ";
        }
      
        displayWrite4x40(15, 2, tempEntries[0]);                          // and update it to the screen
      
      }


      if (editing == true && 
         (PS3.getButtonClick(CROSS) || 
          PS3usb.getButtonClick(CROSS))) {                                // here is where we save the parameter 

        if (tempEntries[0] != savedEntries[0]) {                          // if you have changed the units                     
          
          if (inHg == false) {
            tripPointOneFloat = ((tripPointOne / 10.0) / unitMult) + 0.05;// do some math to convert the trip points, here we're going from kPA to inHg
            tripPointTwoFloat = ((tripPointTwo / 10.0) / unitMult) + 0.05; 
          } else if (kPa == false) {
            tripPointOneFloat = (tripPointOne / 10.0) * unitMult;         // or vica versa, here we're going from inHg to kPa
            tripPointTwoFloat = (tripPointTwo / 10.0) * unitMult;              
          }                                                               // kPa to inHg uses slightly different multiplier to avoid truncation errors
                                                                          
                                                                          // we do math to the "temp" versions so that the normal versions
                                                                          // (which are saved to the EEPROM) don't need to be large variables 

          inHg = inHgTemp;                                                // then update the boolean variables that dictate which units we're using
          kPa = kPaTemp;

          tripPointOneFloat *= 10;                                        // move the decimal one place to the right
          tripPointTwoFloat *= 10;

          tripPointOne = (int)tripPointOneFloat;                          // here we convert the floating point versions into integers, with the
          EEPROM.put(24, tripPointOne);
          tripPointTwo = (int)tripPointTwoFloat;                          // decimal moved over one place to the right, this makess it easier to 
          EEPROM.put(28, tripPointTwo);                                   // print these numbers to the LCD and use them for comparisons when checking
                                                                          // the trip points
                                                                          
          savedEntries[0] = tempEntries[0];                               // we load the string version of the new units ("inHg" or "kPa ") into the array of saved parameters

          EEPROM.put(22, inHg);
          EEPROM.put(23, kPa);
         
          savedEntries[1] = getVacString(tripPointOne);                   // and also load the modified trip point one
          tempEntries[1] = savedEntries[1];
          
          savedEntries[2] = getVacString(tripPointTwo);                   // and modified trip point two
          tempEntries[2] = tempEntries[2];
          
          displayVacuum();                                                // write the new units and vacuum reading to the LCD
        }

        displayWrite4x40(15, 2, savedEntries[0]);                         // write the units to the LCD
        displayWrite4x40(15, 3, savedEntries[1]);                         // and write the trip points in
        displayWrite4x40(15, 4, savedEntries[2]);                         // the correct units to the LCD
        editing = false;                                                  // we are now done editing this parameter

      }
    
    }



//***************************** EDIT TRIP POINT ONE ********************************************************************

    if (menuIndex == 1) {                                                 // the item at index 1 is trip point 1, the warning trip point
      
      if (PS3.getButtonClick(RIGHT) || PS3usb.getButtonClick(RIGHT)) {    // tapping RIGHT raises the trip point
        
        editing = true;
        
        tripPointOneTemp ++; 
        //if (tripPointOneTemp >= 30) {                                     // but don't go higher than 30 (30 arbitrarily chosen, what should it really be?)
          //tripPointOneTemp = 30;
        //}
      
        tempEntries[1] = getVacString(tripPointOneTemp);                  // enter the string version into the array of temporary entries
      
        displayWrite4x40(15, 3, tempEntries[1]);                          // and update it to the screen after it changes
      
      }
    
      if (PS3.getButtonClick(LEFT) || PS3usb.getButtonClick(LEFT)) {      // tapping LEFT lowers the trip point
        
        editing = true;
        
        tripPointOneTemp --; 
        if (tripPointOneTemp <= tripPointTwo) {                           // but don't go lower than trip point two
          tripPointOneTemp = tripPointTwoTemp + 1;
        }
      
        tempEntries[1] = getVacString(tripPointOneTemp);                  // enter the string version into the array of entries
      
        displayWrite4x40(15, 3, tempEntries[1]);                          // and update it to the screen after it changes
      
      }

      if (editing == true && 
         (PS3.getButtonClick(CROSS) || 
          PS3usb.getButtonClick(CROSS))) {                                // here is where we save the parameter 
        
        editing = false;
        displayWrite4x40(15, 3, tempEntries[1]);
        tripPointOne = tripPointOneTemp;
        savedEntries[1] = tempEntries[1];
        
        EEPROM.put(24, tripPointOne);
        
      }

    }




//***************************** EDIT TRIP POINT TWO ********************************************************************

    if (menuIndex == 2) {                                                 // the item at index 2 is trip point 2, the FP lock trip point
      
      if (PS3.getButtonClick(RIGHT) || PS3usb.getButtonClick(RIGHT)) {    // tapping RIGHT raises the trip point
        
        editing = true;
        
        tripPointTwoTemp ++; 
        if (tripPointTwoTemp >= tripPointOne) {                           // but don't go higher than trip point one
          tripPointTwoTemp = tripPointOneTemp - 1;
        }
     
        tempEntries[2] = getVacString(tripPointTwoTemp);                  // enter the string version into the array of temporary entries   
      
        displayWrite4x40(15, 4, tempEntries[2]);                          // and update it to the screen after it changes
      }
    
      if (PS3.getButtonClick(LEFT) || PS3usb.getButtonClick(LEFT)) {      // tapping LEFT lowers the trip point
        
        editing = true;
        
        tripPointTwoTemp --; 
        //if (tripPointTwoTemp <= 10) {                                     // but don't go lower than 10 (10 arbitrarily chosen, what should it really be?)
          //tripPointTwoTemp = 10;
        //}
      
        tempEntries[2] = getVacString(tripPointTwoTemp);                  // enter the string version into the array of entries   
      
        displayWrite4x40(15, 4, tempEntries[2]);                          // and update it to the screen after it changes
      
      }

      if (editing == true && 
         (PS3.getButtonClick(CROSS) || 
          PS3usb.getButtonClick(CROSS))) {                                // here is where we save the parameter 
        
        editing = false;
        displayWrite4x40(15, 4, tempEntries[2]);
        tripPointTwo = tripPointTwoTemp;
        savedEntries[2] = tempEntries[2];
        
        EEPROM.put(28, tripPointTwo);
      
      }
    
    }


//*************************************** CHANGE VACUUM TYPE, VARIABLE OR NORMAL ***************************************************************************


    if (menuIndex == 3) {                                                 // the item at index 3 is vacuum type, either variable or normal
        
      if (PS3.getButtonClick(RIGHT) || PS3.getButtonClick(LEFT)           // if you press LEFT or RIGHT
      || PS3usb.getButtonClick(RIGHT) || PS3usb.getButtonClick(LEFT)) {
  
        editing = true;                                                   // you are now editing this parameter
        entrySaved = false;                                               // but you have not saved it yet
        
        variVacuumTemp = !variVacuumTemp;                                 // flip the temporary version
        
        if (variVacuumTemp == true) {
          tempEntries[3] = "VARI";                                        // enter the new type of vacuum into the array of temporary entries
        } else {
          tempEntries[3] = "NORM";
        }
      
        displayWrite4x40(15, 2, tempEntries[3]);                          // and update it to the screen
    
      }
  
  
      if (editing == true && 
         (PS3.getButtonClick(CROSS) || 
          PS3usb.getButtonClick(CROSS))) {                                // here is where we save the parameter 
  
        variVacuum = variVacuumTemp;                                      // we update the boolean variable      
        EEPROM.put(32, variVacuum);
                                                                                              
        savedEntries[3] = tempEntries[3];                                 // and load the string version of the new vacuum type ("VARI" or "NORM") into the array of saved parameters

        displayWrite4x40(15, 2, savedEntries[3]);                               
        editing = false;                                                  // we are now done editing this parameter
  
      }
    
    }


//*************************************** EDIT FALL PROTECTOR TORQUE SCALING, TURNS UP/DOWN MAX TORQUE ***************************************************************************
    
if (menuIndex == 4) {                                                     // the item at index 4 is torqueScale, turns up/down max FP torque
      
      if (PS3.getButtonClick(RIGHT) || PS3usb.getButtonClick(RIGHT)) {    // tapping RIGHT raises the max torque
        
        editing = true;
        
        torqueScaleTemp ++; 
        if (torqueScaleTemp >= 10) {                                      // but don't go higher than 100
          torqueScaleTemp = 10;
        }
     
        tempEntries[4] = torqueScaleTemp;
      
        displayWrite4x40(15, 3, tempEntries[4]);                          // and update it to the screen after it changes
      }
    
      if (PS3.getButtonClick(LEFT) || PS3usb.getButtonClick(LEFT)) {      // tapping LEFT lowers the max torque
        
        editing = true;
        
        torqueScaleTemp --; 
        if (torqueScaleTemp <= 0) {                                       // but don't go lower than 25 (25 is arbitrary number, it shouldn't be allowed 
          torqueScaleTemp = 0;                                            // to go too low, not sure what it should actually be)
        }
      
        tempEntries[4] = torqueScaleTemp;                                 // enter the string version into the array of entries   
      
        displayWrite4x40(15, 3, tempEntries[4]);                          // and update it to the screen after it changes
      
      }

      if (editing == true && 
         (PS3.getButtonClick(CROSS) || 
          PS3usb.getButtonClick(CROSS))) {                                // here is where we save the parameter 
        
        editing = false;
        displayWrite4x40(15, 3, tempEntries[4]);
        torqueScale = torqueScaleTemp;
        EEPROM.put(33, torqueScale);
        savedEntries[4] = tempEntries[4];
            
      }
      
    }                                                                     // end bracket for menuIndex == 4, FP torque scaling
 
  }                                                                       // end bracket for programming mode while loop

}                                                                         // end bracket for main loop





void vacuumControl() {                                // checks the vacuum switch and controlls the vacuum
  
  vacSwitchOn = digitalRead(2);                       // check the vacuum enable switch
  
  if (vacSwitchOn == HIGH) {
    if (variVacuum == true) {                         // it it's a variable vacuum
      vacScaled = (analogRead(vacPot) * 2) - 800;           
      Serial1.print("P4: ");                          // send the scaled value from the vacuum level pot to aux sabertooth P4
      Serial1.println(vacScaled);                     // (P2 on aux sabertooth, using 3-4 address change)to control the vacuum via an opamp
    
    } else {                                          // but if it's a non variable vacuum
      Serial1.print("P4: ");
      Serial1.println(2047);                          // just send 2047 to open a relay
    }
  } else {                        
    Serial1.print("P4: ");                            // turn the vacuum off
    Serial1.println(-2047);                           // -2047 will turn off either a variable or non variable vacuum
  }
}


void BTonInit() { //runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" BT CONNECTED ");
}



void SerialBTonInit() { //runs once at moment of controller initialization, sets up and writes current values to LCD
  
  //LedState = 0;

  Serial.print(" Serial LCD CONNECTED ");
}


//******************************* 4 X 40 LCD INITIALIZATION FUNCTION ******************************************************* 

void displayInit4x40() {                                      // this initializes the LCD with the stuff that always stays,
                                                              // and fills in speed/torque/vacuum/etc. values
                                                              // called when first plugging in the LCD, and when exiting programming mode
//---------- SETUP LCD

  Serial2.write(0xFE);
  Serial2.write(0xD1);
  Serial2.write(20);  // 20 columns
  Serial2.write(4);   // 4 rows
  delay(10);       
  // we suggest putting delays after each command to make sure the data 
  // is sent and the Serial2 is updated.

  // set the contrast, 200 is a good place to start, adjust as desired
  Serial2.write(0xFE);
  Serial2.write(0x50);
  Serial2.write(230);
  delay(10);       
  
  // set the brightness - we'll max it (255 is max brightness)
  Serial2.write(0xFE);
  Serial2.write(0x99);
  Serial2.write(255);
  delay(10);       
  
  // turn off cursors
  Serial2.write(0xFE);
  Serial2.write(0x4B);
  Serial2.write(0xFE);
  Serial2.write(0x54);

  //DISABLE AUTOSCROLL SO LAST CHARACTER WILL DISPLAY CORRECTLY  
  Serial2.write(0xFE);
  Serial2.write(0x52);



  //+++++++++++++++++++++++++++++++ CREATE THE FRAGMENTS WHICH MAKE UP THE BIG DIGITS +++++++++++++++++++++++++++++++++++++++++++++++++++++

  // GREEN CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)0);     // location #0
  Serial2.write((uint8_t)0x7);  // 8 bytes of character data
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write((uint8_t)0x0);
  delay(10);   // we suggest putting delays after each command 

  // LIGHT GREEN CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)1);     // location #0
  Serial2.write((uint8_t)0x1C);  // 8 bytes of character data
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write((uint8_t)0x1C);
  delay(10);   // we suggest putting delays after each command 

  //RED CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)2);     // location #0
  Serial2.write((uint8_t)0x1F);  // 8 bytes of character data
  Serial2.write(0x1F);
  Serial2.write(0x1F);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write((uint8_t)0x0);
  delay(10);   // we suggest putting delays after each command 

  //PURPLE CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)3);     // location #0
  Serial2.write((uint8_t)0x1F);  // 8 bytes of character data
  Serial2.write(0x1F);
  Serial2.write(0x1F);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write((uint8_t)0x7);
  delay(10);   // we suggest putting delays after each command 

  //BLUE CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)4);     // location #0
  Serial2.write((uint8_t)0x1F);  // 8 bytes of character data
  Serial2.write(0x1F);
  Serial2.write(0x1F);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write(0x1C);
  Serial2.write((uint8_t)0x1C);
  delay(10);   // we suggest putting delays after each command 

  //YELLOW CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)5);     // location #0
  Serial2.write((uint8_t)0x7);  // 8 bytes of character data
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write(0x7);
  Serial2.write((uint8_t)0x7);
  delay(10);   // we suggest putting delays after each command 

  //LIGHT BLUE CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)6);     // location #0
  Serial2.write((uint8_t)0xF);  // 8 bytes of character data
  Serial2.write(0xF);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write(0x0);
  Serial2.write((uint8_t)0x0);
  delay(10);   // we suggest putting delays after each command 

  //ORANGE CHARACTER FRAGMENT
  Serial2.write(0xFE);
  Serial2.write(0x4E);
  Serial2.write((uint8_t)7);     // location #0
  Serial2.write((uint8_t)0xF);  // 8 bytes of character data
  Serial2.write(0xF);
  Serial2.write(0xC);
  Serial2.write(0xC);
  Serial2.write(0xC);
  Serial2.write(0xC);
  Serial2.write(0xC);
  Serial2.write((uint8_t)0xC);
  delay(10);   // we suggest putting delays after each command 


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


  // CLEAR SCREEN
  Serial2.write(0xFE);
  Serial2.write(0x58);                                             
  delay(10);   // we suggest putting delays after each command 



  displayVacuum();                                            // write the proper units and vacuum reading to the LCD
                                                
  displayWrite4x40(16, 1, "FP:");                             // fill in the things that always stay

  displayWrite4x40(9, 2, "ZLIFT:");

  displayWrite4x40(9, 3, "SPEED:");

  displayWrite4x40(9, 4, "DRIVE: ");


  
  //---------- WRITE FP LOCK STATE

  if (vacLow == true) {                                       // vacLow = true means low vacuum
    displayWrite4x40(9, 1, "LOCKED");  
    vibrate = false;
  } else if (vacLow == false) {                               // vacLow = false means yes the vacuum is > tripPointTwo
    displayWrite4x40(9, 1, "      "); 
  }


  //---------- WRITE TORQUE VALUE

  FPtorque = (analogRead(torquePot) * 10) / 103;              // check the torque pot, scale it to 0 to 99
    
  LCDtorqueString = String(FPtorque);                         // turns the value into a string so we can write it to the LCD

  if (FPtorque >= 10) {                                       // if the value is >= 10 write both digits
    displayWrite4x40(19, 1, LCDtorqueString);
  } else {                                                    // if the value is < 10 write a 0 first to keep the formatting nice
    displayWrite4x40(19, 1, "0" + LCDtorqueString);
  } 


  //---------- WRITE SPEED VALUE

  LCDspeed = (analogRead(speedPot) * 10) / 103;               // check the speed pot, scale it to 0 to 99
  
  LCDspeedString = String(LCDspeed);                          // turn the value into a string so we can write it to the LCD
    
  if (LCDspeed >= 10) {                                       // if the value is >= 10 write both digits
    displayWrite4x40(15, 3, LCDspeedString);
  } else {                                                    // if the value is < 10 write a 0 first to keep the formatting nice
    displayWrite4x40(15, 3, "0" + LCDspeedString);
  } 
  



  //---------- WRITE DRIVE TYPE
  
  if (proportionalSpeed == true) {                                       
    displayWrite4x40(15, 4, "ANALOG");
  } else {                                                    
    displayWrite4x40(15, 4, "ON/OFF");
  } 

}    
  


void USBonInit() { //runs once at moment of controller initialization
  LedState = 0;
  Serial.print(" USB CONNECTED ");
}



//***************************** LCD FUNCTIONS FOR NEW HAVEN 2 X 16 SERIAL LCD ********************************************************************

void displayWrite2x16(int screenPosition, String message) {  // prepares and writes to the wireless display, WORKS WITH NEW HAVEN 2 X 16 LCD
/*  
  if (LCDwired) {
    Serial2.write(0xFE);                                     // Command
    Serial2.write(0x45);                                     // Move cursor to
    Serial2.write(screenPosition);                           // Position
    delay(1);                                                // short delay may be required
    Serial2.print(message);                                  // and here we write the actual message
  } else {
    SerialBT.write(0xFE);                                    // this is the section where the
    SerialBT.write(0x45);                                    // LCD is not plugged into the
    SerialBT.write(screenPosition);                          // Control Station, and communicates 
    delay(1);                                                // via bluetooth
    SerialBT.print(message);                                 
  } 
*/
}



void displaySetup2x16(int screenPosition) {                  // just prepares to write to the wireless display, WORKS WITH NEW HAVEN 2 X 16 LCD
/*
  if (LCDwired) {
    Serial2.write(0xFE);                                     // Command
    Serial2.write(0x45);                                     // Move cursor to
    Serial2.write(screenPosition);                           // Position
    delay(1);                                                // short delay may be required
  } else {
    SerialBT.write(0xFE);                                    // this is the section where the
    SerialBT.write(0x45);                                    // LCD is not plugged into the
    SerialBT.write(screenPosition);                          // Control Station, and communicates 
    delay(1);                                                // via bluetooth
  }                                 
*/
}

//****************************************************************************************************************************************************



//***************************** LCD FUNCTIONS FOR NEW HAVEN 4 X 40 LCD WITH ADAFRUIT BACKPACK ********************************************************

void displayWrite4x40(int column, int row, String message) { // prepares and writes to the wireless display, WORKS WITH NEW HAVEN 4 x 20 LCD
  
  if (LCDwired) {
    Serial2.write(0xFE);                                     // Command
    Serial2.write(0x47);                                     // Move cursor to
    Serial2.write(column);                                   // column
    Serial2.write(row);                                      // row
    delay(1);                                                // short delay may be required
    Serial2.print(message);                                  // and here we write the actual message
  } else {
    SerialBT.write(0xFE);                                    // this is the section where the
    SerialBT.write(0x47);                                    // LCD is not plugged into the
    SerialBT.write(column);                                  // Control Station, and communicates 
    SerialBT.write(row);                                     // via bluetooth
    delay(1);                                                 
    SerialBT.print(message);  
  } 
}



void displaySetup4x40(int column, int row) {                 // just prepares to write to the wireless display, WORKS WITH NEW HAVEN 4 x 20 LCD
  
  if (LCDwired) {
    Serial2.write(0xFE);                                     // Command
    Serial2.write(0x47);                                     // Move cursor to
    Serial2.write(column);                                   // column
    Serial2.write(row);                                      // row
    delay(1);                                                // short delay may be required
  } else {
    SerialBT.write(0xFE);                                    // this is the section where the
    SerialBT.write(0x47);                                    // LCD is not plugged into the
    SerialBT.write(column);                                  // Control Station, and communicates 
    SerialBT.write(row);                                     // via bluetooth
    delay(1);                                                                                
  } 
}

//**********************************************************************************************************************************************************





//*************************************** FUNCTION TO CALL BIG DIGIT FUNCTIONS ************************************************************************************

void bigDigit(int value) {                // we pass the value of the big digit to be written,
  switch (value) {                        // which calls one of ten functions that actually prints
    case 0:                               // the big digit
      bigZero(bigDigitPlace);             // we pass the place value, which is a global variable
      break;                              // to whichever individual digit is actually called
    case 1:    
      bigOne(bigDigitPlace);
      break;
    case 2:    
      bigTwo(bigDigitPlace);
      break;
    case 3:    
      bigThree(bigDigitPlace);
      break;
    case 4:    
      bigFour(bigDigitPlace);
      break;
    case 5:    
      bigFive(bigDigitPlace);
      break;
    case 6:    
      bigSix(bigDigitPlace);
      break;
    case 7:    
      bigSeven(bigDigitPlace);
      break;
    case 8:    
      bigEight(bigDigitPlace);
      break;
    case 9:    
      bigNine(bigDigitPlace);
      break; 
  }
}

//*************************************************************************************************************************************************************







//************************ FUNCTIONS FOR PRINTING BIG NUMBERS TO THE LCD ***********************************************************************************

 
void bigZero(int digit) {       // prints a big old 0 to the LCD 
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);         // we pass it information about which column to start at
  Serial2.write(1);             // the rows don't change
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)1);
  Serial2.write((uint8_t)5);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigOne(int digit) {         
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)0);
  Serial2.write((uint8_t)1);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write(" ");
  Serial2.write((uint8_t)1);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)0);
  Serial2.write((uint8_t)2);
}


void bigTwo(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)2);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigThree(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)0);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigFour(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)1); //four[0]
  Serial2.write((uint8_t)5);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)2); //four[1]
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write(" "); //four[2]
  Serial2.write((uint8_t)0);
}

void bigFive(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)2);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigSix(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)1);
  Serial2.write(" ");
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigSeven(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write(" ");
  Serial2.write((uint8_t)5);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write(" ");
  Serial2.write((uint8_t)0);
}

void bigEight(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)2);
}

void bigNine(int digit) {
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit); //col
  Serial2.write(1);  //row
  Serial2.write((uint8_t)4);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(2);
  Serial2.write((uint8_t)2);
  Serial2.write((uint8_t)3);
  Serial2.write(0xFE);
  Serial2.write(0x47);
  Serial2.write(digit);
  Serial2.write(3);
  Serial2.write(" ");
  Serial2.write((uint8_t)0);
}


void bigDecimalPoint(int spot) {                  // make a big decimal pont in the correct spot depending on if you are inHg or kPa
  Serial2.write(0xFE);                            // on if you are working in inHg or kPa
  Serial2.write(0x47);
  Serial2.write(spot);                            // we pass it information about which column to start at
  Serial2.write(3);                               //the row never changes
  Serial2.write((uint8_t)0);
}


//************************************************************************************************************************************************







void setLEDs() {

  //PS3.printStatusString();
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



//********************************** FUNCTION TO GET VACUUM LEVEL AND PRINT IT TO LCD *************************************************************

void getVacuum() {                                    // asks the climber for the value of the vacuum sensor, averages
                                                      // the last eight sensor reading, and then prints the vacuum level
                                                      // to the LCD in either inHg or kPa, in great big digits

  //-----------------------REQUEST FOR VACUUM SENSOR READING-------------------------------

  if (millis() - lastA1 > 100) {                      // only ask for A1 10 times per second
      lastA1 = millis();
      Serial1.println("A1: get");                     // ask main sabertoth for value of the vacuum sensor at A1                          
  }                                                   // if successful, "A1; get" returns vacSense, an int proportional to vacuum pressure

  while (Serial1.available()) {                               // this is serial information from the Sabertooth representing the value of the vacuum sensor

    char inChar = Serial1.read();                             // get the new byte
    
    inputString += inChar;                                    // add it to the inputString
    
    if (inChar == '\n') {                                     // a new line appears, string is done!
      
      if (inputString.startsWith("A1: ") &&                   // if string is not gibberish, congratulations, it's a valid A1!
      inputString.substring(3).toInt() <= 2047 &&             // A1: value of 2000 is a vac display reading of about 0.0 inHg
      inputString.substring(3).toInt() >= 10) {               // A1: value of 14 is a vac display reading of about 8 inHg
        
        vacSense = inputString.substring(3).toInt();          // strip away non useful part of string and convert it to an integer
      
        /*if(runTimeStart == 0){
          machineRunTime = millis();
          runTimeStart = 1;
        }*/
              
        lastValidA1 = millis();                               // note when the last valid A1 reading happened
        
        if (vacSense <= 1700) {                               // here we record how long the machine has been vacuumed to a surface
          if (vacStart == 0) {                                // this is when the vacuuming starts
            machineVacTime = millis();
            vacStart = 1;
          }                                          
        } else if (vacSense > 1700) {                         // 1 inHg = 1600
          if (vacStart == 1) {                                // this is when the vacuuming stops
            //Serial.println("saving vacTime");
            machineVacTime = millis() - machineVacTime;
            EEPROM.get(18, totalVacTime);
            totalVacTime += machineVacTime/1000;              // here we add up the total time sucking has happened
            EEPROM.put(18, totalVacTime);                     // and store it in the EEPROM
            //Serial.print("Saving. VacTimeTotal: ");
            //Serial.println(totalVacTime);
            vacStart = 0;
          }
        }
      }
      
      inputString = "";                                       // clear the string after a newline appears
   
    }                                                         // this is the bracket for the "if (inChar == '\n') {" line
  }                                                           // this is the bracket for the "while (Serial1.available()) {" line
                                                             // this is the bracket for the start of the "serialEvent1()" function

  if (millis() - lastValidA1 >= 500) {                // if it's been more that 500 mS since a valid A1 reading
    vacSense = 2000;                                  // write 0.0 inHg/kPa to the remote and wireless display
  }    

  sensorArray[sensorIndex] = vacSense;                // load the last 8 sensor readings into an array
  sensorIndex ++;                                    

  if (sensorIndex == 8) {                             // once the array is full
    sensorAverage = 0;
    for (int i = 0; i < 8; i ++) {
      sensorAverage += sensorArray[i];                // add up the values
    }
    sensorAverage = sensorAverage >> 3;               // and divide by 8 to get the average (n >> 3 is equivalent to n/8, but faster)
    sensorIndex = 0;                                  // reset the index
  }

  if (sensorAverage > 2000) {                         // sensorAverage > 2000 means inHg/kPa are negative,
    sensorAverage = 2000;                             // ignore any negative readings
  }

/*
  vacPWM = map(vacSense, 2000, 10, 58, 124);          // A1/vacSense is "upside down", 2000 = ~0 inHG, 10 = ~8 inHG. Map function scales A1/vacSense for PWM
      
      //Serial.print("          PWM VALUE IS:   ");   // we don't need this if using an LCD display only, but we should
      //Serial.println(vacPWM);                       // keep it around, as it may be useful for checking calibration
      
      analogWrite(13, vacPWM);                        // PWM out to vacuum display.
*/    




//--------------------------------VAC SENSOR LCD PRINTOUT------------------------------ 

  if (sensorAverage != lastSensor) {                          // only write to the LCD if the value has changed 

    lastSensor = sensorAverage;
  
    if (inHg == true) {                                       // working in inHg

      inHgCheck = map(sensorAverage, 2000, 14, 0, 80);        // find slope of line between sensorAverage and inhg, inHgCheck 
      vacCheck = inHgCheck;                                   // is an integer version of inHg with no decimal point, 
                                                              // (1.8 inHg = 18, for example) so we can strip out ones and tenths
        
      sensorOnes = inHgCheck / 10;                            // get the ones component, for writing big numbers to the LCD
      inHgCheck -= (sensorOnes * 10);
      sensorTenths = inHgCheck;                               // get the tenths component, for writing big numbers to the LCD    

      if (sensorOnes != lastOnes) {                           // print the ones value, but only if it has changed
        lastOnes = sensorOnes;
        bigDigit(sensorOnes);                                 // call the big digit function with the ones value
      }
      
      bigDigitPlace += 3;                                     // move the place up so the tenths come out in the right spot 
      
      
      if (sensorTenths != lastTenths) {                       // print the tenths value, but only if it has changed
        lastTenths = sensorTenths;
        bigDigit(sensorTenths);                               // call the big digit function with the tenths value
      }
            
      bigDigitPlace = 1;                                      // reset the place so next time you start in the right spot
    
    } else if (kPa == true) {                                 // working in kilopascals

      kPaCheck = map(sensorAverage, 2000, 14, 0, 271);        // find slope of line between sensorAverage and kPa, kPaCheck 
      vacCheck = kPaCheck;                                    // is an integer version of kPa with no decimal point, 
                                                              // (21.8 inHg = 218, for example) so we can strip out tens, ones and tenths
                                                                                  
      sensorTens = kPaCheck / 100;                            // get the tens component, for writing big numbers to the LCD  
      kPaCheck -= (sensorTens * 100);
      sensorOnes = kPaCheck /10;                              // get the ones component, for writing big numbers to the LCD
      kPaCheck -= (sensorOnes * 10);
      sensorTenths = kPaCheck;                                // get the tenths component, for writing big numbers to the LCD  

      
      if (sensorTens != lastTens) {                           // print the tens value, but only if it has changed
        lastTens = sensorTens;
        bigDigit(sensorTens);                                 // call the big digit function with the tens value
      }
      
      bigDigitPlace += 2;                                     // move the place up so the ones come out in the right spot 
      
      if (sensorOnes != lastOnes) {                           // print the ones value, but only if it has changed
        lastOnes = sensorOnes;
        bigDigit(sensorOnes);                                 // call the big digit function with the ones value
      }
      
      bigDigitPlace += 3;                                     // move the place up so the tenths come out in the right spot 

      if (sensorTenths != lastTenths) {                       // print the tenths value, but only if it has changed
        lastTenths = sensorTenths;
        bigDigit(sensorTenths);                               // call the big digit function with the tenths value
      }
      
      bigDigitPlace = 1;                                      // reset the place so next time you start in the right spot
    
    }

  }

}

//***************************************************************************************************************************************************







void displayVacuum() {                                      // call this when you change from inHg to kPa or vica versa,
                                                            // when you go from normal mode to programming mode, and when
                                                            // you initialize the LCD
                                                            
                                                            // prints "inHg" or "kPa " to LCD, prints decimal point in 
                                                            // correct place, and prints current vacuum level

  
  for (int c = 1; c < 8; c ++) {                            // first clear the LCD where the vacuum units and big digits go
    for (int r = 1; r < 5; r ++) {                          // clear columns 1 through 7 and rows 1 through 4
      displayWrite4x40(c, r, " "); 
    } 
  }
  
  
  if (inHg == true) {                                       // here we're working in inHg

    displayWrite4x40(1, 4, "inHg");                         // print the units and decimal place in the correct spot
    bigDecimalPoint(3); 

    inHgCheck = map(sensorAverage, 2000, 14, 0, 80);        // find slope of line between sensorAverage and inhg, inHgCheck 
                                                            // is an integer version of inHg with no decimal point, 
    vacCheck = inHgCheck;                                   // (1.8 inHg = 18, for example) so we can strip out ones and tenths
      
    sensorOnes = inHgCheck / 10;                            // get the ones component, for writing big numbers to the LCD
    inHgCheck -= (sensorOnes * 10);
    sensorTenths = inHgCheck;                               // get the tenths component, for writing big numbers to the LCD    
 
    bigDigit(sensorOnes);                                   // call the big digit function with the ones value

    bigDigitPlace += 3;                                     // move the place up so the tenths come out in the right spot 
    
    bigDigit(sensorTenths);                                 // call the big digit function with the tenths value
        
    bigDigitPlace = 1;                                      // reset the place so next time you start in the right spot
    
  
  
  
  } else if (kPa == true) {                                 // here we're working in kilopascals

    displayWrite4x40(1, 4, "kPa");                          // print the units and decimal place in the correct spot
    bigDecimalPoint(5);        

    kPaCheck = map(sensorAverage, 2000, 14, 0, 271);        // find slope of line between sensorAverage and kPa, kPaCheck 
    vacCheck = kPaCheck;                                    // is an integer version of kPa with no decimal point, 
                                                            // (21.8 inHg = 218, for example) so we can strip out tens, ones and tenths
                                                                                
    sensorTens = kPaCheck / 100;                            // get the tens component, for writing big numbers to the LCD  
    kPaCheck -= (sensorTens * 100);
    sensorOnes = kPaCheck /10;                              // get the ones component, for writing big numbers to the LCD
    kPaCheck -= (sensorOnes * 10);
    sensorTenths = kPaCheck;                                // get the tenths component, for writing big numbers to the LCD  
    
    bigDigit(sensorTens);                                   // call the big digit function with the tens value

    bigDigitPlace += 2;                                     // move the place up so the ones come out in the right spot 

    bigDigit(sensorOnes);                                   // call the big digit function with the ones value
    
    
    bigDigitPlace += 3;                                     // move the place up so the tenths come out in the right spot 

    bigDigit(sensorTenths);                                 // call the big digit function with the tenths value
    
    bigDigitPlace = 1;                                      // reset the place so next time you start in the right spot
  
  }
 
}



//********************************** GET VACUUM LEVEL STRING TO PRINT TO LCD ***************************************************************

String getVacString(int input) {                          // returns a string representation of vacuum number in either inHg or kPa,
                                                          // this is just used to display the trip points while in programming mode,
static int      tens      =     0;                        // not to display the actual vacuum level, that is done with bigDigit()
static int      ones      =     0;
static int      tenths    =     0;
static String   output    =     "";

  if (inHg == true) {
    
    ones = input / 10;
    tenths = input % 10;
    
    output = String(ones) + "." + String(tenths) + " ";          // if printing inHg, format is [ones] [decimal point] [tenths] [blank space]

    return output;
  
  } else if (kPa == true) {
    
    tens = input / 100;
    ones = (input / 10) % 10;
    tenths = input % 10; 

    output = String(tens) + String(ones) + "." + String(tenths); // if printing kPa, format is [tens] [ones] [decimal point] [tenths]

    return output; 
  
  }    
}


