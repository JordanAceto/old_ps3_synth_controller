    // Simple NeoPixel test.  Lights just a few pixels at a time so a
    // 1m strip can safely be powered from Arduino 5V pin.  Arduino
    // may nonetheless hiccup when LEDs are first connected and not
    // accept code.  So upload code first, unplug USB, connect pixels
    // to GND FIRST, then +5V and digital pin 6, then re-plug USB.
    // A working strip will show a few pixels moving down the line,
    // cycling between red, green and blue.  If you get no response,
    // might be connected to wrong end of strip (the end wires, if
    // any, are no indication -- look instead for the data direction
    // arrows printed on the strip).
     

    #include <FastLED.h>
    #define LED_PIN    6
    #define NUM_LEDS 16


    CRGB leds[NUM_LEDS]; 


    int index = 0;

    int color = 0;
    
    void setup() {
      FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
      //strip.begin();
      //strip.show();
      Serial.begin(9600);
      
    }
     
    void loop() {

      
      index = analogRead(5);
      
      color = map(index, 0, 1024, 0, 255);
      
      index = map(index, 0, 1024, 0, 17);
      //index = 5;
      
      Serial.println(index);

      //          CHSV(hue, saturation, brightness)
      //leds[index] = CHSV( color, 187, 75);
      leds[index] = CRGB::Red; 
      FastLED.show(); 

      
       
       for (int i = 0; i < 16; i++) {
          if (i != index) {
              leds[i] = CRGB::Black;
              FastLED.show();
          }
      }
  }
     
  

