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
     
    #include <Adafruit_NeoPixel.h>
     
    #define PIN    6
    #define N_LEDS 16
     
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

    int index = 0;

    int color;
    
    void setup() {
      strip.begin();

      Serial.begin(9600);
      
    }
     
    void loop() {

      color = strip.Color(0, 30, 3);
      
      index = analogRead(5);
      index = map(index, 0, 1024, 0, 17);

      Serial.println(index);
      
      strip.setPixelColor(index, color);
      strip.show();

      for (int i = 0; i < 16; i++) {
          if (i != index) {
              strip.setPixelColor(i, 0);
          }
      }

      
      //chase(strip.Color(255, 0, 0)); // Red
      //chase(strip.Color(0, 255, 0)); // Green
      //chase(strip.Color(0, 0, 255)); // Blue
    }
     
  

    static void blank() {
       for(uint16_t i=0; i<strip.numPixels() + 4; i++) {
          strip.setPixelColor(i-3, 0); // Erase pixel a few steps back
          strip.show();
          //delay(500);
    }
  }

    
    static void chase(uint32_t c) {
      for(uint16_t i=0; i<strip.numPixels() + 4; i++) {
          strip.setPixelColor(i  , c); // Draw new pixel
          strip.setPixelColor(i-3, 0); // Erase pixel a few steps back
          strip.show();
          delay(500);
      }
    }
   
