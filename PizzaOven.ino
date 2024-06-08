#include <Arduino.h>
#include <RotaryEncoder.h>
#include "U8glib.h"
#include <EncoderButton.h>
#include <Adafruit_MAX31865.h>

// These pins refer to the Arduino Uno V2                                                                  These pins refer to the pinout of the display that I used, a Creality 12864 OEM LCD Screen that can be found here: https://www.th3dstudio.com/product/creality-12864-oem-lcd-screen/ 
#define PIN_IN1 A5                                                                                      // PB10 Encoder Pin
#define PIN_IN2 A4                                                                                      // PB14 Encoder Pin
#define PIN_IN3 A0                                                                                      // PB2 Encoder Button Pin
#define PIN_OUT7 7                                                                                      // PC6 Piezo Buzzer Pin
#define PIN_OUT8 8

#define RREF 430.0                                                                                      // The value of the Rref resistor. For the PT100, use 430.0
#define RNOMINAL  100.0                                                                                 // The 'nominal' 0C resistance of the sensor. For the PT100, use 100.0

#define MENU_ITEMS 2
#define MAX_TEMP 460                                                                                    // This is in Fahrenheit

U8GLIB_ST7920_128X64_1X u8g(11, 13, 12);	// SPI Com: SCK = en = 11, MOSI = rw = 13, CS = di = 12          Display Object
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);                               // Encoder Object
EncoderButton eb1(PIN_IN3);                                                                             // Encoder Button Object
Adafruit_MAX31865 thermo = Adafruit_MAX31865(A3, A2, A1, 11);                                           // Thermistor Object                    Using software SPI: CS, DI, DO, CLK

char *displayMsg = "dir: ";                                                                             // This is for logging the movement of the encoder, only for troubleshooting.
int *targetTemp = 0;                                                                                    // Temp the user wants the oven to be
int *currentTemp = 0;                                                                                   // Temp the oven is currently at
bool inTempMenu = false;                                                                                // Checks wheter or not the user is in the menu or not
int newDir = 0;                                                                                         // Uses ((int)encoder.getDirection()); to determine which way the encoder moved while the user is in the menu and increment accordingly
bool buttonPressed = false;                                                                             // Keeps track of whether the encoder button is pressed or not

static const unsigned long REFRESH_INTERVAL = 3000;                                                     // Time to poll the thermistor in ms
static const unsigned long REFRESH_INTERVAL_UI = 300;                                                   // Time to refresh the display in ms
static unsigned long lastRefreshTime = 0;                                                               // Helps track the timing for updates

int uiKeyCode = 0;                                                                                      // This program was written with an extensible menu system in mind but I only ended up needing one so it is largely obsolete and could be taken out for simplification in the future
char *menu_strings[MENU_ITEMS] = { "Target Temp - ", "Current Temp - " };                               // Writes the static part of the menu
uint8_t menu_current = 0;                                                                               // Detects whether or not the menu is up to date or not
uint8_t menu_redraw_required = 0;                                                                       // Triggers a redraw of the menu

void onEb1Clicked(EncoderButton& eb) {                                                                  
  buttonPressed = !buttonPressed;                                                                       // Sets the buttonPressed boolean to true when first pressed, and then false the next time it is pressed
  menuHandler();                                                                                        // Triggers the menu handler
}

void menuHandler() {
  if (menu_current == 1){
    inTempMenu = !inTempMenu;                                                                           // Sets the inTempMenu boolean to true when menuHandler is first triggered, and then false the next time it is triggered
    do{
      static int pos = 0;                                                                               // This variable is used to keep track of the encoders last movement
      newDir = ((int)encoder.getDirection());                                                           // This is either 1 or -1
      if (menu_current == 1){                                                                           // Checks again whether or not the user is in the menu
        if (newDir == 1 && targetTemp < MAX_TEMP){                                                      // Checks the direction of the encoder and whether or not the targetTemp has exceeded the maximum allowed temperature, and then increments the targetTemp appropriately
          targetTemp += 5;
        } else if (newDir == -1 && targetTemp > 0){                                                     // Checks the direction of the encoder and whether or not the targetTemp has exceeded the minimum allowed temperature of 0, and then increments the targetTemp appropriately
          targetTemp -= 5;
        }
      }

      char postdata[32] = "Target Temp - ";                                                             // This entire block is just to format the targetTemp correctly so that it can be displayed properly
      char buffer[32];
      char finalizedOut[64];
      int mynumber = targetTemp;
      itoa(targetTemp, buffer, 10);                                                                     // itoa(int, buffer, base)
      strcpy(finalizedOut, postdata);
      strcat(finalizedOut, buffer);
      menu_strings[0] = finalizedOut;

      menu_current--;
      updateScreen();                                                                                   // Updates the screen
      eb1.update();                                                                                     // Updates the state of the encoder button
    }while(inTempMenu == true && buttonPressed == true);
  }
}

void uiStep(int newDir) {                                                                               // Updates the UI after a set amount of time
  if(millis() - lastRefreshTime >= REFRESH_INTERVAL_UI)
	{
		lastRefreshTime += REFRESH_INTERVAL;
    uiKeyCode = newDir;
	}
}

void drawMenu(void) {                                                                                   // Function to draw the menu and push it to the display
  uint8_t i, h;                                                                                         // The below just formats the text and sensor information for use with the display
  u8g_uint_t w, d;

  u8g.setFont(u8g_font_6x13);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  
  h = u8g.getFontAscent()-u8g.getFontDescent() + 1;
  w = u8g.getWidth();

  char *menu_strings_with_values[3] = { "", "", "" };
  menu_strings_with_values[0] = menu_strings[0] + (char)targetTemp;
  menu_strings_with_values[1] = menu_strings[1] + (char)currentTemp + (char)" F";                       // The F at the end is for Fahrenheit

  for( i = 0; i < MENU_ITEMS; i++ ) {                                                                   // Draws the menu line by line, working through the number of menu items
    d = (w-u8g.getStrWidth(menu_strings[i]))/2;
    u8g.setDefaultForegroundColor();
    if ( i == menu_current ) {
      u8g.drawBox(0, i*h+20, w, h+1);
      u8g.setDefaultBackgroundColor();
    }
    u8g.drawStr(d, i*h+20, menu_strings[i]);
  }
}

void updateMenu(void) {                                                                                 // Handles updating the menu
  switch ( uiKeyCode ) {
    case 1:
      menu_current++;
      if ( menu_current > MENU_ITEMS )                                                                  // Handles the looping through of the menu
        menu_current = 0;
      menu_redraw_required = 1;
      break;
  }
}

void setup() { 
  Serial.begin(115200);

  pinMode(PIN_OUT8, OUTPUT);                                                                            // Sets the digital I/O pin as an output for the 120V power relay

  thermo.begin(MAX31865_3WIRE);                                                                         // Initializes the thermistor. Can be set to 2WIRE or 4WIRE as necessary

  pinMode(PIN_OUT7, OUTPUT);                                                                            // Sets the digital I/O pin as an output for the buzzer on the display. Makes one noise when plugged in and two if everything works as it should
  tone(PIN_OUT7, 1000);                                                                                 // Send 1KHz sound signal...
  delay(40);                                                                                            // ...for 1 sec
  noTone(PIN_OUT7);                                                                                     // Stop sound...

  eb1.setClickHandler(onEb1Clicked);                                                                    // Initializes the encoders button
  

  while (! Serial);
  Serial.println("SimplePollRotator example for the RotaryEncoder library.");
  PCICR |= (1 << PCIE1);                                                                                // Enables pin change interrupts for the group of pins PCINT[14:8]
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT8);                                            // Enables interrupts for A5, A4, and A0, the three analog input pins that the encoder and its button are using

  menu_redraw_required = 1;                                                                             // Prompts screen to be drawn
}

ISR(PCINT1_vect) {                                                                                      // PCINT1_vect is triggered when any of the pins in the PCINT[14:8] group change state
  encoder.tick();                                                                                       // Polls the encoder for its current state
  eb1.update();                                                                                         // Polls the encoder button for its current state
}

int returnThermistorTemp() {                                                                            // Returns the temperature reported by the thermistor as an integer
  uint8_t fault = thermo.readFault();                                                                   // Check and print any faults
  if (fault) {                                                                                          // Handles faults
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  
  float fahrenheitConversion = (thermo.temperature(RNOMINAL, RREF) * 9.0) / 5.0 + 32;                   // Converts the native Celsius measurement to Fahrenheit.

  char postdata[32] = "Current Temp - ";                                                                // This entire block is just to format the current temp correctly so that it can be displayed properly
  char buffer[32];
  char finalizedOut[64];
  float mynumber = fahrenheitConversion;
  dtostrf(mynumber, 4, 1, buffer);
  strcpy(finalizedOut, postdata);
  strcat(finalizedOut, buffer);
  menu_strings[1] = finalizedOut;

  menu_current--;
  updateScreen();

  return fahrenheitConversion;
}

void updateScreen() {                                                                                   // Updates the screen with new information
  if (menu_redraw_required != 0) {
      u8g.firstPage();
      do  {
        drawMenu();
      } while( u8g.nextPage() );
      menu_redraw_required = 0;
  }
  updateMenu();                                                                                         // Updates the menu
}

void loop() { 
	if(millis() - lastRefreshTime >= REFRESH_INTERVAL)                                                    // Updates the thermistor information once the refresh interval has elapsed
	{
		lastRefreshTime += REFRESH_INTERVAL;
    currentTemp = returnThermistorTemp();                                                     
	}

  if (currentTemp != NULL && targetTemp != NULL && currentTemp < targetTemp - 27){                      // Checks that everything is initialized and that the current temp is lower than the target temp. The -27 is to help curb the overrun the heater experiences using this method
    digitalWrite(PIN_OUT8, HIGH);                                                                       // Sets the digital I/O pin as high， i.e. turning on the relay and powering the heating coils    
    delay(200);
  }
  if (currentTemp != NULL && targetTemp != NULL && currentTemp > targetTemp + 3){                       // Checks that everything is initialized and that the current temp is lower than the target temp. The 3 is to help curb the relay for the heater turning on and off rapidly
      digitalWrite(PIN_OUT8, LOW);                                                                      // Sets the digital I/O pin as low， i.e. turning off the relay and powering the heating coils
      delay(200);
  }

  static int pos = 0;                                                                                   // Updates the encoder information
  int newPos = encoder.getPosition();
  newDir = ((int)encoder.getDirection());

  if (pos != newPos) {                                                                                  // Detects whether or not the display should be updated
    pos = newPos;
    uiStep(newDir);                                                                                     // Updates the UI after a set amount of time
    updateScreen();                                                                                     // Updates the screen with new information
  }

  eb1.update();                                                                                         // Updates the encoder button information
}
