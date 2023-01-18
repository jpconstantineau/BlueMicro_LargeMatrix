
// Uses Adafruit Feather nRF52832
// needs the following libraries
// - bluemicro_hid
#include <cstdint>
#include <vector>
#include <bluemicro_hid.h>
#include "hid_keycodes.h"



#define MATRIX_ROWS 4
#define MATRIX_COLS 12

#define MATRIX_ROW_PINS {3, 14, 13, 11}
#define MATRIX_COL_PINS {5, 4, 16, 15, 30, 29, 28, 27, 26, 25, 7, 18} 

#define  STATUS_BLE_LED_PIN  19  //blue = 0.19
#define  STATUS_KB_LED_PIN 17  //red = 0.17


byte rows[] MATRIX_ROW_PINS;        // Contains the GPIO Pin Numbers 
byte columns[] MATRIX_COL_PINS;     // Contains the GPIO Pin Numbers  

uint8_t keymap[] =    {KC_ESC,    KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,    KC_U,  KC_I,    KC_O,    KC_P,     KC_BSPACE, \
              KC_TAB,    KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_H,    KC_J,  KC_K,    KC_L,    KC_SCLN,  KC_QUOT, \
              KC_LSFT,   KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,    KC_M,  KC_COMMA,KC_DOT,  KC_SLASH, KC_ENTER, \
              KC_LCTL,   KC_LGUI, KC_LALT, KC_RGUI,LAYER_1, KC_SPC,  KC_SPC, LAYER_2,KC_LEFT, KC_UP,   KC_DOWN,  KC_RIGHT};

std::vector<uint16_t> activeKeys;
std::vector<uint8_t> reportvector;

void setup() {
  // put your setup code here, to run once:
  bluemicro_hid.begin(); 
  Serial.begin(115200);
  Serial.println("BlueMicro_HID Large Matrix Tests");
  activeKeys.reserve(10);
  reportvector.reserve(7);
}




void loop() {
  // put your main code here, to run repeatedly:
    // use to send key release report
    static bool has_key = false;  // must be static to remember previous state
    uint32_t pinreg = 0;

    //Setting up Scanning, Enabling all columns
    for (int i = 0; i < MATRIX_COLS; ++i){
        pinMode(columns[i], INPUT_PULLUP);
    } 


// FIRST ROW
    pinMode(rows[0], OUTPUT);
    digitalWrite(rows[0],LOW);
    nrfx_coredep_delay_us(1);   // need for the GPIO lines to settle down electrically before reading first row;
    // READ FIRST ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[0], INPUT);                                        // 'disables' the row that was just scanned  
// SECOND ROW
    pinMode(rows[1], OUTPUT);
    digitalWrite(rows[1],LOW);
    // PROCESS FIRST ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            uint8_t keycode = keymap[0*MATRIX_COLS + i];
            activeKeys.push_back(keycode);
            Serial.print("row: 0 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ SECOND ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[1], INPUT); 
// THIRD ROW
    pinMode(rows[2], OUTPUT);
    digitalWrite(rows[2],LOW);
    // PROCESS SECOND ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            uint8_t keycode = keymap[1*MATRIX_COLS + i];
            activeKeys.push_back(keycode);
            Serial.print("row: 1 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ THIRD ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[2], INPUT); 
// FOURTH ROW
    pinMode(rows[3], OUTPUT);
    digitalWrite(rows[3],LOW);
    // PROCESS THIRD ROW - need for the GPIO lines to settle down electrically before reading second row;
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            uint8_t keycode = keymap[2*MATRIX_COLS + i];
            activeKeys.push_back(keycode); 
            Serial.print("row: 2 Column");
            Serial.println(i);
            has_key = true;
          }
    }
    // READ FOURTH ROW
    pinreg = (NRF_GPIO->IN)^0xffffffff;  // press is active high regardless of diode dir
    pinMode(rows[3], INPUT); 
    // PROCESS FOURTH ROW - Process 4th row right away...
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            uint8_t keycode = keymap[3*MATRIX_COLS + i];
            activeKeys.push_back(keycode);
            Serial.print("row: 3 Column");
            Serial.println(i);
            has_key = true;
          }
    }

    //Scanning done, disabling all columns - needed to save power
    for (int i = 0; i < MATRIX_COLS; ++i) {                             
        pinMode(columns[i], INPUT);                                     
    }
    if ( activeKeys.empty() ) //&& has_key
    {
      // send empty key report if previously has key pressed
      if (has_key) 
      { 
        bluemicro_hid.keyboardRelease();
        Serial.println("Release"); 
      }
      has_key = false;
      
    } 
    else
    {
        uint8_t currentMod = 0;
        for (auto keycode : activeKeys) 
        {
            auto hidKeycode = static_cast<uint8_t>(keycode & 0x00FF);
            auto extraModifiers = static_cast<uint8_t>((keycode & 0xFF00) >> 8);
    
            if (hidKeycode >= KC_A && hidKeycode <= KC_EXSEL)
            {
              // add hidKeycode to report vector
              reportvector.push_back(hidKeycode);
            }
            
            //check if the hid keycode contains a modifier. // also check for macros.
            switch (hidKeycode) { 
                case KC_LCTRL:  currentMod |= 1;    currentMod |= extraModifiers; break;
                case KC_LSHIFT: currentMod |= 2;    currentMod |= extraModifiers; break;
                case KC_LALT:   currentMod |= 4;    currentMod |= extraModifiers; break;
                case KC_LGUI:   currentMod |= 8;    currentMod |= extraModifiers; break;
                case KC_RCTRL:  currentMod |= 16;   currentMod |= extraModifiers; break;
                case KC_RSHIFT: currentMod |= 32;   currentMod |= extraModifiers; break;
                case KC_RALT:   currentMod |= 64;   currentMod |= extraModifiers; break;
                case KC_RGUI:   currentMod |= 128;  currentMod |= extraModifiers; break;
            }
            //add all of the extra modifiers into the curren modifier 
            currentMod |= extraModifiers;
    
        }

            uint8_t keycode[6] = { 0 };
            uint8_t keycodeposition = 0;
            for (auto thiskeycode : activeKeys) 
            {
              if (keycodeposition<6)
              {
                  keycode[keycodeposition] = thiskeycode;
              }
              keycodeposition++;
            }
            
      
            bluemicro_hid.keyboardReport(currentMod, keycode);
            activeKeys.clear();
            reportvector.clear();
            Serial.println("Send");
    }

    
  bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);


    // poll gpio once each 10 ms
  delay(10);
}
