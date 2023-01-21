
// Uses Adafruit Feather nRF52832
// needs the following libraries
// - bluemicro_hid
#include <cstdint>
#include <vector>
#include <bluemicro_hid.h>
#include "hid_keycodes.h"

#define COL2ROW       0
#define ROW2COL       1 

/**************************************************************************************************************************/
// musialik

#define MATRIX_ROWS 10
#define MATRIX_COLS 10

#define MATRIX_COL_PINS { 22, 23, 9, 16, 15, 20, 11, 24, 12, 19 }
#define MATRIX_ROW_PINS { 25, 26, 27, 28, 29, 30, 31, 2, 3, 4}
#define DIODE_DIRECTION COL2ROW
uint16_t keymap[] =    { \
        KC_ESC,     KC_Q,   KC_E,   KC_G,   KC_J,       KC_SCLN,    KC_SLSH,    KC_P1,      KC_P4,      KC_PDOT, 
        KC_GRAVE,    KC_1,   KC_3,   KC_T,   KC_U,       KC_P,       KC_LCTL,    KC_RGHT,    KC_P7,      KC_P3, 
        KC_TAB,     KC_F1,  KC_F3,  KC_5,   KC_7,       KC_0,       KC_LEFT,    KC_DOWN,    KC_NLCK,    KC_P6, 
        KC_CAPS,      KC_F2,  KC_F4,  KC_F5,  KC_F7,      KC_F10,     KC_RSFT,    KC_UP,      KC_PAUS,    KC_P9, 
        KC_LSFT,    KC_2,   KC_4,   KC_F6,  KC_F8,      KC_F9,      KC_QUOT,    KC_ENT,     KC_SLCK,    KC_PAST, 
        KC_LCTL,    KC_W,   KC_R,   KC_6,   KC_8,       KC_9,       KC_LBRC,    KC_RBRC,    KC_PSLS,    KC_HOME, 
        KC_LGUI,    KC_S,   KC_F,   KC_Y,   KC_I,       KC_O,       KC_MINS,    KC_BSLS,    KC_P8,      KC_END, 
        KC_LALT,      KC_X,   KC_V,   KC_H,   KC_K,       KC_L,       KC_F11,     KC_BSPC,    KC_P5,      KC_PMNS, 
        KC_Z,       KC_C,   KC_SPC, KC_N,   KC_COMM,    KC_DOT,     KC_F12,     KC_DEL,     KC_P2,      KC_PPLS, 
        KC_A,       KC_D,   KC_B,   KC_M,   KC_RALT,    KC_NO,      KC_EQL,     KC_PSCR,    KC_P0,      KC_PENT
        };


/**************************************************************************************************************************/
// LUDDITE
/*
#define MATRIX_ROWS 8
#define MATRIX_COLS 8

#define MATRIX_ROW_PINS {6, 8, 15,17, 20,13,24,9}
#define MATRIX_COL_PINS {30, 26, 29, 2, 45, 3, 28, 43}
#define DIODE_DIRECTION COL2ROW
#define KEYMAP( \
    K00, K01, K02, K03, K04, K05, K06, K07, K10, K11, K12, K13, K14, K15, \
    K16, K17, K20, K21, K22, K23, K24, K25, K26, K27, K30, K31, K32, K33, \
    K34, K35, K36, K37, K40, K41, K42, K43, K44, K45, K46, K47, K50, \
    K51, K52, K53, K54, K55, K56, K57, K60, K61, K62, K63, K64, \
    K65, K66, K67,                K70,                K71, K72, K73, K74\
) { \
     K00, K01, K02, K03, K04, K05, K06, K07 , \
     K10, K11, K12, K13, K14, K15, K16, K17 , \
     K20, K21, K22, K23, K24, K25, K26, K27 , \
     K30, K31, K32, K33, K34, K35, K36, K37 , \
     K40, K41, K42, K43, K44, K45, K46, K47 , \
     K50, K51, K52, K53, K54, K55, K56, K57 , \
     K60, K61, K62, K63, K64, K65, K66, K67 , \
     K70, K71, K72, K73, K74, KC_NO,KC_NO,KC_NO  \
}

uint16_t keymap[] = \
    KEYMAP(    KC_ESC, KC_1,   KC_2,   KC_3,   KC_4,   KC_5,   KC_6,   KC_7,   KC_8,   KC_9,   KC_0,   KC_MINS,KC_EQL, KC_BSPC, \
        KC_TAB,KC_Q,   KC_W,   KC_E,   KC_R,   KC_T,   KC_Y,   KC_U,   KC_I,   KC_O,   KC_P,   KC_LBRC,KC_RBRC,KC_BSLS, \
        LAYER_2,KC_A,   KC_S,   KC_D,   KC_F,   KC_G,   KC_H,   KC_J,   KC_K,   KC_L,   KC_SCLN,KC_QUOT,KC_ENT,  \
        KC_LSFT,KC_Z,   KC_X,   KC_C,   KC_V,   KC_B,   KC_N,   KC_M,   KC_COMM,KC_DOT, KC_SLSH,KC_RSFT, \
        KC_LCTL,KC_LGUI,KC_LALT,          KC_SPC,                     LAYER_1, KC_RALT, KC_APP,KC_RCTL);

*/
/**************************************************************************************************************************/
// CONTRA
/*
#define MATRIX_ROWS 4
#define MATRIX_COLS 12

#define MATRIX_ROW_PINS {3, 14, 13, 11}
#define MATRIX_COL_PINS {5, 4, 16, 15, 30, 29, 28, 27, 26, 25, 7, 18} 
#define  DIODE_DIRECTION COL2ROW

 

uint16_t keymap[] =    {KC_ESC,    KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,    KC_U,  KC_I,    KC_O,    KC_P,     KC_BSPACE, \
              KC_TAB,    KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_H,    KC_J,  KC_K,    KC_L,    KC_SCLN,  KC_QUOT, \
              KC_LSFT,   KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,    KC_M,  KC_COMMA,KC_DOT,  KC_SLASH, KC_ENTER, \
              KC_LCTL,   KC_LGUI, KC_LALT, KC_RGUI,LAYER_1, KC_SPC,  KC_SPC, LAYER_2,KC_LEFT, KC_UP,   KC_DOWN,  KC_RIGHT};*/
/**************************************************************************************************************************/

typedef std::vector <uint16_t> trigger_keycodes_t;
typedef std::vector <uint8_t>  trigger_keys_t;

trigger_keys_t activeKeys;
trigger_keycodes_t activeKeycodes;
std::vector<uint8_t> reportvector;

byte rows[] MATRIX_ROW_PINS;        // Contains the GPIO Pin Numbers 
byte columns[] MATRIX_COL_PINS;     // Contains the GPIO Pin Numbers 
/**************************************************************************************************************************/


void sleepNow()
{
  for(int j = 0; j < MATRIX_ROWS; ++j) {                             
    //set the current row as OUPUT and LOW
    pinMode(rows[j], OUTPUT);
    #if DIODE_DIRECTION == COL2ROW                                         
    digitalWrite(rows[j], LOW);                                       // 'enables' a specific row to be "low" 
    #else
    digitalWrite(rows[j], HIGH);                                       // 'enables' a specific row to be "HIGH"
    #endif
  }
  //loops thru all of the columns
  for (int i = 0; i < MATRIX_COLS; ++i) {
      #if DIODE_DIRECTION == COL2ROW                                         
        pinMode(columns[i], INPUT_PULLUP_SENSE);              // 'enables' the column High Value on the diode; becomes "LOW" when pressed - Sense makes it wake up when sleeping
      #else
        pinMode(columns[i], INPUT_PULLDOWN_SENSE);            // 'enables' the column High Value on the diode; becomes "LOW" when pressed - Sense makes it wake up when sleeping
      #endif
  }
  sd_power_system_off();
}

/**************************************************************************************************************************/
/* These are compile time replacements for the scanMatrix function and depends on DIODE_DIRECTION and nRF52832/nRF52840   */
/**************************************************************************************************************************/
#if DIODE_DIRECTION == COL2ROW
#define writeRow(r) digitalWrite(r,LOW)
#define modeCol(c) pinMode(c, INPUT_PULLUP)
#ifdef NRF52840_XXAA
#define gpioIn (((uint64_t)(NRF_P1->IN)^0xffffffff)<<32)|(NRF_P0->IN)^0xffffffff
#else
#define gpioIn (NRF_GPIO->IN)^0xffffffff
#endif
#else
#define writeRow(r) digitalWrite(r,HIGH)
#define modeCol(c) pinMode(c, INPUT_PULLDOWN)
#ifdef NRF52840_XXAA
#define gpioIn (((uint64_t)NRF_P1->IN)<<32)|(NRF_P0->IN)
#else
#define gpioIn NRF_GPIO->IN
#endif
#endif
#ifdef NRF52840_XXAA
#define PINDATATYPE uint64_t
#else
#define PINDATATYPE uint32_t
#endif
/**************************************************************************************************************************/
trigger_keys_t scanMatrix(trigger_keys_t activeKeys)
{
    bool has_key = false;
    PINDATATYPE pinreg = 0;

    //Setting up Scanning, Enabling all columns
    for (int i = 0; i < MATRIX_COLS; ++i){
        modeCol(columns[i]);
    } 

    // FIRST ROW
    pinMode(rows[0], OUTPUT);
    writeRow(rows[0]);
    nrfx_coredep_delay_us(1);   // need for the GPIO lines to settle down electrically before reading first row;
    // READ FIRST ROW
    pinreg = gpioIn;   // press is active high regardless of diode dir
    pinMode(rows[0], INPUT);                                        // 'disables' the row that was just scanned  

    /*************/

     for (int j = 1; j < MATRIX_ROWS; ++j){       
      // NEXT ROW
        pinMode(rows[j], OUTPUT);
        writeRow(rows[j]);
        // PROCESS PREVIOUS ROW - need for the GPIO lines to settle down electrically before reading NEXT row;
        for (int i = 0; i < MATRIX_COLS; ++i){
            int ulPin = g_ADigitalPinMap[columns[i]]; 
            if((pinreg>>ulPin)&1)  
              {
                uint8_t keynumber = (j-1)*MATRIX_COLS + i;
                Serial.println(keynumber);
                activeKeys.push_back(keynumber);
                has_key = true;
              }
        }
        // READ NEXT ROW
        pinreg = gpioIn;  // press is active high regardless of diode dir
        pinMode(rows[j], INPUT); 
     }

    // PROCESS LAST ROW - Process 4th row right away...
    for (int i = 0; i < MATRIX_COLS; ++i){
        int ulPin = g_ADigitalPinMap[columns[i]]; 
        if((pinreg>>ulPin)&1)  
          {
            uint8_t keynumber = (MATRIX_ROWS-1)*MATRIX_COLS + i;
            Serial.println(keynumber);
            activeKeys.push_back(keynumber);
            has_key = true;
          }
    }
    /*************/ 
    //Scanning done, disabling all columns - needed to save power
    for (int i = 0; i < MATRIX_COLS; ++i) {                             
        pinMode(columns[i], INPUT);                                     
    }
    return activeKeys;
}
/**************************************************************************************************************************/
trigger_keycodes_t processKeys(trigger_keys_t activeKeys, trigger_keycodes_t activeKeycodes)
{
    for (auto pressedkey : activeKeys) 
    {
         uint16_t keycode = keymap[pressedkey]; 
         activeKeycodes.push_back(keycode);
    }
  return activeKeycodes;
}
/**************************************************************************************************************************/
trigger_keycodes_t sendKeys(trigger_keycodes_t activeKeycodes)
{
  static bool has_key = false;  // must be static to remember previous state
  if ( activeKeycodes.empty() )
    {
      // send empty key report if previously has key pressed
      if (has_key) 
      { 
        bluemicro_hid.keyboardRelease();
        has_key = false;
      }
    } 
    else
    {
        has_key = true;
        uint8_t currentMod = 0;
        for (auto keycode : activeKeycodes) 
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
        for (auto thiskeycode : reportvector) 
        {
          if (keycodeposition<6)
          {
              keycode[keycodeposition] = thiskeycode;
          }
          keycodeposition++;
        }
        bluemicro_hid.keyboardReport(currentMod, keycode);
        activeKeycodes.clear();
        reportvector.clear();
    }
  return activeKeycodes;
}
/**************************************************************************************************************************/
void pause(unsigned long timestamp, uint8_t cycletime, bool nokeys)
{
  static unsigned long lasttime =0;
  static unsigned long lastkeytime =0;
  unsigned long diff = timestamp - lasttime;
  unsigned long diffkey = timestamp - lastkeytime;
  lasttime = timestamp;
  if (!nokeys)
  {
    lastkeytime = timestamp;
  }
  if (diffkey> 60000) 
  {
    sleepNow();
  }
  if ((diff) < 15*cycletime/10)
  {
    delay(cycletime);
  } // else don't delay and we are already slow (probably typing)
}

/**************************************************************************************************************************/
void setup() {
  // put your setup code here, to run once:
  bluemicro_hid.begin(); 
  Serial.begin(115200);
  Serial.println("BlueMicro_HID Large Matrix Tests");
  activeKeys.reserve(10);
  activeKeycodes.reserve(10);
  reportvector.reserve(7);
}
/**************************************************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:  
  activeKeys = scanMatrix(activeKeys);
  bool nokeyspresssed = activeKeys.empty();
  activeKeycodes = processKeys(activeKeys,activeKeycodes);
  activeKeys.clear();
  activeKeycodes = sendKeys(activeKeycodes); 
  bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);
  pause(millis(),10,nokeyspresssed);
}
