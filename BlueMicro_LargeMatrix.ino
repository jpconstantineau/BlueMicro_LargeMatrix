// SPDX-FileCopyrightText: 2023 Pierre Constantineau
//
// SPDX-License-Identifier: MIT

// Supports 
// - Adafruit Feather nRF52832
// - Nordic nRF52840 DK
// - Raspberry Pi Pico
// needs the following libraries
// - bluemicro_hid   V0.0.9 or above
// - bluemicro_nrf52 V0.0.3 or above
// - bluemicro_rp2040 V0.0.0 or above

#ifdef ARDUINO_ARCH_NRF52 // includes both NRF52832_XXAA and NRF52840_XXAA 
  #include <bluemicro_nrf52.h>
#endif
#ifdef ARDUINO_ARCH_RP2040 // for RP2040 Boards
  #include <bluemicro_rp2040.h>
#endif

/**************************************************************************************************************************/
// Pico87
/**************************************************************************************************************************/
/*
#define MATRIX_COL_PINS { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 }
#define MATRIX_ROW_PINS { 18,19,20,21,22,26}
#define DIODE_DIRECTION COL2ROW
uint16_t keymap[] =    { \
KC_ESC, KC_NO,  KC_F1,  KC_F2,  KC_F3,  KC_F4,  KC_NO,  KC_F5,  KC_F6,  KC_F7,  KC_F8,  KC_F9,  KC_F10, KC_F11, KC_F12,      KC_PSCR,KC_SLCK,KC_PAUS,  
KC_GRAVE,KC_1,  KC_2,   KC_3,   KC_4,   KC_5,   KC_6,   KC_7,   KC_8,   KC_9,   KC_0,   KC_MINS,KC_EQL, KC_BSPC, KC_BSPC, KC_INS,KC_HOME,KC_PGUP,\
KC_TAB, KC_NO,  KC_Q,   KC_W,   KC_E,   KC_R,   KC_T,   KC_Y,   KC_U,   KC_I,   KC_O,   KC_P,   KC_LBRC,KC_RBRC,KC_BSLS,  KC_DEL,  KC_END,  KC_PGDN,  \
KC_CAPS,KC_NO,  KC_A,   KC_S,   KC_D,   KC_F,   KC_G,   KC_H,   KC_J,   KC_K,   KC_L,   KC_SCLN,KC_QUOT,KC_ENT, KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,   \
KC_LSFT,KC_Z,   KC_X,   KC_C,   KC_V,   KC_B,   KC_N,   KC_M,   KC_COMM,KC_DOT, KC_SLSH,KC_NO,  KC_RSFT,KC_NO,  KC_NO,  KC_UP,  KC_NO, \
KC_LCTL,KC_LGUI,KC_NO,  KC_LALT,KC_NO,  KC_NO,  KC_SPC, KC_NO,  KC_NO,  KC_NO,  KC_RALT,KC_RGUI,KC_NO,  LAYER_1, KC_RCTL, KC_LEFT,  KC_DOWN,  KC_RIGHT };*/
/**************************************************************************************************************************/
// musialik
/**************************************************************************************************************************/
/*

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

*/
/**************************************************************************************************************************/
// LUDDITE
/**************************************************************************************************************************/
/*

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
/**************************************************************************************************************************/

#define MATRIX_ROW_PINS {3, 14, 13, 11}
#define MATRIX_COL_PINS {5, 4, 16, 15, 30, 29, 28, 27, 26, 25, 7, 18} 
#define  DIODE_DIRECTION COL2ROW

// below is to test the ROW2COL instead of standard COL2ROW
//#define MATRIX_COL_PINS {3, 14, 13, 11}
//#define MATRIX_ROW_PINS {5, 4, 16, 15, 30, 29, 28, 27, 26, 25, 7, 18} 
//#define  DIODE_DIRECTION ROW2COL

uint16_t keymap[] =    {KC_ESC,    KC_Q,    KC_W,    KC_E,   KC_R,    KC_T,    KC_Y,    KC_U,  KC_I,    KC_O,    KC_P,     KC_BSPACE, \
              KC_TAB,    KC_A,    KC_S,    KC_D,   KC_F,    KC_G,    KC_H,    KC_J,  KC_K,    KC_L,    KC_SCLN,  KC_QUOT, \
              KC_LSFT,   KC_Z,    KC_X,    KC_C,   KC_V,    KC_B,    KC_N,    KC_M,  KC_COMMA,KC_DOT,  KC_SLASH, KC_ENTER, \
              KC_LCTL,   KC_LGUI, KC_LALT, KC_RGUI,LAYER_1, KC_SPC,  KC_SPC, LAYER_2,KC_LEFT, KC_UP,   KC_DOWN,  KC_RIGHT};
              
/**************************************************************************************************************************/


  #if DIODE_DIRECTION == COL2ROW
    #define sleep(r,c)   sleep_C2R(r,c)
    #define scanMatrix(a,b,c) scanMatrix_C2R(a,b,c)
  #else
    #define sleep(r,c)   sleep_R2C(r,c)
    #define scanMatrix(a,b,c) scanMatrix_R2C(a,b,c)
  #endif


trigger_keys_t activeKeys;
trigger_keycodes_t activeKeycodes;


byte rows[] MATRIX_ROW_PINS;        // Contains the GPIO Pin Numbers 
byte columns[] MATRIX_COL_PINS;     // Contains the GPIO Pin Numbers 
/**************************************************************************************************************************/


trigger_keycodes_t processKeys(trigger_keys_t activeKeys, trigger_keycodes_t activeKeycodes)
{
  //Serial.print("P");
    for (auto pressedkey : activeKeys) 
    {
         // Serial.print(pressedkey);
        //  Serial.print(" ");
         uint16_t keycode = keymap[pressedkey]; 
         activeKeycodes.push_back(keycode);
    }
  //  Serial.println("");
  return activeKeycodes;
}



/**************************************************************************************************************************/
void pause(unsigned long timestamp, uint16_t cycletime, bool nokeys, unsigned long sleeptime)
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
  if (diffkey> sleeptime) 
  {                                        
        sleep(rows,columns);
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
  pinMode(27, OUTPUT);
}
/**************************************************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:                                         
  activeKeys = scanMatrix(activeKeys,rows,columns);
  bool nokeyspresssed = activeKeys.empty();
  if (!nokeyspresssed){
  //Serial.println(activeKeys[0]);
  digitalWrite(27, HIGH);
  }
  else
  {
    digitalWrite(27, LOW);
  }
  activeKeycodes = processKeys(activeKeys,activeKeycodes);
  activeKeycodes = sendKeys(activeKeycodes); 
  bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);
  pause(millis(),10,nokeyspresssed,60000);
}
