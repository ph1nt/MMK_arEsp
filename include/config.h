#include <Arduino.h>
#include <BleKeyboardMouse.h>
#include <EEPROM.h>
#include <ESP32Time.h>
#include <ESPmDNS.h>
#include <U8g2lib.h>
#include <WiFiUdp.h>
#include <action_code.h>
#include <keycode.h>
#include <quantum_keycodes.h>
#include <stdio.h>
#include <version.h>

#include "ArduinoOTA.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/rtc_io.h"
#include "driver/touch_pad.h"
#include "esp32/ulp.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "passwords.h"
#include "rom/ets_sys.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"

#define MODULE_ID "LOLIN 32"
#define GATTS_TAG "MMK V0.5"  // The device's name
#define MAX_BT_DEVICENAME_LENGTH 40
#define SLEEP_DISPLAY 600  // 60 seconds to power off display
#define SLEEP_CPU 30000    // 50 min
#define MODTAP_TIME 200    // mod_tap time
#define DEBOUNCE 2         // debounce time in ms
#define BASE_LAYER 0
#define KEYBOARD_REPORT_SIZE (8)
#define CONSUMER_REPORT_SIZE (1)

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))
#define SET_BIT(var, pos) (var |= 1 << pos);

// Encoder definitions
#define ENCODER_A_PIN GPIO_NUM_16  // encoder phase A pin
#define ENCODER_B_PIN GPIO_NUM_17  // encoder phase B pin

// OLED Parameters
#define OLED_SDA_PIN GPIO_NUM_32
#define OLED_SCL_PIN GPIO_NUM_33

/* Battery monitoring
 * resistor divider 91k/91k
 * use ADC1 only,  */
#define Vout_min 3.00
#define Vout_max 4.10
#define BATT_PIN 35  // gpio pin 35

const gpio_num_t rowPins[] = {GPIO_NUM_4, GPIO_NUM_18, GPIO_NUM_23,
                              GPIO_NUM_19};
const gpio_num_t colPins[] = {
    GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_2,  GPIO_NUM_34, GPIO_NUM_5,
    GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12};

#define maxBTdev 3
uint8_t MACAddress[maxBTdev][6] = {{0x14, 0x88, 0xE6, 0x08, 0x3D, 0x03},
                                   {0xA0, 0x78, 0x17, 0x74, 0x2F, 0xA6},
                                   {0x31, 0xAE, 0xAC, 0x42, 0x0A, 0x31}};

enum keyState { KS_UP = 0, KS_DOWN, KS_HOLD, KS_TAP, KS_DTAP, KS_RELASE };
uint8_t keysState[4][10];
uint64_t keysPress[4][10];

/* key matrix position */
typedef struct {
  uint8_t col;
  uint8_t row;
} keypos_t;

/* key event */
typedef struct {
  bool pressed;
  bool curState;
  uint8_t state;
  uint64_t time_debounce;
  uint64_t time_press;
} keyevent_t;
typedef struct {
  keypos_t pos;
  uint8_t state;
  uint8_t tap;
  uint8_t mod;
  uint64_t time;
} keypressed_t;

// ESP32Time rtc;
ESP32Time rtc(0);  // offset in seconds GMT+2, for ntp 0?

const char* devs[3] = {"iPad", "MBP", "iMac"};
const char* layers[4] = {"QWERTY", "Symbol", "Number", "Funct."};

uint16_t deviceChose;
// Define matrix
#define MATRIX_ROWS 4
#define MATRIX_COLS 10
#define MATRIX_LAYERS 4    // number of layers defined
uint8_t matrixChange = 0;  // semafor to make analise
uint8_t reportReady = 0;
KeyReport report = {0};
KeyReport releaseReport = {0};
uint8_t powerSave = 0;
uint64_t msec, lsec = 0;
uint64_t matrixTick = 0;
uint8_t _curLayer = 0;
uint8_t curLayer = 0;
uint8_t _modifiers = 0;
#define XXXXX KC_NO
#define TO(layer) (0x8000 | (((layer) & 0xF) << 8))
#define Symbol TO(1)
#define Number TO(2)
#define Funct TO(3)
#define KC_SPWR KC_SYSTEM_POWER

keyevent_t keyEvents[MATRIX_ROWS][MATRIX_COLS] = {};
/*
 * Mod bits:    43210
 *   bit 0      ||||+- Control
 *   bit 1      |||+-- Shift
 *   bit 2      ||+--- Alt
 *   bit 3      |+---- Gui
 *   bit 4      +----- LR flag(Left:0, Right:1)
 * 000r|mods| keycode     Modifiers + Key (Modified key)
 * 001r|mods| keycode     Modifiers + Tap Key(Dual role)
 * 0100|01| usage(10)     Consumer control(0x01) - Consumer page(0x0C)
 * 1000|LLLL| keycode     Layer + Tap Key(0x00-DF)[TAP]
 */
// clang-format off
uint16_t keyMap[MATRIX_LAYERS][MATRIX_ROWS][MATRIX_COLS] = {
    {{KC_Q,   KC_W,   KC_E,   KC_R,   KC_T,   KC_Y,   KC_U,   KC_I,   KC_O,   KC_P},
     {KC_A,   KC_S,   KC_D,   KC_F,   KC_G,   KC_H,   KC_J,   KC_K,   KC_L,   KC_SCLN},
     {KC_Z,   KC_X,   KC_C,   KC_V,   KC_B,   KC_N,   KC_M,   KC_COMM,KC_DOT, KC_SLSH},
     {Funct,  KC_LCTL,KC_LGUI,XXXXX,  KC_LALT,KC_SPC, XXXXX,  Number, KC_RSFT,KC_RALT}},
    // Symbol,
    {{KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_SPWR,KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO},
     {KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO},
     {KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_NO,  DEBUG,  KC_NO,  BT_1,   BT_2,   BT_3},
     {Funct,  KC_LCTL,KC_LGUI,XXXXX,  KC_LALT,KC_SPC, XXXXX,  Number, KC_RSFT,KC_RALT}},
    // Number
    {{KC_1,   KC_2,   KC_3,   KC_4,   KC_5,   KC_6,   KC_7,   KC_8,   KC_9,   KC_0},
     {KC_TAB, KC_MPLY,KC_MPRV,KC_MNXT,KC_MUTE,KC_DOWN,KC_UP,  KC_RIGHT,KC_QUOT},
     {KC_NUBS,KC_GRV, KC_VOLD,KC_VOLU,KC_NO,  KC_HOME,KC_PGDN,KC_PGUP,KC_END, KC_BSLS},
     {Symbol, KC_LCTL,KC_LGUI,XXXXX,  KC_LALT,KC_SPC, XXXXX,  Number, KC_RSFT,KC_RALT}},
    // Function
    {{KC_ESC, KC_NO,  KC_NO,  KC_NO,  KC_NO,  KC_LBRC,KC_RBRC,KC_MINS,KC_EQUAL,KC_BSPACE},
     {KC_F1,  KC_F2,  KC_F3,  KC_F4,  KC_F5,  KC_LEFT,KC_DOWN,KC_UP,  KC_RIGHT,KC_ENTER},
     {KC_F6,  KC_F7,  KC_F8,  KC_F9,  KC_F10, KC_F11, KC_F12, KC_F13, KC_F14, KC_F15},
     {Funct,  KC_LCTL,KC_LGUI,XXXXX,  KC_LALT,KC_SPC, XXXXX,  Symbol, KC_RSFT,KC_RALT}}};
// clang-format on

#define cat_width 32
#define cat_height 32
static unsigned char cat[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x80, 0x7e, 0x06,
    0x00, 0xe0, 0xc0, 0x07, 0x00, 0x1e, 0x00, 0x04, 0x80, 0x03, 0x00, 0x02,
    0xc0, 0x00, 0x28, 0x00, 0x20, 0x00, 0xef, 0x01, 0x20, 0x80, 0x00, 0x03,
    0x10, 0x00, 0x00, 0x06, 0x10, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x04,
    0x10, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x04, 0x30, 0x00, 0x00, 0x04,
    0x20, 0x00, 0x00, 0x06, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x01, 0x80, 0x01,
    0x00, 0x0e, 0x70, 0x00, 0x00, 0xf8, 0x0f, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
