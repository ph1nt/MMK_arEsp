#include <Arduino.h>
#include <BleKeyboard.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <action_code.h>
#include <keycode.h>
#include <quantum_keycodes.h>

#define MODULE_ID "LOLIN 32"
#define GATTS_TAG "MMK V0.5"  // The device's name
#define MAX_BT_DEVICENAME_LENGTH 40
#define SLEEP_DELAY 300  // 5 * 60  // 5 min to sleep
#define MODTAP_TIME 220
#define DEBOUNCE 4  // debounce time in ms

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))
#define SET_BIT(var, pos) (var |= 1 << pos);

// Encoder definitions
#define ENCODER_A_PIN GPIO_NUM_16  // encoder phase A pin
#define ENCODER_B_PIN GPIO_NUM_17  // encoder phase B pin

// OLED Parameters
#define OLED_SDA_PIN GPIO_NUM_32
#define OLED_SCL_PIN GPIO_NUM_33

/*Battery monitoring
 * Please read check battery_monitor.h for resistor values before applying
 * use ADC1 only,  */
#define Vout_min 3.00
#define Vout_max 4.10
#define BATT_PIN 35  // gpio pin 35

const gpio_num_t rowPins[] = {GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_23,
                              GPIO_NUM_19};
const gpio_num_t colPins[] = {
    GPIO_NUM_13, GPIO_NUM_15, GPIO_NUM_2,  GPIO_NUM_34, GPIO_NUM_4,
    GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12};

#define maxBTdev 3
uint8_t MACAddress[maxBTdev][6] = {{0x14, 0x88, 0xE6, 0x08, 0x3D, 0x03},
                                   {0xA0, 0x78, 0x17, 0x74, 0x2F, 0xA6},
                                   {0x31, 0xAE, 0xAC, 0x42, 0x0A, 0x31}};

enum keyState { KS_UP = 0, KS_DOWN, KS_HOLD, KS_TAP, KS_DTAP, KS_RELASE };
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
  uint16_t time_debounce;
  uint16_t time_press;
} keyevent_t;

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
uint8_t curLayer = 0;
#define LT(layer, kc) (0x8000 | (((layer)&0xF) << 8) | ((kc)&0xFF))
#define MT(mod, kc) (0x2000 | (((mod)&0x1F) << 8) | ((kc)&0xFF))
#define TO(layer) (0x8000 | (((layer)&0xF) << 8))
#define LOWER TO(1)
#define RAISE LT(2, KC_BSPACE)
#define NUM LT(3, KC_COMMA)
#define ENT_SFT MT(MOD_LSFT, KC_ENTER)
#define LGUI_ESC MT(MOD_LGUI, KC_ESC)
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
uint16_t keyMap[MATRIX_LAYERS][MATRIX_ROWS][MATRIX_COLS] = {
    {{KC_Q, KC_W, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P},
     {KC_A, KC_S, KC_D, KC_F, KC_G, KC_H, KC_J, KC_L, KC_K, KC_SCLN},
     {KC_Z, KC_X, KC_C, KC_V, KC_B, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH},
     {NUM, KC_LCTRL, KC_LALT, KC_NO, LOWER, KC_SPACE, KC_NO, RAISE, ENT_SFT,
      LGUI_ESC}},
    {{KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0},
     {KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_LEFT, KC_DOWN, KC_UP, KC_RIGHT,
      KC_ENTER},
     {KC_F11, KC_F12, KC_F13, KC_F14, KC_F15, KC_F6, KC_F7, KC_F8, KC_F9,
      KC_F10},
     {NUM, KC_LCTRL, KC_LALT, LOWER, KC_NO, KC_BSLS, KC_NO, RAISE, KC_ENTER,
      KC_LGUI}},
    {{KC_1, KC_2, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0},
     {KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_LEFT, KC_DOWN, KC_UP, KC_RIGHT,
      KC_ENTER},
     {KC_F11, KC_F12, KC_F13, KC_F14, KC_F15, KC_F6, KC_F7, KC_F8, KC_F9,
      KC_F10},
     {NUM, KC_LCTRL, KC_LALT, LOWER, KC_NO, KC_BSLS, KC_NO, RAISE, KC_ENTER,
      KC_LGUI}},
    {{KC_GRV, KC_3, KC_4, KC_5, KC_6, KC_7, KC_8, KC_9, KC_0, KC_BSPC},
     {KC_ESC, KC_E, KC_R, KC_T, KC_Y, KC_U, KC_I, KC_O, KC_P, KC_BSPC},
     {KC_TAB, KC_D, KC_F, KC_G, KC_H, KC_J, KC_L, KC_K, KC_SCLN, KC_QUOT},
     {NUM, KC_LCTRL, KC_LALT, LOWER, KC_NO, KC_BSLS, KC_NO, RAISE, KC_ENTER,
      KC_LGUI}}};