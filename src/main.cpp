#include <Arduino.h>
#include <config.h>

#define Xpos(_col) (6 * _col)
#define Ypos(_row) (12 * _row)
#define posY(_row) (127 - Ypos(_row))

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8x8(U8G2_R3, /* reset=*/U8X8_PIN_NONE,
                                            /* clock=*/OLED_SCL_PIN,
                                            /* data=*/OLED_SDA_PIN);
BleKeyboard bleKeyboard(GATTS_TAG, "HNA", 100);

boolean otaUpdate = false;
unsigned int otaProgress = 0;
unsigned int otaTotal = 30;

class Keys {
#define _pressedSize 10
 private:
  keypressed_t keyPressed[_pressedSize] = {};
  uint8_t _count = 0;
  uint8_t _layer = 0;
  uint8_t _mods = 0;
  uint16_t _keycode = 0;
  uint8_t i = 0;
  uint8_t kbd_report[KEYBOARD_REPORT_SIZE + 2] = {
      KEYBOARD_REPORT_SIZE + 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t consumer_report[CONSUMER_REPORT_SIZE + 2] = {CONSUMER_REPORT_SIZE + 1,
                                                       2, 0};
  KeyReport _report = {0};

 public:
  Keys() {
    // clear entire pressed table
    for (uint8_t n = 0; n < _pressedSize; n++) {
      keyPressed[n].pos = {0, 0};
      keyPressed[n].state = 0;  // empty cell
      keyPressed[n].time = 0;
    }
  }
  uint64_t pressed(uint8_t _row, uint8_t _col) {
    for (uint8_t n = 0; n < _count; n++) {
      if ((keyPressed[n].pos.row == _row) and (keyPressed[n].pos.col == _col)) {
        return keyPressed[n].time;
      }
    }
    return 0;
  }
  uint8_t add(uint8_t _row, uint8_t _col, uint64_t _time) {
    if (_count == _pressedSize - 1) {
      return 1;
    }
    keyPressed[_count].state = 1;  // key down
    keyPressed[_count].pos.row = _row;
    keyPressed[_count].pos.col = _col;
    _count++;
    return 0;
  }
  uint64_t del(uint8_t _row, uint8_t _col) {
    for (uint8_t n = 0; n < _count; n++) {
      if ((keyPressed[n].pos.row == _row) and (keyPressed[n].pos.col == _col)) {
        for (uint8_t _n = _count; _n < _pressedSize; _n++) {
          keyPressed[_n] = keyPressed[_n + 1];
        }
        return 0;
      }
    }
    return 1;
  }
  uint8_t isConsumer(uint16_t _keycode) {
    switch (_keycode) {
      case KC_VOLU:
        return (1UL << 6);
        break;
      case KC_VOLD:
        return (1UL << 7);
        break;
      case KC_MPLY:
        return (1UL << 4);
        break;
      case KC_MNXT:
        return (1UL << 0);
        break;
      case KC_MPRV:
        return (1UL << 1);
        break;
      default:
        break;
    }
  }
  uint16_t analyzeKey(uint8_t _row, uint8_t _col) {
    if (i < 6) {
      uint8_t sl = _layer;
      do {
        if (sl < 0) {
          _keycode = keyMap[BASE_LAYER][_row][_col];
          break;
        }
        _keycode = keyMap[sl--][_row][_col];
      } while (_keycode == KC_TRNS);
      if IS_MOD (_keycode) {
        _report.modifiers |= MOD_BIT(_keycode);
      }
      // TODO mouse
      if IS_KEY (_keycode) {
        _report.keys[i++] = _keycode;
      } else {
        if IS_CONSUMER (_keycode) {
          consumer_report[2] |= isConsumer(_keycode);
        }
      }
    } else {
      _keycode = 0;
    }
    return _keycode;
  }
  void sendKeys() {
    bleKeyboard.sendReport(&_report);
    bleKeyboard.press(consumer_report[2]);
    bleKeyboard.release(consumer_report[2]);
    consumer_report[2] = 0;
    i = 0;
  }
  void process() {
    for (uint8_t n = 0; n < _count; n++) {
      uint16_t _keycode =
          keyMap[_layer][keyPressed[n].pos.row][keyPressed[n].pos.col];
      switch (keyPressed[n].state) {
        case 1:
          // pressed uncheck
          keyPressed[n].state = 2;
          if (_keycode & 0x8000) {
            // layer change
            _layer |= ((_keycode & 0x0F00) >> 8);
          }
          if (_keycode & 0x4000) {
            // TODO media key
          }
          if (_keycode & 0x2000) {
            // mods change
            _mods |= ((_keycode & 0x1F00) >> 8);
          }
          break;
        case 2:
          if ((matrixTick - keyPressed[n].time) > MODTAP_TIME) {
            if (_keycode & 0x8000) {
              // layer change
              keyPressed[n].state = 4;
              _layer |= ((_keycode & 0x0F00) >> 8);
            } else {
              // mod pressed
              keyPressed[n].state = 3;
              _mods |= 1 << (_keycode - KC_LCTL);
            }
          }
          break;
        case 3:
          break;
        case 4:
          break;
        case 8 + 3:
          _mods &= !(1 << (_keycode - KC_LCTL));
          break;
        case 8 + 4:
          _layer &= !((_keycode & 0x0F00) >> 8);
          break;
        default:
          break;
      }
    }
  }
};

void getFreeRAM() {
  // Internal RAM
  uint32_t getHeapSize();      // total heap size
  uint32_t getFreeHeap();      // available heap
  uint32_t getMinFreeHeap();   // lowest level of free heap since boot
  uint32_t getMaxAllocHeap();  // largest block that can be allocated at once
}

float_t getBatteryVoltage(void) {
  return analogReadMilliVolts(BATT_PIN) / 520.0;
}

uint32_t getBatteryLevel(void) {
  int32_t battery_percent =
      (getBatteryVoltage() - Vout_min) * 100.0 / (Vout_max - Vout_min);
  battery_percent = (battery_percent < 100) ? battery_percent : 100;
  battery_percent = (battery_percent < 0) ? 0 : battery_percent;
  return battery_percent;
}

// Returns ESP32 core temperature in celcius degrees
float tempCpu() {
  SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3,
                    SENS_FORCE_XPD_SAR_S);
  SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10,
                    SENS_TSENS_CLK_DIV_S);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
  ets_delay_us(100);
  SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
  ets_delay_us(5);
  float temp_f = (float)GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG,
                                           SENS_TSENS_OUT, SENS_TSENS_OUT_S);
  float temp_c = round((temp_f - 32.0) * 50 / 9) / 10;
  return temp_c;
}

uint16_t fps, tmOut, sleepDebug = 0, powerOff = 0;

// deinitializing rtc matrix pins on  deep sleep wake up
void rtc_matrix_deinit(void) {
  // Deinitializing columns
  for (uint8_t col = 0; col < MATRIX_COLS; col++) {
    if (rtc_gpio_is_valid_gpio(colPins[col]) == 1) {
      rtc_gpio_set_level(colPins[col], 0);
      rtc_gpio_set_direction(colPins[col], RTC_GPIO_MODE_DISABLED);
      gpio_reset_pin(colPins[col]);
    }
  }
  // Deinitializing rows
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    if (rtc_gpio_is_valid_gpio(rowPins[row]) == 1) {
      rtc_gpio_set_level(rowPins[row], 0);
      rtc_gpio_set_direction(rowPins[row], RTC_GPIO_MODE_DISABLED);
      gpio_reset_pin(rowPins[row]);
    }
  }
}

// Initializing rtc matrix pins for deep sleep wake up
void rtc_matrix_setup(void) {
  uint64_t rtc_mask = 0;
  // Initializing columns
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    if (rtc_gpio_is_valid_gpio(rowPins[row]) == 1) {
      rtc_gpio_init((rowPins[row]));
      rtc_gpio_set_direction(rowPins[row], RTC_GPIO_MODE_INPUT_OUTPUT);
      rtc_gpio_set_drive_capability(rowPins[row], GPIO_DRIVE_CAP_0);
      rtc_gpio_set_level(rowPins[row], 1);
    } else {
      digitalWrite(rowPins[row], 1);
      esp_sleep_enable_gpio_wakeup();
    }
  }
  // Initializing rows
  for (uint8_t col = 0; col < MATRIX_COLS; col++) {
    if (rtc_gpio_is_valid_gpio(colPins[col]) == 1) {
      rtc_gpio_init((colPins[col]));
      rtc_gpio_set_direction(colPins[col], RTC_GPIO_MODE_INPUT_OUTPUT);
      rtc_gpio_set_level(colPins[col], 0);
      rtc_gpio_wakeup_enable(colPins[col], GPIO_INTR_HIGH_LEVEL);
      SET_BIT(rtc_mask, colPins[col]);
    } else {
      gpio_wakeup_enable(colPins[col], GPIO_INTR_HIGH_LEVEL);
    }
    esp_sleep_enable_ext1_wakeup(rtc_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  }
}

void oledOta() {
  char tmpStr[8];
  u8x8.drawGlyphX2(4, 40 + 2 * (rtc.getSecond() % 10), 0xe061);
  u8x8.drawGlyphX2(4, 80, 0xe028);
  if (otaTotal != 0) {
    sprintf(tmpStr, "%d%%", (100 * otaProgress / otaTotal));
    u8x8.drawStr(4, 100, tmpStr);
    sprintf(tmpStr, "%dk", uint16_t(otaTotal / 1024));
    u8x8.drawStr(0, 120, tmpStr);
  }
}

void oledDebug() {
  u8x8.setDisplayRotation(U8G2_R0);
  u8x8.drawStr(0, 10, "batt:");
  u8x8.drawStr(30, 10, String(getBatteryVoltage()).c_str());
  u8x8.drawStr(54, 10, "V");
  u8x8.drawStr(80, 10, "fps");
  u8x8.drawStr(98, 10, String(fps).c_str());
  u8x8.drawStr(0, 20, VERSION_SHORT);
  u8x8.drawStr(80, 20, "cpu");
  u8x8.drawStr(98, 20, String(tempCpu()).c_str());
  u8x8.drawGlyph(121, 20, 0x00b0);
  u8x8.drawStr(0, 31, "IP:");
  u8x8.drawStr(24, 31, WiFi.localIP().toString().c_str());
}

void oledSleep() {
  u8x8.drawStr(0, 36, "Power");
  u8x8.drawStr(0, 48, "off..");
}

void oledDraw(uint8_t _stage = 0) {
  // X:  6 12 18 24 30 | 1 pix left
  // Y: 12 24 36 48 60 72 84 96 108 120 | 8 pix left
  // from bottom: 115 103 91 79 67 55 43 31 19
  u8x8.clearBuffer();
  u8x8.setFont(u8g2_font_siji_t_6x10);
  switch (_stage) {
    case 1:
      oledOta();
      break;
    case 2:
      oledDebug();
      break;
    case 3:
      oledSleep();
      break;

    default:
      u8x8.setDisplayRotation(U8G2_R3);
      u8x8.setFont(u8g2_font_5x8_tr);
      u8x8.drawStr(0, Ypos(5), layers[curLayer]);
      u8x8.setFont(u8g2_font_6x12_mf);
      u8x8.drawStr(0, posY(0), devs[deviceChose]);
      if (rtc.getYear() > 2010) {
        u8x8.drawStr(0, Ypos(7), rtc.getTime("%H:%M").c_str());
        u8x8.drawStr(Xpos(1), Ypos(8), rtc.getTime("%d").c_str());
        // u8x8.setFont(u8g2_font_4x6_tf);
        u8x8.drawStr(Xpos(2), Ypos(9), rtc.getTime("%b").c_str());
      }
      // u8x8.setFont(u8g2_font_emoticons21_tr);
      // u8x8.setFont(u8g2_font_battery19_tn);
      u8x8.setFont(u8g2_font_siji_t_6x10);
      if (bleKeyboard.isConnected()) {
        u8x8.drawGlyph(0, 12, 0xe00b);
      }
      uint8_t tmpi = -WiFi.RSSI();
      if (tmpi > 0) {
        u8x8.drawGlyph(
            16, 8,
            0xe258 + (tmpi < 80) + (tmpi < 70) + (tmpi < 60) + (tmpi < 50));
      } else {
        u8x8.drawGlyph(16, 8, 0xe142);
      }
      u8x8.drawGlyph(16, 16, 0xe241 + uint8_t(getBatteryLevel() / 10));
      // cat
      if (curLayer > 0) {
        u8x8.setFont(u8g2_font_streamline_all_t);
        u8x8.drawGlyph(5, 43, 0x0125 + curLayer);
      } else {
        u8x8.drawXBM(0, 20, 32, 32, cat);
      }
      break;
  }
  u8x8.sendBuffer();
  u8x8.refreshDisplay();
}

// Write to eprom, reset ESP 32
void changeID(int DevNum) {
  if (DevNum < maxBTdev) {
    EEPROM.write(0, DevNum);
    EEPROM.commit();
    esp_sleep_enable_timer_wakeup(1);
    esp_deep_sleep_start();
  }
}

// Initializing matrix pins
void matrixSetup(void) {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    gpio_pad_select_gpio(rowPins[row]);
    gpio_set_direction(rowPins[row], GPIO_MODE_OUTPUT);
    gpio_set_level(rowPins[row], 0);
  }
  for (uint8_t col = 0; col < MATRIX_COLS; col++) {
    gpio_pad_select_gpio(colPins[col]);
    gpio_set_direction(colPins[col], GPIO_MODE_INPUT);
    gpio_pulldown_en(colPins[col]);
    gpio_set_pull_mode(colPins[col], GPIO_PULLDOWN_ONLY);
  }
  // encoder setup
  gpio_pad_select_gpio(ENCODER_A_PIN);
  gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
  gpio_pad_select_gpio(ENCODER_B_PIN);
  gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
  // arrays initialization
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      keysState[row][col] = 0;
      keysPress[row][col] = 0;
    }
  }
}
void keysSend() {
  /*
    bleKeyboard.sendReport(report);
    bleKeyboard.press(consumer_report[2]);
    bleKeyboard.release(consumer_report[2]);
    consumer_report[2] = 0;
  */
}

void keysProcess() {
  for (uint8_t _row = 0; _row < MATRIX_ROWS; _row++) {
    for (uint8_t _col = 0; _col < MATRIX_COLS; _col++) {
      switch (keysState[_row][_col]) {
        // = 0, KS_DOWN, KS_HOLD, KS_TAP, KS_DTAP, KS_RELASE
        case KS_UP:  // relax
          break;
        case KS_DOWN:  // check mod/tap
          if ((matrixTick - keysPress[_row][_col]) > MODTAP_TIME) {
            keysState[_row][_col] = KS_HOLD;
          }
          break;
        case KS_HOLD:
          // TODO report BLE press
          break;
        case KS_TAP:
          // TODO all mod/tap to mod, send key
          break;

        case KS_RELASE:
          // TODO unpresss key
          keysState[_row][_col] = KS_UP;
          break;

        default:
          break;
      }
    }
  }
  keysSend();
}

// process press or release key
void keyProcess(uint8_t _row, uint8_t _col, uint8_t _state) {
  if (_state) {
    // key pressed
    switch (keysState[_row][_col]) {
      // = 0, KS_DOWN, KS_HOLD, KS_TAP, KS_DTAP, KS_RELASE
      case KS_UP:  // key_down
        keysState[_row][_col] = KS_DOWN;
        keysPress[_row][_col] = matrixTick;
        break;

      default:
        break;
    }
  } else {
    // key released
    switch (keysState[_row][_col]) {
      case KS_HOLD:
        // TODO report BLE release
        keysState[_row][_col] = KS_RELASE;
        break;
      case KS_DOWN:  // KS_TAP
        // TODO report BLE press
        keysState[_row][_col] = KS_TAP;
        break;

      default:
        break;
    }
  }
}

// Scanning physical keys matrix
void matrixScan() {
  uint8_t curState = 0;
  matrixTick++;
  // Serial.printf("tick: %d ", matrixTick);
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    gpio_set_level(rowPins[row], 1);
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      curState = gpio_get_level(colPins[col]);
      if (keyEvents[row][col].curState != curState) {
        keyEvents[row][col].time_debounce = matrixTick;
        // DEBUG
        Serial.printf("!=curState row:%d, col:%d, time:%d, state:%d\n", row,
                      col, matrixTick, curState);
      }
      if ((matrixTick - keyEvents[row][col].time_debounce) > DEBOUNCE) {
        if (keyEvents[row][col].pressed != curState) {
          Serial.printf("matrixChange row:%d, col:%d, time:%d, state:%d\n", row,
                        col, matrixTick, curState);
          matrixChange = 1;
          keyEvents[row][col].pressed = curState;
          // TODO call keyboard process
          keyProcess(row, col, curState);
        }
      }
      keyEvents[row][col].curState = curState;
    }
    gpio_set_level(rowPins[row], 0);
  }
  keysProcess();
}

const uint8_t mediacode[] = {16, 32, 64, 1, 2, 4, 8};
void matrixPress(uint16_t keycode, uint8_t _hold) {
  uint8_t k = 0;
  if (keycode != DEBUG) {
    uint8_t mediaReport[2] = {0, 0};
    if (_hold) {
      curLayer = _curLayer;
      report.modifiers = _modifiers;
      if (keycode & 0x2000) {
        report.modifiers |=
            ((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
      }
    } else {
      if (keycode & 0x8000) {
        curLayer = ((keycode & 0x0F00) >> 8);
      }
    }
    // consumer control key
    if ((keycode & 0x4000) || ((keycode >= 168) && (keycode <= 174))) {
      mediaReport[0] = mediacode[(keycode & 0x00FF) - 168];
      bleKeyboard.press(mediaReport);
    }

    if (keycode & 0x1F00) {
      report.modifiers |= ((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
      k = (keycode & 0x00FF);
    } else {
      if ((keycode >= KC_MS_U) & (keycode <= KC_ACL2)) {
        // mouse
        switch (keycode) {
          case KC_MS_U:
            bleKeyboard.move(0, -1);
            break;
          case KC_MS_D:
            bleKeyboard.move(0, 1);
            break;
          case KC_MS_L:
            bleKeyboard.move(-1, 0);
            break;
          case KC_MS_R:
            bleKeyboard.move(1, 0);
            break;
          case KC_MS_BTN1:
            bleKeyboard.click(1);
            break;
          case KC_MS_BTN2:
            bleKeyboard.click(2);
            break;
          case KC_MS_BTN3:
            bleKeyboard.click(4);
            break;
          default:
            break;
        }
      } else {
        if ((keycode >= KC_LCTRL) && (keycode <= KC_RGUI)) {
          report.modifiers |= (1 << (keycode - KC_LCTRL));
        } else {
          k = (keycode & 0x00FF);
        }
      }
    }
  }
  if (k != 0) {
    if ((report.keys[0] != k) && (report.keys[1] != k) &&
        (report.keys[2] != k) && (report.keys[3] != k) &&
        (report.keys[4] != k) && (report.keys[5] != k)) {
      for (uint8_t i = 0; i < 6; i++) {
        if (report.keys[i] == 0x00) {
          report.keys[i] = k;
          break;
        }
      }
    }
  }
}

void matrixRelease(uint16_t keycode) {
  // TODO mod release
  if (keycode != DEBUG) {
    uint8_t k;
    uint8_t mediaReport[2] = {0, 0};
    if (keycode & 0x8000) {
      curLayer = 0;
    }
    if ((keycode & 0x4000) || ((keycode >= 168) && (keycode <= 174))) {
      mediaReport[0] = mediacode[(keycode & 0x00FF) - 168];
      bleKeyboard.release(mediaReport);
      return;
    } else {
      if (keycode & 0x1F00) {
        report.modifiers &=
            ~((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
      }
      k = (keycode & 0x00FF);
      for (uint8_t i = 0; i < 6; i++) {
        if (0 != k && report.keys[i] == k) {
          releaseReport.keys[i] = k;
        }
      }
      if ((keycode >= KC_LCTRL) && (keycode <= KC_RGUI)) {
        report.modifiers &= !(1 << (keycode - KC_LCTRL));
      }
    }
  }
}

// press >time> hold; release tap >time> release
void matrixProces() {
  uint16_t keycode;
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      keycode = keyMap[curLayer][row][col];  // TODO transparent
      switch (keyEvents[row][col].state) {
        case KS_UP:
          if (keyEvents[row][col].pressed == 1) {
            keyEvents[row][col].state = KS_DOWN;
            keyEvents[row][col].time_press = matrixTick;
            Serial.printf("KS_DOWN keycode:%d row:%d col:%d time:%d\n", keycode,
                          row, col, matrixTick);
            if (keycode & 0x8000) {
              _curLayer = ((keycode & 0x0F00) >> 8);
            }
            if (keycode & 0x2000) {
              _modifiers |=
                  ((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
            }
          }
          break;
        case KS_DOWN:
          if (keyEvents[row][col].pressed == 1) {
            if (matrixTick >
                uint64_t(keyEvents[row][col].time_press + MODTAP_TIME)) {
              keyEvents[row][col].state = KS_HOLD;
              Serial.printf("KS_HOLD keycode:%d row:%d col:%d time:%d \n",
                            keycode, row, col, matrixTick);
              if ((keycode >= BT_1) && (keycode <= BT_3)) {
                // reboot to select diffrent BT device
                changeID(keycode - BT_1);
              }
              switch (keycode) {
                case DEBUG:
                  sleepDebug = 1;
                  break;
                case KC_SYSTEM_POWER:
                  powerOff = 1;
                  Serial.println("Power off..");
                  break;

                default:
                  matrixPress(keycode, 1);
                  break;
              }
            }
          } else {
            keyEvents[row][col].state = KS_TAP;
            Serial.printf("KS_TAP row:%d col:%d time:%d keycode:%d\n\n", row,
                          col, matrixTick, keycode);
          }
          break;
        case KS_HOLD:
          if (keyEvents[row][col].pressed == 0) {
            keyEvents[row][col].state = KS_UP;
            Serial.printf("KS_UP row:%d col:%d time:%d keycode:%d\n\n", row,
                          col, matrixTick, keycode);
            if (keycode == DEBUG) {
              sleepDebug = 0;
            } else {
              matrixRelease(keycode);
              reportReady++;
            }
          } else {
            switch (keycode) {
              case KC_MS_U:
                bleKeyboard.move(0, -1);
                break;
              case KC_MS_D:
                bleKeyboard.move(0, 1);
                break;
              case KC_MS_L:
                bleKeyboard.move(-1, 0);
                break;
              case KC_MS_R:
                bleKeyboard.move(1, 0);
                break;
            }
          }
          break;
        case KS_TAP:
          keyEvents[row][col].state = KS_RELASE;
          matrixPress(keycode, 0);
          reportReady++;
          break;
        case KS_RELASE: {
          matrixRelease(keycode);
          reportReady++;
          keyEvents[row][col].state = KS_UP;
        } break;
        default:
          Serial.println(
              "Upss.. wrong keyEvents[row][col].state in matrixProcess()");
          break;
      }
    }
  }
}

// Keyboard state array initialize
void keyboardSetup() {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      keyEvents[row][col].state = KS_UP;
      keyEvents[row][col].curState = 0;
      keyEvents[row][col].time_press = 0;
    }
  }
}

// Task for continually scaning keyboard
void keyboardTask(void *pvParameters) {
  while (1) {
    if (!otaUpdate) {
      matrixScan();
      matrixProces();
      if (matrixChange == 1) {
        matrixChange = 0;
        tmOut = 0;
        if (powerSave == 1) powerSave++;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void otaTask(void *pvParameters) {
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(9 / portTICK_PERIOD_MS);
  }
}

void otaSetup() {
  ArduinoOTA
      .onStart([]() {
        otaUpdate = true;
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        otaUpdate = false;
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        otaProgress = progress;
        otaTotal = total;
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        otaUpdate = false;
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });
  ArduinoOTA.setHostname("MMK");
  // ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();
  Serial.println(WiFi.localIP());
}

void WifiTask(void *pvParameters) {
  while (1) {
    int16_t n = WiFi.scanNetworks();
    int16_t ni = 0;
    if ((n > 0) && (!otaUpdate)) {
      while (WiFi.status() != WL_CONNECTED) {
        for (int i = 0; i < WiFis; i++) {
          for (ni = 0; ni < n; ni++) {
            if (WiFi.SSID(ni) == String(ssid[i])) {
              WiFi.begin(ssid[i], password[i]);
              if (WiFi.waitForConnectResult() == WL_CONNECTED) {
                configTime(3600, 00, "europe.pool.ntp.org");
                otaSetup();
                break;
              }
              WiFi.disconnect();
            }
          }
        }
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

int16_t PastEncoderCount = 0;

// setup encoder
void encoderSetup(void) {
  pcnt_config_t pcnt_config_a = {
      .pulse_gpio_num = ENCODER_A_PIN,
      .ctrl_gpio_num = ENCODER_B_PIN,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .pos_mode = PCNT_COUNT_DIS,
      .neg_mode = PCNT_COUNT_INC,
      .counter_h_lim = INT16_MAX,
      .counter_l_lim = INT16_MIN,
      .unit = PCNT_UNIT_0,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config_a);
  pcnt_set_filter_value(PCNT_UNIT_0, 1023);
  pcnt_filter_enable(PCNT_UNIT_0);
  gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
  gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

// Check encoder state
uint8_t encoderState(void) {
  uint8_t EncoderState = 0x00;
  int16_t EncoderCount;
  pcnt_get_counter_value(PCNT_UNIT_0, &EncoderCount);
  // Serial.println(EncoderCount);
  if (EncoderCount > PastEncoderCount) {
    EncoderState = 1;
  }
  if (EncoderCount < PastEncoderCount) {
    EncoderState = 2;
  }
  PastEncoderCount = EncoderCount;
  return EncoderState;
}

void encoderTask(void *pvParameters) {
  while (1) {
    switch (encoderState()) {
      case 1:
        if (!curLayer) {
          bleKeyboard.press(KEY_MEDIA_VOLUME_UP);
          bleKeyboard.release(KEY_MEDIA_VOLUME_UP);
        } else {
          if (curLayer == 2) {
            bleKeyboard.move(0, 0, 1, 0);
          } else {
            matrixPress(KC_F15, 0);
            matrixRelease(KC_F15);
          }
        }
        break;
      case 2:
        if (!curLayer) {
          bleKeyboard.press(KEY_MEDIA_VOLUME_DOWN);
          bleKeyboard.release(KEY_MEDIA_VOLUME_DOWN);
        } else {
          if (curLayer == 2) {
            bleKeyboard.move(0, 0, -1, 0);
          } else {
            matrixPress(KC_F14, 0);
            matrixRelease(KC_F14);
          }
        }
        break;

      default:
        break;
    };
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void u8x8start(uint8_t _stage) {
  switch (_stage) {
    case 0:
      u8x8.begin();
      u8x8.clearBuffer();
      u8x8.setPowerSave(0);
      u8x8.setContrast(1);
      u8x8.setFont(u8g2_font_streamline_all_t);
      u8x8.drawGlyph(5, 83, 0x0093);
      break;
    case 1:
      u8x8.setFont(u8g2_font_streamline_all_t);
      u8x8.drawGlyph(5, 62, 0x0083);
      break;
    case 2:
      u8x8.setFont(u8g2_font_streamline_all_t);
      u8x8.drawGlyph(5, 26, 0x0091);
      break;
    case 3:
      u8x8.clearBuffer();
      u8x8.setFont(u8g2_font_streamline_all_t);
      u8x8.drawGlyph(5, 43, 0x018d);
      u8x8.drawGlyph(5, 83, 0x0164);
      break;

    default:
      break;
  }
  u8x8.sendBuffer();
  u8x8.refreshDisplay();
}

void eepromStart() {
  EEPROM.begin(4);
  deviceChose = EEPROM.read(0);
  if (deviceChose > (maxBTdev - 1)) {
    deviceChose = 0;
    EEPROM.write(0, deviceChose);
  }
  esp_base_mac_addr_set(&MACAddress[deviceChose][0]);
}

uint8_t checkKey(uint16_t _key) {
  uint16_t _cur = 0;
  gpio_set_level(rowPins[0], 1);
  for (int n = 0; n < 2000; n++) {
    _cur = 0;
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      _cur |= (gpio_get_level(colPins[col]) << col);
    }
    if (_cur == _key) {
      return (1);
    }
    delay(1);
  }
  return (0);
}

void checkOK() {
  if (checkKey(0)) {
    if (checkKey(256)) {
      if (checkKey(0)) {
        return;
      }
    }
  }
  rtc_matrix_setup();
  u8x8.setPowerSave(1);
  esp_deep_sleep_start();
}

void setup() {
  u8x8start(0);
  rtc_matrix_deinit();
  Serial.begin(115200);
  delay(500);
  u8x8start(1);
  delay(500);
  u8x8start(2);
  Serial.printf("\nStarting MMK %s\n", VERSION_SHORT);
  eepromStart();
  bleKeyboard.setName(GATTS_TAG);
  bleKeyboard.begin();
  matrixSetup();
  encoderSetup();
  delay(1000);
  u8x8start(3);
  checkOK();
  xTaskCreate(&keyboardTask, "keyboard task", 2048, NULL, 5, NULL);
  xTaskCreate(&WifiTask, "wifi task", 2048, NULL, 5, NULL);
  xTaskCreate(&otaTask, "OTA task", 2048, NULL, 5, NULL);
  xTaskCreate(&encoderTask, "encoder task", 2048, NULL, 5, NULL);
}

void loop(void) {
  msec = esp_timer_get_time();
  fps++;
  if ((msec - lsec) > uint64_t(100000)) {  // every 0.1 mili second
    lsec = msec;
    if (!otaUpdate) {
      tmOut++;
    } else {
      if (powerSave == 1) {
        powerSave = 2;
      }
    }
    // ArduinoOTA.handle();
    if (reportReady > 0) {
      Serial.printf("reportReady = %d ", reportReady);
      bleKeyboard.sendReport(&report);
      for (uint8_t i = 0; i < 6; i++) {
        if (report.keys[i] == releaseReport.keys[i]) {
          releaseReport.keys[i] = 0x00;
          report.keys[i] = 0x00;
        }
      }
      delay(9);
      bleKeyboard.sendReport(&report);
      Serial.printf("reportReady = %d\n", reportReady);
      reportReady = 0;
    }
    if ((powerSave == 2) || ((powerSave == 1) && (tmOut == 1))) {
      powerSave = 0;
      u8x8.setPowerSave(powerSave);
      Serial.println("Display on");
    }
    if ((tmOut > SLEEP_DISPLAY) | (powerOff == 1)) {
      if ((powerSave != 1) & (powerOff != 1)) {
        powerSave = 1;
        u8x8.setPowerSave(powerSave);
        Serial.println("Display off");
      }
      if ((tmOut > SLEEP_CPU) | (powerOff == 1)) {
        oledDraw(3);
        rtc_matrix_setup();
        Serial.println("Going to sleep now");
        Serial.flush();
        delay(3000);
        u8x8.setPowerSave(1);
        Serial.println("bye ..");
        esp_deep_sleep_start();
        Serial.println("Wake up from sleep");
        esp_restart();
        rtc_matrix_deinit();
        powerSave = 2;
      }
    } else {
      bleKeyboard.setBatteryLevel(getBatteryLevel());
      if (otaUpdate) {
        oledDraw(1);
      } else {
        if (sleepDebug == 1) {
          oledDraw(2);
        } else {
          oledDraw();
        }
      }
    }
    fps = 0;
  }
}
