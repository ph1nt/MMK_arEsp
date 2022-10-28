#include <Arduino.h>
#include <config.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8x8(U8G2_R3, /* reset=*/U8X8_PIN_NONE,
                                            /* clock=*/OLED_SCL_PIN,
                                            /* data=*/OLED_SDA_PIN);
BleKeyboard bleKeyboard(GATTS_TAG, "HNA", 100);

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
  float temp_c = (temp_f - 32) / 1.8;
  return temp_c;
}

uint16_t fps, tmOut, sleepDebug = 0;

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

#define Xpos(_col) (6 * _col)
#define Ypos(_row) (12 * _row)
#define posY(_row) (127 - Ypos(_row))
void drawOled() {
  u8x8.clearBuffer();
  u8x8.setFont(u8g2_font_5x7_tf);
  u8x8.drawStr(0, 8, "FPS");
  u8x8.drawStr(0, 16, String(fps).c_str());
  bleKeyboard.setBatteryLevel(getBatteryLevel());
  if (curLayer == 0) {
    u8x8.setFont(u8g2_font_4x6_tf);
  }
  u8x8.drawStr(0, Ypos(5), layers[curLayer]);
  u8x8.setFont(u8g2_font_6x12_mf);
  u8x8.drawStr(0, Ypos(3), rtc.getTime("%H:%M").c_str());
  u8x8.drawStr(0, Ypos(4), rtc.getTime("%d%h").c_str());
  // X:  6 12 18 24 30 | 1 pix left
  // Y: 12 24 36 48 60 72 84 96 108 120 | 8 pix left
  // from bottom: 115 103 91 79 67 55 43 31 19
  u8x8.drawStr(0, posY(0), String(getBatteryVoltage()).c_str());
  u8x8.drawStr(24, posY(0), "V");
  u8x8.drawStr(0, posY(1), String(tempCpu(), 1).c_str());
  u8x8.drawStr(25, posY(1), String(char(176)).c_str());
  if (bleKeyboard.isConnected()) {
    u8x8.drawStr(0, posY(3), "BLE");
    u8x8.drawStr(22, posY(3), String(deviceChose).c_str());
    // TODO read string from eprom
    switch (deviceChose) {
      case 0:
      case 1:
      case 2:
        u8x8.drawStr(0, posY(4), devs[deviceChose]);
        break;
    }
  } else {
    u8x8.drawStr(0, posY(4), "no BL");
    u8x8.drawStr(0, posY(3), " dev.");
  }
  u8x8.sendBuffer();
  u8x8.refreshDisplay();
}

void changeID(int DevNum) {
  // Serial.println("changing MAC...");
  // Make sure the selection is valid
  if (DevNum < maxBTdev) {
    // Write and commit to storage, reset ESP 32
    EEPROM.write(0, DevNum);
    EEPROM.commit();
    // esp_reset();
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
  gpio_pad_select_gpio(ENCODER_A_PIN);
  gpio_set_direction(ENCODER_A_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_A_PIN, GPIO_PULLUP_ONLY);
  gpio_pad_select_gpio(ENCODER_B_PIN);
  gpio_set_direction(ENCODER_B_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(ENCODER_B_PIN, GPIO_PULLUP_ONLY);
}

// Scanning physical keys matrix
void matrixScan() {
  uint8_t curState = 0;
  matrixTick++;
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    gpio_set_level(rowPins[row], 1);
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      curState = gpio_get_level(colPins[col]);
      if (keyEvents[row][col].curState != curState) {
        keyEvents[row][col].time_debounce = matrixTick;
      }
      if ((matrixTick - keyEvents[row][col].time_debounce) > DEBOUNCE) {
        if (keyEvents[row][col].pressed != curState) {
          matrixChange = 1;
          keyEvents[row][col].pressed = curState;
        }
      }
      keyEvents[row][col].curState = curState;
    }
    gpio_set_level(rowPins[row], 0);
  }
}

void matrixPress(uint16_t keycode, uint8_t _hold) {
  uint8_t k = 0;
  uint8_t mediaReport[2] = {0, 0};
  Serial.printf("\nat matrixTick:%lld matrixPress(0x%04x, %d)\n", matrixTick,
                keycode, _hold);
  if (keycode & 0x8000) {
    if (_hold) {
      curLayer = (keycode & 0x0F00) >> 8;
    } else {
      k = (keycode & 0x00FF);
    }
    Serial.printf("layer 0x%02x or TAP key 0x%02x ", (keycode & 0x0F00) >> 8,
                  (keycode & 0x00FF));
  } else {
    if (keycode & 0x4000) {
      mediaReport[0] = (1 << ((keycode & 0x00FF) - 0x0C));
      bleKeyboard.press(mediaReport);
      Serial.printf("consumer control 0x%02x ", (keycode & 0x00FF));
    } else {
      if (keycode & 0x2000) {
        if (_hold) {
          report.modifiers |=
              ((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
        } else {
          k = (keycode & 0x00FF);
        }
        Serial.printf("modifier 0x%02x or TAP key 0x%02x ", report.modifiers,
                      (keycode & 0x00FF));
      } else {
        if (keycode & 0x1F00) {
          report.modifiers |=
              ((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
          k = (keycode & 0x00FF);
          Serial.printf("modifier 0x%02x and key 0x%02x ",
                        (keycode & 0x1F00) >> 8, (keycode & 0x00FF));
        } else {
          k = (keycode & 0x00FF);
          if ((keycode >= KC_LCTRL) && (keycode <= KC_RGUI)) {
            report.modifiers |= (1 << (keycode - KC_LCTRL));
            Serial.printf("report.modifiers |= 0x%04x, keycode 0x%04x\n",
                          (1 << (keycode - KC_LCTRL)), keycode);
          }
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
          Serial.printf("report.keys[%d] = %d", i, k);
          report.keys[i] = k;
          break;
        }
      }
    }
  }
  Serial.printf(" layer:%d keycode:0x%04x\n", curLayer, k);
}

void matrixRelease(uint16_t keycode) {
  // TODO mod release
  uint8_t k;
  uint8_t mediaReport[2] = {0, 0};
  Serial.printf("\nat matrixTick:%lld matrixRelease(0x%04x)\n", matrixTick,
                keycode);
  if (keycode & 0x8000) {
    curLayer = 0;
  }
  if (keycode & 0x4000) {
    mediaReport[0] = (1 << ((keycode & 0x00FF) - 0x0C));
    bleKeyboard.release(mediaReport);
    return;
  } else {
    if (keycode & 0x1F00) {
      report.modifiers &= ~((keycode & 0x0F00) >> (4 + 4 * (keycode & 0x1000)));
      Serial.printf("release modifier 0x%02x and key 0x%02x\n",
                    (keycode & 0x1F00) >> 8, (keycode & 0x00FF));
    }
    k = (keycode & 0x00FF);
    for (uint8_t i = 0; i < 6; i++) {
      if (0 != k && report.keys[i] == k) {
        releaseReport.keys[i] = k;
      }
      Serial.printf("0x%04x ", releaseReport.keys[i]);
    }
    if ((keycode >= KC_LCTRL) && (keycode <= KC_RGUI)) {
      report.modifiers &= !(1 << (keycode - KC_LCTRL));
      Serial.printf("report.modifiers &= 0x%04x, keycode 0x%04x\n",
                    (1 << (keycode - KC_LCTRL)), keycode);
    }
    Serial.printf("mod:0x%04x\n", report.modifiers);
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
          }
          break;
        case KS_DOWN:
          if (keyEvents[row][col].pressed == 1) {
            if (matrixTick >
                uint64_t(keyEvents[row][col].time_press + MODTAP_TIME)) {
              keyEvents[row][col].state = KS_HOLD;
              // Serial.printf("press %d 0x%04x .. ", keycode,
              // keyMap[curLayer][row][col]);
              if ((keycode >= BT_1) && (keycode <= BT_3)) {
                changeID(keycode -
                         BT_1);  // reboot to select diffrent BT device
              }
              if (keycode == DEBUG) {
                sleepDebug = 1;
              }
              matrixPress(keycode, 1);
            }
          } else {
            keyEvents[row][col].state = KS_TAP;
          }
          break;
        case KS_HOLD:
          if (keyEvents[row][col].pressed == 0) {
            keyEvents[row][col].state = KS_UP;
            // Serial.printf("release %d\n", keycode);
            matrixRelease(keycode);
          }
          break;
        case KS_TAP:
          keyEvents[row][col].state = KS_RELASE;
          Serial.printf("write %d\n", keycode);
          matrixPress(keycode, 0);
          break;
        case KS_RELASE: {
          matrixRelease(keycode);
          keyEvents[row][col].state = KS_UP;
        } break;
        default:
          Serial.println(
              "Upss.. wrong keyEvents[row][col].state in matrixProcess()");
          break;
      }
    }
  }
  // bleKeyboard.sendReport(&report);
  reportReady = 1;
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
    if (sleepDebug == 0) {
      matrixScan();
    }
    matrixProces();
    if (matrixChange == 1) {
      matrixChange = 0;
      tmOut = 0;
      if (powerSave == 1) powerSave++;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  u8x8.begin();
  u8x8.clearBuffer();
  u8x8.setPowerSave(0);
  u8x8.setContrast(1);
  u8x8.setFont(u8g2_font_5x7_tf);
  u8x8.drawStr(0, 8, "Booting..");
  u8x8.sendBuffer();
  u8x8.refreshDisplay();
  rtc_matrix_deinit();
  // rtc.setTime(0, 27, 10, 28, 10, 2022);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting MMK");
  EEPROM.begin(4);
  deviceChose = EEPROM.read(0);
  if (deviceChose > 2) {
    deviceChose = 0;
    EEPROM.write(0, deviceChose);
  }
  esp_base_mac_addr_set(&MACAddress[deviceChose][0]);
  bleKeyboard.setName(GATTS_TAG);
  bleKeyboard.begin();
  matrixSetup();
  xTaskCreate(&keyboardTask, "keyboard task", 2048, NULL, 5, NULL);
}

// TODO deep sleep
void loop(void) {
  msec = esp_timer_get_time();
  fps++;
  if ((msec - lsec) > uint64_t(100000)) {  // every 0.1 mili second
    lsec = msec;
    tmOut++;
    if (reportReady > 0) {
      bleKeyboard.sendReport(&report);
      for (uint8_t i = 0; i < 6; i++) {
        if (report.keys[i] == releaseReport.keys[i]) {
          releaseReport.keys[i] = 0x00;
          report.keys[i] = 0x00;
        }
      }
      delay(1);
      bleKeyboard.sendReport(&report);
      reportReady = 0;
    }
    if ((powerSave == 2) || ((powerSave == 1) && (tmOut == 1))) {
      powerSave = 0;
      u8x8.setPowerSave(powerSave);
      Serial.println("Display on");
    }
    if ((tmOut > SLEEP_DISPLAY) || (sleepDebug == 1)) {
      if (powerSave != 1) {
        powerSave = 1;
        u8x8.setPowerSave(powerSave);
        Serial.println("Display off");
      }
      if ((tmOut > SLEEP_CPU) || (sleepDebug == 1)) {
        sleepDebug = 0;
        rtc_matrix_setup();
        Serial.println("Going to sleep now");
        Serial.flush();
        delay(1000);
        esp_deep_sleep_start();
        Serial.println("Wake up from sleep");
        esp_restart();
        rtc_matrix_deinit();
        powerSave = 2;
      }
    } else {
      drawOled();
    }
    fps = 0;
  }
}  // TODO pin35 encoder wakeup