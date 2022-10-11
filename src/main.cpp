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

uint16_t fps, tmOut = 0;

void drawOled() {
  u8x8.clearBuffer();
  u8x8.setFont(u8g2_font_5x7_tf);
  u8x8.drawStr(0, 8, "FPS");
  u8x8.drawStr(0, 16, String(fps).c_str());
  bleKeyboard.setBatteryLevel(getBatteryLevel());
  u8x8.setFont(u8g2_font_12x6LED_tr);
  u8x8.drawStr(0, 127, String(getBatteryVoltage()).c_str());
  u8x8.drawStr(22, 127, "V");
  if (bleKeyboard.isConnected()) {
    u8x8.drawStr(0, 111, "BLE1");
    u8x8.drawStr(22, 111, String(deviceChose).c_str());
    switch (deviceChose) {
      case 0:
        u8x8.drawStr(0, 95, "iPad");
        break;
      case 1:
        u8x8.drawStr(0, 95, "MBP");
        break;
      case 2:
        u8x8.drawStr(0, 95, "iMac");
        break;
    }
  } else {
    u8x8.drawStr(0, 111, "----");
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
  Serial.printf("\nat matrixTick:%d matrixPress(0x%04x, %d)\n", matrixTick,
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
  Serial.printf("\nat matrixTick:%d matrixRelease(0x%04x)\n", matrixTick,
                keycode);
  if (keycode & 0x8000) {
    curLayer = 0;
  }
  return;
  if (keycode & 0x4000) {
    mediaReport[0] = (1 << ((keycode & 0x00FF) - 0x0C));
    bleKeyboard.release(mediaReport);
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
          }
          break;
        case KS_DOWN:
          if (matrixTick >
              uint32_t(keyEvents[row][col].time_press + MODTAP_TIME)) {
            if (keyEvents[row][col].pressed == 1) {
              keyEvents[row][col].state = KS_HOLD;
              // Serial.printf("press %d 0x%04x .. ", keycode,
              // keyMap[curLayer][row][col]);
              matrixPress(keycode, 1);
            } else {
              keyEvents[row][col].state = KS_TAP;
            }
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
          if (keyEvents[row][col].pressed == 0) {
            keyEvents[row][col].state = KS_RELASE;
            Serial.printf("write %d\n", keycode);
            matrixPress(keycode, 0);
          }
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

void keyboardSetup() {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      keyEvents[row][col].state = KS_UP;
      keyEvents[row][col].time_press = 0;
    }
  }
}

// Task for continually scaning keyboard
void keyboardTask(void *pvParameters) {
  while (1) {
    matrixScan();
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
  Serial.begin(115200);
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
  u8x8.begin();
  u8x8.clearBuffer();
  u8x8.setPowerSave(0);
  u8x8.setContrast(1);
  matrixSetup();
  xTaskCreate(&keyboardTask, "keyboard task", 2048, NULL, 5, NULL);
}

// TODO deep sleep
void loop(void) {
  msec = esp_timer_get_time();
  fps++;
  if ((msec - lsec) > uint64_t(100000)) {  // every 1 mili second
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
    if (powerSave == 2) {
      powerSave = 0;
      u8x8.setPowerSave(powerSave);
    }
    if (powerSave == 0) {
      if (tmOut > SLEEP_DELAY) {
        powerSave = 1;
        u8x8.setPowerSave(powerSave);
      } else {
        drawOled();
      }
      fps = 0;
    }
  }
  // bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
  // bleKeyboard.press(KEY_LEFT_CTRL);
  // bleKeyboard.releaseAll();
}