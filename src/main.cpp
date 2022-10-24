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
            report.modifiers |= (1 >> (keycode - KC_LCTRL));
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
        if ((keycode >= KC_LCTRL) && (keycode <= KC_RGUI)) {
          report.modifiers &= (1 >> (keycode - KC_LCTRL));
        }
      }
      Serial.printf("0x%04x ", releaseReport.keys[i]);
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
          if (matrixTick >
              uint32_t(keyEvents[row][col].time_press + MODTAP_TIME)) {
            if (keyEvents[row][col].pressed == 1) {
              keyEvents[row][col].state = KS_HOLD;
              // Serial.printf("press %d 0x%04x .. ", keycode,
              // keyMap[curLayer][row][col]);
              if ((keycode >= BT_1) && (keycode <= BT_3)) {
                changeID(keycode -
                         BT_1);  // reboot to select diffrent BT device
              }
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

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

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
      ESP_LOGI(GPIO_TAG, "%d is level %d", rowPins[row],
               gpio_get_level(rowPins[row]));
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
      ESP_LOGI(GPIO_TAG, "%d is level %d", colPins[col],
               gpio_get_level(colPins[col]));
    }
    esp_sleep_enable_ext1_wakeup(rtc_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  }
}

void setup() {
  rtc_matrix_deinit();
  Serial.begin(115200);
  delay(1000);
  print_wakeup_reason();
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
    if (tmOut > SLEEP_DISPLAY) {
      // Serial.printf("tmOut:%d\n", tmOut);
      if (powerSave != 1) {
        powerSave = 1;
        u8x8.setPowerSave(powerSave);
        Serial.println("Display off");
      }
      if (tmOut > SLEEP_CPU) {
        rtc_matrix_setup();
        Serial.println("Going to sleep now");
        Serial.flush();
        delay(1000);
        // esp_light_sleep_start();
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