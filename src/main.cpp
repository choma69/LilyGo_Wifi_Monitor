#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <vector>
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

TFT_eSPI tft = TFT_eSPI();

// ---------- USER TUNABLES (font sizes at one place) ----------
const int ENTRY_TEXT_SIZE = 2;            // menu SSID + RSSI text size
const int BAT_TEXT_SIZE = 2;              // battery percent text size in menu
const int MONITOR_SSID_SIZE = 2;
const int MONITOR_RSSI_SIZE = 6;
const int COMPACT_BAR_W = 48;
const int COMPACT_BAR_H = 8;
// ------------------------------------------------------------

uint8_t BTN_NEXT_PIN   = 35; // NEXT (scroll) - na T-Display bývá GPIO35 (bez interního pullupu)
uint8_t BTN_SELECT_PIN = 0;  // SELECT (enter/back), long-press = manual rescan

// battery / charge pin (changed to 14)
const int BAT_ADC_PIN = 34;   // ADC-only pin for battery voltage via divider
const int CHARGE_PIN  = 14;   // charger STAT pin (GPIO14)
const bool CHARGE_ACTIVE_LOW = true; // STAT low when charging (typical)
const float VOLTAGE_DIVIDER_RATIO = 0.5f; // e.g. 100k/100k

// ADC settings
const adc_bits_width_t ADC_WIDTH = ADC_WIDTH_BIT_12;
const adc_atten_t ADC_ATTEN_CAL = ADC_ATTEN_DB_11;
const adc_attenuation_t ADC_ATTEN_HAL = ADC_11db;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long LONG_PRESS_MS = 900;
const unsigned long MONITOR_SCAN_INTERVAL_MIN_GAP_MS = 20;
const unsigned long BATT_READ_MS = 400;

// App state (no CHANNELS)
enum AppState { STATE_MENU, STATE_MONITOR, STATE_SCANNING };
AppState appState = STATE_SCANNING;

struct NetItem {
  String ssid;
  int rssi;
  int32_t encryptionType;
  String bssid;
  int channel;
};
std::vector<NetItem> networks;
int menuIndex = 0;
int menuOffset = 0;
String selectedSSID = "";

// scanning state
volatile bool scanInProgress = false;
unsigned long lastScanStart = 0;
unsigned long lastMonitorScanComplete = 0;
AppState scanReturnState = STATE_MENU;

// button queue
typedef struct { uint8_t pin; uint32_t tick; } BtnMsg;
static QueueHandle_t btnQueue = NULL;

// button runtime state
struct BtnState {
  int lastStable;
  unsigned long pressStart;
  bool longReported;
  unsigned long lastEventMillis;
};
BtnState btnNextState, btnSelectState;

// battery ADC
esp_adc_cal_characteristics_t *adc_chars;
float emaBatteryV = 0.0f;
bool emaBattInit = false;

// calibration V->% (keeps previous mapping)
struct CalPoint { float v; int pct; };
CalPoint calibPoints[] = {
  {4.20f, 100},{4.10f, 98},{4.00f, 95},{3.95f, 90},
  {3.85f, 80},{3.70f, 50},{3.50f, 20},{3.30f, 0}
};
const int CALIB_N = sizeof(calibPoints) / sizeof(calibPoints[0]);

// forward decls
void startAsyncScan(AppState returnState);
void processScanResults();
void drawMenu();
void drawMonitor(const String &ssid, int rssi);
int findRssiForSSID(const String &ssid);
int findChannelForSSID(const String &ssid);
int rssiToPercent(int rssi);
void drawHorizontalBar(int x, int y, int w, int h, int percent);
int calcRowsVisible(int lineH, int y0, int footerH);
uint16_t rssiColor(int rssi);
void drawBatteryIcon(int x, int y, int w, int h, int pct, bool charging);
int batteryPercentFromVoltageCalibrated(float v);
bool isCharging();
void initBatteryMonitor();
float readBatteryVoltage();

void IRAM_ATTR isrNext() {
  if (btnQueue) { BtnMsg m = { BTN_NEXT_PIN, (uint32_t)xTaskGetTickCountFromISR() }; xQueueSendFromISR(btnQueue, &m, NULL); }
}
void IRAM_ATTR isrSelect() {
  if (btnQueue) { BtnMsg m = { BTN_SELECT_PIN, (uint32_t)xTaskGetTickCountFromISR() }; xQueueSendFromISR(btnQueue, &m, NULL); }
}

void startAsyncScan(AppState returnState) {
  if (scanInProgress) return;
  if (millis() - lastScanStart < MONITOR_SCAN_INTERVAL_MIN_GAP_MS) return;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(8);
  WiFi.scanNetworks(true, true);
  scanInProgress = true;
  lastScanStart = millis();
  scanReturnState = returnState;
}

void processScanResults() {
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) return;
  if (n >= 0) {
    networks.clear(); networks.reserve(n);
    for (int i = 0; i < n; ++i) {
      NetItem it;
      it.ssid = WiFi.SSID(i);
      it.rssi = WiFi.RSSI(i);
      it.encryptionType = WiFi.encryptionType(i);
      uint8_t *b = WiFi.BSSID(i);
      char mac[32];
      sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", b[0],b[1],b[2],b[3],b[4],b[5]);
      it.bssid = String(mac);
      it.channel = WiFi.channel(i);
      networks.push_back(it);
    }
    WiFi.scanDelete();
    scanInProgress = false;
    lastMonitorScanComplete = millis();

    if (menuIndex >= (int)networks.size()) menuIndex = max(0, (int)networks.size()-1);
    int menuLineH = (12 + 8 * (ENTRY_TEXT_SIZE - 1));
    int menuVisible = calcRowsVisible(menuLineH, 30, 0);
    if (menuOffset > max(0, (int)networks.size() - menuVisible)) menuOffset = max(0, (int)networks.size() - menuVisible);
    if (menuIndex < menuOffset) menuOffset = menuIndex;
    if (menuIndex >= menuOffset + menuVisible) menuOffset = menuIndex - menuVisible + 1;

    if (appState == STATE_SCANNING || appState == scanReturnState) appState = scanReturnState;

    if (appState == STATE_MENU) drawMenu();
    else if (appState == STATE_MONITOR) {
      int r = findRssiForSSID(selectedSSID);
      drawMonitor(selectedSSID, r);
      if (!scanInProgress) startAsyncScan(STATE_MONITOR);
    }
    return;
  }
}

int calcRowsVisible(int lineH, int y0, int footerH) {
  int avail = tft.height() - y0 - footerH;
  int per = max(1, avail / (lineH + 6));
  return per;
}

uint16_t rssiColor(int rssi) {
  if (rssi > -65) return TFT_GREEN;
  if (rssi > -80) return TFT_ORANGE;
  return TFT_RED;
}

void drawHorizontalBar(int x, int y, int w, int h, int percent) {
  tft.fillRect(x, y, w, h, TFT_DARKGREY);
  int pw = (w * percent) / 100;
  if (pw > 0) {
    uint16_t col = (percent > 66) ? TFT_GREEN : (percent > 33) ? TFT_ORANGE : TFT_RED;
    tft.fillRect(x, y, pw, h, col);
  }
}

void drawBatteryIcon(int x, int y, int w, int h, int pct, bool charging) {
  int capW = 6;
  int bodyW = max(8, w - capW);
  int bodyH = h;
  int bodyX = x - bodyW - capW + 0;
  int bodyY = y;
  int capX = bodyX + bodyW;
  int capY = bodyY + bodyH / 4;
  int capH = bodyH / 2;

  tft.drawRect(bodyX, bodyY, bodyW, bodyH, TFT_WHITE);
  tft.fillRect(capX, capY, capW, capH, TFT_WHITE);

  int innerW = bodyW - 2;
  int innerH = bodyH - 2;
  int fillW = (innerW * pct) / 100;
  if (fillW < 0) fillW = 0;
  tft.fillRect(bodyX + 1, bodyY + 1, innerW, innerH, TFT_BLACK);
  if (fillW > 0) {
    uint16_t col = (pct > 66) ? TFT_GREEN : (pct > 33) ? TFT_ORANGE : TFT_RED;
    tft.fillRect(bodyX + 1, bodyY + 1, fillW, innerH, col);
  }

  if (charging) {
    int bx = bodyX + bodyW / 2 - 5;
    int by = bodyY + bodyH / 2 - 8;
    tft.fillTriangle(bx + 3, by + 0, bx + 0, by + 8, bx + 6, by + 6, TFT_YELLOW);
    tft.fillTriangle(bx + 3, by + 0, bx + 6, by + 6, bx + 10, by + 0, TFT_YELLOW);
  }
}

void drawMenu() {
  tft.fillScreen(TFT_BLACK);

  int battPct = batteryPercentFromVoltageCalibrated(emaBatteryV);
  bool charging = isCharging();

  int iconW = 40;
  int iconH = 14;
  int iconRightX = tft.width() - 6;
  int iconY = 8;

  drawBatteryIcon(iconRightX, iconY, iconW, iconH, battPct, charging);

  tft.setTextSize(BAT_TEXT_SIZE);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(charging ? TFT_GREEN : TFT_YELLOW, TFT_BLACK);
  tft.drawString(String(battPct) + "%", iconRightX - iconW - 6, 12);

  tft.setTextSize(ENTRY_TEXT_SIZE);
  int lineH = (12 + 8 * (ENTRY_TEXT_SIZE - 1));
  int y0 = 30;
  int maxVisible = calcRowsVisible(lineH, y0, 0);
  if (menuOffset > max(0, (int)networks.size() - maxVisible)) menuOffset = max(0, (int)networks.size() - maxVisible);
  if (menuIndex < menuOffset) menuOffset = menuIndex;
  if (menuIndex >= menuOffset + maxVisible) menuOffset = menuIndex - maxVisible + 1;

  int y = y0;
  const int markerX = 8;
  const int textX = markerX + 18;
  const int rssiX = tft.width() - 8 - COMPACT_BAR_W - 8;
  const int barX = tft.width() - COMPACT_BAR_W - 6;

  for (int i = 0; i < maxVisible; ++i) {
    int idx = menuOffset + i;
    int boxH = lineH + 6;
    if (idx >= (int)networks.size()) {
      tft.fillRect(8, y - 2, tft.width() - 16, boxH, TFT_BLACK);
    } else {
      bool sel = (idx == menuIndex);
      if (sel) tft.fillRect(8, y - 2, tft.width() - 16, boxH, TFT_BLUE);
      else tft.fillRect(8, y - 2, tft.width() - 16, boxH, TFT_BLACK);

      tft.setTextSize(1);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(sel ? TFT_WHITE : TFT_DARKGREY, sel ? TFT_BLUE : TFT_BLACK);
      tft.drawString(sel ? "★" : " ", markerX, y);

      tft.setTextSize(ENTRY_TEXT_SIZE);
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(sel ? TFT_WHITE : TFT_CYAN, sel ? TFT_BLUE : TFT_BLACK);
      String ss = networks[idx].ssid;
      int approxCharW = 7 * ENTRY_TEXT_SIZE;
      int maxChars = max(6, (rssiX - textX) / approxCharW);
      if (ss.length() > maxChars) ss = ss.substring(0, maxChars) + "...";
      tft.drawString(ss, textX, y);

      int r = networks[idx].rssi;
      uint16_t rc = rssiColor(r);
      tft.setTextSize(1);
      tft.setTextDatum(TR_DATUM);
      tft.setTextColor(sel ? TFT_WHITE : rc, sel ? TFT_BLUE : TFT_BLACK);
      tft.drawString(String(r) + " dB", rssiX, y + 2);

      int p = rssiToPercent(r);
      drawHorizontalBar(barX, y + (boxH - COMPACT_BAR_H)/2, COMPACT_BAR_W, COMPACT_BAR_H, p);
    }
    y += lineH + 6;
  }
}

void drawMonitor(const String &ssid, int rssi) {
  tft.fillScreen(TFT_BLACK);

  tft.setTextSize(MONITOR_SSID_SIZE);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString(ssid.length() ? ssid : "(unknown)", tft.width()/2, 12);

  tft.setTextSize(MONITOR_RSSI_SIZE);
  tft.setTextDatum(MC_DATUM);
  uint16_t rc = rssiColor(rssi);
  tft.setTextColor(rc, TFT_BLACK);
  String dbm = (rssi <= -998) ? "--- dBm" : String(rssi) + " dBm";
  tft.drawString(dbm, tft.width()/2, tft.height()/2 - 6);

  int barW = tft.width() - 32;
  int barH = 12;
  int barX = 16;
  int barY = tft.height() - 36;
  int perc = (rssi <= -999) ? 0 : rssiToPercent(rssi);
  drawHorizontalBar(barX, barY, barW, barH, perc);
}

int findRssiForSSID(const String &ssid) {
  for (auto &n : networks) if (n.ssid == ssid) return n.rssi;
  return -999;
}
int findChannelForSSID(const String &ssid) {
  for (auto &n : networks) if (n.ssid == ssid) return n.channel;
  return -1;
}
int rssiToPercent(int rssi) {
  int p = map(rssi, -100, -50, 0, 100);
  return constrain(p, 0, 100);
}

void initBatteryMonitor() {
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_ATTEN_HAL);
  analogSetWidth(ADC_WIDTH);
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_CAL, ADC_WIDTH, 0, adc_chars);
}
float readBatteryVoltage() {
  uint32_t raw = analogRead(BAT_ADC_PIN);
  uint32_t v_adc_mV = esp_adc_cal_raw_to_voltage(raw, adc_chars);
  float v_adc = (float)v_adc_mV / 1000.0f;
  float vbat = v_adc / VOLTAGE_DIVIDER_RATIO;
  if (!emaBattInit) { emaBatteryV = vbat; emaBattInit = true; }
  else emaBatteryV = 0.35f * vbat + (1.0f - 0.35f) * emaBatteryV;
  return vbat;
}
int batteryPercentFromVoltageCalibrated(float v) {
  if (v >= calibPoints[0].v) return calibPoints[0].pct;
  if (v <= calibPoints[CALIB_N - 1].v) return calibPoints[CALIB_N - 1].pct;
  for (int i=0;i<CALIB_N-1;i++){
    float v_hi = calibPoints[i].v, v_lo = calibPoints[i+1].v;
    if (v <= v_hi && v >= v_lo) {
      int p_hi = calibPoints[i].pct, p_lo = calibPoints[i+1].pct;
      float t = (v - v_lo) / (v_hi - v_lo);
      return (int)round(p_lo + t * (p_hi - p_lo));
    }
  }
  return 0;
}
bool isCharging() {
  int val = digitalRead(CHARGE_PIN);
  return CHARGE_ACTIVE_LOW ? (val == LOW) : (val == HIGH);
}

void setup() {
  Serial.begin(115200); delay(50);
  Serial.println("WiFi menu - battery icon + bolt (charge pin GPIO14)");

  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);

  btnQueue = xQueueCreate(8, sizeof(BtnMsg));
  btnNextState.lastStable = digitalRead(BTN_NEXT_PIN);
  btnNextState.pressStart = 0; btnNextState.longReported = false; btnNextState.lastEventMillis = 0;
  btnSelectState.lastStable = digitalRead(BTN_SELECT_PIN);
  btnSelectState.pressStart = 0; btnSelectState.longReported = false; btnSelectState.lastEventMillis = 0;

  pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
  pinMode(BTN_SELECT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_NEXT_PIN), isrNext, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BTN_SELECT_PIN), isrSelect, CHANGE);

  initBatteryMonitor();
  pinMode(CHARGE_PIN, INPUT_PULLUP);

  startAsyncScan(STATE_MENU);
  appState = STATE_SCANNING;
}

void loop() {
  if (scanInProgress) processScanResults();

  unsigned long now = millis();
  if (appState == STATE_MONITOR) {
    if (!scanInProgress && (now - lastMonitorScanComplete >= MONITOR_SCAN_INTERVAL_MIN_GAP_MS)) startAsyncScan(STATE_MONITOR);
  }

  static unsigned long lastBattRead = 0;
  if (now - lastBattRead >= BATT_READ_MS) { readBatteryVoltage(); lastBattRead = now; }

  BtnMsg m;
  while (btnQueue && xQueueReceive(btnQueue, &m, 0) == pdTRUE) {
    unsigned long evNow = millis();
    BtnState *s = (m.pin == BTN_NEXT_PIN) ? &btnNextState : (m.pin == BTN_SELECT_PIN) ? &btnSelectState : nullptr;
    if (!s) continue;
    if (evNow - s->lastEventMillis < DEBOUNCE_MS) continue;
    s->lastEventMillis = evNow;

    int cur = digitalRead(m.pin);
    if (cur == LOW && s->lastStable == HIGH) {
      s->pressStart = evNow; s->longReported = false; s->lastStable = LOW;
    } else if (cur == HIGH && s->lastStable == LOW) {
      unsigned long dur = (s->pressStart == 0) ? 0 : (evNow - s->pressStart);
      s->pressStart = 0; s->lastStable = HIGH;
      if (s->longReported) { s->longReported = false; }
      else {
        if (m.pin == BTN_NEXT_PIN) {
          if (appState == STATE_MENU) {
            if (networks.size()) {
              menuIndex = (menuIndex + 1) % max(1, (int)networks.size());
              int lineH = (12 + 8 * (ENTRY_TEXT_SIZE - 1));
              int rows = calcRowsVisible(lineH, 30, 0);
              if (menuIndex < menuOffset) menuOffset = menuIndex;
              if (menuIndex >= menuOffset + rows) menuOffset = menuIndex - rows + 1;
              drawMenu();
            }
          } else if (appState == STATE_MONITOR) {
            if (!scanInProgress) startAsyncScan(STATE_MONITOR);
          }
        } else if (m.pin == BTN_SELECT_PIN) {
          if (appState == STATE_MENU) {
            if (networks.size()) {
              selectedSSID = networks[menuIndex].ssid;
              appState = STATE_MONITOR;
              if (!scanInProgress) startAsyncScan(STATE_MONITOR);
              else drawMonitor(selectedSSID, findRssiForSSID(selectedSSID));
            }
          } else if (appState == STATE_MONITOR) {
            scanReturnState = STATE_MENU;
            appState = STATE_MENU;
            drawMenu();
          }
        }
      }
    }
  }

  if (btnNextState.lastStable == LOW && btnNextState.pressStart != 0 && !btnNextState.longReported) {
    if (now - btnNextState.pressStart >= LONG_PRESS_MS) {
      btnNextState.longReported = true;
      Serial.println("NEXT long-press detected (no channel-scan configured)");
    }
  }
  if (btnSelectState.lastStable == LOW && btnSelectState.pressStart != 0 && !btnSelectState.longReported) {
    if (now - btnSelectState.pressStart >= LONG_PRESS_MS) {
      btnSelectState.longReported = true;
      Serial.println("SELECT long-press -> manual rescan");
      startAsyncScan(appState == STATE_MONITOR ? STATE_MONITOR : STATE_MENU);
    }
  }

  delay(2);
}
