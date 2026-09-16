#define BLYNK_TEMPLATE_ID "TMPL3NrfkI_p4"
#define BLYNK_TEMPLATE_NAME "SMART EV"
#define BLYNK_AUTH_TOKEN "GhHcwVlXlPKvGT808c9piLYbMRmL31Dy"

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

/******************************************************************************
 * ELITE EMBEDDED BMS - 100% COMPLIANT FREERTOS EDITION
 * Tasks 1-6: Includes Anomalies, Timed Recovery, Event-Telemetry & Ledger
 ******************************************************************************/

const char* WIFI_SSID = "Wokwi-GUEST"; 
const char* WIFI_PASS = "";
const char* BLYNK_AUTH = "GhHcwVlXlPKvGT808c9piLYbMRmL31Dy"; // MUST UPDATE THIS

// Datastream Pin Mappings
#define VPIN_CELL1 V0   
#define VPIN_CELL2 V1   
#define VPIN_CELL3 V2   
#define VPIN_CELL4 V3   
#define VPIN_WEAKEST_CELL V4   
#define VPIN_STRONGEST_CELL V5   
#define VPIN_IMBALANCE V6   
#define VPIN_SOC V7   
#define VPIN_RELAY_STATUS V8   
#define VPIN_FAULT_STATE V9   
#define VPIN_FAULT_NAME V10  
#define VPIN_WIFI_RSSI V11  
#define VPIN_QUEUE_DEPTH V12  
#define VPIN_CONNECTIVITY_STR V13  
#define VPIN_RISK_SCORE V20  
#define VPIN_HEALTH_SCORE V21  
#define VPIN_SEVERITY_COLOR V22  
#define VPIN_OPERATOR_RECOMMEND V23  
#define VPIN_EXECUTIVE_SUMMARY V24  
#define VPIN_FAULT_HISTORY_LOG V25  
#define VPIN_LIFETIME_FAULT_COUNT V26  
#define VPIN_SYSTEM_UPTIME V27  
#define VPIN_IMBALANCE_TREND_PTS V28  
#define VPIN_SOC_GRAPH V29  

// Hardware Pins
#define RELAY_PIN 19  
#define RELAY_FEEDBACK_PIN 18  
#define LED_GREEN_PIN 27
#define LED_YELLOW_PIN 26
#define LED_RED_PIN 25
#define BUZZER_PIN 4

constexpr uint8_t NUM_CELLS = 4; 
const int adcPins[NUM_CELLS] = {33, 32, 34, 35}; 
const float MIN_CELL_VOLTAGE = 3.00f;
const float MAX_CELL_VOLTAGE = 4.20f;
#define WINDOW_SIZE 5

// RTOS Mutex & NVM
SemaphoreHandle_t systemMutex;
Preferences nvm;
LiquidCrystal_I2C lcd(0x27, 16, 2);

enum SystemState { STATE_NORMAL = 0, STATE_DEGRADED, STATE_FAILSAFE, STATE_SHUTDOWN };
enum FaultID { FAULT_NONE = 0, FAULT_CELL_IMBALANCE, FAULT_CELL_OUT_OF_RANGE, FAULT_ADC_FROZEN, FAULT_ADC_JUMP, FAULT_RELAY_MISMATCH };
enum TrendType { TREND_INCREASING, TREND_DECREASING, TREND_STABLE };
enum LcdPage { PAGE_BATTERY_STATUS, PAGE_SYSTEM_STATE, PAGE_TELEMETRY };

struct BatteryInfo {
  float voltage[NUM_CELLS];
  int weakestCell, strongestCell;
  float lowestVoltage, highestVoltage, imbalance, previousImbalance, smoothedImbalance, imbalanceDerivative, soc, threshold;
  TrendType trend;
  bool imbalanceFault, rangeFault, jumpFault, frozenFault;
} battery;

struct TelemetrySnapshot {
  uint32_t timestamp;
  float cellVoltages[NUM_CELLS];
  float imbalance, imbalanceDerivative, soc;
  bool relayClosed;
  SystemState state;
  FaultID fault;
  int32_t rssi;
};

struct TransitionEvent {
  uint32_t timestamp;
  SystemState prevState;
  SystemState newState;
  FaultID fault;
};

// Global State
SystemState currentState = STATE_NORMAL;
FaultID activeFault = FAULT_NONE;
LcdPage currentPage = PAGE_BATTERY_STATUS;
uint32_t totalLifetimeFaults = 0;
float compositeRiskScore = 0.0f;
float overallHealthScore = 100.0f;
const char* activeRecommendation = "NOMINAL";

// Filters & Anomalies Variables
float voltageSamples[WINDOW_SIZE];
int sampleIndex = 0;
float previousPackVoltage = 0.0f;
uint32_t frozenStart = 0;
float lastFrozenCheckValue = 0.0f;

// History Ledger
TransitionEvent faultHistory[10];
uint8_t historyCount = 0;

// Recovery Variables
bool recoveryActive = false;
uint32_t recoveryStart = 0;

class NVMTelemetryQueue {
private:
  static constexpr uint8_t CAPACITY = 40;
  TelemetrySnapshot buffer[CAPACITY];
  uint8_t head = 0, tail = 0, count = 0;

  void persist() {
    nvm.putBytes("q_buf", buffer, sizeof(buffer));
    nvm.putUChar("q_head", head);
    nvm.putUChar("q_tail", tail);
    nvm.putUChar("q_count", count);
  }

public:
  void init() {
    if (nvm.getBytesLength("q_buf") == sizeof(buffer)) {
      nvm.getBytes("q_buf", buffer, sizeof(buffer));
      head = nvm.getUChar("q_head", 0);
      tail = nvm.getUChar("q_tail", 0);
      count = nvm.getUChar("q_count", 0);
    }
  }
  bool enqueue(const TelemetrySnapshot& item) {
    if (count >= CAPACITY) { tail = (tail + 1) % CAPACITY; count--; }
    buffer[head] = item; head = (head + 1) % CAPACITY; count++;
    persist(); return true;
  }
  bool dequeue(TelemetrySnapshot& item) {
    if (count == 0) return false;
    item = buffer[tail]; tail = (tail + 1) % CAPACITY; count--;
    persist(); return true;
  }
  uint8_t size() const { return count; }
  bool isEmpty() const { return count == 0; }
} tQueue;

String stateToString(SystemState s) {
  if (s == STATE_NORMAL) return "NORMAL";
  if (s == STATE_DEGRADED) return "DEGRADED";
  if (s == STATE_FAILSAFE) return "FAILSAFE";
  return "SHUTDOWN";
}

String faultToString(FaultID f) {
  if (f == FAULT_NONE) return "NONE";
  if (f == FAULT_CELL_IMBALANCE) return "IMBALANCE";
  if (f == FAULT_CELL_OUT_OF_RANGE) return "OUT_OF_RANGE";
  if (f == FAULT_ADC_JUMP) return "ADC_JUMP";
  if (f == FAULT_ADC_FROZEN) return "ADC_FROZEN";
  if (f == FAULT_RELAY_MISMATCH) return "RELAY_FROZEN";
  return "ERR";
}

const char* getSeverityColor(SystemState state) {
  switch (state) {
    case STATE_NORMAL: return "#00E676"; 
    case STATE_DEGRADED: return "#FFD600"; 
    case STATE_FAILSAFE: return "#FF6D00"; 
    case STATE_SHUTDOWN: return "#D50000"; 
    default: return "#FFFFFF";
  }
}

void triggerTransition(SystemState nextState, FaultID fault) {
  if (currentState == nextState && activeFault == fault) return; 
  
  if (fault != FAULT_NONE && currentState == STATE_NORMAL) {
    totalLifetimeFaults++;
    nvm.putUInt("faults", totalLifetimeFaults); 
  }
  
  if (historyCount < 10) {
    faultHistory[historyCount++] = { millis(), currentState, nextState, fault };
  } else {
    for (int i = 1; i < 10; i++) faultHistory[i-1] = faultHistory[i];
    faultHistory[9] = { millis(), currentState, nextState, fault };
  }
  
  currentState = nextState;
  activeFault = fault;
}

void vTaskBMSEngine(void *pvParameters) {
  for(;;) {
    xSemaphoreTake(systemMutex, portMAX_DELAY);
    float totalVolts = 0;
    battery.lowestVoltage = 5.0f; battery.highestVoltage = 0.0f;
    
    for (int i = 0; i < NUM_CELLS; i++) {
      float v = MIN_CELL_VOLTAGE + ((float)analogRead(adcPins[i]) / 4095.0f) * (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE);
      battery.voltage[i] = v;
      totalVolts += v;
      if (v < battery.lowestVoltage) { battery.lowestVoltage = v; battery.weakestCell = i; }
      if (v > battery.highestVoltage) { battery.highestVoltage = v; battery.strongestCell = i; }
    }
    
    float rawAvgVolts = totalVolts / NUM_CELLS;
    
    voltageSamples[sampleIndex] = rawAvgVolts;
    sampleIndex = (sampleIndex + 1) % WINDOW_SIZE;
    float filteredVolts = 0;
    for(int i = 0; i < WINDOW_SIZE; i++) filteredVolts += voltageSamples[i];
    filteredVolts /= WINDOW_SIZE;

    battery.jumpFault = (previousPackVoltage != 0.0f && fabs(filteredVolts - previousPackVoltage) > 0.40f);
    previousPackVoltage = filteredVolts;

    if (fabs(filteredVolts - lastFrozenCheckValue) < 0.0005f) {
      if (frozenStart == 0) frozenStart = millis();
      battery.frozenFault = (millis() - frozenStart > 8000); 
    } else {
      frozenStart = 0;
      battery.frozenFault = false;
    }
    lastFrozenCheckValue = filteredVolts;
    
    float currentImbalance = battery.highestVoltage - battery.lowestVoltage;
    battery.imbalanceDerivative = (currentImbalance - battery.imbalance) / 0.25f;
    battery.imbalance = currentImbalance;
    
    battery.smoothedImbalance = (0.3f * battery.imbalance) + (0.7f * battery.previousImbalance);
    if (battery.smoothedImbalance > battery.previousImbalance + 0.004f) battery.trend = TREND_INCREASING;
    else if (battery.smoothedImbalance < battery.previousImbalance - 0.004f) battery.trend = TREND_DECREASING;
    else battery.trend = TREND_STABLE;
    battery.previousImbalance = battery.smoothedImbalance;

    battery.soc = constrain(((filteredVolts - MIN_CELL_VOLTAGE) / (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE)) * 100.0f, 0, 100);
    battery.threshold = (battery.soc > 80.0f) ? 0.05f : 0.10f; 
    battery.imbalanceFault = (battery.imbalance > battery.threshold);
    battery.rangeFault = (battery.lowestVoltage < MIN_CELL_VOLTAGE || battery.highestVoltage > MAX_CELL_VOLTAGE);
    
    xSemaphoreGive(systemMutex);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void vTaskSafetyKernel(void *pvParameters) {
  uint32_t debounceStart = 0;
  for(;;) {
    xSemaphoreTake(systemMutex, portMAX_DELAY);
    
    bool relayMismatch = (digitalRead(RELAY_FEEDBACK_PIN) == HIGH) && (millis() > 2000); 

    FaultID detected = FAULT_NONE;
    if (relayMismatch) detected = FAULT_RELAY_MISMATCH;
    else if (battery.frozenFault) detected = FAULT_ADC_FROZEN;
    else if (battery.jumpFault) detected = FAULT_ADC_JUMP;
    else if (battery.rangeFault) detected = FAULT_CELL_OUT_OF_RANGE;
    else if (battery.imbalanceFault) detected = FAULT_CELL_IMBALANCE;

    if (currentState == STATE_NORMAL) {
      if (detected != FAULT_NONE) {
        if (debounceStart == 0) debounceStart = millis();
        if (millis() - debounceStart > 500) { 
          if (detected == FAULT_RELAY_MISMATCH) {
            digitalWrite(RELAY_PIN, LOW);
            triggerTransition(STATE_SHUTDOWN, detected);
          } else if (detected == FAULT_CELL_IMBALANCE) {
            triggerTransition(STATE_DEGRADED, detected);
          } else {
            digitalWrite(RELAY_PIN, LOW);
            triggerTransition(STATE_FAILSAFE, detected);
          }
        }
      } else { debounceStart = 0; }
    } 
    else if (currentState == STATE_DEGRADED) {
       if (detected == FAULT_NONE) triggerTransition(STATE_NORMAL, FAULT_NONE);
       else if (detected != FAULT_CELL_IMBALANCE) {
          digitalWrite(RELAY_PIN, LOW);
          triggerTransition(STATE_FAILSAFE, detected);
       }
    } 
    else if (currentState == STATE_SHUTDOWN || currentState == STATE_FAILSAFE) {
      if (detected == FAULT_NONE) {
        if (!recoveryActive) {
          recoveryActive = true;
          recoveryStart = millis();
        } else if (millis() - recoveryStart >= 3000) { 
          recoveryActive = false;
          digitalWrite(RELAY_PIN, HIGH);
          triggerTransition(STATE_NORMAL, FAULT_NONE);
        }
      } else {
        recoveryActive = false; 
        if (detected == FAULT_RELAY_MISMATCH && currentState != STATE_SHUTDOWN) {
           triggerTransition(STATE_SHUTDOWN, detected); 
        }
      }
    }

    digitalWrite(LED_GREEN_PIN, currentState == STATE_NORMAL);
    digitalWrite(LED_YELLOW_PIN, currentState == STATE_DEGRADED);
    digitalWrite(LED_RED_PIN, currentState == STATE_FAILSAFE || currentState == STATE_SHUTDOWN);
    digitalWrite(BUZZER_PIN, (currentState == STATE_FAILSAFE || currentState == STATE_SHUTDOWN) ? HIGH : LOW);
    
    xSemaphoreGive(systemMutex);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void vTaskHMI(void *pvParameters) {
  for(;;) {
    xSemaphoreTake(systemMutex, portMAX_DELAY);
    SystemState st = currentState; FaultID f = activeFault;
    float s = battery.soc, imb = battery.imbalance;
    TrendType tr = battery.trend;
    xSemaphoreGive(systemMutex);

    if (st == STATE_FAILSAFE || st == STATE_SHUTDOWN) {
      lcd.setCursor(0, 0); lcd.print(st == STATE_SHUTDOWN ? "!! SHUTDOWN !!  " : "!! FAILSAFE !!  ");
      lcd.setCursor(0, 1); 
      String errStr = faultToString(f);
      while(errStr.length() < 16) errStr += " ";
      lcd.print(errStr);
    } else {
      currentPage = (LcdPage)((millis() / 3000) % 3);
      if (currentPage == PAGE_BATTERY_STATUS) {
        lcd.setCursor(0, 0); lcd.print("SOC:"); lcd.print(s, 1); lcd.print("%   ");
        lcd.setCursor(0, 1); lcd.print("Imb:"); lcd.print(imb, 2); lcd.print("V ");
        if(tr == TREND_INCREASING) lcd.print("UP "); else if (tr == TREND_DECREASING) lcd.print("DN "); else lcd.print("ST ");
      } else if (currentPage == PAGE_SYSTEM_STATE) {
        lcd.setCursor(0, 0); lcd.print("SYS:"); lcd.print(stateToString(st)); lcd.print("  ");
        lcd.setCursor(0, 1); lcd.print("RLY:"); lcd.print(digitalRead(RELAY_PIN) ? "ON " : "OFF"); lcd.print(" FLT:"); lcd.print((int)f);
      } else {
        lcd.setCursor(0, 0); lcd.print("Wk:"); lcd.print(battery.weakestCell + 1); lcd.print(" St:"); lcd.print(battery.strongestCell + 1); lcd.print("   ");
        lcd.setCursor(0, 1); lcd.print("LTD Flts: "); lcd.print(totalLifetimeFaults); lcd.print("    ");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

bool evaluateSignificance(const TelemetrySnapshot& cur, const TelemetrySnapshot& last, uint32_t lastHb) {
  if (cur.state != last.state) return true;
  if (cur.fault != last.fault) return true;
  if (cur.relayClosed != last.relayClosed) return true;
  if (millis() - lastHb > 2000) return true; 
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (fabs(cur.cellVoltages[i] - last.cellVoltages[i]) > 0.015f) return true;
  }
  return false;
}

void vTaskTelemetry(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // Call begin ONCE outside the loop
  Blynk.config(BLYNK_AUTH);
  
  TelemetrySnapshot lastReported = {0};
  uint32_t lastHeartbeat = 0;
  
  for(;;) {
    bool online = false;
    
    // Safely attempt connection without spamming the module
    if (WiFi.status() == WL_CONNECTED) {
       if (!Blynk.connected()) Blynk.connect(2000); 
       online = Blynk.connected();
    } else {
       // Wait patiently for Wokwi's internal network to bridge 
       vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    xSemaphoreTake(systemMutex, portMAX_DELAY);
    TelemetrySnapshot snap = {millis(), {battery.voltage[0], battery.voltage[1], battery.voltage[2], battery.voltage[3]}, battery.imbalance, battery.imbalanceDerivative, battery.soc, digitalRead(RELAY_PIN), currentState, activeFault, WiFi.RSSI()};
    
    float risk = constrain((snap.imbalance / 0.080f) * 35.0f, 0.0f, 35.0f);
    if (snap.imbalanceDerivative > 0.001f) risk += 20.0f;
    if (snap.state == STATE_DEGRADED) risk += 15.0f;
    else if (snap.state == STATE_FAILSAFE || snap.state == STATE_SHUTDOWN) risk += 30.0f;
    compositeRiskScore = constrain(risk + (totalLifetimeFaults * 2.0f), 0.0f, 100.0f);

    overallHealthScore = constrain(100.0f - (compositeRiskScore * 0.4f) - (totalLifetimeFaults * 1.5f), 5.0f, 100.0f);

    if (snap.state == STATE_SHUTDOWN) activeRecommendation = "CRITICAL: Hard interlock active.";
    else if (snap.state == STATE_FAILSAFE) activeRecommendation = "FAILSAFE: Load disconnected.";
    else if (snap.imbalanceDerivative > 0.002f) activeRecommendation = "WARNING: Imbalance rate rising.";
    else if (snap.soc < 20.0f) activeRecommendation = "NOTICE: Pack depleted.";
    else activeRecommendation = "OPTIMAL: All parameters nominal.";
    
    String ledgerString = "--- FAULT LEDGER ---\n";
    for(int i = historyCount - 1; i >= 0; i--) {
      ledgerString += "[" + String(faultHistory[i].timestamp/1000) + "s] " + stateToString(faultHistory[i].prevState) + "->" + stateToString(faultHistory[i].newState) + " | " + faultToString(faultHistory[i].fault) + "\n";
    }
    xSemaphoreGive(systemMutex);

    if (evaluateSignificance(snap, lastReported, lastHeartbeat)) {
      lastReported = snap;
      lastHeartbeat = millis();
      
      if (online) {
        Blynk.run();
        
        while (!tQueue.isEmpty()) {
          TelemetrySnapshot q; tQueue.dequeue(q);
          Blynk.virtualWrite(VPIN_CONNECTIVITY_STR, "FLUSHING QUEUE");
          Blynk.virtualWrite(VPIN_SOC, q.soc);
          vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        
        Blynk.virtualWrite(VPIN_CONNECTIVITY_STR, "LIVE");
        Blynk.virtualWrite(VPIN_CELL1, snap.cellVoltages[0]);
        Blynk.virtualWrite(VPIN_CELL2, snap.cellVoltages[1]);
        Blynk.virtualWrite(VPIN_CELL3, snap.cellVoltages[2]);
        Blynk.virtualWrite(VPIN_CELL4, snap.cellVoltages[3]);
        Blynk.virtualWrite(VPIN_SOC, snap.soc);
        Blynk.virtualWrite(VPIN_SOC_GRAPH, snap.soc);
        Blynk.virtualWrite(VPIN_IMBALANCE, snap.imbalance);
        Blynk.virtualWrite(VPIN_IMBALANCE_TREND_PTS, snap.imbalanceDerivative * 1000.0f);
        Blynk.virtualWrite(VPIN_FAULT_STATE, stateToString(snap.state));
        Blynk.virtualWrite(VPIN_FAULT_NAME, faultToString(snap.fault));
        Blynk.virtualWrite(VPIN_RELAY_STATUS, snap.relayClosed ? 1 : 0);
        Blynk.virtualWrite(VPIN_WIFI_RSSI, snap.rssi);
        Blynk.virtualWrite(VPIN_RISK_SCORE, compositeRiskScore);
        Blynk.virtualWrite(VPIN_HEALTH_SCORE, overallHealthScore);
        Blynk.virtualWrite(VPIN_SEVERITY_COLOR, getSeverityColor(snap.state));
        Blynk.virtualWrite(VPIN_OPERATOR_RECOMMEND, activeRecommendation);
        Blynk.virtualWrite(VPIN_LIFETIME_FAULT_COUNT, totalLifetimeFaults);
        Blynk.virtualWrite(VPIN_QUEUE_DEPTH, tQueue.size());
        Blynk.virtualWrite(VPIN_FAULT_HISTORY_LOG, ledgerString); 
        
        char summaryBuf[128];
        snprintf(summaryBuf, sizeof(summaryBuf), "SOH: %.1f%% | SOC: %.1f%% | UPTIME: %lus", overallHealthScore, snap.soc, millis() / 1000);
        Blynk.virtualWrite(VPIN_EXECUTIVE_SUMMARY, summaryBuf);
      } else {
        tQueue.enqueue(snap);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void setup() {
  Serial.begin(115200);
  
  for(int i = 0; i < WINDOW_SIZE; i++) voltageSamples[i] = 3.8f;

  nvm.begin("bms", false);
  totalLifetimeFaults = nvm.getUInt("faults", 0);
  tQueue.init(); 
  
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, HIGH);
  pinMode(RELAY_FEEDBACK_PIN, INPUT_PULLUP);
  pinMode(LED_GREEN_PIN, OUTPUT); pinMode(LED_YELLOW_PIN, OUTPUT); 
  pinMode(LED_RED_PIN, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);

  lcd.init(); lcd.backlight();
  systemMutex = xSemaphoreCreateMutex();

  // STACK SIZES INCREASED: Prevent FreeRTOS stack overflow crashes
  xTaskCreatePinnedToCore(vTaskBMSEngine, "BMS", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTaskSafetyKernel, "Safety", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTaskHMI, "HMI", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTaskTelemetry, "Net", 16384, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
