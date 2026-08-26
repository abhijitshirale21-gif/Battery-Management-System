#define BLYNK_TEMPLATE_ID "TMPL3NrfkI_p4"
#define BLYNK_TEMPLATE_NAME "SMART EV"
#define BLYNK_AUTH_TOKEN "GhHcwVlXlPKvGT808c9piLYbMRmL31Dy"

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/******************************************************************************
 * FULL MODULAR BMS & ENTERPRISE ANALYTICS ENGINE (TASKS 1 - 6)
 * Board: ESP32 | Target: Hardware / Wokwi Simulator
 ******************************************************************************/

const char* WIFI_SSID = "Wokwi-GUEST"; // Use "Wokwi-GUEST" for sim, or your Router SSID
const char* WIFI_PASS = "";

// Task 5 Datastream Pin Mappings (V0 - V13)
#define VPIN_CELL1                 V0   
#define VPIN_CELL2                 V1   
#define VPIN_CELL3                 V2   
#define VPIN_CELL4                 V3   
#define VPIN_WEAKEST_CELL          V4   
#define VPIN_STRONGEST_CELL        V5   
#define VPIN_IMBALANCE             V6   
#define VPIN_SOC                   V7   
#define VPIN_RELAY_STATUS          V8   
#define VPIN_FAULT_STATE           V9   
#define VPIN_FAULT_NAME            V10  
#define VPIN_WIFI_RSSI             V11  
#define VPIN_QUEUE_DEPTH           V12  
#define VPIN_CONNECTIVITY_STR      V13  

// Task 6 Enterprise Analytics Pin Mappings (V20 - V29)
#define VPIN_RISK_SCORE            V20  
#define VPIN_HEALTH_SCORE          V21  
#define VPIN_SEVERITY_COLOR        V22  
#define VPIN_OPERATOR_RECOMMEND    V23  
#define VPIN_EXECUTIVE_SUMMARY     V24  
#define VPIN_FAULT_HISTORY_LOG     V25  
#define VPIN_LIFETIME_FAULT_COUNT  V26  
#define VPIN_SYSTEM_UPTIME         V27  
#define VPIN_IMBALANCE_TREND_PTS   V28  
#define VPIN_SOC_GRAPH             V29  

// Hardware Pin Definitions
#define SENSOR_PIN          39  // Changed from 27 to 39 (ADC1) to avoid Wi-Fi crash
#define RELAY_PIN           19  
#define RELAY_FEEDBACK_PIN  18  // Left defined, but bypassed in logic below
#define CURRENT_SENSOR_PIN  36  

// LED Indicators
#define LED_GREEN_PIN       27
#define LED_YELLOW_PIN      26
#define LED_RED_PIN         25
#define Buzzer_Pin          4
bool actualState = !digitalRead(RELAY_FEEDBACK_PIN);


// Task 1: Scalable Compile-Time Constant
constexpr uint8_t NUM_CELLS = 4; 
const int adcPins[NUM_CELLS] = {33, 32, 34, 35}; // All ADC1 pins (Safe)

// Voltage, Capacity & Safety Thresholds
const float MIN_CELL_VOLTAGE    = 3.00f;
const float MAX_CELL_VOLTAGE    = 4.20f;
const float NOMINAL_CAPACITY_AH = 2.50f;
const float FAULT_THRESHOLD     = 4.10f;
const float HYSTERESIS          = 0.05f;

// Non-blocking Timer Intervals (ms)
const unsigned long DEBOUNCE_TIME         = 500;   
const unsigned long RECOVERY_TIME         = 1000;  
const unsigned long FROZEN_TIME           = 8000;  
const unsigned long BMS_INTERVAL          = 250;   
const unsigned long LCD_REFRESH_INTERVAL  = 250;   
const unsigned long PAGE_ROTATE_INTERVAL  = 3000;  

// ==================== TASK 3: HARDWARE LCD ENGINE ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Filter Buffer
#define WINDOW_SIZE 5
float samples[WINDOW_SIZE];
int sampleIndex = 0;

// ==================== ENUMS & STRUCTS (TASKS 1, 4, 6) ====================
enum SystemState {
  STATE_NORMAL = 0,
  STATE_DEGRADED,
  STATE_FAILSAFE,
  STATE_SHUTDOWN
};

enum FaultID {
  FAULT_NONE              = 0,
  FAULT_CELL_IMBALANCE    = 1,
  FAULT_CELL_OUT_OF_RANGE = 2,
  FAULT_PACK_OVERVOLT     = 3,
  FAULT_ADC_FROZEN        = 4,
  FAULT_ADC_JUMP          = 5,
  FAULT_RELAY_MISMATCH    = 6,
  FAULT_CRITICAL_HARDWARE = 7
};

enum TrendType {
  TREND_INCREASING,
  TREND_DECREASING,
  TREND_STABLE
};

enum LcdPage {
  PAGE_BATTERY_STATUS,
  PAGE_SYSTEM_STATE,
  PAGE_TELEMETRY
};

struct BatteryInfo {
  float voltage[NUM_CELLS];
  int weakestCell;
  int strongestCell;
  float lowestVoltage;
  float highestVoltage;
  float imbalance;
  float previousImbalance;
  float smoothedImbalance;
  float imbalanceDerivative;
  TrendType trend;
  float soc;
  float packCurrent;
  float cRate;
  float threshold;
  bool imbalanceFault;
  bool rangeFault;
};

struct TransitionEvent {
  uint32_t timestamp;
  SystemState prevState;
  SystemState newState;
  FaultID fault;
  float riskAtTransition;
};

struct TelemetrySnapshot {
  uint32_t timestamp;
  float cellVoltages[NUM_CELLS];
  float minVoltage;
  float maxVoltage;
  uint8_t weakestCellIndex;
  uint8_t strongestCellIndex;
  float voltageImbalance;
  float imbalanceDerivative;
  float soc;
  bool relayClosed;
  SystemState state;
  FaultID fault;
  int32_t rssi;
  bool isQueuedData;
};

// ==================== TASK 5: OFFLINE RING BUFFER QUEUE ====================
class TelemetryQueue {
private:
  static constexpr uint8_t CAPACITY = 40;
  TelemetrySnapshot buffer[CAPACITY];
  uint8_t head = 0;
  uint8_t tail = 0;
  uint8_t count = 0;

public:
  bool enqueue(const TelemetrySnapshot& item) {
    if (count >= CAPACITY) {
      tail = (tail + 1) % CAPACITY; 
      count--;
    }
    buffer[head] = item;
    head = (head + 1) % CAPACITY;
    count++;
    return true;
  }

  bool dequeue(TelemetrySnapshot& item) {
    if (count == 0) return false;
    item = buffer[tail];
    tail = (tail + 1) % CAPACITY;
    count--;
    return true;
  }

  uint8_t size() const { return count; }
  bool isEmpty() const { return count == 0; }
};

// ==================== TASK 5: NON-BLOCKING NETWORK MANAGER ====================
enum NetworkState {
  NET_DISCONNECTED,
  NET_CONNECTING_WIFI,
  NET_CONNECTING_BLYNK,
  NET_CONNECTED
};

class BMSNetworkManager {
private:
  NetworkState netState = NET_DISCONNECTED;
  uint32_t lastAttemptTime = 0;
  const uint32_t reconnectInterval = 4000;

public:
  void init() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    netState = NET_DISCONNECTED;
  }

  void update() {
    uint32_t now = millis();
    switch (netState) {
      case NET_DISCONNECTED:
        if (now - lastAttemptTime >= reconnectInterval) {
          lastAttemptTime = now;
          WiFi.begin(WIFI_SSID, WIFI_PASS);
          netState = NET_CONNECTING_WIFI;
        }
        break;

      case NET_CONNECTING_WIFI:
        if (WiFi.status() == WL_CONNECTED) {
          // Note: Add your BLYNK_AUTH_TOKEN if deploying to physical hardware
           Blynk.config("GhHcwVlXlPKvGT808c9piLYbMRmL31Dy");
           Blynk.connect(3000);
          netState = NET_CONNECTING_BLYNK;
        } else if (now - lastAttemptTime >= reconnectInterval) {
          WiFi.disconnect();
          netState = NET_DISCONNECTED;
        }
        break;

      case NET_CONNECTING_BLYNK:
        if (Blynk.connected()) {
          netState = NET_CONNECTED;
        } else if (now - lastAttemptTime >= reconnectInterval) {
          netState = NET_DISCONNECTED;
        }
        break;

      case NET_CONNECTED:
        if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
          netState = NET_DISCONNECTED;
          lastAttemptTime = now;
        } else {
          Blynk.run();
        }
        break;
    }
  }

  bool isReady() const { return (netState == NET_CONNECTED); }
  int32_t getRSSI() const { return (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : -100; }
};

// Forward Declarations
String stateToString(SystemState s);
String faultToString(FaultID f);
void updateIndicators(); // Added for LEDs
void updateBuzzer(); // Added for buzzer

// ==================== TASK 6: ENTERPRISE ANALYTICS & ADVISORY ====================
class EnterpriseAnalyticsEngine {
private:
  static constexpr uint8_t HISTORY_DEPTH = 10;
  TransitionEvent faultHistory[HISTORY_DEPTH];
  uint8_t historyCount = 0;
  uint32_t totalLifetimeFaults = 0;

  float compositeRiskScore = 0.0f;
  float overallHealthScore = 100.0f;
  const char* activeRecommendation = "NOMINAL";

public:
  void recordTransition(SystemState prev, SystemState next, FaultID fault) {
    if (fault != FAULT_NONE) {
      totalLifetimeFaults++;
    }
    if (historyCount < HISTORY_DEPTH) {
      faultHistory[historyCount++] = { millis(), prev, next, fault, compositeRiskScore };
    } else {
      for (uint8_t i = 1; i < HISTORY_DEPTH; i++) faultHistory[i - 1] = faultHistory[i];
      faultHistory[HISTORY_DEPTH - 1] = { millis(), prev, next, fault, compositeRiskScore };
    }
  }

  void evaluateAnalytics(const TelemetrySnapshot& data) {
    float risk = 0.0f;
    risk += constrain((data.voltageImbalance / 0.080f) * 35.0f, 0.0f, 35.0f);
    if (data.imbalanceDerivative > 0.001f) {
      risk += constrain((data.imbalanceDerivative / 0.005f) * 25.0f, 5.0f, 25.0f);
    }
    if (data.state == STATE_DEGRADED) risk += 15.0f;
    else if (data.state == STATE_FAILSAFE) risk += 25.0f;
    else if (data.state == STATE_SHUTDOWN) risk += 30.0f;
    risk += constrain((float)totalLifetimeFaults * 2.0f, 0.0f, 10.0f);
    compositeRiskScore = constrain(risk, 0.0f, 100.0f);

    float health = 100.0f - (compositeRiskScore * 0.4f) - (totalLifetimeFaults * 1.5f);
    if (data.soc < 10.0f || data.soc > 95.0f) health -= 5.0f;
    overallHealthScore = constrain(health, 5.0f, 100.0f);

    if (data.state == STATE_SHUTDOWN || data.fault == FAULT_RELAY_MISMATCH) {
      activeRecommendation = "CRITICAL: Hard interlock active. Check relay feedback and bus lines.";
    } else if (data.state == STATE_FAILSAFE) {
      activeRecommendation = "FAILSAFE: Load disconnected. Verify cell delta before auto-recovery.";
    } else if (data.imbalanceDerivative > 0.002f) {
      activeRecommendation = "WARNING: Imbalance rate rising. Active cell balancing recommended.";
    } else if (compositeRiskScore > 40.0f) {
      activeRecommendation = "DEGRADED: Thermal or cell divergence above target. Lower load current.";
    } else if (data.soc < 20.0f) {
      activeRecommendation = "NOTICE: Pack depleted. Recharging sequence recommended.";
    } else {
      activeRecommendation = "OPTIMAL: All battery cell parameters operating nominal.";
    }
  }

  const char* getSeverityColor(SystemState state) {
    switch (state) {
      case STATE_NORMAL:   return "#00E676"; 
      case STATE_DEGRADED: return "#FFD600"; 
      case STATE_FAILSAFE: return "#FF6D00"; 
      case STATE_SHUTDOWN: return "#D50000"; 
      default:             return "#FFFFFF";
    }
  }

  void pushBlynkAnalytics(const TelemetrySnapshot& data) {
    char summaryBuf[256];
    snprintf(summaryBuf, sizeof(summaryBuf), 
             "STATE: %s | SOH: %.1f%% | SOC: %.1f%% | IMB: %.1fmV | FAULTS: %u | UPTIME: %lus",
             stateToString(data.state).c_str(),
             overallHealthScore, data.soc, data.voltageImbalance * 1000.0f,
             totalLifetimeFaults, millis() / 1000);

    String faultLedger = "--- SYSTEM FAULT HISTORY LEDGER ---\n";
    for (int8_t i = historyCount - 1; i >= 0; i--) {
      char entry[64];
      snprintf(entry, sizeof(entry), "[%lums] State:%d->%d | FltID:%02d | Risk:%.0f\n",
               faultHistory[i].timestamp, faultHistory[i].prevState, 
               faultHistory[i].newState, faultHistory[i].fault, faultHistory[i].riskAtTransition);
      faultLedger += entry;
    }

    Blynk.virtualWrite(VPIN_RISK_SCORE, compositeRiskScore);
    Blynk.virtualWrite(VPIN_HEALTH_SCORE, overallHealthScore);
    Blynk.virtualWrite(VPIN_SEVERITY_COLOR, getSeverityColor(data.state));
    Blynk.virtualWrite(VPIN_OPERATOR_RECOMMEND, activeRecommendation);
    Blynk.virtualWrite(VPIN_EXECUTIVE_SUMMARY, summaryBuf);
    Blynk.virtualWrite(VPIN_FAULT_HISTORY_LOG, faultLedger);
    Blynk.virtualWrite(VPIN_LIFETIME_FAULT_COUNT, totalLifetimeFaults);
    Blynk.virtualWrite(VPIN_SYSTEM_UPTIME, millis() / 1000);
    Blynk.virtualWrite(VPIN_IMBALANCE_TREND_PTS, data.imbalanceDerivative * 1000.0f);
    Blynk.virtualWrite(VPIN_SOC_GRAPH, data.soc);
  }
};

// ==================== TASK 5: TELEMETRY TRANSMITTER ====================
class EventTelemetryEngine {
private:
  TelemetryQueue queue;
  TelemetrySnapshot lastReported;
  uint32_t lastHeartbeatTime = 0;

  bool evaluateSignificance(const TelemetrySnapshot& cur) {
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      if (fabs(cur.cellVoltages[i] - lastReported.cellVoltages[i]) >= 0.015f) return true;
    }
    if (cur.state != lastReported.state) return true;
    if (cur.fault != lastReported.fault) return true;
    if (cur.relayClosed != lastReported.relayClosed) return true;
    if (millis() - lastHeartbeatTime >= 2000) return true; 
    return false;
  }

  void publishSnapshot(const TelemetrySnapshot& data, EnterpriseAnalyticsEngine& analytics) {
    Blynk.virtualWrite(VPIN_CELL1, data.cellVoltages[0]);
    Blynk.virtualWrite(VPIN_CELL2, data.cellVoltages[1]);
    Blynk.virtualWrite(VPIN_CELL3, data.cellVoltages[2]);
    Blynk.virtualWrite(VPIN_CELL4, data.cellVoltages[3]);
    Blynk.virtualWrite(VPIN_WEAKEST_CELL, data.weakestCellIndex + 1);
    Blynk.virtualWrite(VPIN_STRONGEST_CELL, data.strongestCellIndex + 1);
    Blynk.virtualWrite(VPIN_IMBALANCE, data.voltageImbalance * 1000.0f);
    Blynk.virtualWrite(VPIN_SOC, data.soc);
    Blynk.virtualWrite(VPIN_RELAY_STATUS, data.relayClosed ? 1 : 0);
    Blynk.virtualWrite(VPIN_FAULT_STATE, stateToString(data.state));
    Blynk.virtualWrite(VPIN_FAULT_NAME, faultToString(data.fault));
    Blynk.virtualWrite(VPIN_WIFI_RSSI, data.rssi);
    Blynk.virtualWrite(VPIN_QUEUE_DEPTH, queue.size());
    Blynk.virtualWrite(VPIN_CONNECTIVITY_STR, data.isQueuedData ? "QUEUED" : "LIVE");

    analytics.evaluateAnalytics(data);
    analytics.pushBlynkAnalytics(data);
  }

public:
  void process(const BatteryInfo& bat, SystemState state, FaultID fault, bool relayState, int32_t rssi, bool netReady, EnterpriseAnalyticsEngine& analytics) {
    TelemetrySnapshot current;
    current.timestamp = millis();
    current.minVoltage = bat.lowestVoltage;
    current.maxVoltage = bat.highestVoltage;
    current.weakestCellIndex = (uint8_t)bat.weakestCell;
    current.strongestCellIndex = (uint8_t)bat.strongestCell;
    current.voltageImbalance = bat.imbalance;
    current.imbalanceDerivative = bat.imbalanceDerivative;
    current.soc = bat.soc;
    current.relayClosed = relayState;
    current.state = state;
    current.fault = fault;
    current.rssi = rssi;
    current.isQueuedData = false;

    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      current.cellVoltages[i] = bat.voltage[i];
    }

    if (evaluateSignificance(current)) {
      lastHeartbeatTime = millis();
      lastReported = current;

      if (netReady) {
        while (!queue.isEmpty()) {
          TelemetrySnapshot hist;
          queue.dequeue(hist);
          hist.isQueuedData = true;
          publishSnapshot(hist, analytics);
        }
        publishSnapshot(current, analytics);
      } else {
        queue.enqueue(current);
      }
    }
  }
};

// ==================== GLOBAL REGISTERS ====================
BMSNetworkManager netManager;
EventTelemetryEngine telemetryEngine;
EnterpriseAnalyticsEngine analytics;

SystemState currentState = STATE_NORMAL;
FaultID activeFault = FAULT_NONE;
BatteryInfo battery;
LcdPage currentPage = PAGE_BATTERY_STATUS;
LcdPage lastRenderedPage = (LcdPage)-1;

float packVoltage = 0;
float filteredPackVoltage = 0;
float previousPackVoltage = 0;

unsigned long frozenStart = 0;
unsigned long debounceStart = 0;
unsigned long recoveryVerificationStart = 0;
unsigned long lastBmsUpdate = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastPageRotate = 0;

bool recoveryVerificationActive = false;
int verificationPasses = 0;

// ==================== FORWARD DECLARATIONS ====================
void transitionTo(SystemState nextState, FaultID fault, String reason);
float readVoltage();
float movingAverage(float newValue);
bool detectFrozen(float value);
bool detectJump(float value);
bool detectOutOfRange(float value);
bool detectThreshold(float value);
bool detectRelayMismatch();

void runBMSEngine();
void readCellVoltages();
void readPackCurrent();
void analyzeCells();
void calculateSOC();
void calculateAdaptiveThreshold();
void detectTrend();

void processFaultStateMachine(FaultID detectedFault);
void updateLcdEngine();
void renderPageBatteryStatus();
void renderPageSystemState();
void renderPageTelemetry();
void renderFaultScreen();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RELAY_FEEDBACK_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, HIGH); // Engaged nominal
  
  // LED Setup
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(Buzzer_Pin, OUTPUT);

  netManager.init();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" SMART EV SYSTEM ");
  lcd.setCursor(0, 1);
  lcd.print(" Initializing... ");
  delay(1000);
  lcd.clear();

  for (int i = 0; i < WINDOW_SIZE; i++) samples[i] = 3.8f;
  battery.previousImbalance = 0.0f;
  battery.smoothedImbalance = 0.0f;
  battery.trend = TREND_STABLE;

  analytics.recordTransition(STATE_NORMAL, STATE_NORMAL, FAULT_NONE);
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Maintain Network Connection Non-Blockingly (Task 5)
  netManager.update();

  // 2. Hardware Pack Sensing & Moving Filter (Task 2)
  packVoltage = readVoltage();
  filteredPackVoltage = movingAverage(packVoltage);

  // 3. Periodic BMS Cell Array Processing (Task 1)
  if (currentMillis - lastBmsUpdate >= BMS_INTERVAL) {
    lastBmsUpdate = currentMillis;
    runBMSEngine();
  }

  // 4. Hardware Fault Diagnostics & Anomaly Isolation (Tasks 2 & 4)
  FaultID detectedFault = FAULT_NONE;
  if (detectRelayMismatch()) {
    detectedFault = FAULT_RELAY_MISMATCH;
  } else if (detectOutOfRange(filteredPackVoltage)) {
    detectedFault = FAULT_CELL_OUT_OF_RANGE;
  } else if (detectThreshold(filteredPackVoltage)) {
    detectedFault = FAULT_PACK_OVERVOLT;
  } else if (detectJump(filteredPackVoltage)) {
    detectedFault = FAULT_ADC_JUMP;
  } else if (detectFrozen(filteredPackVoltage)) {
    detectedFault = FAULT_ADC_FROZEN;
  } else if (battery.imbalanceFault) {
    detectedFault = FAULT_CELL_IMBALANCE;
  } else if (battery.rangeFault) {
    detectedFault = FAULT_CELL_OUT_OF_RANGE;
  }

  // 5. Fault State Machine & Recovery Verification Sequence (Task 4)
  processFaultStateMachine(detectedFault);
  
  // Update LEDs based on current State
  updateIndicators();

  // 6. Event-Driven Telemetry & Enterprise Decision Logs (Tasks 5 & 6)
  telemetryEngine.process(
    battery,
    currentState,
    activeFault,
    digitalRead(RELAY_PIN) == HIGH,
    netManager.getRSSI(),
    netManager.isReady(),
    analytics
  );

  // 7. Local Differential LCD Screen Update (Task 3)
  if (currentMillis - lastLcdUpdate >= LCD_REFRESH_INTERVAL) {
    lastLcdUpdate = currentMillis;
    updateLcdEngine();
  }
}

// ==================== INDICATOR CONTROLLER ====================
void updateIndicators() {
  // Clear all LEDs first
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  // Clear all LEDs first
  digitalWrite(LED_GREEN_PIN, LOW);

  // Activate the correct LED based on system state
  if (currentState == STATE_NORMAL) {
    digitalWrite(LED_GREEN_PIN, HIGH);
  } 
  else if (currentState == STATE_DEGRADED) {
    digitalWrite(LED_YELLOW_PIN, HIGH);
  } 
  else if (currentState == STATE_FAILSAFE || currentState == STATE_SHUTDOWN) {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(Buzzer_Pin, HIGH); // Activate buzzer for critical states
  } 
}

// ==================== TASK 4: DETERMINISTIC STATE MACHINE ====================
void processFaultStateMachine(FaultID detectedFault) {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case STATE_NORMAL:
      if (detectedFault != FAULT_NONE) {
        if (debounceStart == 0) debounceStart = currentMillis;
        if (currentMillis - debounceStart >= DEBOUNCE_TIME) {
          if (detectedFault == FAULT_RELAY_MISMATCH) {
            digitalWrite(RELAY_PIN, LOW);
            transitionTo(STATE_SHUTDOWN, detectedFault, "Relay Hardware Mismatch");
          } else if (detectedFault == FAULT_CELL_IMBALANCE) {
            transitionTo(STATE_DEGRADED, detectedFault, "Adaptive Imbalance Threshold Exceeded");
          } else {
            digitalWrite(RELAY_PIN, LOW);
            transitionTo(STATE_FAILSAFE, detectedFault, "Safety Threshold Tripped");
          }
        }
      } else {
        debounceStart = 0;
      }
      break;

    case STATE_DEGRADED:
      if (detectedFault == FAULT_NONE) {
        transitionTo(STATE_NORMAL, FAULT_NONE, "Balanced Normal Restored");
      } else if (detectedFault != FAULT_CELL_IMBALANCE) {
        digitalWrite(RELAY_PIN, LOW);
        transitionTo(STATE_FAILSAFE, detectedFault, "Hard Fault Escalation");
      }
      break;

    case STATE_FAILSAFE:
      if (detectedFault == FAULT_NONE) {
        if (!recoveryVerificationActive) {
          recoveryVerificationActive = true;
          recoveryVerificationStart = currentMillis;
          verificationPasses = 0;
        } else {
          if (currentMillis - recoveryVerificationStart >= (RECOVERY_TIME / 2)) {
            verificationPasses++;
            recoveryVerificationStart = currentMillis;
          }
          if (verificationPasses >= 2) {
            recoveryVerificationActive = false;
            digitalWrite(RELAY_PIN, HIGH);
            transitionTo(STATE_NORMAL, FAULT_NONE, "5s Multi-stage Verification Passed");
          }
        }
      } else {
        recoveryVerificationActive = false;
        activeFault = detectedFault;
        if (detectedFault == FAULT_RELAY_MISMATCH) {
          transitionTo(STATE_SHUTDOWN, detectedFault, "Escalated to SHUTDOWN due to Relay Feedback Failure");
        }
      }
      break;

    case STATE_SHUTDOWN:
      digitalWrite(RELAY_PIN, LOW); // Ensure relay is OFF during shutdown
      
      // Wait for the switch to be turned OFF (LOW)
      if (digitalRead(RELAY_FEEDBACK_PIN) == LOW) {
        
        // Before returning to normal, make sure the battery has no other errors
        if (detectOutOfRange(filteredPackVoltage) == false && 
            detectThreshold(filteredPackVoltage) == false && 
            battery.imbalanceFault == false) {
            
          digitalWrite(RELAY_PIN, HIGH); // Turn the relay back ON
          transitionTo(STATE_NORMAL, FAULT_NONE, "Switch OFF, system normal");
        }
      }
      break;
  }
}

void transitionTo(SystemState nextState, FaultID fault, String reason) {
  Serial.printf("[Timestamp: %lu ms] | %s -> %s | Fault ID: %s | Reason: %s\n",
                millis(), stateToString(currentState).c_str(), 
                stateToString(nextState).c_str(), faultToString(fault).c_str(), reason.c_str());

  analytics.recordTransition(currentState, nextState, fault);
  currentState = nextState;
  activeFault = fault;
  debounceStart = 0;
}

// ==================== TASK 2: SENSING & ANOMALY CHECKS ====================
String stateToString(SystemState s) {
  switch(s) {
    case STATE_NORMAL:   return "NORMAL";
    case STATE_DEGRADED: return "DEGRADED";
    case STATE_FAILSAFE: return "FAILSAFE";
    case STATE_SHUTDOWN: return "SHUTDOWN";
    default:             return "UNKNOWN";
  }
}

String faultToString(FaultID f) {
  switch(f) {
    case FAULT_NONE:              return "NONE";
    case FAULT_CELL_IMBALANCE:    return "CELL_IMBALANCE";
    case FAULT_CELL_OUT_OF_RANGE: return "CELL_OUT_OF_RANGE";
    case FAULT_PACK_OVERVOLT:     return "PACK_OVERVOLT";
    case FAULT_ADC_FROZEN:        return "ADC_FROZEN";
    case FAULT_ADC_JUMP:          return "ADC_JUMP";
    case FAULT_RELAY_MISMATCH:    return "RELAY_MISMATCH";
    default:                      return "NONE";
  }
}

float readVoltage() {
  float total = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    total += battery.voltage[i];
  }
  return total / NUM_CELLS;

}

float movingAverage(float newValue) {
  samples[sampleIndex] = newValue;
  sampleIndex = (sampleIndex + 1) % WINDOW_SIZE;
  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) sum += samples[i];
  return sum / WINDOW_SIZE;
}

bool detectFrozen(float value) {
  static float lastValue = 0.0f;
  if (lastValue == 0.0f) {
    lastValue = value;
    return false;
  }
  if (fabs(value - lastValue) < 0.0005f) {
    if (frozenStart == 0) frozenStart = millis();
    if (millis() - frozenStart > FROZEN_TIME) return true;
  } else {
    frozenStart = 0;
  }
  lastValue = value;
  return false;
}

bool detectJump(float value) {
  if (previousPackVoltage == 0.0f) {
    previousPackVoltage = value;
    return false;
  }
  bool jump = fabs(value - previousPackVoltage) > 0.40f;
  previousPackVoltage = value;
  return jump;
}

bool detectOutOfRange(float value) {
  return (value < (MIN_CELL_VOLTAGE - 0.1f) || value > (MAX_CELL_VOLTAGE + 0.1f));
}

bool detectThreshold(float value) {
  if (currentState == STATE_NORMAL) return value > FAULT_THRESHOLD;
  return value > (FAULT_THRESHOLD - HYSTERESIS);
}

bool detectRelayMismatch() {
// Give the Wokwi simulator 2 seconds to load before checking the switch
  if (millis() < 2000) {
    return false;
  }

  // If the switch is HIGH, trigger the shutdown fault
  if (digitalRead(RELAY_FEEDBACK_PIN) == HIGH) {
    return true;  
  }
  
  // If the switch is LOW, there is no fault
  return false;
}

// ==================== TASK 1: SCALABLE BMS ENGINE ====================
void runBMSEngine() {
  readCellVoltages();
  readPackCurrent();
  analyzeCells();
  calculateSOC();
  calculateAdaptiveThreshold();
  detectTrend();
}

void readCellVoltages() {
  for (int i = 0; i < NUM_CELLS; i++) {
    int adc = analogRead(adcPins[i]);
    // Calculate the perfect simulated voltage
    float baseVoltage = MIN_CELL_VOLTAGE + ((float)adc / 4095.0f) * (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE);
    
    // Inject +/- 1mV to 2mV of simulated electrical noise
    float noise = random(-20, 21) / 10000.0f; 
    
    battery.voltage[i] = baseVoltage + noise;
  }
}

void readPackCurrent() {
  int rawAdc = analogRead(CURRENT_SENSOR_PIN);
  battery.packCurrent = ((float)rawAdc / 4095.0f) * 10.0f;
  battery.cRate = battery.packCurrent / NOMINAL_CAPACITY_AH;
}

void analyzeCells() {
  battery.lowestVoltage  = battery.voltage[0];
  battery.highestVoltage = battery.voltage[0];
  battery.weakestCell    = 0;
  battery.strongestCell  = 0;
  battery.rangeFault     = false;

  for (int i = 0; i < NUM_CELLS; i++) {
    if (battery.voltage[i] < battery.lowestVoltage) {
      battery.lowestVoltage = battery.voltage[i];
      battery.weakestCell = i;
    }
    if (battery.voltage[i] > battery.highestVoltage) {
      battery.highestVoltage = battery.voltage[i];
      battery.strongestCell = i;
    }
    if (battery.voltage[i] < MIN_CELL_VOLTAGE || battery.voltage[i] > MAX_CELL_VOLTAGE) {
      battery.rangeFault = true;
    }
  }
  
  float currentImbalance = battery.highestVoltage - battery.lowestVoltage;
  battery.imbalanceDerivative = (currentImbalance - battery.imbalance) / ((float)BMS_INTERVAL / 1000.0f);
  battery.imbalance = currentImbalance;
}

void calculateSOC() {
  float avg = 0;
  for (int i = 0; i < NUM_CELLS; i++) avg += battery.voltage[i];
  avg /= NUM_CELLS;

  battery.soc = ((avg - MIN_CELL_VOLTAGE) / (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE)) * 100.0f;
  battery.soc = constrain(battery.soc, 0.0f, 100.0f);
}

void calculateAdaptiveThreshold() {
  float socThreshold = (battery.soc > 80.0f) ? 0.05f : ((battery.soc > 50.0f) ? 0.08f : 0.12f);
  float cRateFactor = (battery.cRate > 2.0f) ? 1.5f : 1.0f;
  battery.threshold = socThreshold * cRateFactor;
  battery.imbalanceFault = (battery.imbalance > battery.threshold);
}

void detectTrend() {
  battery.smoothedImbalance = (0.3f * battery.imbalance) + (0.7f * battery.previousImbalance);
  float deadband = 0.004f;

  if (battery.smoothedImbalance > battery.previousImbalance + deadband) {
    battery.trend = TREND_INCREASING;
  } else if (battery.smoothedImbalance < battery.previousImbalance - deadband) {
    battery.trend = TREND_DECREASING;
  } else {
    battery.trend = TREND_STABLE;
  }
  battery.previousImbalance = battery.smoothedImbalance;
}

// ==================== TASK 3: FLICKER-FREE LCD ENGINE ====================
void updateLcdEngine() {
  unsigned long now = millis();

  if (currentState == STATE_FAILSAFE || currentState == STATE_SHUTDOWN) {
    if (lastRenderedPage != (LcdPage)99) {
      lcd.clear();
      lastRenderedPage = (LcdPage)99;
    }
    renderFaultScreen();
    return;
  }

  if (now - lastPageRotate >= PAGE_ROTATE_INTERVAL) {
    lastPageRotate = now;
    currentPage = (LcdPage)((currentPage + 1) % 3);
  }

  if (currentPage != lastRenderedPage) {
    lcd.clear();
    lastRenderedPage = currentPage;
  }

  switch (currentPage) {
    case PAGE_BATTERY_STATUS: renderPageBatteryStatus(); break;
    case PAGE_SYSTEM_STATE:   renderPageSystemState(); break;
    case PAGE_TELEMETRY:      renderPageTelemetry(); break;
  }
}

void renderPageBatteryStatus() {
  lcd.setCursor(0, 0);
  lcd.print("SOC:"); lcd.print(battery.soc, 1); lcd.print("%   ");
  lcd.setCursor(0, 1);
  lcd.print("Imb:"); lcd.print(battery.imbalance, 2); lcd.print("V ");
  switch (battery.trend) {
    case TREND_INCREASING: lcd.print("TR:UP"); break;
    case TREND_DECREASING: lcd.print("TR:DN"); break;
    case TREND_STABLE:     lcd.print("TR:ST"); break;
  }
}

void renderPageSystemState() {
  lcd.setCursor(0, 0);
  lcd.print("SYS:"); lcd.print(stateToString(currentState)); lcd.print("  ");
  lcd.setCursor(0, 1);
  lcd.print("RLY:");
  lcd.print(digitalRead(RELAY_PIN) == HIGH ? "ON " : "OFF");
  lcd.print(" FLT:"); lcd.print((int)activeFault);
}

void renderPageTelemetry() {
  lcd.setCursor(0, 0);
  lcd.print("CUR:"); lcd.print(battery.packCurrent, 1); lcd.print("A "); lcd.print(battery.cRate, 1); lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("W:"); lcd.print(battery.weakestCell + 1); lcd.print(" S:"); lcd.print(battery.strongestCell + 1); lcd.print(" D:"); lcd.print(battery.imbalance, 2);
}

void renderFaultScreen() {
  lcd.setCursor(0, 0);
  if (currentState == STATE_SHUTDOWN) lcd.print("!! SHUTDOWN !!  ");
  else lcd.print("!! FAILSAFE !!  ");

  lcd.setCursor(0, 1);
  switch (activeFault) {
    case FAULT_CELL_IMBALANCE:    lcd.print("CELL IMBALANCE "); break;
    case FAULT_CELL_OUT_OF_RANGE: lcd.print("CELL OUT OF RNG"); break;
    case FAULT_ADC_FROZEN:        lcd.print("ADC SENSOR FROZ"); break;
    case FAULT_ADC_JUMP:          lcd.print("ADC STEP JUMP  "); break;
    case FAULT_RELAY_MISMATCH:    lcd.print("RELAY MISMATCH "); break;
    default:                      lcd.print("OVERVOLT TRIP  "); break;
  }
}