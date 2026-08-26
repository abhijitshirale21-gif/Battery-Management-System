#define BLYNK_TEMPLATE_ID "TMPL3NrfkI_p4"
#define BLYNK_TEMPLATE_NAME "SMART EV"
#define BLYNK_AUTH_TOKEN "GhHcwVlXlPKvGT808c9piLYbMRmL31Dy"



#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*******************************************************
 * Integrated Deterministic Protection Engine & BMS
 * Board: ESP32
 *******************************************************/

// ==================== PIN DEFINITIONS ====================
#define SENSOR_PIN          27  // Pack / Aux Sensor ADC
#define RELAY_PIN           19  // Main Protection Relay Control
#define RELAY_FEEDBACK_PIN  18  // Auxiliary Feedback Sense for Mismatch Detection
#define CURRENT_SENSOR_PIN  36  // ADC pin for Current Sensing (VP)

#define NUM_CELLS            4  
const int adcPins[NUM_CELLS] = {33, 32, 34, 35};

// ==================== VOLTAGE & FAULT LIMITS ====================
const float MIN_CELL_VOLTAGE    = 3.00;
const float MAX_CELL_VOLTAGE    = 4.20;
const float NOMINAL_CAPACITY_AH = 2.5;

const float FAULT_THRESHOLD     = 4.10;
const float HYSTERESIS          = 0.05;

// ==================== TIMERS & INTERVALS ====================
const unsigned long DEBOUNCE_TIME         = 500;   // 500 ms fault debounce
const unsigned long RECOVERY_TIME         = 5000;  // 5 sec recovery verification window
const unsigned long FROZEN_TIME           = 3000;  // 3 sec frozen sensor detect
const unsigned long BMS_INTERVAL          = 1000;  // Run BMS loop every 1 sec
const unsigned long LCD_REFRESH_INTERVAL  = 250;   // Update LCD every 250 ms
const unsigned long PAGE_ROTATE_INTERVAL  = 3000;  // Rotate LCD pages every 3 seconds

// ==================== HARDWARE LCD CONFIG ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== MOVING AVERAGE CONFIG ====================
#define WINDOW_SIZE 5
float samples[WINDOW_SIZE];
int sampleIndex = 0;

// ==================== ENUMS & STRUCTURES ====================
// Required 4-State Fault State Machine
enum SystemState {
  STATE_NORMAL,
  STATE_DEGRADED,
  STATE_FAILSAFE,
  STATE_SHUTDOWN
};

// Structured Fault Identification Tracking
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
  TrendType trend;
  float soc;
  float packCurrent;
  float cRate;
  float threshold;
  bool imbalanceFault;
  bool rangeFault;
};

// Global System Registers
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

// Verification flags for structured Failsafe recovery
bool recoveryVerificationActive = false;
int verificationPasses = 0;

// ==================== FUNCTION DECLARATIONS ====================
void transitionTo(SystemState nextState, FaultID fault, String reason);
String stateToString(SystemState s);
String faultToString(FaultID f);

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
void printBatteryInfo();

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
  digitalWrite(RELAY_PIN, HIGH); // Engaged in NORMAL state

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" SMART EV SYSTEM ");
  lcd.setCursor(0, 1);
  lcd.print(" Initializing... ");
  delay(1000);
  lcd.clear();

  for (int i = 0; i < WINDOW_SIZE; i++) {
    samples[i] = 3.8;
  }

  battery.previousImbalance = 0.0;
  battery.smoothedImbalance = 0.0;
  battery.trend = TREND_STABLE;

  Serial.println("\n==================================================");
  Serial.println(" Deterministic Fault State Machine Activated ");
  Serial.println("==================================================\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();

  // 1. HARDWARE SENSING & FILTERING
  packVoltage = readVoltage();
  filteredPackVoltage = movingAverage(packVoltage);

  // 2. ISOLATE AND IDENTIFY SPECIFIC FAULTS
  FaultID detectedFault = FAULT_NONE;

  if (detectRelayMismatch()) {
    detectedFault = FAULT_RELAY_MISMATCH;
  } else if (detectFrozen(filteredPackVoltage)) {
    detectedFault = FAULT_ADC_FROZEN;
  } else if (detectJump(filteredPackVoltage)) {
    detectedFault = FAULT_ADC_JUMP;
  } else if (detectOutOfRange(filteredPackVoltage)) {
    detectedFault = FAULT_CELL_OUT_OF_RANGE;
  } else if (detectThreshold(filteredPackVoltage)) {
    detectedFault = FAULT_PACK_OVERVOLT;
  }

  // 3. BMS ENGINE EVALUATION
  if (currentMillis - lastBmsUpdate >= BMS_INTERVAL) {
    lastBmsUpdate = currentMillis;
    runBMSEngine();
  }

  // Elevate to BMS Faults if hardware sensing is clear
  if (detectedFault == FAULT_NONE) {
    if (battery.imbalanceFault) detectedFault = FAULT_CELL_IMBALANCE;
    else if (battery.rangeFault) detectedFault = FAULT_CELL_OUT_OF_RANGE;
  }

  // 4. PROCESS STATE MACHINE & RECOVERY ENGINE
  processFaultStateMachine(detectedFault);

  // 5. LCD ENGINE TASK
  if (currentMillis - lastLcdUpdate >= LCD_REFRESH_INTERVAL) {
    lastLcdUpdate = currentMillis;
    updateLcdEngine();
  }
}

// ==================== STATE MACHINE ENGINE ====================

void processFaultStateMachine(FaultID detectedFault) {
  unsigned long currentMillis = millis();

  switch (currentState) {
    
    case STATE_NORMAL:
      if (detectedFault != FAULT_NONE) {
        if (debounceStart == 0) debounceStart = currentMillis;

        if (currentMillis - debounceStart >= DEBOUNCE_TIME) {
          if (detectedFault == FAULT_RELAY_MISMATCH) {
            // Hardware safety failure trips immediately to SHUTDOWN
            digitalWrite(RELAY_PIN, LOW);
            transitionTo(STATE_SHUTDOWN, detectedFault, "Critical Relay Hardware Mismatch");
          } else if (detectedFault == FAULT_CELL_IMBALANCE) {
            // Non-critical divergence moves to DEGRADED first
            transitionTo(STATE_DEGRADED, detectedFault, "Cell Divergence Warning");
          } else {
            // Voltage/ADC faults open relay and trigger FAILSAFE
            digitalWrite(RELAY_PIN, LOW);
            transitionTo(STATE_FAILSAFE, detectedFault, "Protection Parameter Violation");
          }
        }
      } else {
        debounceStart = 0;
      }
      break;

    case STATE_DEGRADED:
      if (detectedFault == FAULT_NONE) {
        transitionTo(STATE_NORMAL, FAULT_NONE, "Cell imbalance restored within threshold");
      } else if (detectedFault != FAULT_CELL_IMBALANCE) {
        digitalWrite(RELAY_PIN, LOW);
        transitionTo(STATE_FAILSAFE, detectedFault, "Degraded state escalated by hard fault");
      }
      break;

    case STATE_FAILSAFE:
      if (detectedFault == FAULT_NONE) {
        // Multi-stage verification before returning to NORMAL
        if (!recoveryVerificationActive) {
          recoveryVerificationActive = true;
          recoveryVerificationStart = currentMillis;
          verificationPasses = 0;
        } else {
          // Verify fault absence continuously during window
          if (currentMillis - recoveryVerificationStart >= (RECOVERY_TIME / 2)) {
            verificationPasses++;
            recoveryVerificationStart = currentMillis; // reset half-window
          }

          if (verificationPasses >= 2) {
            recoveryVerificationActive = false;
            digitalWrite(RELAY_PIN, HIGH); // Re-engage relay
            transitionTo(STATE_NORMAL, FAULT_NONE, "Verification Passed: System Safe");
          }
        }
      } else {
        // Reset verification if fault reappears
        recoveryVerificationActive = false;
        activeFault = detectedFault;
        if (detectedFault == FAULT_RELAY_MISMATCH) {
          transitionTo(STATE_SHUTDOWN, detectedFault, "Escalated to SHUTDOWN due to Relay Failure");
        }
      }
      break;

    case STATE_SHUTDOWN:
      // Terminal latch state: Relay is disabled until system hard reset
      digitalWrite(RELAY_PIN, LOW);
      break;
  }
}

void transitionTo(SystemState nextState, FaultID fault, String reason) {
  Serial.print("[Timestamp: ");
  Serial.print(millis());
  Serial.print(" ms] | TRANSITION: ");
  Serial.print(stateToString(currentState));
  Serial.print(" -> ");
  Serial.print(stateToString(nextState));
  Serial.print(" | Fault ID: ");
  Serial.print(faultToString(fault));
  Serial.print(" | Reason: ");
  Serial.println(reason);

  currentState = nextState;
  activeFault = fault;
  debounceStart = 0;
}

// ==================== HELPER SENSING FUNCTIONS ====================

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
    case FAULT_NONE:              return "NONE (0x00)";
    case FAULT_CELL_IMBALANCE:    return "CELL_IMBALANCE (0x01)";
    case FAULT_CELL_OUT_OF_RANGE: return "CELL_OUT_OF_RANGE (0x02)";
    case FAULT_PACK_OVERVOLT:     return "PACK_OVERVOLT (0x03)";
    case FAULT_ADC_FROZEN:        return "ADC_FROZEN (0x04)";
    case FAULT_ADC_JUMP:          return "ADC_JUMP (0x05)";
    case FAULT_RELAY_MISMATCH:    return "RELAY_MISMATCH (0x06)";
    default:                      return "HARDWARE_CRITICAL";
  }
}

float readVoltage() {
  int adc = analogRead(SENSOR_PIN);
  return 3.0 + ((float)adc / 4095.0) * 1.2;
}

float movingAverage(float newValue) {
  samples[sampleIndex] = newValue;
  sampleIndex = (sampleIndex + 1) % WINDOW_SIZE;

  float sum = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) sum += samples[i];
  return sum / WINDOW_SIZE;
}

bool detectFrozen(float value) {
  static float lastValue = value;
  if (abs(value - lastValue) < 0.002) {
    if (frozenStart == 0) frozenStart = millis();
    if (millis() - frozenStart > FROZEN_TIME) return true;
  } else {
    frozenStart = 0;
  }
  lastValue = value;
  return false;
}

bool detectJump(float value) {
  bool jump = abs(value - previousPackVoltage) > 0.35;
  previousPackVoltage = value;
  return jump;
}

bool detectOutOfRange(float value) {
  return (value < MIN_CELL_VOLTAGE || value > MAX_CELL_VOLTAGE);
}

bool detectThreshold(float value) {
  if (currentState == STATE_NORMAL) return value > FAULT_THRESHOLD;
  return value > (FAULT_THRESHOLD - HYSTERESIS);
}

bool detectRelayMismatch() {
  bool commandedState = digitalRead(RELAY_PIN);
  bool actualFeedback = digitalRead(RELAY_FEEDBACK_PIN);
  return (commandedState != actualFeedback);
}

// ==================== MODULAR BMS ENGINE ====================

void runBMSEngine() {
  readCellVoltages();
  readPackCurrent();
  analyzeCells();
  calculateSOC();
  calculateAdaptiveThreshold();
  detectTrend();
  printBatteryInfo();
}

void readCellVoltages() {
  for (int i = 0; i < NUM_CELLS; i++) {
    int adc = analogRead(adcPins[i]);
    battery.voltage[i] = MIN_CELL_VOLTAGE + ((float)adc / 4095.0) * (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE);
  }
}

void readPackCurrent() {
  int rawAdc = analogRead(CURRENT_SENSOR_PIN);
  battery.packCurrent = ((float)rawAdc / 4095.0) * 10.0;
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
  battery.imbalance = battery.highestVoltage - battery.lowestVoltage;
}

void calculateSOC() {
  float average = 0;
  for (int i = 0; i < NUM_CELLS; i++) average += battery.voltage[i];
  average /= NUM_CELLS;

  battery.soc = ((average - MIN_CELL_VOLTAGE) / (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE)) * 100.0;
  battery.soc = constrain(battery.soc, 0.0, 100.0);
}

void calculateAdaptiveThreshold() {
  float socThreshold;
  if (battery.soc > 80.0)      socThreshold = 0.05;
  else if (battery.soc > 50.0) socThreshold = 0.08;
  else if (battery.soc > 20.0) socThreshold = 0.12;
  else                         socThreshold = 0.18;

  float cRateFactor = (battery.cRate > 2.0) ? 1.5 : ((battery.cRate > 1.0) ? 1.25 : 1.0);
  battery.threshold = socThreshold * cRateFactor;
  battery.imbalanceFault = battery.imbalance > battery.threshold;
}

void detectTrend() {
  battery.smoothedImbalance = (0.3 * battery.imbalance) + (0.7 * battery.previousImbalance);
  float deadband = 0.004;

  if (battery.smoothedImbalance > battery.previousImbalance + deadband) {
    battery.trend = TREND_INCREASING;
  } else if (battery.smoothedImbalance < battery.previousImbalance - deadband) {
    battery.trend = TREND_DECREASING;
  } else {
    battery.trend = TREND_STABLE;
  }
  battery.previousImbalance = battery.smoothedImbalance;
}

void printBatteryInfo() {
  Serial.println("\n----------------- BMS STATUS -----------------");
  Serial.print("System State   : "); Serial.println(stateToString(currentState));
  Serial.print("Active Fault   : "); Serial.println(faultToString(activeFault));
  Serial.print("Pack SOC       : "); Serial.print(battery.soc, 1); Serial.println(" %");
  Serial.print("Imbalance      : "); Serial.print(battery.imbalance, 3); Serial.println(" V");
  Serial.println("----------------------------------------------");
}

// ==================== LCD DISPLAY ENGINE ====================

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