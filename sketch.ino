#define NUM_CELLS 16      // Change only this to 8 or 16

// ADC pins for ESP32
const int adcPins[NUM_CELLS] = {36, 39, 34, 35};

// Battery limits
const float MIN_CELL_VOLTAGE = 3.0;
const float MAX_CELL_VOLTAGE = 4.2;

// Trend Type

enum TrendType
{
  TREND_INCREASING,
  TREND_DECREASING,
  TREND_STABLE
};


// Battery Information Structure
struct BatteryInfo
{
  float voltage[NUM_CELLS];

  int weakestCell;
  int strongestCell;

  float lowestVoltage;
  float highestVoltage;

  float imbalance;

  float previousImbalance;

  TrendType trend;

  float soc;

  float threshold;

  bool imbalanceFault;
};

BatteryInfo battery;

// Function Prototypes
void readVoltages();
void analyzeCells();
void calculateSOC();
void calculateAdaptiveThreshold();
void detectTrend();
void printBatteryInfo();

void setup()
{
  Serial.begin(115200);

  analogReadResolution(12);

  battery.previousImbalance = 0;

  Serial.println("\n========== Modular BMS ==========");
}

void loop()
{
  readVoltages();

  analyzeCells();

  calculateSOC();

  calculateAdaptiveThreshold();

  detectTrend();

  printBatteryInfo();

  delay(2000);
}

// Read Cell Voltages

void readVoltages()
{
  for (int i = 0; i < NUM_CELLS; i++)
  {
    int adc = analogRead(adcPins[i]);

    battery.voltage[i] =
      MIN_CELL_VOLTAGE +
      ((float)adc / 4095.0) *
      (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE);
  }
}

// Analyze Cells

void analyzeCells()
{
  battery.lowestVoltage = battery.voltage[0];
  battery.highestVoltage = battery.voltage[0];

  battery.weakestCell = 0;
  battery.strongestCell = 0;

  for (int i = 1; i < NUM_CELLS; i++)
  {
    if (battery.voltage[i] < battery.lowestVoltage)
    {
      battery.lowestVoltage = battery.voltage[i];
      battery.weakestCell = i;
    }

    if (battery.voltage[i] > battery.highestVoltage)
    {
      battery.highestVoltage = battery.voltage[i];
      battery.strongestCell = i;
    }
  }

  battery.imbalance =
      battery.highestVoltage -
      battery.lowestVoltage;
}

// Simple SoC Estimation

void calculateSOC()
{
  float average = 0;

  for (int i = 0; i < NUM_CELLS; i++)
    average += battery.voltage[i];

  average /= NUM_CELLS;

  battery.soc =
      ((average - MIN_CELL_VOLTAGE) /
      (MAX_CELL_VOLTAGE - MIN_CELL_VOLTAGE))
      * 100.0;

  battery.soc = constrain(battery.soc, 0, 100);
}


// Adaptive Threshold

void calculateAdaptiveThreshold()
{
  if (battery.soc > 80)
      battery.threshold = 0.05;

  else if (battery.soc > 50)
      battery.threshold = 0.08;

  else if (battery.soc > 20)
      battery.threshold = 0.12;

  else
      battery.threshold = 0.18;

  battery.imbalanceFault =
      battery.imbalance >
      battery.threshold;
}


// Trend Detection


void detectTrend()
{
  float tolerance = 0.003;

  if (battery.imbalance >
      battery.previousImbalance + tolerance)
  {
    battery.trend = TREND_INCREASING;
  }
  else if (battery.imbalance <
           battery.previousImbalance - tolerance)
  {
    battery.trend = TREND_DECREASING;
  }
  else
  {
    battery.trend = TREND_STABLE;
  }

  battery.previousImbalance =
      battery.imbalance;
}

// Display Results

void printBatteryInfo()
{
  Serial.println();
  Serial.println("===================================");

  for (int i = 0; i < NUM_CELLS; i++)
  {
    Serial.print("Cell ");
    Serial.print(i + 1);
    Serial.print(" : ");
    Serial.print(battery.voltage[i], 3);
    Serial.println(" V");
  }

  Serial.println("-----------------------------------");

  Serial.print("Weakest Cell : ");
  Serial.println(battery.weakestCell + 1);

  Serial.print("Strongest Cell : ");
  Serial.println(battery.strongestCell + 1);

  Serial.print("Lowest Voltage : ");
  Serial.print(battery.lowestVoltage, 3);
  Serial.println(" V");

  Serial.print("Highest Voltage : ");
  Serial.print(battery.highestVoltage, 3);
  Serial.println(" V");

  Serial.print("Imbalance : ");
  Serial.print(battery.imbalance, 3);
  Serial.println(" V");

  Serial.print("SoC : ");
  Serial.print(battery.soc, 1);
  Serial.println(" %");

  Serial.print("Adaptive Threshold : ");
  Serial.print(battery.threshold, 3);
  Serial.println(" V");

  Serial.print("Trend : ");

  switch (battery.trend)
  {
    case TREND_INCREASING:
      Serial.println("Increasing");
      break;

    case TREND_DECREASING:
      Serial.println("Decreasing");
      break;

    case TREND_STABLE:
      Serial.println("Stable");
      break;
  }

  Serial.print("Status : ");

  if (battery.imbalanceFault)
      Serial.println("IMBALANCE DETECTED");
  else
      Serial.println("NORMAL");

  Serial.println("===================================");
}


// Reusable Interface

BatteryInfo getBatteryInfo()
{
  return battery;
}
