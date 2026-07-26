/*
===========================================================
Modular Battery Management System (BMS)
ESP32
===========================================================

Features
--------
✓ Number of cells changed using one constant
✓ Read all battery cell voltages
✓ Calculate State of Charge (SoC)
✓ Find weakest cell
✓ Find strongest cell
✓ Calculate voltage imbalance
✓ Detect increasing/decreasing imbalance
✓ Adaptive imbalance threshold based on SoC
✓ Clean interface using BatteryPack structure
✓ Easy to expand from 4 cells to 16 cells

===========================================================
*/

#define NUM_CELLS 4          // Change only this value to 16 if required

// ADC Pins
int cellPins[NUM_CELLS] = {36,39,34,35};

//-------------------------------
// Structure for one battery cell
//-------------------------------
struct Cell
{
  float voltage;
  int soc;
};

//--------------------------------
// Structure for complete battery
//--------------------------------
struct BatteryPack
{
  Cell cell[NUM_CELLS];

  int weakestCell;
  int strongestCell;

  float weakestVoltage;
  float strongestVoltage;

  float imbalance;

  float previousImbalance;

  bool imbalanceIncreasing;

  float warningThreshold;
  float faultThreshold;

} pack;

//=========================================================
// Convert Voltage to SoC
//=========================================================
int calculateSOC(float voltage)
{
  if(voltage >= 4.2)
    return 100;

  if(voltage <= 3.0)
    return 0;

  return ((voltage-3.0)/1.2)*100;
}

//=========================================================
// Read Battery Cells
//=========================================================
void readCells()
{
  for(int i=0;i<NUM_CELLS;i++)
  {
    int adc = analogRead(cellPins[i]);

    // Convert ADC to Voltage (Approximation)

    pack.cell[i].voltage =
    3.0 + ((float)adc/4095.0)*1.2;

    pack.cell[i].soc =
    calculateSOC(pack.cell[i].voltage);
  }
}

//=========================================================
// Analyse Battery Pack
//=========================================================
void analyseBattery()
{
  pack.weakestCell=0;
  pack.strongestCell=0;

  for(int i=1;i<NUM_CELLS;i++)
  {
    if(pack.cell[i].voltage <
       pack.cell[pack.weakestCell].voltage)

       pack.weakestCell=i;

    if(pack.cell[i].voltage >
       pack.cell[pack.strongestCell].voltage)

       pack.strongestCell=i;
  }

  pack.weakestVoltage =
  pack.cell[pack.weakestCell].voltage;

  pack.strongestVoltage =
  pack.cell[pack.strongestCell].voltage;

  pack.imbalance =
  pack.strongestVoltage-pack.weakestVoltage;

  //------------------------------
  // Increasing or Decreasing
  //------------------------------

  if(pack.imbalance >
     pack.previousImbalance)

      pack.imbalanceIncreasing=true;

  else

      pack.imbalanceIncreasing=false;

  pack.previousImbalance=
  pack.imbalance;
}

//=========================================================
// Adaptive Threshold
//=========================================================
void adaptiveThreshold()
{
  int totalSOC=0;

  for(int i=0;i<NUM_CELLS;i++)
      totalSOC+=pack.cell[i].soc;

  int averageSOC=totalSOC/NUM_CELLS;

  // High SoC -> Larger Threshold

  if(averageSOC>=80)
  {
      pack.warningThreshold=0.10;
      pack.faultThreshold=0.20;
  }

  // Medium SoC

  else if(averageSOC>=40)
  {
      pack.warningThreshold=0.08;
      pack.faultThreshold=0.15;
  }

  // Low SoC -> Smaller Threshold

  else
  {
      pack.warningThreshold=0.05;
      pack.faultThreshold=0.10;
  }
}

//=========================================================
// Display Battery Information
//=========================================================
void displayBattery()
{
  Serial.println("--------------------------------");

  for(int i=0;i<NUM_CELLS;i++)
  {
    Serial.print("Cell ");
    Serial.print(i+1);

    Serial.print(" Voltage = ");

    Serial.print(pack.cell[i].voltage);

    Serial.print(" V");

    Serial.print("   SoC = ");

    Serial.print(pack.cell[i].soc);

    Serial.println("%");
  }

  Serial.println();

  Serial.print("Weakest Cell : ");
  Serial.println(pack.weakestCell+1);

  Serial.print("Strongest Cell : ");
  Serial.println(pack.strongestCell+1);

  Serial.print("Imbalance : ");
  Serial.print(pack.imbalance);
  Serial.println(" V");

  if(pack.imbalanceIncreasing)
      Serial.println("Imbalance Increasing");

  else
      Serial.println("Imbalance Decreasing");

  Serial.print("Warning Threshold : ");
  Serial.println(pack.warningThreshold);

  Serial.print("Fault Threshold : ");
  Serial.println(pack.faultThreshold);

  if(pack.imbalance>=pack.faultThreshold)
      Serial.println("FAULT");

  else if(pack.imbalance>=pack.warningThreshold)
      Serial.println("WARNING");

  else
      Serial.println("Battery Healthy");

  Serial.println("--------------------------------");
}

//=========================================================
// Setup
//=========================================================
void setup()
{
  Serial.begin(115200);

  for(int i=0;i<NUM_CELLS;i++)
      pinMode(cellPins[i],INPUT);
}

//=========================================================
// Loop
//=========================================================
void loop()
{
  readCells();

  analyseBattery();

  adaptiveThreshold();

  displayBattery();

  delay(1000);
}