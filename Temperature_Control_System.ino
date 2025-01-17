#include <Arduino.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------

#define NTC_PIN          A0     // Analog pin for thermistor reading
#define RELAY_CONTROL_PIN 8     // Digital pin controlling the transistor/relay

// ---------------------------------------------------------------------------
// NTC Thermistor / Voltage Divider Parameters
// ---------------------------------------------------------------------------

#define ADC_MAX_VALUE    1023.0  // 10-bit ADC on Arduino Uno yields 0–1023
#define VREF             5.0     // Voltage reference (5V for Arduino Uno)
#define R_FIXED          10000.0 // e.g., 10kΩ fixed resistor in voltage divider
#define R0               10000.0 // 10kΩ NTC nominal resistance at 25°C
#define BETA             3892.0  // Beta constant for the thermistor
#define T0_K             298.15  // 25°C in Kelvin (25 + 273.15)

// ---------------------------------------------------------------------------
// Control Algorithm Parameters
// ---------------------------------------------------------------------------

// "Boil detection" thresholds
#define BOIL_SLOPE_THRESHOLD  0.05  // Example slope threshold in °C/sec
#define NEAR_BOIL_TEMP        90.0  // Approx. near-boil threshold for plate reading

// Simmer maintenance
#define SIMMER_TARGET         95.0  // Target temperature for simmer
#define HYSTERESIS            2.0   // Hysteresis window in °C

// Control loop timing
#define CONTROL_LOOP_INTERVAL 1000  // in milliseconds

// ---------------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------------

unsigned long previousMillis = 0;  // For timing
float previousTemp = 25.0;         // Assume room temperature at startup
bool hasBoiled = false;            // Flag to indicate if boiling is detected

bool heaterOn = false;             // Track heater state

// Reads the NTC thermistor from A0, calculates temperature in °C
// ---------------------------------------------------------------------------
float readTemperatureNTC()
{
  // 1) Read the raw ADC
  int adcValue = analogRead(NTC_PIN);

  // 2) Convert the ADC reading to a voltage
  float vMeas = (adcValue / ADC_MAX_VALUE) * VREF;

  float rNtc = (R_FIXED * (VREF - vMeas)) / vMeas; 
  float tempK = 1.0 / ( (1.0 / T0_K) + (1.0 / BETA) * log(rNtc / R0) );
  float tempC = tempK - 273.15;

  return tempC;
}


// Turns the relay (heater) ON or OFF
// ---------------------------------------------------------------------------
void setHeaterPower(bool on)
{
  heaterOn = on;
  digitalWrite(RELAY_CONTROL_PIN, (on ? HIGH : LOW));
}


// Determines if the plate is at or near boiling by checking temp slope
// ---------------------------------------------------------------------------
bool detectBoil(float currentTemp, float previousTemp, float deltaTime)
{
  float slope = (currentTemp - previousTemp) / deltaTime; // °C/s

  // If slope is below threshold and currentTemp is above NEAR_BOIL_TEMP
  if (slope < BOIL_SLOPE_THRESHOLD && currentTemp >= NEAR_BOIL_TEMP)
  {
    return true;
  }
  return false;
}


// Simple bang-bang control around SIMMER_TARGET
// ---------------------------------------------------------------------------
void maintainSimmer(float currentTemp)
{
  // Turn ON if below target - hysteresis
  if (currentTemp < (SIMMER_TARGET - HYSTERESIS))
  {
    setHeaterPower(true);
  }
  // Turn OFF if above target + hysteresis
  else if (currentTemp > (SIMMER_TARGET + HYSTERESIS))
  {
    setHeaterPower(false);
  }
  // Else, do nothing (remain in previous state)
}

// Main routine to manage boil detection and simmer control
// ---------------------------------------------------------------------------
void controlLoop()
{
  // Track elapsed time
  unsigned long currentMillis = millis();
  float deltaTimeSec = (currentMillis - previousMillis) / 1000.0;

  if (deltaTimeSec < (CONTROL_LOOP_INTERVAL / 1000.0))
  {
    return; // Not enough time has passed
  }
  previousMillis = currentMillis;

  // 1) Read current temperature from NTC
  float currentTemp = readTemperatureNTC();

  // 2) If we haven't detected boiling yet, keep heater on
  if (!hasBoiled)
  {
    setHeaterPower(true);

    // Check if we've reached boil
    if (detectBoil(currentTemp, previousTemp, deltaTimeSec))
    {
      hasBoiled = true;
    }
  }
  else
  {
    // 3) If boiling is detected, maintain simmer
    maintainSimmer(currentTemp);
  }

  // 4) Debug print
  Serial.print("Temp: ");
  Serial.print(currentTemp);
  Serial.print(" °C | Heater: ");
  Serial.println(heaterOn ? "ON" : "OFF");

  // 5) Update previousTemp
  previousTemp = currentTemp;
}

// Arduino setup routine
// ---------------------------------------------------------------------------
void setup()
{
  // Serial monitor for debugging
  Serial.begin(9600);

  // Set up the relay control pin
  pinMode(RELAY_CONTROL_PIN, OUTPUT);
  setHeaterPower(false); // Start with the heater off

  // If needed, configure the analog pin. On Arduino Uno, A0 is already analog capable
  // pinMode(NTC_PIN, INPUT); // Not strictly required for Uno analog pins
}

void loop()
{
  controlLoop();
  delay(50); // Brief delay to avoid spamming the loop
}
