# Arduino NTC Thermistor-Based Heater Control

A simple Arduino sketch that:
1. Reads temperature from an NTC thermistor (via an analog input).
2. Detects when the temperature reaches boiling.
3. Maintains a simmer temperature using on/off (bang-bang) control with hysteresis.

## Quick Setup
1. **Hardware**  
   - Arduino board (e.g., Uno).  
   - NTC thermistor (10 kΩ at 25°C) and a 10 kΩ fixed resistor in a voltage divider.  
   - Relay (or transistor + relay) connected to a digital pin for heater control.

2. **Wiring**  
   - NTC + resistor in series from 5V to GND; junction to A0 (NTC_PIN).  
   - Digital pin 8 to relay input; relay switches power to the heater.

3. **Code**  
   - Upload the provided code to the Arduino.  
   - It keeps the heater ON until boiling is detected (based on temperature slope), then switches to a simmer control mode.

4. **Usage**  
   - Open Serial Monitor at 9600 baud to see temperature and heater status.  
   - Adjust thresholds (`BOIL_SLOPE_THRESHOLD`, `NEAR_BOIL_TEMP`, `SIMMER_TARGET`, etc.) as needed.

Please follow proper safety procedures when dealing with high-power or high-temperature systems.
