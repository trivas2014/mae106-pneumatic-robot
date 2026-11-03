/* 
 * MAE 106 - Experimental Test B.6: 10-Impulse Comparison
 * This code fires the piston exactly 10 times and records 
 * cumulative distance for comparison with the simulator.
 */

const int SOLENOID_PIN = 2;
const int REED_PIN     = 2;
const float INCHES_PER_TICK = 8.51 / 2.0;

volatile int tickCount = 0;
void handleReedSwitch() { tickCount++; }

void setup() {
  Serial.begin(9600);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), handleReedSwitch, FALLING);

  delay(5000); // Safety delay

  for (int i = 0; i < 10; i++) {
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(150); // Fire
    digitalWrite(SOLENOID_PIN, LOW);
    delay(200); // Recovery (Off-time)
    
    // Print distance after each fire for the report graph
    Serial.print("Fire: "); Serial.print(i+1);
    Serial.print(" | Dist: "); Serial.println(tickCount * INCHES_PER_TICK);
  }
}

void loop() { /* End of Test */ }
