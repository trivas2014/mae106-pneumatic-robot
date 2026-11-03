/* 
 * MAE 106 - Experimental Test B.5: Single Impulse Response
 * This code fires the piston ONCE and records the time of every 
 * wheel rotation to calculate impulse response.
 */

const int SOLENOID_PIN = 2;
const int REED_PIN     = 2; // ISR Pin
volatile int tickCount = 0;
volatile unsigned long tickTimes[3]; // Store times of each tick

void handleReedSwitch() {
  if (tickCount < 50) {
    tickTimes[tickCount] = millis();
    tickCount++;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REED_PIN), handleReedSwitch, FALLING);
  
  delay(5000); // Wait 5 seconds to clear the area
  
  // FIRE ONCE
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(150); // Optimized Piston On-Time from Section B.4
  digitalWrite(SOLENOID_PIN, LOW);
  
  // Monitor for 3 seconds of movement
  unsigned long startTime = millis();
  while(millis() - startTime < 3000) { 
    // Just waiting for ISR to collect data
  }
  
  // Output results for plotting
  Serial.println("Tick, Time (ms)");
  for(int i=0; i < tickCount; i++) {
    Serial.print(i+1); Serial.print(", ");
    Serial.println(tickTimes[i]);
  }
}

void loop() { /* End of Test */ }
