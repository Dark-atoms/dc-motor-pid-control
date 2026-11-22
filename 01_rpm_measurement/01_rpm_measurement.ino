// Stable and Correct MH Hall Sensor RPM Measurement
// Fixes inverse-speed issue (use FALLING edge, debounce logic)

const byte sensorPin = 2;              // MH Hall sensor output pin
const byte pulsesPerRevolution = 1;    // Use 1 if one magnet; 2 if two pulses per rev
const unsigned long zeroTimeout = 800000;  // µs before RPM = 0 (~0.8s)
const unsigned long debounceMicros = 1000; // ignore pulses within 1 ms (for bounce)

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

float rpm = 0;
float rpmFiltered = 0;
float alpha = 0.2;  // smoothing factor (0.1 = smooth, 0.3 = responsive)

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);

  // MH Hall sensors usually pull LOW when magnet passes → FALLING edge
  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseISR, FALLING);

  Serial.println("Fixed MH Hall Sensor RPM Measurement (1200 RPM Range)");
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long currentMicros = micros();
  unsigned long intervalCopy;

  noInterrupts();
  intervalCopy = pulseInterval;
  interrupts();

  if ((currentMicros - lastPulseTime) > zeroTimeout) {
    rpm = 0;  // motor stopped
  } else if (intervalCopy > 0) {
    float frequency = 1000000.0 / intervalCopy;
    rpm = (frequency / pulsesPerRevolution) * 60.0;
  }

  // Smooth output
  rpmFiltered = rpmFiltered + alpha * (rpm - rpmFiltered);

  // Print every 300 ms
  if (millis() - lastPrint >= 300) {
    Serial.print("Interval (µs): ");
    Serial.print(intervalCopy);
    Serial.print("\tRPM: ");
    Serial.println(rpmFiltered, 1);
    lastPrint = millis();
  }
}

void pulseISR() {
  unsigned long now = micros();
  
  // Debounce: ignore too-fast pulses (sensor bounce or noise)
  if (now - lastPulseTime > debounceMicros) {
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
  }
}
