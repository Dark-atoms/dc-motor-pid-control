// --- PID Closed-Loop Motor Speed Control ---
// Feedback: Hall Sensor (1 pulse/rev)
// PWM output: Pin 9
// Direction pin: Pin 8 (optional)
// Serial interface for tuning: S:<rpm>  Kp:<val>  Ki:<val>  Kd:<val>  RESET

const byte sensorPin = 2;              // Hall sensor input
const byte pwmPin = 9;                 // Motor PWM output
const byte dirPin = 8;                 // Optional direction pin
const byte pulsesPerRevolution = 1;    // Update if you have multiple magnets

const unsigned long debounceMicros = 1000UL;  // Debounce threshold
const unsigned long zeroTimeout = 1000000UL;  // 1 sec no pulse => motor stopped
const float rpmFilterAlpha = 0.12f;           // Low-pass filter factor

// --- PID constants (initial) ---
float setpointRPM = 280.0;    // Desired fixed RPM
float Kp = 0.8;
float Ki = 0.6;
float Kd = 0.2;

// --- Internal variables ---
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulseIntervalMicros = 0;
volatile unsigned long lastPulseSeenMicros = 0;

float measuredRPM = 0.0f;
float filteredRPM = 0.0f;
float integralTerm = 0.0f;
float previousError = 0.0f;

const int pwmMin = 0;
const int pwmMax = 255;

unsigned long lastControlMillis = 0;
unsigned long lastPrintMillis = 0;
unsigned long lastPIDMillis = 0;
float controlOutput = 0;

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);

  attachInterrupt(digitalPinToInterrupt(sensorPin), pulseISR, FALLING);

  Serial.println("PID Closed-loop Motor Speed Control Started");
  Serial.println("Commands: Kp:<val>  Ki:<val>  Kd:<val>  S:<rpm>  RESET");
}

void loop() {
  unsigned long nowMillis = millis();

  // --- Calculate RPM every 100 ms ---
  if (nowMillis - lastControlMillis >= 100UL) {
    unsigned long intervalCopy, lastPulseCopy;
    noInterrupts();
    intervalCopy = pulseIntervalMicros;
    lastPulseCopy = lastPulseSeenMicros;
    interrupts();

    unsigned long nowMicros = micros();

    if ((nowMicros - lastPulseCopy) > zeroTimeout) {
      measuredRPM = 0.0f;
    } else if (intervalCopy > 0) {
      float freqHz = 1000000.0f / (float)intervalCopy;
      measuredRPM = (freqHz / (float)pulsesPerRevolution) * 60.0f;
    }

    // Apply low-pass filter
    filteredRPM = filteredRPM + rpmFilterAlpha * (measuredRPM - filteredRPM);
    lastControlMillis = nowMillis;
  }

  // --- PID Controller update (every ~100 ms) ---
  unsigned long nowPIDMillis = millis();
  float dt = (nowPIDMillis - lastPIDMillis) / 1000.0f;  // seconds
  if (dt >= 0.1f) {
    float error = setpointRPM - filteredRPM;

    // Integral term with anti-windup
    integralTerm += error * dt;
    if (integralTerm > 255) integralTerm = 255;
    if (integralTerm < -255) integralTerm = -255;

    // Derivative term (rate of change)
    float derivative = (error - previousError) / dt;

    // PID Output
    controlOutput = (Kp * error) + (Ki * integralTerm) + (Kd * derivative);

    // Clamp output
    int pwmOut = constrain((int)round(controlOutput), pwmMin, pwmMax);

    // Apply PWM
    analogWrite(pwmPin, pwmOut);

    previousError = error;
    lastPIDMillis = nowPIDMillis;

    // --- Debug print every 400 ms ---
    if (nowPIDMillis - lastPrintMillis >= 400UL) {
      Serial.print("Setpoint: "); Serial.print(setpointRPM, 1);
      Serial.print("\tRPM: "); Serial.print(filteredRPM, 1);
      Serial.print("\tErr: "); Serial.print(error, 1);
      Serial.print("\tPWM: "); Serial.print((int)controlOutput);
      Serial.print("\tKp: "); Serial.print(Kp, 2);
      Serial.print("\tKi: "); Serial.print(Ki, 2);
      Serial.print("\tKd: "); Serial.println(Kd, 2);
      lastPrintMillis = nowPIDMillis;
    }
  }

  // --- Serial commands for tuning ---
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("S:")) {
      setpointRPM = cmd.substring(2).toFloat();
      Serial.print("New setpoint: "); Serial.println(setpointRPM);
    } else if (cmd.startsWith("Kp:")) {
      Kp = cmd.substring(3).toFloat();
      Serial.print("New Kp: "); Serial.println(Kp, 3);
    } else if (cmd.startsWith("Ki:")) {
      Ki = cmd.substring(3).toFloat();
      Serial.print("New Ki: "); Serial.println(Ki, 3);
    } else if (cmd.startsWith("Kd:")) {
      Kd = cmd.substring(3).toFloat();
      Serial.print("New Kd: "); Serial.println(Kd, 3);
    } else if (cmd == "RESET") {
      integralTerm = 0;
      previousError = 0;
      analogWrite(pwmPin, 0);
      Serial.println("PID controller reset");
    }
  }
}

void pulseISR() {
  unsigned long now = micros();
  if (now - lastPulseMicros > debounceMicros) {
    if (lastPulseMicros != 0) {
      pulseIntervalMicros = now - lastPulseMicros;
    }
    lastPulseMicros = now;
    lastPulseSeenMicros = now;
  }
}
