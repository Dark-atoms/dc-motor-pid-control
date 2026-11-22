// Full H-Bridge PWM with Serial Control
// 2 kHz complementary PWM, 5 µs dead time
// Duty < 50% = Reverse, Duty = 50% = Stop, Duty > 50% = Forward

const int PWM_A = 10;      // H-bridge side A
const int PWM_B = 6;       // H-bridge side B
const int DEAD_TIME_US = 5;

float duty = 0.5;          // Start stopped
int inputValue = 5;

void setup() {
  Serial.begin(9600);
  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);

  // Timer1 setup: Fast PWM with ICR1 as TOP -> 2 kHz
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  ICR1   = 1000;                       // TOP → 16 MHz/(8*1000)=2 kHz
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  TIMSK1 |= (1 << TOIE1) | (1 << OCIE1A);  // Enable interrupts
}

void loop() {
  if (Serial.available()) {
    inputValue = Serial.parseInt();
    while (Serial.available()) Serial.read(); // clear buffer

    if (inputValue >= 0 && inputValue <= 10) {
      duty = inputValue / 10.0; // map 0–10 → 0.0–1.0
      Serial.print("Input: ");
      Serial.print(inputValue);
      Serial.print("  Duty: ");
      Serial.print(duty * 100, 1);
      Serial.println("%");
    } else {
      Serial.println("Enter a number 0–10");
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  if (duty > 0.5) { // Forward
    digitalWrite(PWM_A, HIGH);
    delayMicroseconds(DEAD_TIME_US);
    digitalWrite(PWM_B, LOW);
  } 
  else if (duty < 0.5) { // Reverse
    digitalWrite(PWM_B, HIGH);
    delayMicroseconds(DEAD_TIME_US);
    digitalWrite(PWM_A, LOW);
  } 
  else { // Stop
    digitalWrite(PWM_A, LOW);
    digitalWrite(PWM_B, LOW);
  }
}

ISR(TIMER1_OVF_vect) {
  if (duty > 0.5) { // Forward
    digitalWrite(PWM_A, LOW);
    delayMicroseconds(DEAD_TIME_US);
    digitalWrite(PWM_B, HIGH);
  } 
  else if (duty < 0.5) { // Reverse
    digitalWrite(PWM_B, LOW);
    delayMicroseconds(DEAD_TIME_US);
    digitalWrite(PWM_A, HIGH);
  } 
  else { // Stop
    digitalWrite(PWM_A, LOW);
    digitalWrite(PWM_B, LOW);
  }
}
