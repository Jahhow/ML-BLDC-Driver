// https://simple-circuit.com/arduino-brushless-dc-motor-controller/

#define SPEED_UP          A0 // BLDC motor speed-up button
#define SPEED_DOWN        A1 // BLDC motor speed-down button
#define SoundSensorPin    A5 // this pin read the analog voltage from the sound level meter
#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      50
#define PWM_START_DUTY    100
#define VREF              5.0// voltage on AREF pin,default:operating voltage

float getDbValue(){
  return analogRead(SoundSensorPin) / 1024.0 * VREF * 50.0;
}
void printDbValue(){
  Serial.print(getDbValue(), 1); // print to 1 decimal places
  Serial.println(" dBA");
}

byte bldc_step = 0, pwm_duty;
unsigned int commutationCounter=0;

void setup() {
  DDRB = 0b1110;   // Configure pins 9(OC1A), 10(OC1B) and 11(OC2A) as outputs.  Used as PWM pins.
  DDRD = 0b111000; // Configure pins 3, 4 and 5 as outputs.  Use as normal pins.

  // Timer1 (no prescaling)
  TCCR1B = 1;

  // Timer2 (no prescaling)
  TCCR2B = 1;

  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
  pinMode(SPEED_UP,   INPUT_PULLUP);
  pinMode(SPEED_DOWN, INPUT_PULLUP);

  Serial.begin(115200);
}
// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce
  for (i = 0; i < 10; i++) {
    while ((bldc_step & 1) != (ACSR & (1 << ACO))) // ACO: Analog Comparator Output
      ;
  }
  bldc_move();
  bldc_step = (bldc_step + 1) % 6;
  if(++commutationCounter>1000){
    printDbValue();
    commutationCounter = 0;
  }
}
void bldc_move() {       // BLDC motor commutation function
  switch (bldc_step) {
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}

void loop() {
  SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  unsigned int i = 5000;
  // Motor start
  while (i > 100) {
    delayMicroseconds(i);
    bldc_move();
    bldc_step++;
    bldc_step %= 6;
    i = i - 20;
  }
  pwm_duty = PWM_START_DUTY;
  ACSR |= 0x08;                    // Enable analog comparator interrupt
  while (1) {
    while (!(digitalRead(SPEED_UP)) && pwm_duty < PWM_MAX_DUTY) {
      pwm_duty++;
      SET_PWM_DUTY(pwm_duty);
      delay(100);
    }
    while (!(digitalRead(SPEED_DOWN)) && pwm_duty > PWM_MIN_DUTY) {
      pwm_duty--;
      SET_PWM_DUTY(pwm_duty);
      delay(100);
    }
  }
}

void BEMF_A_RISING() {
  ADCSRB = 0;    // Select AIN1 (Pin7) as comparator negative input
  ACSR |= 3;            // Set interrupt on rising edge
}
void BEMF_A_FALLING() {
  ADCSRB = 0;    // Select AIN1 (Pin7) as comparator negative input
  ACSR &= ~1;           // Set interrupt on falling edge
}
void BEMF_B_RISING() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  ACSR |= 3;
}
void BEMF_B_FALLING() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  ACSR &= ~1;
}
void BEMF_C_RISING() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  ACSR |= 3;
}
void BEMF_C_FALLING() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  ACSR &= ~1;
}

////////////////////////////////////////////////

void AH_BL() {
  PORTD  = 1 << 4; // pin 4 on
  TCCR1A = 0;            // pin 9, 10 OFF
  TCCR2A = 0x81;         // pin 11 (OC2A) PWM ON
}
void AH_CL() {
  PORTD  = 1 << 3; // pin 3 on
  TCCR1A = 0;
  TCCR2A = 0x81;
}

void BH_CL() {
  PORTD  = 1 << 3; // pin 3 on
  TCCR1A = 0x21;         // pin 9  OFF, pin 10 (OC1B) PWM ON,
  TCCR2A = 0;            // pin 11 OFF
}
void BH_AL() {
  PORTD  = 1 << 5; // pin 5 on
  TCCR1A = 0x21;
  TCCR2A = 0;
}

void CH_AL() {
  PORTD  = 1 << 5; // pin 5 on
  TCCR1A = 0x81;         // pin 9 (OC1A) PWM ON, pin 10 OFF
  TCCR2A = 0;            // pin 11 OFF
}
void CH_BL() {
  PORTD  = 1 << 4; // pin 4 on
  TCCR1A = 0x81;
  TCCR2A = 0;
}

void SET_PWM_DUTY(byte duty) {
  if (duty < PWM_MIN_DUTY)
    duty  = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty  = PWM_MAX_DUTY;
  OCR1A  = duty;                   // Set pin 9  PWM duty cycle
  OCR1B  = duty;                   // Set pin 10 PWM duty cycle
  OCR2A  = duty;                   // Set pin 11 PWM duty cycle
}