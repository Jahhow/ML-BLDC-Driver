// https://simple-circuit.com/arduino-brushless-dc-motor-controller/

#define SPEED_UP          A0          // BLDC motor speed-up button
#define SPEED_DOWN        A1          // BLDC motor speed-down button
#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      50
#define PWM_START_DUTY    100

byte bldc_step = 0, motor_speed;
unsigned int i;
void setup() {
  DDRD = 0b111000; // Configure pins 3, 4 and 5 as outputs (PWM function is available on pins 3, 5, 6, 9, 10, 11)
  DDRB = 0b1110;   // Configure pins 9, 10 and 11 as outputs

  // Timer1 (no prescaling)
  TCCR1B = 1;

  // Timer2 (no prescaling)
  TCCR2B = 1;

  // Analog comparator setting
  ACSR   = 0x10;           // Disable and clear (flag bit) analog comparator interrupt
  pinMode(SPEED_UP,   INPUT_PULLUP);
  pinMode(SPEED_DOWN, INPUT_PULLUP);
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
  i = 5000;
  // Motor start
  while (i > 100) {
    delayMicroseconds(i);
    bldc_move();
    bldc_step++;
    bldc_step %= 6;
    i = i - 20;
  }
  motor_speed = PWM_START_DUTY;
  ACSR |= 0x08;                    // Enable analog comparator interrupt
  while (1) {
    while (!(digitalRead(SPEED_UP)) && motor_speed < PWM_MAX_DUTY) {
      motor_speed++;
      SET_PWM_DUTY(motor_speed);
      delay(100);
    }
    while (!(digitalRead(SPEED_DOWN)) && motor_speed > PWM_MIN_DUTY) {
      motor_speed--;
      SET_PWM_DUTY(motor_speed);
      delay(100);
    }
  }
}

void BEMF_A_RISING() {
  ADCSRB = (0 << ACME);    // Select AIN1 (Pin7) as comparator negative input
  ACSR |= 3;            // Set interrupt on rising edge
}
void BEMF_A_FALLING() {
  ADCSRB = (0 << ACME);    // Select AIN1 (Pin7) as comparator negative input
  ACSR &= ~1;           // Set interrupt on falling edge
}
void BEMF_B_RISING() {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  ACSR |= 3;
}
void BEMF_B_FALLING() {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  ACSR &= ~1;
}
void BEMF_C_RISING() {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  ACSR |= 3;
}
void BEMF_C_FALLING() {
  ADCSRA = (0 << ADEN);   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  ACSR &= ~1;
}

////////////////////////////////////////////////

void AH_BL() {
  PORTD &= ~0b101000;
  PORTD |=  0b010000;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void AH_CL() {
  PORTD &= ~0b110000;
  PORTD |=  0b001000;
  TCCR1A =  0;            // Turn pin 11 (OC2A) PWM ON (pin 9 & pin 10 OFF)
  TCCR2A =  0x81;         //
}
void BH_CL() {
  PORTD &= ~0b110000;
  PORTD |=  0b001000;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void BH_AL() {
  PORTD &= ~0b011000;
  PORTD |=  0b100000;
  TCCR2A =  0;            // Turn pin 10 (OC1B) PWM ON (pin 9 & pin 11 OFF)
  TCCR1A =  0x21;         //
}
void CH_AL() {
  PORTD &= ~0b011000;
  PORTD |=  0b100000;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
}
void CH_BL() {
  PORTD &= ~0b101000;
  PORTD |=  0b010000;
  TCCR2A =  0;            // Turn pin 9 (OC1A) PWM ON (pin 10 & pin 11 OFF)
  TCCR1A =  0x81;         //
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
