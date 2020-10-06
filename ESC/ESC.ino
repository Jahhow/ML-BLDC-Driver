// https://simple-circuit.com/arduino-brushless-dc-motor-controller/

#define SoundSensorPin    A5 // this pin read the analog voltage from the sound level meter
#define PWM_START_DUTY    70
#define ARRLEN            32
#define NUM_DEBOUNCE_CHECK_LOW_SPEED 8;
#define NUM_DEBOUNCE_CHECK_HIGH_SPEED 8; // Lower this number can gain a little more maximum motor speed

byte bldc_step = 0, torque = PWM_START_DUTY;
byte pwm[ARRLEN];
size_t indexPwm = 0;
unsigned int isr_bldc_moveCount = 0;
unsigned int numDebounceCheck = NUM_DEBOUNCE_CHECK_LOW_SPEED;
int finalIndexPwm;

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce
  for (size_t i = 0; i < numDebounceCheck; ++i) {
    if ((bldc_step & 1) != ((ACSR >> ACO) & 1))
      return;
  }
  finalIndexPwm = indexPwm;
  if(TIMSK1)
    SET_PWM_DUTY(pwm[0]);
  indexPwm = 1;
  bldc_move();
  ++isr_bldc_moveCount;
}

ISR(TIMER1_OVF_vect){
  SET_PWM_DUTY(pwm[indexPwm]);
  if (indexPwm < ARRLEN - 1)
    ++indexPwm;
}

void bldc_move() { // BLDC motor commutation function
  noInterrupts();
  switch (bldc_step) {
    case 0: AH_BL(); BEMF_C_RISE(); ++bldc_step; interrupts(); return;
    case 1: AH_CL(); BEMF_B_FALL(); ++bldc_step; interrupts(); return;
    case 2: BH_CL(); BEMF_A_RISE(); ++bldc_step; interrupts(); return;
    case 3: BH_AL(); BEMF_C_FALL(); ++bldc_step; interrupts(); return;
    case 4: CH_AL(); BEMF_B_RISE(); ++bldc_step; interrupts(); return;
    case 5: CH_BL(); BEMF_A_FALL(); bldc_step = 0; interrupts(); return;
  }
}

void setup() {
  DDRB = 0b1110;   // Configure pins 9(OC1A), 10(OC1B) and 11(OC2A) as outputs.  Use as PWM pins.
  DDRD = 0b111000; // Configure pins 3, 4 and 5 as outputs.  Use as normal pins.
  
  // Timer1 no prescaling, Fast PWM
  TCCR1A = 1;         // pin 11 (OC2A) PWM OFF
  TCCR1B = 1 | (1<<WGM12); // Timer1 no prescaling, Fast PWM

  // Timer2 no prescaling, Fast PWM
  TCCR2A = 1 | (1<<WGM21);         // pin 11 (OC2A) PWM OFF
  TCCR2B = 1; // Timer2 no prescaling

  TIMSK1 = (1 << TOIE1); // enable timer overflow interrupt

  pinMode(LED_BUILTIN, OUTPUT);

  // Analog comparator setting
  //ACSR = 1 << ACI; // Disable and clear (flag bit) analog comparator interrupt
  ACSR = 1 << ACIE; // Enable Analog Comparator Interrupt

  SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY

  // init pwm array
  const int nSteps=10;
  const int step = PWM_START_DUTY/nSteps;
  int curDuty = PWM_START_DUTY-step*nSteps/2;
  size_t i = 0;
  for (; i < nSteps; ++i) {
    curDuty += step;
    pwm[i] = curDuty;
  }
  for (; i < ARRLEN - nSteps; ++i) {
    pwm[i] = curDuty;
  }
  for (; i < ARRLEN; ++i) {
    curDuty -= step;
    pwm[i] = curDuty;
  }
  Serial.begin(9600);
}
void loop() {
  if (torque == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    ACSR = 1 << ACI; // Disable and clear (flag bit) analog comparator interrupt
    MOTOR_IDLE();
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    if (isr_bldc_moveCount == 0) {
      // Motor start
      ACSR = 1 << ACIE;
      TIMSK1 = 0; // disable timer overflow interrupt
      SET_PWM_DUTY(PWM_START_DUTY);
      bldc_move();
    } else {
      TIMSK1 = (1 << TOIE1); // enable timer overflow interrupt
      if (isr_bldc_moveCount > 60) {
        numDebounceCheck = NUM_DEBOUNCE_CHECK_HIGH_SPEED;
      } else {
        numDebounceCheck = NUM_DEBOUNCE_CHECK_LOW_SPEED;
      }
      isr_bldc_moveCount = 0;
    }
  }

  Serial.println(finalIndexPwm);

  // while (Serial.available()) {
  //   torque = Serial.read();
  // }

  delay(30);
}

void BEMF_A_RISE() {
  ADCSRB = 0;    // Select AIN1 (Pin7) as comparator negative input
  //ACSR |= 3;            // Set interrupt on rising edge
}
void BEMF_A_FALL() {
  ADCSRB = 0;    // Select AIN1 (Pin7) as comparator negative input
  //ACSR &= ~1;           // Set interrupt on falling edge
}
void BEMF_B_RISE() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  //ACSR |= 3;
}
void BEMF_B_FALL() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 2;              // Select analog channel 2 (A2) as comparator negative input
  //ACSR &= ~1;
}
void BEMF_C_RISE() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  //ACSR |= 3;
}
void BEMF_C_FALL() {
  ADCSRA = 0;   // Disable the ADC module
  ADCSRB = (1 << ACME);
  ADMUX = 3;              // Select analog channel 3 (A3) as comparator negative input
  //ACSR &= ~1;
}

/////////////////////////////////////////////////////////////

void AH_BL() {
  PORTD  = 1 << 4; // pin 4 on
  TCCR1A = 1;            // pin 9, 10 OFF
  TCCR2A = 0x81 | (1<<WGM21);         // pin 11 (OC2A) PWM ON
}
void AH_CL() {
  PORTD  = 1 << 3; // pin 3 on
  TCCR1A = 1;
  TCCR2A = 0x81 | (1<<WGM21);
}

void BH_CL() {
  PORTD  = 1 << 3; // pin 3 on
  TCCR1A = 0x21;         // pin 9  OFF, pin 10 (OC1B) PWM ON,
  TCCR2A = 0x1 | (1<<WGM21);     // pin 11 OFF
}
void BH_AL() {
  PORTD  = 1 << 5; // pin 5 on
  TCCR1A = 0x21;
  TCCR2A = 0x1 | (1<<WGM21);     // pin 11 OFF
}

void CH_AL() {
  PORTD  = 1 << 5; // pin 5 on
  TCCR1A = 0x81;         // pin 9 (OC1A) PWM ON, pin 10 OFF
  TCCR2A = 0x1 | (1<<WGM21);     // pin 11 OFF
}
void CH_BL() {
  PORTD  = 1 << 4; // pin 4 on
  TCCR1A = 0x81;
  TCCR2A = 0x1 | (1<<WGM21);     // pin 11 OFF
}
void MOTOR_IDLE() {
  PORTD  = 0; // pin 3, 4, 5 OFF
  TCCR1A = 0; // pin 9, 10 OFF
  TCCR2A = 0; // pin 11 OFF
}

void SET_PWM_DUTY(byte duty) {
  OCR1A = duty; // Set pin 9  PWM duty cycle
  OCR1B = duty; // Set pin 10 PWM duty cycle
  OCR2A = duty; // Set pin 11 PWM duty cycle
}
