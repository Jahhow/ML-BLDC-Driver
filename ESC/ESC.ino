// https://simple-circuit.com/arduino-brushless-dc-motor-controller/

#define SPEED_UP          A0 // BLDC motor speed-up button
#define SPEED_DOWN        A1 // BLDC motor speed-down button
#define SoundSensorPin    A5 // this pin read the analog voltage from the sound level meter
#define PWM_MAX_DUTY     255
#define PWM_MIN_DUTY       0
#define PWM_START_DUTY    30
#define ARRLEN            64
#define VREF             5.0 // voltage on AREF pin,default:operating voltage

#define getOriginalSoundLevel analogRead(SoundSensorPin) // (int) 0 ~ 1023

float getDbValue() {
  return getOriginalSoundLevel / 1024.0 * VREF * 50.0;
}
void printDbValue() {
  Serial.print(getDbValue(), 1); // print to 1 decimal places
  Serial.println(" dBA");
}
void printOriginalSoundLevel() {
  Serial.print("Sound Level: ");
  Serial.println(getOriginalSoundLevel);
}
int Loss() {
  return getOriginalSoundLevel;
}

byte bldc_step = 0, pwm_duty = PWM_START_DUTY;
byte pwm[ARRLEN];
unsigned int commutationCounter = 0;
unsigned int bldc_moveCount = 0;
int innerEpoch = 5, lose;
size_t iarr = 0;
unsigned long ms = 0;

// Analog comparator ISR
ISR (ANALOG_COMP_vect) {
  // BEMF debounce
  for (int i = 0; i < 16; i++) {
    if ((bldc_step & 1) != ((ACSR >> ACO) & 1))
      return;
  }

  bldc_move();
  /*if (++commutationCounter > 1000) {
    printOriginalSoundLevel();
    commutationCounter = 0;
    }*/
}
void bldc_move() {       // BLDC motor commutation function
  switch (bldc_step) {
    case 0: AH_BL(); BEMF_C_RISE(); break;
    case 1: AH_CL(); BEMF_B_FALL(); break;
    case 2: BH_CL(); BEMF_A_RISE(); break;
    case 3: BH_AL(); BEMF_C_FALL(); break;
    case 4: CH_AL(); BEMF_B_RISE(); break;
    case 5: CH_BL(); BEMF_A_FALL(); break;
  }
  ++bldc_moveCount;
  bldc_step = (bldc_step + 1) % 6;
}

void setup() {
  DDRB = 0b1110;   // Configure pins 9(OC1A), 10(OC1B) and 11(OC2A) as outputs.  Used as PWM pins.
  DDRD = 0b111000; // Configure pins 3, 4 and 5 as outputs.  Use as normal pins.

  TCCR1B = 1;// Timer1 (no prescaling)
  TCCR2B = 1;// Timer2 (no prescaling)

  pinMode(SPEED_UP,   INPUT_PULLUP);
  pinMode(SPEED_DOWN, INPUT_PULLUP);

  // Analog comparator setting
  //ACSR  = 0x10; // Disable and clear (flag bit) analog comparator interrupt
  ACSR |= 0x08; // Enable analog comparator interrupt

  SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  Serial.begin(9600);
}
void loop() {
  // Motor start
  unsigned long curMs = millis();
  unsigned long distanceMs = curMs - ms;
  //Serial.println(distanceMs);
  if (distanceMs > 10) {
    if(bldc_moveCount==0){
      bldc_move();
    }
    ms = curMs;
    bldc_moveCount=0;
  }

  if (pwm_duty < PWM_MAX_DUTY && !(digitalRead(SPEED_UP))) {
    SET_PWM_DUTY(++pwm_duty);
    delay(30);
  }
  if (pwm_duty > PWM_MIN_DUTY && !(digitalRead(SPEED_DOWN))) {
    SET_PWM_DUTY(--pwm_duty);
    delay(30);
  }

  // Noise Canceling Algorithm
  // for (iarr = 0; iarr < ARRLEN; iarr++)
  /* {
    int direction = 1;
    int loseDiff;
    int v = 1;
    lose = Loss();
    //printf("\n iarr: %u, Loss %d ", iarr, lose);
    for (size_t iInnerEpoch = 0; iInnerEpoch < innerEpoch; iInnerEpoch++)
    {
      uint8_t oldCurPwm = pwm[iarr];
      pwm[iarr] = max(min((int)oldCurPwm + v, 255), 0);
      int newLose = Loss();
      loseDiff = newLose - lose;
      //printf("%d ", newLose);
      if (loseDiff >= 0)
      {
        pwm[iarr] = oldCurPwm;
        direction = -direction;
      }
      lose = newLose;
      if (loseDiff == 0)
        break;
      v = direction * abs(loseDiff);
    }
    }
    iarr = (iarr + 1) & ~ARRLEN; // (iarr + 1) % ARRLEN
  */
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
//  if (duty < PWM_MIN_DUTY)
//    duty  = PWM_MIN_DUTY;
//  if (duty > PWM_MAX_DUTY)
//    duty  = PWM_MAX_DUTY;
  OCR1A  = duty;                   // Set pin 9  PWM duty cycle
  OCR1B  = duty;                   // Set pin 10 PWM duty cycle
  OCR2A  = duty;                   // Set pin 11 PWM duty cycle
}
