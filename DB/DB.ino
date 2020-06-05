#define SoundSensorPin A5 //this pin read the analog voltage from the sound level meter
#define VREF 5.0 //voltage on AREF pin,default:operating voltage
#define getOriginalSoundLevel analogRead(SoundSensorPin) // (int) 0 ~ 1023
void setup()
{
  //ACSR=0x8;
  Serial.begin(9600);
}
void loop()
{
    printOriginalSoundLevel();
}

float getDbValue(){
    return analogRead(SoundSensorPin) / 1024.0 * VREF * 50.0;
}
void printDbValue() {
  Serial.print(getDbValue(), 1); // print to 1 decimal places
  Serial.println(" dBA");
}
void printOriginalSoundLevel() {
  //Serial.print("Sound Level: ");
  Serial.println(getOriginalSoundLevel);
}
