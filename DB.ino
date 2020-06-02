#define SoundSensorPin A5 //this pin read the analog voltage from the sound level meter
#define VREF 5.0 //voltage on AREF pin,default:operating voltage
 
void setup()
{
    Serial.begin(115200);
}
 
void loop()
{
    printDbValue();
    delay(125);
}

float getDbValue(){
    return analogRead(SoundSensorPin) / 1024.0 * VREF * 50.0;
}
void printDbValue(){
    Serial.print(getDbValue(), 1); // print to 1 decimal places
    Serial.println(" dBA");
}