#include "SoftwareSerial.h"
#include "TinyGPS++.h"

#define gpsRx 6
#define gpsTx 5
#define gpsBitrate 9600
SoftwareSerial ss(gpsTx, gpsRx);
TinyGPSPlus gps;
//TinyGPSCustom gpsValid(gps, "GPRMC", 2);

#define bitrate 9600  //Bitrate for the serial interface
#define vs 5.0        //Supply voltage
#define loopTime 1000 //Duration of one loop

//***** Sensor calibration *****

// NTC Temp sensor
#define r1 10000.0
const float a = 3.354016E-3;
const float b = 2.569850E-4;
const float c = 2.620131E-6;
const float d = 6.383091E-8;

// Pressure sensor
#define pressureSens 0.009
#define pressureOff 0.095
float prs;

// LM35DZ Temp sensor
#define tmpSens 0.01      //Sensitivity
#define tmpOff 0.0733138  //Offset

// Accelerometer
#define accSensMode true  //Accelerometer sensitivity mode. false: 1.5 g, true: 6 g
#define acc15 0.8         //Sensitivity at 1.5 g (from datasheet)
#define acc6 0.206        //Sensitivity at 6 g (from datasheet)
#define accXOff 1.61      //0 g outputs. Slightly wrong, since
#define accYOff 1.68      //the sensor is very hard to calibrate
#define accZOff 1.71      //due to the inconsistency of these values.

//***** Altitude calculation *****
#define tmpGrad -0.0065   //Temperature gradient
const float R = 287.06;   //Specific gas constant
const float g = 9.81;     //Gravitational acceleration

#define alertAltitude 100.0

#define ledPin 3              //Pin of LED
//#define ledPin2 10            //Pin of LED 2
#define ledDelayOn 250        //Blink delay of LED 1 in ms
#define ledDelayOff 1750      //Blink delay of LED 1 in ms
//#define ledDelayOn2 250       //Blink delay of LED 2 in ms
//#define ledDelayOff2Real 1750 //Blink delay of LED 2 in ms
bool ledToggle = false;       //If true, LED 1 will toggle when led() is called.
//bool ledToggle2 = false;      //If true, LED 2 will toggle when led2() is called.
bool ledOn = true;           //
//bool led2On = false;          //

#define buzzerPin 2           //Pin of buzzer
#define buzzerDelayOn 250
#define buzzerDelayOff 1750
bool buzzerToggle = false;
bool buzzerOn = true;

bool useLed = true;
//bool useLed2 = true;
bool useBuzzer = true;
bool forceAlert = false;
bool forceSilence = false;

#define switchPinHelp 7
#define switchPinOk 8

unsigned long counter = 0;    //Used to check how many times the program has run
bool calculate = true;        //Onboard calculations on/off
float alt = 0.0;
#define delimiter ','

void setup()
{
  Serial.begin(bitrate);
  ss.begin(gpsBitrate);
  pinMode(ledPin, OUTPUT);
  //pinMode(ledPin2, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  //analogWrite(buzzerPin, 255);
  pinMode(switchPinHelp, INPUT_PULLUP);
  pinMode(switchPinOk, INPUT_PULLUP);
  /*
  Serial.print("MESS,Status,Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,");                                                                                                      //Heading row
  Serial.println("Acceleration Z-axis,Pressure / kPa,Temperature (LM35) / K,Temperature (NTC) / K,Acceleration X-axis / g,Acceleration Y-axis / g,Acceleration Z-axis / g,Altitude / m,GPS Valid,Latitude,Longitude,Altitude (GPS),Speed,Course");//for the output
  Serial.print("MESS,Status,Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),Acceleration X-axis,Acceleration Y-axis,");
  Serial.println("Acceleration Z-axis,GPS Valid,Latitude,Longitude,Altitude (GPS),Speed,Course");
  */
  Serial.print("MESS,Status,Counter,Time / ms,Pressure,Temperature (LM35),Temperature (NTC),");                                                                                                      //Heading row
  Serial.println("Pressure / kPa,Temperature (LM35) / K,Temperature (NTC) / K,Altitude / m,GPS Valid,Latitude,Longitude,Altitude (GPS),Speed,Course");//for the output
}

float bitToVolt(int n)  //Function to convert raw ADC-data (0-1023) to volt (from shield test)
{
  int raw = analogRead(n);
  float volt = (float)raw * vs / 1023;
  return volt;
}

float ntc()             //Function to calculate temperature in K
{
  float v = bitToVolt(0);
  float r = vs * r1 / v - r1;
  float t = 0.97806063 * (1.0 / (a + b * log(r / r1) + c * pow(log(r / r1), 2.0) + d * pow(log(r / r1), 3.0)));
  return t;
}

float pressure()        //Function to calculate pressure in Pa
{
  float v = bitToVolt(1);
  float p = 1.004833 * (((v / vs + pressureOff) / pressureSens) * 1000.0);
  return p;
}

float temp()            //Function to calculate temperature in K
{
  float v = bitToVolt(5);
  float tmp = 1.004783 * ((v - tmpOff) / tmpSens + 273.5);
  return tmp;
}

float accCalc(int axis) //Function to calculate acceleration in g
{
  float off, sens;
  switch (axis)
  {
    case 2:             //x-axis
      off = accXOff;
      break;
    case 3:             //y-axis
      off = accYOff;
      break;
    case 4:             //z-axis
      off = accZOff;
      break;
  }
  if (accSensMode)
  {
    sens = acc6;
  }
  else
  {
    sens = acc15;
  }
  float acc = (bitToVolt(axis) - off) / sens;
  return acc;
}

float altitude()        //Function to calculate altitude in m
{
  static float startTmp = temp();      //Measure start temperature
  static float startPrs = pressure();  //Measure start pressure
  float alt = (startTmp / tmpGrad) * (pow(prs / startPrs, -tmpGrad * R / g) - 1.0);
  return alt;
}

int switchRead()
{
  if (digitalRead(switchPinHelp) == LOW)
  {
    return 1;
  }
  else if (digitalRead(switchPinOk) == LOW)
  {
    return 0;
  }
  else
  {
    return 2;
  }
}

double accX = accCalc(2);
double accY = accCalc(3);
double accZ = accCalc(4);

double accCalcMagnitude()
{
  return sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2));
}

int switchState = switchRead();

//unsigned long latitude = gps.location.rawLat().deg;// * 1000000000 + gps.location.rawLat().billionths;
//unsigned long longitude = gps.location.rawLng().deg;// * 1000000000 + gps.location.rawLng().billionths;

void printData()
{
  Serial.print("MESS");
  Serial.print(delimiter);
  Serial.print(switchState);
  Serial.print(delimiter);
  Serial.print(counter);
  Serial.print(delimiter);
  Serial.print(millis());
  Serial.print(delimiter);
  Serial.print(analogRead(1));
  Serial.print(delimiter);
  Serial.print(analogRead(5));
  Serial.print(delimiter);
  Serial.print(analogRead(0));
  Serial.print(delimiter);
  /*
  Serial.print(analogRead(2));
  Serial.print(delimiter);
  Serial.print(analogRead(3));
  Serial.print(delimiter);
  Serial.print(analogRead(4));
  Serial.print(delimiter);
  */
  if (calculate);
  {
    prs = pressure();
    Serial.print(prs);
    Serial.print(delimiter);
    Serial.print(temp());
    Serial.print(delimiter);
    Serial.print(ntc());
    Serial.print(delimiter);
    /*
    accX = accCalc(2);
    accY = accCalc(3);
    accZ = accCalc(4);
    Serial.print(accX);
    Serial.print(delimiter);
    Serial.print(accY);
    Serial.print(delimiter);
    Serial.print(accZ);
    Serial.print(delimiter);
    Serial.print(accCalcMagnitude());
    Serial.print(delimiter);
    */
    alt = altitude();
    Serial.print(alt);
    Serial.print(delimiter);
  }
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }
  Serial.print(gps.location.isValid());
  Serial.print(delimiter);
  //latitude = gps.location.rawLat().deg;// * 1000000000// + gps.location.rawLat().billionths;
  //longitude = gps.location.rawLng().deg;// * 1000000000// + gps.location.rawLng().billionths;
  Serial.print(gps.location.rawLat().negative ? '-' : '+');
  Serial.print(gps.location.rawLat().deg);
  Serial.print('.');
  Serial.print(gps.location.rawLat().billionths);
  //Serial.print(latitude * 1000000000 + gps.location.rawLat().billionths);
  Serial.print(delimiter);
  Serial.print(gps.location.rawLng().negative ? '-' : '+');
  Serial.print(gps.location.rawLng().deg);
  Serial.print('.');
  Serial.print(gps.location.rawLng().billionths);
  //Serial.print(longitude * 1000000000 + gps.location.rawLng().billionths);
  Serial.print(delimiter);
  Serial.print(gps.altitude.value());
  Serial.print(delimiter);
  Serial.print(gps.speed.value());
  Serial.print(delimiter);
  Serial.println(gps.course.value());
}

void led()
{
  if (ledOn)
  {
    digitalWrite(ledPin, LOW);
    ledOn = false;
  }
  else
  {
    if (!forceSilence && useLed)
    {
      if (alt < alertAltitude || forceAlert)
        digitalWrite(ledPin, HIGH);
    }
    ledOn = true;
  }
  ledToggle = false;
}

/*
void led2()
{
  if (led2On)
  {
    digitalWrite(ledPin2, LOW);
    led2On = false;
  }
  else
  {
    if (!forceSilence)
    {
      if (useLed2)
      {
        if (alt < alertAltitude)
          digitalWrite(ledPin2, HIGH);
        else if (forceAlert)
          digitalWrite(ledPin2, HIGH);
      }
    }
    led2On = true;
  }
  ledToggle2 = false;
}
*/

void buzzer()
{
  if (buzzerOn)
  {
    digitalWrite(buzzerPin, LOW);
    buzzerOn = false;
  }
  else
  {
    if (!forceSilence && useBuzzer)
    {
      if (alt < alertAltitude || forceAlert)
        digitalWrite(buzzerPin, HIGH);
    }
    buzzerOn = true;
  }
  buzzerToggle = false;
}

void manual()
{
  while (Serial.available())
  {
    char rxChar = (char)Serial.read();
    switch (rxChar)
    {
      case '1':
        if (useLed)
          useLed = false;
        else
          useLed = true;
        break;
      /*
      case '2':
        if (useLed2)
          useLed2 = false;
        else
          useLed2 = true;
        break;
      */
      case 'b':
        if (useBuzzer)
          useBuzzer = false;
        else
          useBuzzer = true;
        break;
      case 't':
        if (forceAlert)
          forceAlert = false;
        else
          forceAlert = true;
        break;
      case 's':
        if (forceSilence)
          forceSilence = false;
        else
          forceSilence = true;
        break;
      case 'c':
        if (calculate)
          calculate = false;
        else
          calculate = true;
    }
  }
}

void loop()
{
  static unsigned long loopStart = 0;                                       //Determines whether data should be printed.
  static unsigned long ledStart = 0;                                       //Determines whether LED 1 state should change when altitude < 100.
  //static unsigned long ledStart2 = 0;                                       //Determines whether LED 2 state should change when altitude < 100.
  static unsigned long buzzerStart = 0;
  static unsigned long loopEnd = 0;                                         //Initialized as an arbitrary high value to make everything run the first time.
  //static unsigned long ledDelayOff2 = (ledDelayOn2 + ledDelayOff2Real) / 2; //This sets the first off delay so that the LEDs are in opposite phase.

  if (ledOn)                                //If LED is on…
  {
    if (loopEnd - ledStart > ledDelayOn)  //…and enough time has passed since it last toggled…
    {
      ledToggle = true;                     //…set ledToggle to true, which will make LED 1 toggle the next time led() is called.
      ledStart = millis();                 //Assign a new starting point to compare the loop end time with. This lets us know how long it's been since the LED toggle.
    }                                       //The following conditionals do the same thing, but check if LED 1 is off and if LED 2 is on or off. also buzzer
  }
  else
  {
    if (loopEnd - ledStart > ledDelayOff)
    {
      ledToggle = true;
      ledStart = millis();
    }
  }
  /*
  if (led2On)
  {
    if (loopEnd - ledStart2 > ledDelayOn2)
    {
      ledToggle2 = true;
      ledStart2 = millis();
    }
  }
  else
  {
    if (loopEnd - ledStart2 > ledDelayOff2)
    {
      ledToggle2 = true;
      ledStart2 = millis();
      ledDelayOff2 = ledDelayOff2Real;
    }
  }
  */
  if (buzzerOn)
  {
    if (loopEnd - buzzerStart > buzzerDelayOn)
    {
      buzzerToggle = true;
      buzzerStart = millis();
    }
  }
  else
  {
    if (loopEnd - buzzerStart > buzzerDelayOff)
    {
      buzzerToggle = true;
      buzzerStart = millis();
    }
  }

  switchState = switchRead();
  if (switchState != 2)
    forceSilence = true;

  manual();

  if (ledToggle)
    led();
  //if (ledToggle2)
    //led2();
  if (buzzerToggle)
    buzzer();
  
  if (loopEnd - loopStart > loopTime)
  {
    printData();
    counter++;
    loopStart = millis();
  }

  loopEnd = millis();
}
